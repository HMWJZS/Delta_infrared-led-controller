/*
 * ota_app.c - 通过 HTTP 下载固件并写入备用分区的 OTA 实现
 *
 * 原理（对应 STM32 的 IAP 思路）：
 *   Flash 里有 ota_0 / ota_1 两个固件区，当前跑 A 区时，
 *   把新固件下载写进 B 区 -> esp_ota_set_boot_partition 切启动标志 -> 重启进 B 区。
 *   写坏/校验不过，ROM 引导程序会回退到旧分区，不会变砖（升级中断电除外）。
 */
#include <string.h>
#include "ota_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "ota";

#define OTA_BUF_SIZE   4096        /* 每次下载写入的分块大小 */
#define OTA_TASK_STACK 8192

/* OTA 任务主体：下载 -> 写分区 -> 切启动 -> 重启 */
static void ota_task(void *arg)
{
    char *url = (char *)arg;                     /* 由启动方 strdup 传入 */
    char *buf = malloc(OTA_BUF_SIZE);
    if (!buf) { ESP_LOGE(TAG, "内存不足"); goto fail_free_url; }

    /* ① 打开 HTTP 连接 */
    esp_http_client_config_t cfg = { .url = url, .timeout_ms = 10000 };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) { ESP_LOGE(TAG, "URL 无效: %s", url); goto fail_free_buf; }

    esp_err_t err = esp_http_client_open(cli, 0);          /* 发起 GET */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "连接失败: %s（检查电脑上的 http 服务和 IP）", esp_err_to_name(err));
        esp_http_client_cleanup(cli);
        goto fail_free_buf;
    }
    int status = esp_http_client_fetch_headers(cli);
    status = esp_http_client_get_status_code(cli);
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP 状态码 %d，确认文件路径存在", status);
        esp_http_client_cleanup(cli);
        goto fail_free_buf;
    }
    int64_t total = esp_http_client_get_content_length(cli);
    ESP_LOGI(TAG, "开始下载 %s（%d 字节）", url, (int)total);

    /* ② 找到备用（非当前运行的）app 分区并开始 OTA 会话 */
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) { ESP_LOGE(TAG, "找不到备用分区，检查分区表"); esp_http_client_cleanup(cli); goto fail_free_buf; }
    esp_ota_handle_t h;
    err = esp_ota_begin(part, total > 0 ? (size_t)total : OTA_SIZE_UNKNOWN, &h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_ota_begin 失败: %s", esp_err_to_name(err)); esp_http_client_cleanup(cli); goto fail_free_buf; }

    /* ③ 边下边写，每 25% 打一次进度
     *
     * 循环退出条件不能只看 read 返回 0：在跨网段、链路有抖动的情况下，
     * 单次 read 返回 0 可能只是"这一瞬间没数据"，不代表下载结束。
     * 如果据此提前跳出，esp_ota_end 会因长度不符而校验失败（不会变砖，但白下一次）。
     * 所以：连续多次空读才判定结束，同时用 is_complete_data_received 做正判。 */
    int written = 0, next_mark = 25, idle = 0;
    while (1) {
        int n = esp_http_client_read(cli, buf, OTA_BUF_SIZE);
        if (n < 0) { ESP_LOGE(TAG, "下载中断"); esp_ota_abort(h); esp_http_client_cleanup(cli); goto fail_free_buf; }
        if (n == 0) {
            if (esp_http_client_is_complete_data_received(cli)) break;   /* 确实下完了 */
            if (++idle >= 200) {                                         /* 空转 2 秒 = 断流 */
                ESP_LOGE(TAG, "下载卡住（已收到 %d 字节），放弃本次升级", written);
                esp_ota_abort(h); esp_http_client_cleanup(cli); goto fail_free_buf;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        idle = 0;
        err = esp_ota_write(h, buf, n);
        if (err != ESP_OK) { ESP_LOGE(TAG, "写入 Flash 失败"); esp_ota_abort(h); esp_http_client_cleanup(cli); goto fail_free_buf; }
        written += n;
        if (total > 0 && written * 100 / (int)total >= next_mark) {
            ESP_LOGI(TAG, "进度 %d%% (%d/%d)", next_mark, written, (int)total);
            next_mark += 25;
        }
    }
    esp_http_client_cleanup(cli);
    ESP_LOGI(TAG, "下载完成，共 %d 字节", written);

    /* ④ 收尾校验 + 切换启动分区 + 重启（重启后跑新固件） */
    err = esp_ota_end(h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "固件校验失败(%s)，已放弃本次升级", esp_err_to_name(err)); goto fail_free_buf; }
    err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) { ESP_LOGE(TAG, "设置启动分区失败"); goto fail_free_buf; }
    ESP_LOGI(TAG, "OTA 完成！1 秒后重启进入新固件...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

fail_free_buf:
    free(buf);
fail_free_url:
    free(url);
    vTaskDelete(NULL);                           /* 任务自杀，设备继续跑旧固件 */
}

void ota_start(const char *url)
{
    /* strdup 一份 URL 传给任务（任务里 free），立即返回不阻塞 TCP 应答 */
    char *u = strdup(url);
    if (xTaskCreate(ota_task, "ota", OTA_TASK_STACK, u, 5, NULL) != pdPASS) {
        free(u);
        ESP_LOGE(TAG, "OTA 任务创建失败");
    }
}
