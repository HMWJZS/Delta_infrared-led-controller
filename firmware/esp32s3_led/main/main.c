/*
 * main.c - ESP32-S3 + LP5860 10x10 红外 LED 矩阵 + WiFi 上位机控制
 *
 * 上电流程：
 *   1. 初始化 LP5860（SPI + 读回自检）
 *   2. 连 WiFi，串口打印分配到的 IP
 *   3. 启动 TCP 服务（端口 8266），等上位机发指令
 *
 * 显示策略：上电默认全灭，画面完全由 TCP 命令控制。
 * （固件里不要加"开机自播图案"之类的代码 —— 每次重启/OTA 都会盖掉上位机设的图案。）
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"          /* esp_restart / esp_get_free_heap_size */
#include "ledmatrix.h"
#include "wifi_app.h"
#include "tcp_server.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 LED 矩阵启动 ===");

    /* 1. LED 驱动初始化
     *    注意：SPI 读回自检失败也不停机——读不通未必写不通（常见是 MISO 没接线）。
     *    真正的证据是灯亮不亮，所以这里无论结果都继续往下跑。 */
    if (ledmatrix_init() != ESP_OK) {
        ESP_LOGW(TAG, "LP5860 初始化异常，仍继续运行（看灯亮不亮来判断写方向）");
    }
    matrix_clear();              /* 上电全灭，画面等上位机发命令 */

    /* 2. 连 WiFi，阻塞直到拿到 IP（IP 同时打印到串口、存入 g_ip_addr） */
    if (wifi_app_start() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi 连接失败，重启重试");
        esp_restart();
    }

    /* 3. 启动 TCP 服务，等上位机 */
    tcp_server_start();

    /* 4. 主循环：每 15 秒把 IP 和关键资源重新打一遍，方便随时确认状态
     *    - 可用堆：OTA 需要 malloc(4KB) 下载缓冲 + HTTP 客户端，堆太少会升级失败
     *    - 顺带当"心跳"，看到这行就说明程序还活着没进崩溃循环 */
    while (1) {
        ESP_LOGI(TAG, ">>> 本机 IP: %s   TCP 端口: %d   可用堆: %u 字节 <<<",
                 g_ip_addr, TCP_SERVER_PORT, (unsigned)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}
