/*
 * wifi_app.c - 连接公司内网 Delta-device，拿到 IP 后打印到串口
 */
#include <string.h>
#include "wifi_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* ---------------- WiFi 参数：改网络只改这两行 ---------------- */
#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"

#define RECONNECT_DELAY_US  (3 * 1000 * 1000)   /* 断线后 3 秒重连 */

static const char *TAG = "wifi";
static EventGroupHandle_t s_events;
static esp_timer_handle_t s_retry_timer;
#define EVT_CONNECTED  BIT0
#define EVT_FAIL       BIT1

char g_ip_addr[16] = "0.0.0.0";

/* 定时器回调：3 秒到点后发起重连。
 *
 * 为什么不直接在事件回调里 vTaskDelay 再重连？
 *   事件回调跑在系统事件循环任务（sys_evt）上，在那里阻塞 3 秒会把整个事件循环
 *   卡住——期间 WiFi/IP 事件全部排队无法处理。改成定时器延时，事件循环立刻返回。 */
static void retry_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "发起重连...");
    esp_wifi_connect();
}

/* 事件回调：WiFi 状态机推进全在这里（必须快速返回，不许阻塞） */
static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();                       /* STA 启动，开始连 AP */
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi 断开，3 秒后重连...");
        /* 延时交给定时器，事件循环立即返回。已在计时则重置，避免重连风暴 */
        if (esp_timer_is_active(s_retry_timer)) {
            esp_timer_restart(s_retry_timer, RECONNECT_DELAY_US);
        } else {
            esp_timer_start_once(s_retry_timer, RECONNECT_DELAY_US);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        /* 拿到 DHCP 分配的 IP —— 就是你串口里要看的那个 */
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(g_ip_addr, sizeof(g_ip_addr), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  WiFi 已连接，IP 地址: %s", g_ip_addr);
        ESP_LOGI(TAG, "========================================");
        xEventGroupSetBits(s_events, EVT_CONNECTED);
    }
}

esp_err_t wifi_app_start(void)
{
    /* NVS 是 WiFi 驱动的依赖，先初始化（失败通常是首次格式化，重试一次） */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    s_events = xEventGroupCreate();

    /* 重连定时器：只创建一次，之后重复使用（one-shot 模式，用 restart 复用） */
    const esp_timer_create_args_t targs = {
        .callback = &retry_timer_cb,
        .name     = "wifi_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_retry_timer));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册两类事件：WiFi 自身状态 + IP 层拿地址 */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;   /* 允许开放/任意加密方式，兼容公司 AP */

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 死等连上（事件里会置位）；实际产品里可以加超时 */
    EventBits_t bits = xEventGroupWaitBits(s_events, EVT_CONNECTED | EVT_FAIL,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    return (bits & EVT_CONNECTED) ? ESP_OK : ESP_FAIL;
}
