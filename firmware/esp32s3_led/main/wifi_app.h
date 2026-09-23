/*
 * wifi_app.h - WiFi STA 连接（Delta-device）
 */
#pragma once
#include "esp_err.h"

/* 连接成功后保存的 IP 字符串（如 "192.168.1.100"），供全局使用 */
extern char g_ip_addr[16];

/** 启动 WiFi 并阻塞直到拿到 IP；失败返回错误码 */
esp_err_t wifi_app_start(void);
