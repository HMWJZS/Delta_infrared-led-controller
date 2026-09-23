/*
 * tcp_server.h - 上位机控制用的 TCP 服务
 */
#pragma once
#include "esp_err.h"

#define TCP_SERVER_PORT  8266

/** 启动 TCP 服务（内部建 FreeRTOS 任务，永久监听） */
esp_err_t tcp_server_start(void);
