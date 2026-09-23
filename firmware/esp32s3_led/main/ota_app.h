/*
 * ota_app.h - WiFi OTA 升级模块
 *
 * 用法：给一个 http:// 开头的固件 .bin 下载地址即可，
 *       例如 http://192.168.1.50:8000/led_matrix_wifi.bin
 * 流程：下载 -> 校验 -> 写入备用分区 -> 切启动标志 -> 自动重启
 */
#pragma once

/** 启动 OTA（内部建任务异步执行，立即返回；成功后设备会自己重启） */
void ota_start(const char *url);
