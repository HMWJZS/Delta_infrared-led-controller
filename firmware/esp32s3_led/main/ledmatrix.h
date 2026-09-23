/*
 * ledmatrix.h - 10x10 红外 LED 矩阵显示接口（对上层暴露的唯一 API）
 *
 * 屏幕坐标约定：
 *   (0,0) = 左上角
 *   x: 0~9 从左到右  -> 对应 LP5860 的 CS0~CS9（列阴极）
 *   y: 0~9 从上到下  -> 对应 LP5860 的 SW0~SW9（行阳极/扫描线）
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#define MATRIX_W 10
#define MATRIX_H 10

/** 初始化矩阵（内部调用 lp5860_init，并清屏） */
esp_err_t ledmatrix_init(void);

/**
 * 点亮/熄灭一个点 —— 你要的那个函数
 * @param x 0~9 列（从左到右）
 * @param y 0~9 行（从上到下）
 * @param v 亮度 0~255，0=灭
 */
esp_err_t set_led(int x, int y, uint8_t v);

/** 批量设置：pts 是 (x,y) 坐标对数组，n 个点，全部按 level 亮度点亮 */
esp_err_t set_leds(const uint8_t (*pts)[2], int n, uint8_t level);

/** 全屏填同一亮度（0=全灭） */
esp_err_t matrix_fill(uint8_t v);

/** 全灭（等价 matrix_fill(0)） */
esp_err_t matrix_clear(void);

/** 把一整帧（100 字节，行优先）写到屏幕上 */
esp_err_t matrix_show_frame(const uint8_t *frame100);

/** 读回当前帧（调试用） */
void matrix_get_frame(uint8_t *out100);
