/*
 * ledmatrix.c - 10x10 矩阵帧缓冲 + 到 LP5860 点阵的映射
 *
 * 映射关系（依数据手册 Table 8-64）：
 *   行 y（SW 扫描线）-> 点索引基址 y*18
 *   列 x（CS 通道）  -> +x
 *   Mode 1（8-bit PWM）：PWM 地址 = 0x200 + 点索引，**每个点 1 字节**，值就是 0~255
 *   （Mode 3 则是 0x200 + 点索引*2、每点 2 字节、低字节在前 —— 两条路完全不同，别混）
 */
#include <string.h>
#include "ledmatrix.h"
#include "lp5860.h"
#include "esp_log.h"

/* 帧缓冲：100 字节，fb[y*10+x] = (x,y) 的亮度，行优先 */
static uint8_t s_fb[MATRIX_W * MATRIX_H];

esp_err_t ledmatrix_init(void)
{
    esp_err_t err = lp5860_init();
    if (err != ESP_OK) return err;
    memset(s_fb, 0, sizeof(s_fb));
    return ESP_OK;
}

esp_err_t set_led(int x, int y, uint8_t v)
{
    if (x < 0 || x >= MATRIX_W || y < 0 || y >= MATRIX_H) return ESP_ERR_INVALID_ARG;
    s_fb[y * MATRIX_W + x] = v;
    /* 单点直写：Mode 1 下 PWM 地址 = 0x200 + (行*18+列)，只动这一个点。
     * 亮度值直接就是 0~255 的占空比，不需要再做位扩展。 */
    return lp5860_set_dot_pwm(y * LP5860_NUM_CS + x, v);
}

esp_err_t set_leds(const uint8_t (*pts)[2], int n, uint8_t level)
{
    for (int i = 0; i < n; i++) {
        esp_err_t err = set_led(pts[i][0], pts[i][1], level);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t matrix_fill(uint8_t v)
{
    memset(s_fb, v, sizeof(s_fb));
    /* 连写 10 行 × 18 通道 = 180 个点（其中 CS10~17 没接 LED，写 0 即可）。
     * Mode 1 下每点 1 字节，一次 SPI 传输 180 字节 + 2 地址字节，约 1.5ms，无撕裂。 */
    uint8_t line[LP5860_NUM_LINES * LP5860_NUM_CS] = { 0 };
    for (int y = 0; y < MATRIX_H; y++)
        for (int x = 0; x < MATRIX_W; x++)
            line[y * LP5860_NUM_CS + x] = v;
    return lp5860_write_pwm_burst(0, line, LP5860_NUM_LINES * LP5860_NUM_CS);
}

esp_err_t matrix_clear(void)
{
    return matrix_fill(0);
}

esp_err_t matrix_show_frame(const uint8_t *frame100)
{
    memcpy(s_fb, frame100, sizeof(s_fb));
    uint8_t line[LP5860_NUM_LINES * LP5860_NUM_CS] = { 0 };
    for (int y = 0; y < MATRIX_H; y++)
        for (int x = 0; x < MATRIX_W; x++)
            line[y * LP5860_NUM_CS + x] = frame100[y * MATRIX_W + x];
    return lp5860_write_pwm_burst(0, line, LP5860_NUM_LINES * LP5860_NUM_CS);
}

void matrix_get_frame(uint8_t *out100)
{
    memcpy(out100, s_fb, sizeof(s_fb));
}
