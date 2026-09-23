/*
 * tcp_server.c - 简单文本协议的 TCP 服务端，给上位机控制 LED 图案
 *
 * 协议（每条命令以换行 \n 结尾，纯 ASCII，网络调试助手就能测）：
 *   PING                -> 回 PONG（测试连通）
 *   IP                  -> 回当前 IP
 *   CLEAR               -> 全灭
 *   FILL <v>            -> 全屏按亮度 v (0~255) 点亮
 *   PX <x> <y> <v>      -> 设置单点，如: PX 3 4 200
 *   ROW <y> <v0> ... <v9>  -> 整行 10 个亮度
 *   FRAME <v0> ... <v99>   -> 整帧 100 个亮度（行优先）
 *   BLINK <hz>          -> 固件内部定时闪烁（0.5~20 Hz）
 *   BLINK STOP           -> 停止闪烁并熄灭
 *   GRID                -> 回当前帧内容（调试）
 *   FAULT               -> LED 开/短路检测：让芯片自己报"这些 LED 有没有导通"
 *                          （红外看不见 / 手机摄像头看不到 940nm 时，用它拿电气证据）
 *   RREG <addr>         -> 读一个寄存器（验证 SPI 读写通路，地址可写 0x01 或 1）
 *   WREG <addr> <val>   -> 写一个寄存器（实验用，慎用）
 *   DEFAULTS            -> 重跑 lp5860_init()，把芯片恢复成初始化后的已知状态
 *   VERSION             -> 回固件版本号
 *   OTA <url>           -> 从该 URL 下载固件做 WiFi 升级
 *   REBOOT              -> 重启
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>          /* strtol / strdup */
#include <errno.h>           /* errno */
#include <stdbool.h>
#include "tcp_server.h"
#include "ledmatrix.h"
#include "lp5860.h"          /* FAULT / RREG / WREG / DEFAULTS 直接用寄存器级 API */
#include "wifi_app.h"
#include "ota_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

/* 固件版本号：每次 OTA 迭代改这里，方便确认刷没刷上（TCP 命令 VERSION 可查） */
#define FIRMWARE_VERSION  "v3.1"

static const char *TAG = "tcp";
static esp_timer_handle_t blink_timer;
static uint8_t blink_frame[MATRIX_W * MATRIX_H];
static bool blink_on;

static void blink_cb(void *arg)
{
    (void)arg;
    blink_on = !blink_on;
    if (blink_on) matrix_show_frame(blink_frame);
    else matrix_clear();
}

static void blink_stop(void)
{
    if (blink_timer) esp_timer_stop(blink_timer);
    blink_on = false;
}

static bool blink_start(float hz)
{
    if (hz < 0.5f || hz > 20.0f) return false;
    if (!blink_timer) {
        const esp_timer_create_args_t args = { .callback = blink_cb, .name = "led_blink" };
        if (esp_timer_create(&args, &blink_timer) != ESP_OK) return false;
    }
    matrix_get_frame(blink_frame);
    blink_stop();
    blink_on = true;
    matrix_show_frame(blink_frame);
    return esp_timer_start_periodic(blink_timer, (uint64_t)(500000.0f / hz)) == ESP_OK;
}

/* 把一行文本解析成整数数组（容错逗号或空格分隔） */
static int parse_ints(const char *s, int *out, int maxn)
{
    int n = 0;
    while (*s && n < maxn) {
        char *end;
        long v = strtol(s, &end, 10);
        if (end == s) { s++; continue; }        /* 跳过非数字字符 */
        out[n++] = (int)v;
        s = end;
    }
    return n;
}

/* 解析一个数：支持十进制和 0x 前缀的十六进制（寄存器地址用十六进制更自然）。
 * 只给 RREG/WREG 用 —— 其它命令仍走 parse_ints（纯十进制），避免 "PX 010" 被当成八进制。 */
static long parse_num(const char *s, int *ok)
{
    char *end;
    *ok = 0;
    while (*s == ' ') s++;
    long v = strtol(s, &end, 0);        /* base=0：自动识别 0x / 0 前缀 */
    if (end != s) *ok = 1;
    return v;
}

/* 依次解析两个数（WREG <addr> <val> 用）。
 *
 * ⚠️ 这里踩过一个坑：一开始用 strchr(line+4,' ') 找分隔符，但 line+4 的第一个字符
 * 本身就是"WREG"和地址之间那个空格 —— 于是解析出来的"值"其实是地址，
 * `WREG 0x01 0x56` 会把 0x01 写进去（还把全局亮度从 0xFF 误改成 0x05）。
 * 正确做法是用 strtol 的 endptr 连续取两个数，别自己找空格。 */
static int parse_two_nums(const char *s, long *a, long *b)
{
    char *end;
    while (*s == ' ' || *s == '\t') s++;
    *a = strtol(s, &end, 0);
    if (end == s) return 0;                 /* 第一个数没解析出来 */
    s = end;
    while (*s == ' ' || *s == '\t') s++;
    *b = strtol(s, &end, 0);
    if (end == s) return 0;                 /* 第二个数没解析出来 */
    return 1;
}

/* 处理一条命令，返回要回复的文本（写入 resp）；conn 用于 REBOOT 前先把应答发出去 */
static void handle_line(char *line, char *resp, size_t rsz, int conn)
{
    int vals[MATRIX_W * MATRIX_H];

    if (!strncmp(line, "PING", 4)) {
        snprintf(resp, rsz, "PONG\n");
    } else if (!strncmp(line, "IP", 2)) {
        snprintf(resp, rsz, "IP %s\n", g_ip_addr);
    } else if (!strncmp(line, "CLEAR", 5)) {
        blink_stop();
        matrix_clear();
        snprintf(resp, rsz, "OK\n");
    } else if (!strncmp(line, "FILL", 4)) {
        blink_stop();
        int n = parse_ints(line + 4, vals, 1);
        matrix_fill((uint8_t)(n == 1 ? vals[0] : 255));
        snprintf(resp, rsz, "OK\n");
    } else if (!strncmp(line, "PX", 2)) {
        blink_stop();
        int n = parse_ints(line + 2, vals, 3);
        if (n == 3) {
            esp_err_t e = set_led(vals[0], vals[1], (uint8_t)vals[2]);
            snprintf(resp, rsz, e == ESP_OK ? "OK\n" : "ERR X/Y 越界\n");
        } else {
            snprintf(resp, rsz, "ERR 用法: PX x y v\n");
        }
    } else if (!strncmp(line, "ROW", 3)) {
        blink_stop();
        int n = parse_ints(line + 3, vals, 1 + MATRIX_W);
        if (n == 1 + MATRIX_W && vals[0] >= 0 && vals[0] < MATRIX_H) {
            for (int x = 0; x < MATRIX_W; x++)
                set_led(x, vals[0], (uint8_t)vals[1 + x]);
            snprintf(resp, rsz, "OK\n");
        } else {
            snprintf(resp, rsz, "ERR 用法: ROW y v0..v9\n");
        }
    } else if (!strncmp(line, "FRAME", 5)) {
        blink_stop();
        int n = parse_ints(line + 5, vals, MATRIX_W * MATRIX_H);
        if (n == MATRIX_W * MATRIX_H) {
            uint8_t frame[MATRIX_W * MATRIX_H];
            for (int i = 0; i < n; i++) frame[i] = (uint8_t)vals[i];
            matrix_show_frame(frame);
            snprintf(resp, rsz, "OK\n");
        } else {
            snprintf(resp, rsz, "ERR 需要 %d 个亮度值，收到 %d\n", MATRIX_W * MATRIX_H, n);
        }
    } else if (!strncmp(line, "BLINK STOP", 10)) {
        blink_stop();
        matrix_clear();
        snprintf(resp, rsz, "BLINK STOPPED\n");
    } else if (!strncmp(line, "BLINK", 5)) {
        char *end;
        float hz = strtof(line + 5, &end);
        if (end == line + 5 || !blink_start(hz))
            snprintf(resp, rsz, "ERR 用法: BLINK <0.5..20 Hz> 或 BLINK STOP\n");
        else
            snprintf(resp, rsz, "BLINK %.2f Hz\n", hz);
    } else if (!strncmp(line, "GRID", 4)) {
        uint8_t f[MATRIX_W * MATRIX_H];
        matrix_get_frame(f);
        int off = 0;
        off += snprintf(resp + off, rsz - off, "GRID\n");
        for (int y = 0; y < MATRIX_H && off < (int)rsz - 8; y++) {
            for (int x = 0; x < MATRIX_W; x++)
                off += snprintf(resp + off, rsz - off, "%3d ", f[y * MATRIX_W + x]);
            off += snprintf(resp + off, rsz - off, "\n");
        }
    } else if (!strncmp(line, "VERSION", 7)) {
        snprintf(resp, rsz, "VERSION %s\n", FIRMWARE_VERSION);
    } else if (!strncmp(line, "FAULT", 5)) {
        /* LED 开路/短路检测：红外灯本来就看不见，很多手机摄像头也看不到 940nm，
         * 所以"看不见光"不能当证据。这里让 LP5860 自己报告 LED 有没有真的导通。
         *
         * ⚠️ 检测要求 PWM 达标才会触发（Mode 1/2 需 ≥25），所以本命令先把**整屏点亮**，
         *    这意味着它的结论是在"全屏 255"下测出来的，**会覆盖你之前设的图案**。
         *    想单独验证某一列，别用 FAULT：CLEAR + PX 点那一列，再 WREG 0xA7 0xFF
         *    清标志、读 0x64/0x65 即可（注意 LOD 标志是锁存的，不清就是历史值）。 */
        matrix_fill(255);
        vTaskDelay(pdMS_TO_TICKS(30));          /* 留几个扫描周期让检测电路出结论 */
        lp5860_fault_check(resp, rsz);
        size_t used = strlen(resp);
        if (used + 48 < rsz)                    /* 位置够才补这句，避免截断 */
            snprintf(resp + used, rsz - used,
                     "  (矩阵保持全亮，方便同时用摄像头看)\n");
    } else if (!strncmp(line, "RREG", 4)) {
        /* 读一个寄存器：RREG <addr>  地址可写十进制或 0x 十六进制
         * 用来验证 SPI 读写通路：写进去再读回来一致，才算真的通了。 */
        int ok = 0;
        long a = parse_num(line + 4, &ok);
        if (ok && a >= 0 && a <= 0x3FF) {
            uint8_t v = 0;
            esp_err_t e = lp5860_read_reg((uint16_t)a, &v);
            snprintf(resp, rsz, e == ESP_OK ? "RREG 0x%03lX = 0x%02X\n" : "ERR 读取失败\n", a, v);
        } else {
            snprintf(resp, rsz, "ERR 用法: RREG <addr>   (addr 0 ~ 0x3FF，可写 0x01 或 1)\n");
        }
    } else if (!strncmp(line, "WREG", 4)) {
        /* 写一个寄存器：WREG <addr> <val>
         * ⚠️ 慎用：直接改芯片内部寄存器，写错（例如把 Maximum_Current 拉到最大）
         * 可能让 LED 过流。正常显示不需要它，只做实验时用。 */
        int ok1 = 0, ok2 = 0;
        long a = parse_num(line + 4, &ok1);
        long v = 0;
        if (ok1) ok2 = (int)parse_two_nums(line + 4, &a, &v);
        if (ok1 && ok2 && a >= 0 && a <= 0x3FF && v >= 0 && v <= 0xFF) {
            esp_err_t e = lp5860_write_reg((uint16_t)a, (uint8_t)v);
            snprintf(resp, rsz, e == ESP_OK ? "WREG 0x%03lX <- 0x%02lX\n" : "ERR 写入失败\n", a, v);
        } else {
            snprintf(resp, rsz, "ERR 用法: WREG <addr> <val>   (例: WREG 0x01 0x56)\n");
        }
    } else if (!strncmp(line, "DEFAULTS", 8)) {
        /* 重跑一遍 lp5860_init()：把芯片恢复到上电初始化后的已知状态。
         * 用 WREG 把亮度/配置改乱了之后，不用整机重启就能恢复（重启会断 TCP、要重连）。 */
        esp_err_t e = lp5860_init();
        matrix_clear();
        snprintf(resp, rsz, e == ESP_OK ? "OK 已重新初始化 LP5860，显示已清空\n"
                                       : "ERR 重新初始化失败\n");
    } else if (!strncmp(line, "REBOOT", 6)) {
        snprintf(resp, rsz, "OK 重启中...\n");
        send(conn, resp, strlen(resp), 0);                /* 先把应答发出去再重启 */
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (!strncmp(line, "OTA ", 4)) {
        /* 例: OTA http://192.168.1.50:8000/led_matrix_wifi.bin
         * 立即回 OTA START，之后升级进度看串口，成功会自动重启（连接断开） */
        ota_start(line + 4);
        snprintf(resp, rsz, "OTA START %s\n", line + 4);
    } else {
        snprintf(resp, rsz, "ERR 未知命令\n");
    }
}

/* 服务任务：一个客户端一个循环，断开后回到 accept 等下一个 */
static void tcp_server_task(void *arg)
{
    /* 这三个缓冲区加起来近 3KB，放栈上会把任务栈挤爆（曾是栈溢出崩溃的根因），
     * 所以改成 static。本任务全程只跑一个实例、命令串行处理，不存在重入问题。 */
    static char rxbuf[1024];        /* 一次 recv 收上来的原始字节 */
    static char line[512];          /* 攒出一整行命令 */
    static char resp[1400];         /* 待发回的应答（GRID 最长约 420 字节，余量充足） */

    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd < 0) {
        ESP_LOGE(TAG, "socket() 创建失败: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in addr = { 0 };
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);   /* 监听本机所有网卡 */
    addr.sin_port        = htons(TCP_SERVER_PORT);
    /* 允许地址重用：OTA/重启后立刻重连不会因为 TIME_WAIT 而 bind 失败 */
    int opt = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (bind(listenfd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind 失败: errno=%d（端口 %d 被占？）", errno, TCP_SERVER_PORT);
    }
    listen(listenfd, 1);
    ESP_LOGI(TAG, "TCP 服务已启动，端口 %d，等上位机连接...", TCP_SERVER_PORT);

    while (1) {
        int conn = accept(listenfd, NULL, NULL);
        if (conn < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        /* 打印栈高水位（uxTaskGetStackHighWaterMark 返回的是"历史最低剩余字节数"，
         * ESP-IDF 下单位为字节）。这个数字越小越危险，接近 0 就该加栈了——
         * 上次 tcp_srv 栈溢出就是靠这类观测点才能提前发现。 */
        ESP_LOGI(TAG, "上位机已连接（本任务栈历史最低剩余 %u 字节）",
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
        snprintf(resp, sizeof(resp), "HELLO %s\n", g_ip_addr);
        send(conn, resp, strlen(resp), 0);

        int got = 0;                            /* line 里已积累的字节数 */
        while (1) {
            int len = recv(conn, rxbuf, sizeof(rxbuf), 0);
            if (len <= 0) break;                /* 对端断开 */
            for (int i = 0; i < len; i++) {
                char c = rxbuf[i];
                if (c == '\n') {                /* 一条命令结束 */
                    line[got] = '\0';
                    /* 关键：去掉行尾的 \r 和空格。
                     * 很多客户端（PowerShell 的 WriteLine、telnet、串口助手默认设置）
                     * 发的是 CRLF，只按 \n 切分会把 \r 留在字符串里。
                     * PX/ROW/FRAME 靠 strtol 解析数字不受影响，但 OTA 命令是**整串取用**，
                     * 尾巴上多个 \r 会让 URL 变成 ...bin\r -> 服务器 404 -> 升级失败。 */
                    while (got > 0 && (line[got - 1] == '\r' ||
                                       line[got - 1] == ' '  ||
                                       line[got - 1] == '\t')) {
                        line[--got] = '\0';
                    }
                    handle_line(line, resp, sizeof(resp), conn);
                    send(conn, resp, strlen(resp), 0);
                    got = 0;
                } else if (got < (int)sizeof(line) - 1) {
                    line[got++] = c;            /* 攒字符直到换行 */
                }
            }
        }
        close(conn);
        ESP_LOGW(TAG, "上位机断开，继续等待新连接");
    }
}

esp_err_t tcp_server_start(void)
{
    /* 栈给 8192：缓冲区虽已改 static，但 handle_line 内部还有 vals[400B]/frame[100B]，
     * 以及 lwip 的 recv/send 调用链较深，留足余量（S3 内存充裕，不必省这点）。 */
    BaseType_t ok = xTaskCreate(tcp_server_task, "tcp_srv", 8192, NULL, 5, NULL);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}
