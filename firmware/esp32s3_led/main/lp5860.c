/*
 * lp5860.c - TI LP5860 SPI 驱动实现
 * 寄存器地址/位定义来自 TI 数据手册 SNVSBU8A 与寄存器手册 SNVU786
 */
#include <string.h>
#include <stdlib.h>         /* malloc/free：批量写寄存器要用 */
#include "lp5860.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "lp5860";

static spi_device_handle_t s_spi;   /* SPI 设备句柄（CS 由硬件控制） */

/* ---------------- 底层传输 ---------------- */

/*
 * 构造 SPI 帧并写入。
 *
 * 帧格式（数据手册 Table 8-7 + Figure 8-17/8-18）：
 *   字节1 = 寄存器地址 bit[9:2]      （Bit7=A9 … Bit0=A2，MSB 先发）
 *   字节2 = bit[7]=A1  bit[6]=A0  bit[5]=R/W(0=读 1=写)  bit[4:0]=Don't Care
 *   之后  = 数据字节，寄存器地址自动递增，可一次连写多个
 *
 * ⚠️ byte2 的位排列是最容易写错、且后果最严重的一处（曾经把整套系统写瘫）：
 *   地址低 2 位必须放在 bit[7:6]、读写位放在 bit[5]。若按"低位靠后"的习惯写成
 *   bit[2:1]/bit[0]，芯片会把所有写命令当成读命令、地址也解析错，
 *   表现为配置一个字节都进不去、灯永远不亮。改这里务必回归一次显示验证。
 */
static esp_err_t lp5860_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = { 0 };
    t.length    = len * 8;          /* 单位是 bit */
    t.tx_buffer = tx;
    t.rx_buffer = rx;               /* 全双工：读时同 length */
    t.rxlength  = len * 8;
    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI 传输失败: %s", esp_err_to_name(err));
    }
    return err;
}

/* byte2 的组装：地址低 2 位在 bit[7:6]，读写位在 bit[5]（0=读 1=写） */
#define LP5860_ADDR_BYTE2(addr, is_write) \
    ((uint8_t)((((addr) & 0x03) << 6) | ((is_write) ? 0x20 : 0x00)))

esp_err_t lp5860_write_reg(uint16_t addr, uint8_t val)
{
    uint8_t buf[3];
    buf[0] = (addr >> 2) & 0xFF;                        /* 地址 bit[9:2] */
    buf[1] = LP5860_ADDR_BYTE2(addr, 1);                 /* bit5 = 1 -> 写 */
    buf[2] = val;
    return lp5860_transfer(buf, NULL, 3);
}

esp_err_t lp5860_write_burst(uint16_t addr, const uint8_t *data, size_t len)
{
    /* 地址 2 字节 + 数据 len 字节，一次 CS 低电平内发完，芯片自动递增地址 */
    uint8_t *buf = malloc(2 + len);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = (addr >> 2) & 0xFF;
    buf[1] = LP5860_ADDR_BYTE2(addr, 1);
    memcpy(&buf[2], data, len);
    esp_err_t err = lp5860_transfer(buf, NULL, 2 + len);
    free(buf);
    return err;
}

esp_err_t lp5860_read_reg(uint16_t addr, uint8_t *val)
{
    uint8_t tx[3] = { 0 }, rx[3] = { 0 };
    tx[0] = (addr >> 2) & 0xFF;
    tx[1] = LP5860_ADDR_BYTE2(addr, 0);                 /* bit5 = 0 -> 读 */
    tx[2] = 0x00;                                       /* 哑元字节，读回数据在这里 */
    esp_err_t err = lp5860_transfer(tx, rx, 3);
    if (err == ESP_OK) *val = rx[2];
    return err;
}

esp_err_t lp5860_set_dot_pwm(int dot, uint8_t v)
{
    if (dot < 0 || dot >= LP5860_NUM_DOTS) return ESP_ERR_INVALID_ARG;
    /* Mode 1（8-bit PWM）：地址 = 0x200 + 点索引，每个点 1 字节。
     * 别照 Mode 3 的 16-bit 写法去乘 2（手册 Table 8-64）：那是另一套地址，
     * 且字节序为小端（低字节在前）。 */
    return lp5860_write_burst(LP5860_REG_PWM + dot, &v, 1);
}

esp_err_t lp5860_write_pwm_burst(int start_dot, const uint8_t *pwm, size_t ndots)
{
    if (start_dot + (int)ndots > LP5860_NUM_DOTS) return ESP_ERR_INVALID_ARG;
    /* Mode 1：每个点就是 1 个字节，直接连写，无需位扩展 */
    return lp5860_write_burst(LP5860_REG_PWM + start_dot, pwm, ndots);
}

/* ---------------- 初始化 ---------------- */

/*
 * 挂载 SPI 总线 + LP5860 设备。
 * 允许重复调用（DEFAULTS 命令会重跑 init）：先摘掉旧设备句柄，避免重复挂载泄漏。
 */
static esp_err_t lp5860_bus_setup(void)
{
    if (s_spi) { spi_bus_remove_device(s_spi); s_spi = NULL; }

    /* ① 挂载 SPI 主机总线（ESP32-S3 的 FSPI；IO10~13 是它的原生 IO_MUX 引脚） */
    spi_bus_config_t bus = { 0 };
    bus.mosi_io_num = LP5860_MOSI_GPIO;
    bus.miso_io_num = LP5860_MISO_GPIO;
    bus.sclk_io_num = LP5860_SCLK_GPIO;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    /* ② 挂载 LP5860 设备：数据在 SCLK 上升沿采样 -> SPI mode 0；
     *    CS 由硬件自动控制（传输时拉低、结束拉高，拉高即复位芯片接口） */
    spi_device_interface_config_t dev = { 0 };
    dev.clock_speed_hz = LP5860_SPI_HZ;
    dev.mode           = 0;
    dev.spics_io_num   = LP5860_CS_GPIO;
    dev.queue_size     = 1;
    return spi_bus_add_device(SPI2_HOST, &dev, &s_spi);
}

esp_err_t lp5860_init(void)
{
    esp_err_t err = lp5860_bus_setup();
    if (err != ESP_OK) return err;

    /* ③ 必须先使能芯片，再写任何配置寄存器。
     *
     * Chip_EN=0 时芯片处于 STANDBY，手册 8.4 原文："the I²C/SPI are still available
     * for Chip_EN only" —— 此时除 Chip_EN 外的所有写入都被丢弃。
     * 若按"先关芯片→写配置→最后使能"的顺序写，配置会全部丢失，而读回值恰好等于
     * 出厂默认值，日志上看不出任何异常。可用 RREG/WREG 复现：
     *     Chip_EN=0 写 0x05=0xAA → 读回 0xFF（被忽略）；Chip_EN=1 写 0xBB → 读回 0xBB。 */
    lp5860_write_reg(LP5860_REG_CHIP_EN, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));                      /* 手册要求使能后延时 t_chip_en */

    /* ④ 先把所有点关掉，再清 PWM：否则残留的 PWM 数据会立刻亮出来 */
    uint8_t onoff_off[LP5860_ONOFF_REGS];
    memset(onoff_off, 0x00, sizeof(onoff_off));
    lp5860_write_burst(LP5860_REG_DOT_ONOFF, onoff_off, LP5860_ONOFF_REGS);

    /* ⑤ PWM SRAM 清零（全灭），避免上电乱亮。Mode 1 下每点 1 字节，共 198 字节 */
    uint8_t zero[LP5860_NUM_DOTS];
    memset(zero, 0, sizeof(zero));
    lp5860_write_pwm_burst(0, zero, LP5860_NUM_DOTS);

    /* ⑥ 扫描行数 / 刷新模式 / PWM 频率 */
    lp5860_write_reg(LP5860_REG_DEV_INITIAL, LP5860_DEV_INITIAL_MODE1);

    /* ⑦ 配置寄存器：Dev_config3 的 MC 拉到 125mA（LP5860T 顶格）。
     *    出厂默认 MC=37.5mA 且 CC=50.4%，实际每点仅约 9.5~19mA。
     *    ⚠️ 125mA 的前提是 VLED 电源能扛峰值（10 点/行 × 125mA ≈ 1.25A），
     *       散热不足时降档：0x47=37.5 / 0x49=50 / 0x4B=75 / 0x4D=100 / 0x4F=125mA */
    lp5860_write_reg(LP5860_REG_DEV_CONFIG1, 0x00);
    lp5860_write_reg(LP5860_REG_DEV_CONFIG2, 0x00);
    lp5860_write_reg(LP5860_REG_DEV_CONFIG3, LP5860_DEV_CONFIG3_MC125);

    /* ⑦b 色组电流(CC) 0x40(50.4%) -> 0x7F(100%)。
     *     18 个电流阱固定分 3 组、每组 6 个通道，CS0~CS9 三组都有份，三个都要写。
     *     放在 init 里随使能周期配置：模拟增益在运行中写不一定立刻生效。 */
    lp5860_write_reg(LP5860_REG_CC_GROUP1, 0x7F);
    lp5860_write_reg(LP5860_REG_CC_GROUP2, 0x7F);
    lp5860_write_reg(LP5860_REG_CC_GROUP3, 0x7F);

    /* ⑧ 全局亮度拉满 */
    lp5860_write_reg(LP5860_REG_GLOBAL_BRI, 0xFF);

    /* ⑨ 每点点电流校正(DC) 全部 0xFF（不做每点衰减） */
    uint8_t dc[LP5860_NUM_DOTS];
    memset(dc, 0xFF, sizeof(dc));
    lp5860_write_burst(LP5860_REG_DC, dc, LP5860_NUM_DOTS);

    /* ⑩ 打开所有点的 ON/OFF 位（PWM 仍是 0，所以还是不亮，等上位机送图） */
    uint8_t onoff_on[LP5860_ONOFF_REGS];
    memset(onoff_on, 0xFF, sizeof(onoff_on));
    lp5860_write_burst(LP5860_REG_DOT_ONOFF, onoff_on, LP5860_ONOFF_REGS);

    /* ⑪ 自检：读回 Dev_initial，写 0x50 读 0x50 才算读写通路都通。
     *    注意读回失败 ≠ 写入失败：MISO 没接或芯片还在 I2C 模式时必然读回 0x00，
     *    但写方向可能完全正常 —— 所以这里只警告、不停机，灯亮不亮才是硬证据。 */
    uint8_t chk = 0;
    for (int i = 0; i < 3; i++) {                      /* 连读 3 次，避免偶发时序抖 */
        lp5860_read_reg(LP5860_REG_DEV_INITIAL, &chk);
        if (chk == LP5860_DEV_INITIAL_MODE1) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (chk == LP5860_DEV_INITIAL_MODE1) {
        ESP_LOGI(TAG, "LP5860 初始化完成，读回校验通过 (Dev_initial=0x%02X, 10行/Mode1/125kHz)",
                 chk);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "SPI 读回校验未通过：写 0x%02X 读到 0x%02X（连读 3 次都一样）",
             (unsigned)LP5860_DEV_INITIAL_MODE1, chk);
    ESP_LOGW(TAG, "  已按「写入成功」继续运行（读不通不代表写不通），看 LED 亮不亮");
    ESP_LOGW(TAG, "  硬件按顺序查：① VIO_EN / VCC / VLED(第16脚) 供电  ② IFS 必须为高(SPI 模式)");
    ESP_LOGW(TAG, "                ③ CS=IO10→38脚  ④ SCLK=IO12→35脚  ⑤ MOSI=IO11→36脚");
    ESP_LOGW(TAG, "                ⑥ MISO=IO13→37脚  ⑦ ESP32 与 LP5860 共地");
    ESP_LOGW(TAG, "  灯全不亮时，用 TCP 命令 FAULT 让芯片自报 LED 有没有电流流过");
    return ESP_OK;
}

/* ---------------- LED 开/短路检测（红外看不见时用它拿电气证据） ---------------- */

/*
 * 背景：这是红外(850/940nm)灯阵 —— 人眼看不见，很多手机摄像头也看不到 940nm
 * （IR-cut 滤光片把它滤掉了）。所以"我看不见光"不能证明灯没亮。
 *
 * LP5860 自带检测电路，可给出电气层面的证据：
 *   LOD (LED Open Detection)：开路阈值 0.25V。扫到某行时若 CSn 电压连续 4 个子周期
 *        低于阈值，说明这一路没有电流 → 该点开路。
 *   LSD (LED Short Detection)：短路阈值 (VLED - 1)V。
 *   条件：Mode 1/2 下要求 PWM ≥ 25（v=255 远超阈值，检测正常工作）。
 *
 * 结果寄存器：0x64 Fault_state(bit1=Global_LOD, bit0=Global_LSD)、
 *             0x65~0x67 Dot_lod0/1/2、0x86~0x88 Dot_lsd0/1/2，
 *             0xA7/0xA8 写 0x0F 清标志。
 * ⚠️ LOD/LSD 是按 CS 通道聚合的，不是逐点。
 *
 * ⭐ 为什么要分两阶段（阳性对照，别删）：
 *   直接读 LOD 有致命歧义 —— 读到 0 到底是"真的没有开路"，还是"检测压根没跑"？
 *   CS10~CS17 上没有 LED，把它们（幽灵点）开到阈值以上，检测正常时**必然**判开路。
 *     - 幽灵点报开路   → 检测电路是活的，阶段 B 的结论才可信
 *     - 幽灵点也干净   → 检测没生效，阶段 B 的结论无效（不能当"LED 正常"用）
 */
esp_err_t lp5860_fault_check(char *report, size_t len)
{
    int off = 0;
    uint8_t fault = 0, lod0 = 0, lod1 = 0, lod2 = 0;
    uint8_t lsd0 = 0, lsd1 = 0, lsd2 = 0;

    off += snprintf(report + off, len - off, "LED 开/短路检测 (LOD/LSD)\n");

    /* ======= 阶段 A：阳性对照（幽灵点必然开路） ======= */
    for (int y = 0; y < LP5860_USED_LINES; y++)
        for (int cs = LP5860_USED_CS; cs < LP5860_NUM_CS; cs++)
            lp5860_set_dot_pwm(y * LP5860_NUM_CS + cs, 0xFF);

    lp5860_write_reg(LP5860_REG_LOD_CLEAR, 0x0F);      /* 清标志，否则读到旧状态 */
    lp5860_write_reg(LP5860_REG_LSD_CLEAR, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(60));                      /* 等检测电路出结论（需数个扫描周期） */

    uint8_t pf = 0, a0 = 0, a1 = 0, a2 = 0;
    lp5860_read_reg(LP5860_REG_FAULT_STATE, &pf);
    lp5860_read_reg(LP5860_REG_DOT_LOD0, &a0);
    lp5860_read_reg(LP5860_REG_DOT_LOD1, &a1);
    lp5860_read_reg(LP5860_REG_DOT_LOD2, &a2);
    uint32_t pa = (uint32_t)a0 | ((uint32_t)a1 << 8) | ((uint32_t)(a2 & 0x03) << 16);

    /* 清掉幽灵点，别让它们一直占着电流 */
    for (int y = 0; y < LP5860_USED_LINES; y++)
        for (int cs = LP5860_USED_CS; cs < LP5860_NUM_CS; cs++)
            lp5860_set_dot_pwm(y * LP5860_NUM_CS + cs, 0);

    int ph_open = 0;
    for (int cs = LP5860_USED_CS; cs < LP5860_NUM_CS; cs++) ph_open += (pa >> cs) & 1;

    off += snprintf(report + off, len - off,
                    "  [对照] 幽灵点(CS10~CS17 无LED)标为开路的: %d/8  %s\n",
                    ph_open, ph_open ? "-> 检测电路工作正常" : "-> ⚠️ 检测没生效！");
    if (ph_open == 0) {
        off += snprintf(report + off, len - off,
            "  ⚠️ 阳性对照失败：连没接 LED 的通道都没报开路，说明检测电路没起作用。\n"
            "     常见原因：PWM 没到阈值(Mode1 需 >=25)、芯片未使能、或选了 Mode 2/3 却没给 VSYNC。\n"
            "     ==> 下面的结果**不能**当作\"LED 正常\"的证据，只能当作\"测不出来\"。\n");
    }

    /* ======= 阶段 B：读真实通道 ======= */
    lp5860_write_reg(LP5860_REG_LOD_CLEAR, 0x0F);
    lp5860_write_reg(LP5860_REG_LSD_CLEAR, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(60));

    lp5860_read_reg(LP5860_REG_FAULT_STATE, &fault);
    lp5860_read_reg(LP5860_REG_DOT_LOD0, &lod0);
    lp5860_read_reg(LP5860_REG_DOT_LOD1, &lod1);
    lp5860_read_reg(LP5860_REG_DOT_LOD2, &lod2);
    lp5860_read_reg(LP5860_REG_DOT_LSD0, &lsd0);
    lp5860_read_reg(LP5860_REG_DOT_LSD1, &lsd1);
    lp5860_read_reg(LP5860_REG_DOT_LSD2, &lsd2);

    uint32_t lod = (uint32_t)lod0 | ((uint32_t)lod1 << 8) | ((uint32_t)(lod2 & 0x03) << 16);
    uint32_t lsd = (uint32_t)lsd0 | ((uint32_t)lsd1 << 8) | ((uint32_t)(lsd2 & 0x03) << 16);

    off += snprintf(report + off, len - off,
                    "  Fault_state(0x64) = 0x%02X  Global_LOD=%d Global_LSD=%d\n",
                    fault, (fault >> 1) & 1, fault & 1);
    off += snprintf(report + off, len - off,
                    "  开路位图 CS17..CS0 = %02X %02X %02X  (1=开路)\n",
                    lod2 & 0x03, lod1, lod0);
    off += snprintf(report + off, len - off,
                    "  短路位图 CS17..CS0 = %02X %02X %02X  (1=短路)\n",
                    lsd2 & 0x03, lsd1, lsd0);

    int bad_open = 0, bad_short = 0;
    off += snprintf(report + off, len - off, "  已用通道 CS0~CS9: ");
    for (int cs = 0; cs < LP5860_USED_CS; cs++) {
        int o = (lod >> cs) & 1, s = (lsd >> cs) & 1;
        bad_open  += o;
        bad_short += s;
        off += snprintf(report + off, len - off, "CS%d:%s%s ", cs,
                        o ? "开路" : "--", s ? "短路" : "");
    }
    off += snprintf(report + off, len - off, "\n");

    /* ======= 结论 ======= */
    if (ph_open == 0) {
        off += snprintf(report + off, len - off,
            "  结论: 【无法判定】阳性对照没过，本次读数不可信。\n"
            "        先解决\"检测为什么没工作\"，再来看 CS0~CS9。\n");
    } else if (bad_open == 0 && bad_short == 0) {
        off += snprintf(report + off, len - off,
            "  结论: 对照通过 + CS0~CS9 全部导通 —— 这 100 颗 LED 的电流通路是真的通的，\n"
            "        芯片确实在驱动它们发光。看不到光 = 观测手段问题：\n"
            "        · 940nm 红外很多手机摄像头看不到（IR-cut 滤掉了）\n"
            "        · 自测办法：电视遥控器对着手机摄像头按一下，看不见就说明手机不行\n"
            "        · 靠谱办法：红外感应卡 / 红外摄像头 / 光电二极管 + 万用表\n");
    } else {
        off += snprintf(report + off, len - off,
            "  结论: 已用通道里有 %d 路开路、%d 路短路 —— 硬件层面的问题（对照已通过，可信）。\n"
            "        整片开路: 先量 VLED(第16脚，2.7~5.5V) —— 它是 SW 高侧开关的电源输入，\n"
            "                  悬空时芯片通信/扫描一切正常，但 LED 一条电流都没有\n"
            "        单列开路: 该列 LED 虚焊/装反/限流电阻没贴/排线断，或行列接反\n"
            "        短路: 该列对地或对 VLED 短路\n",
            bad_open, bad_short);
    }
    return ESP_OK;
}
