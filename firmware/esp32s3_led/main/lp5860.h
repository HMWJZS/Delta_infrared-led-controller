/*
 * lp5860.h - TI LP5860 LED 矩阵驱动器 SPI 驱动（ESP-IDF）
 *
 * 硬件连接：
 *   CS   -> IO10   片选（低电平有效，拉高复位 SPI 接口）
 *   MOSI -> IO11   主出从入（LP5860 的 SDA_MOSI）
 *   SCLK -> IO12   时钟（LP5860 的 SCL_SCLK）
 *   MISO -> IO13   主入从出（LP5860 的 ADDR0_MISO）
 *
 * 依据数据手册 SNVSBU8A（LP5860）：
 *   - SPI 帧格式（Table 8-7 / Figure 8-17、8-18）：2 字节地址 + 数据
 *       字节1 = 寄存器地址 bit[9:2]（Bit7=A9 … Bit0=A2，MSB 先发）
 *       字节2 = bit[7]=A1  bit[6]=A0  bit[5]=R/W(0=读 1=写)  bit[4:0]=Don't Care
 *       之后为数据字节，寄存器地址自动递增，可一次连写多个
 *     ⚠️ byte2 里 A1 A0 必须在最高两位，读写位紧跟其后 —— 详见 lp5860.c 的说明
 *   - MISO 平时高阻，只有读周期的数据字节才被驱动；
 *     SS（片选）为高会复位接口，因此每帧之间 CS 必须拉高
 *   - 寄存器 0x200 起为 PWM SRAM（本工程用 Mode 1：每个 LED 点 1 字节）
 *   - LED 点索引 = 扫描行 × 18 + CS 通道号
 *   - SW0~SW10：扫描高侧开关（接 LED 行阳极）
 *     CS0~CS17：恒流阴极（接 LED 列阴极）
 *   - ⭐ Chip_EN=0 时芯片进 STANDBY，SPI 只能写 Chip_EN 这一个寄存器，
 *     其余配置写入一律被丢弃 → 初始化必须"先使能再配置"
 *   - 内置 LED 开/短路检测（LOD/LSD），结果在 0x64~0x67 / 0x86~0x88
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/* ---------------- 引脚配置（改硬件只改这里） ---------------- */
#define LP5860_SCLK_GPIO   12      /* 时钟    -> IO12 */
#define LP5860_MOSI_GPIO   11      /* 数据出  -> IO11 */
#define LP5860_MISO_GPIO   13      /* 数据入  -> IO13 */
#define LP5860_CS_GPIO     10      /* 片选    -> IO10 */
#define LP5860_SPI_HZ      (1000 * 1000)  /* SPI 时钟 1MHz（芯片最高 12MHz，先求稳） */

/* ---------------- 寄存器地址（数据手册 Table 8-8） ---------------- */
#define LP5860_REG_CHIP_EN        0x000  /* 芯片使能：bit0=1 开启 */
#define LP5860_REG_DEV_INITIAL    0x001  /* 扫描行数/刷新模式/PWM频率，默认 0x5E */
#define LP5860_REG_DEV_CONFIG1    0x002  /* SW消隐/PWM缩放/相位/CS移位，默认 0x00 */
#define LP5860_REG_DEV_CONFIG2    0x003  /* 补偿组/开短路移除，默认 0x00 */
#define LP5860_REG_DEV_CONFIG3    0x004  /* 消隐电平/最大电流，默认 0x47 */
#define LP5860_REG_GLOBAL_BRI     0x005  /* 全局 PWM 亮度，默认 0xFF */
#define LP5860_REG_GROUP_PWM1     0x006  /* PWM 组1 占空比，默认 0xFF */
#define LP5860_REG_GROUP_PWM2     0x007  /* PWM 组2 占空比，默认 0xFF */
#define LP5860_REG_GROUP_PWM3     0x008  /* PWM 组3 占空比，默认 0xFF */
/* 色组电流(CC)：7 位有效，0~127 = 0%~100% × IOUT_MAX，默认 0x40（50.4%）。
 * I_OUT(mA) = IOUT_MAX(MC) × (CC/127) × (DC/255) */
#define LP5860_REG_CC_GROUP1      0x009  /* CS0,CS3,CS6,CS9,CS12,CS15 电流 */
#define LP5860_REG_CC_GROUP2      0x00A  /* CS1,CS4,CS7,CS10,CS13,CS16 电流 */
#define LP5860_REG_CC_GROUP3      0x00B  /* CS2,CS5,CS8,CS11,CS14,CS17 电流 */
/* Dev_config3 的 bit[3:1] = Maximum_Current(MC)，LP5860T 档 7.5~125mA 共 8 档：
 *   011b(默认)=37.5mA  100b=50mA  101b=75mA  110b=100mA  111b=125mA
 * 0x4F = 0b0100_1111 -> MC=111(125mA)，其余位保持默认 */
#define LP5860_DEV_CONFIG3_MC125  0x4F   /* MC=125mA/点（顶格，注意散热与电源能力） */
#define LP5860_REG_DOT_ONOFF      0x043  /* 每个 LED 点的开/关位，共 33 字节覆盖 198 点 */
#define LP5860_REG_FAULT_STATE    0x064  /* bit1=Global_LOD(开路) bit0=Global_LSD(短路) */
#define LP5860_REG_DOT_LOD0       0x065  /* CS0~CS7  的开路状态，bit=1 表示开路 */
#define LP5860_REG_DOT_LOD1       0x066  /* CS8~CS15 的开路状态 */
#define LP5860_REG_DOT_LOD2       0x067  /* CS16~CS17（只有低 2 位有效） */
#define LP5860_REG_DOT_LSD0       0x086  /* CS0~CS7  的短路状态 */
#define LP5860_REG_DOT_LSD1       0x087  /* CS8~CS15 的短路状态 */
#define LP5860_REG_DOT_LSD2       0x088  /* CS16~CS17 */
#define LP5860_REG_LOD_CLEAR      0x0A7  /* 写 0x0F 清 LOD 标志 */
#define LP5860_REG_LSD_CLEAR      0x0A8  /* 写 0x0F 清 LSD 标志 */
#define LP5860_REG_RESET          0x0A9  /* 软件复位（写 0xFF） */
#define LP5860_REG_DC             0x100  /* 每点点电流校正(DC)，198 字节 */
#define LP5860_REG_PWM            0x200  /* PWM SRAM，Mode 1 下每点 1 字节 */

#define LP5860_NUM_LINES   11   /* 扫描行总数（SW0~SW10） */
#define LP5860_NUM_CS      18   /* 恒流通道总数（CS0~CS17） */
#define LP5860_NUM_DOTS    (LP5860_NUM_LINES * LP5860_NUM_CS)  /* 198 点 */
#define LP5860_ONOFF_REGS  33   /* ONOFF 寄存器字节数 33*8=264 >= 198 */

/* ---------------- Dev_initial(0x01) ---------------- */
/*
 * 字段定义（LP5860 手册 Table 8-8 / Table 8-11）：
 *   bit[7:3] Max_Line_Num   扫描行数，出厂默认 B=11（芯片有 11 个内部 MOSFET）
 *   bit[2:1] Data_Ref_Mode  0=Mode 1  1=Mode 2  2/3=Mode 3
 *   bit[0]   PWM_Fre        0=125kHz  1=62.5kHz
 *
 * 本工程用 **Mode 1**：8-bit PWM、数据收到即刷新、**不需要 VSYNC**，
 * 并且 8-bit 正好对应帧缓冲里的 0~255，读回即可自检。
 * （Mode 2/3 必须由 MCU 提供 VSYNC 脉冲才会输出帧数据，本板没有 SYNC 线，不要选。）
 */
#define LP5860_DEV_INITIAL_MODE1  0x50   /* Max_Line_Num=10(0b01010) + Mode1(00) + 125kHz(0) */

/* 本板实际接出来的矩阵规模：10 行 × 10 列。
 * 放在这里是为了让驱动自己知道"哪些 CS 通道上真的挂了 LED" —— FAULT 检测要靠它
 * 把"真通道"和"幽灵通道"（CS10~CS17 没接 LED）区分开，用幽灵通道做阳性对照。 */
#define LP5860_USED_LINES  10   /* 用了 SW0~SW9 */
#define LP5860_USED_CS     10   /* 用了 CS0~CS9 */

/**
 * @brief 初始化 SPI 总线并配置 LP5860
 *        流程：先使能芯片 -> 写配置(10 行扫描 / Mode 1 8-bit PWM) -> 清 PWM -> 开全部点
 *        可重复调用（DEFAULTS 命令依赖这一点）
 */
esp_err_t lp5860_init(void);

/** @brief 写单个 8-bit 寄存器 */
esp_err_t lp5860_write_reg(uint16_t addr, uint8_t val);

/**
 * @brief 连续写一段寄存器（利用芯片的地址自动递增特性，一次 CS 内完成）
 * @param addr 起始寄存器地址
 * @param data 数据缓冲区
 * @param len  字节数
 */
esp_err_t lp5860_write_burst(uint16_t addr, const uint8_t *data, size_t len);

/** @brief 读单个 8-bit 寄存器（用于读写通路自检） */
esp_err_t lp5860_read_reg(uint16_t addr, uint8_t *val);

/**
 * @brief 写一个 LED 点的 8-bit PWM 值（Mode 1）
 * @param dot  点索引 = 行*18 + CS 通道
 * @param v    0~255 亮度（255 = 100%）
 *
 * 地址规律（手册 Table 8-64）：
 *   Mode 1/2（8-bit）：地址 = 0x200 + 点索引，**每个点 1 字节**
 *   Mode 3（16-bit）：地址 = 0x200 + 点索引*2，每点 2 字节，低字节在前（小端）
 */
esp_err_t lp5860_set_dot_pwm(int dot, uint8_t v);

/** @brief 整片连写 PWM SRAM（Mode 1 下从点 start_dot 开始，ndots 个点） */
esp_err_t lp5860_write_pwm_burst(int start_dot, const uint8_t *pwm, size_t ndots);

/**
 * @brief LED 开/短路检测报告：让芯片自己回答"这些 LED 到底有没有导通"
 *
 * 为什么需要它：850/940nm 红外 LED 人眼看不见，很多手机摄像头也看不到 940nm，
 * 所以"我看不见光"不能证明灯没亮，这时就得靠电气证据。
 * LP5860 自带开路检测(LOD，阈值 0.25V)与短路检测(LSD，阈值 VLED-1V)，
 * 逐 CS 通道把结果记在 0x64~0x67 / 0x86~0x88。
 *
 * 内部流程：先把"幽灵点"(CS10~CS17，板上没接 LED)开到阈值以上做**阳性对照**，
 * 确认检测电路真的在工作，再读真实通道 —— 否则"读到 0"无法区分
 * "真的没开路"和"检测压根没跑"。检测要求 Mode 1/2 下 PWM ≥ 25。
 *
 * 注意：LOD/LSD 是**按 CS 通道**聚合的，不是逐点。
 *
 * @param report 输出报告文本（多行，含对照结论与通道位图）
 * @param len    report 缓冲区长度，建议 >= 1024
 */
esp_err_t lp5860_fault_check(char *report, size_t len);
