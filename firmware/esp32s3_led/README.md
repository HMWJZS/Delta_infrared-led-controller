# ESP32-S3 + LP5860 10×10 红外 LED 矩阵 + WiFi 上位机控制

ESP32-S3-WROOM-1-N16R8 通过 SPI 驱动 TI LP5860 点阵控制器，点亮 10×10 红外 LED（850/940nm）；
WiFi 联网后开 TCP 服务，上位机（Tkinter 图形界面）实时画图案；支持 WiFi OTA 升级，不用再插串口线。

## 文件结构

```
esp32s3_led/
├── CMakeLists.txt          # 工程入口
├── partitions.csv          # OTA 双分区表（ota_0 / ota_1）
├── sdkconfig.defaults      # 默认配置（esp32s3 + 自定义分区表）
├── tools/
│   ├── led_gui.py          # 上位机：Tkinter 图形界面，点格子控制 LED
│   ├── build.py            # 一键编译（封装 ESP-IDF 环境变量）
│   ├── push_ota.py         # 一键 OTA 推送 + 版本核验
│   └── capture_log.py      # 串口日志抓取 + 自动判定（固件版本/崩溃/掉电）
└── main/
    ├── main.c              # 上电流程（初始化 -> 连 WiFi -> 起 TCP -> 心跳）
    ├── lp5860.h / .c       # LP5860 SPI 驱动（含 LED 开/短路检测）
    ├── ledmatrix.h / .c    # 10x10 帧缓冲 + set_led API
    ├── wifi_app.h / .c     # WiFi STA 连接 + 断线自动重连
    ├── tcp_server.h / .c   # 上位机控制协议（端口 8266）
    └── ota_app.h / .c      # WiFi OTA 固件升级
```

## 硬件接线

| 信号 | ESP32-S3 | LP5860 |
|---|---|---|
| CS | IO10 | 第 38 脚 SS |
| MOSI | IO11 | 第 36 脚 SDA_MOSI |
| SCLK | IO12 | 第 35 脚 SCL_SCLK |
| MISO | IO13 | 第 37 脚 ADDR0_MISO |

- **IFS 脚必须接高电平**（选 SPI 模式；拉低是 I2C）
- 矩阵接法：**行(y) 阳极 → SW0~SW9，列(x) 阴极 → CS0~CS9**
- 坐标约定：`(0,0)` 左上角，x 向右（列，CS0~9），y 向下（行，SW0~9）

### ⚠️ 三个电源引脚彼此独立，都要接

| 引脚 | 名称 | 作用 | 没接的后果 |
|---|---|---|---|
| 40 | VCC | 芯片自身供电（2.7~5.5V） | 完全不工作、SPI 不通 |
| **16** | **VLED** | **SW 高侧开关的电源输入（2.7~5.5V）** | **通信/扫描/LOD 全正常，但灯一颗不亮** |
| 39 | VIO_EN | IO 电平 + 芯片使能（1.65~5.5V） | IO 不稳定 / 芯片不使能 |

**VLED 最容易被漏掉**：它夹在 LED 引脚区中间，不在电源引脚那一堆里。
SW0~SW9 只是"把 VLED 的电压按扫描时序切到 LED 阳极"的开关，芯片内部没有给 LED 用的
电源/升压电路 —— LED 的电流是从第 16 脚流进来的。

手册要求 VLED 供电必须"**能提供 LED 配置所需的峰值电流而不掉压**"：
10 行扫描、每颗 125mA 时，峰值 = 10 点/行 × 125mA ≈ **1.25A**，
且不能掉到 `LED Vf + VSAT` 以下。

### 其它硬件注意

- 模组是 **N16R8**（八线 PSRAM），**IO35/36/37 被 PSRAM 占用，任何电路都不要接**
- ESP32 与 LP5860 必须**共地**

## 拿到工程后要先改的几处

| # | 改什么 | 位置 | 说明 |
|---|---|---|---|
| 1 | **WiFi 账号密码** | `main/wifi_app.c` 第 15~16 行 | `WIFI_SSID` / `WIFI_PASS`。不改只会看到反复重连日志，TCP 服务起不来 |
| 2 | 上位机默认 IP（可选） | `tools/led_gui.py` 的 `DEFAULT_IP`、`tools/push_ota.py` 的 `DEFAULT_BOARD` | 不改也行：GUI 里能直接填，脚本能加 `--ip` / `-H` 覆盖 |
| 3 | 不用改 | `tools/build.py` | IDF 路径自动探测 |
| 4 | 按需装依赖 | `pyserial`（只有 `tools/capture_log.py` 抓串口日志用得到） | `pip install pyserial` |

- **VS Code**：`.vscode/settings.json` 里机器相关的项（IDF 安装路径、clangd 路径、串口号）已清空，
  第一次打开用命令面板 `ESP-IDF: Select current ESP-IDF version`、底部状态栏选串口即可。
- **IDF 版本**：本工程在 **ESP-IDF v6.1** 下验证（`sdkconfig` 由 v6.1 生成）。
  换大版本时先删掉 `sdkconfig`，让它按 `sdkconfig.defaults` 重新生成再编译。

## 编译烧录（VS Code ESP-IDF）

1. VS Code 打开 `esp32s3_led` 文件夹
2. 底部状态栏选串口 → 🔨 Build → ⚡ Flash → 🖥️ Monitor
3. 串口里找这两行，记下 IP：

```
WiFi 已连接，IP 地址: 192.168.1.x
>>> 本机 IP: 192.168.1.x   TCP 端口: 8266   可用堆: xxxxx 字节 <<<
```

命令行编译（不想开 VS Code 时）：

```powershell
python esp32s3_led\tools\build.py
```

> `build.py` 会自己找 IDF：先看环境变量 `IDF_PATH` / `IDF_TOOLS_PATH`，没设就在常见位置
> （`C:\Espressif\frameworks\esp-idf-*`、`~\esp\esp-idf` 等）搜一遍，最后才用开发机的默认值。
> 所以换台电脑也能直接用，不用改代码。它在非交互环境里也顺手绕开了两个坑：
> `IDF_TOOLS_PATH` 必须写到 `...\tools` 这一层；Git Bash 的 `MSYSTEM` 变量会让 `idf.py`
> 直接拒绝执行**且退出码仍是 0**（看起来像编译成功却没输出），必须剥掉。

## 上位机（图形界面）

`tools/led_gui.py`：只用 Python 标准库自带的 Tkinter，**不需要 pip 装任何东西**。

```powershell
python esp32s3_led\tools\led_gui.py
```

> 必须用**带 tkinter 的解释器**（Windows 官方安装包 / miniconda 都自带；某些精简版
> Python 没装 tkinter，会报 `No module named 'tkinter'`，换一个解释器即可）。
> IP 变了就加参数：`... led_gui.py --ip 192.168.1.x`

| 操作 | 效果 |
|---|---|
| 左键单击格子 | 点亮 / 熄灭该点（用亮度滑块的值） |
| 按住左键拖动 | 连续涂鸦 |
| 右键单击格子 | 强制熄灭 |
| 侧边按钮 | 全灭 / 全亮 / 边框 / 反色 / 发送整帧 / 读取当前帧 / LED 检测 |
| 快捷键 | `C` 清空、`F` 全亮、`B` 边框、`R` 反色、`S` 发送整帧 |

单击/拖动只发 `PX x y v`（延迟最低）；整屏操作发 `FRAME v0..v99`（一次写满，无撕裂）。

## 命令协议（每条以 `\n` 结尾，纯 ASCII）

| 命令 | 作用 |
|---|---|
| `PING` | 连通测试，回 `PONG` |
| `IP` | 查询设备 IP |
| `CLEAR` | 全灭 |
| `FILL v` | 全屏按亮度 v (0~255) 点亮 |
| `PX x y v` | 单点，如 `PX 3 4 200` |
| `ROW y v0..v9` | 整行 10 个亮度 |
| `FRAME v0..v99` | 整帧 100 个亮度（行优先） |
| `GRID` | 读回当前屏幕内容 |
| `FAULT` | **LED 开/短路检测**（红外看不见时的电气证据，见下文） |
| `RREG <addr>` | 读寄存器，地址可写 `0x01` 或 `1` |
| `WREG <addr> <val>` | 写寄存器（实验用，`DEFAULTS` 可恢复） |
| `DEFAULTS` | 重跑初始化，把芯片恢复到已知状态 |
| `VERSION` | 查询固件版本 |
| `REBOOT` | 远程重启 |
| `OTA <url>` | 从 URL 下载固件升级，成功后自动重启 |

连上后设备会先主动发一行 `HELLO <ip>`，客户端需要先吃掉这行。

## 亮度：三级电流链

每颗 LED 的实际电流（手册公式）：

```
I_OUT(mA) = IOUT_MAX(MC) × (CC/127) × (DC/255)
```

| 因素 | 寄存器 | 出厂默认 | 本工程固化值 |
|---|---|---|---|
| MC 峰值电流 | `0x04` bit[3:1]（LP5860T 档 7.5~125mA） | 0x47 → 37.5mA | **0x4F → 125mA** |
| CC 色组电流 | `0x09/0A/0B`（各管 6 个 CS 通道） | 0x40 → 50.4% | **0x7F → 100%** |
| DC 点电流 | `0x100~` | 0x80 → 50.2% | **0xFF → 100%** |

MC 分档：`011`=37.5 / `100`=50 / `101`=75 / `110`=100 / `111`=125mA（只动 bit[3:1]）。

两个注意点：
1. **调亮度用 PWM（`FILL`/`PX`）才即时可靠**。模拟增益（MC/CC）在运行中用 `WREG` 改
   不一定立刻生效，所以它们放在 `lp5860_init()` 里随使能周期配置。
2. **125mA 是芯片上限，不是 LED 上限** —— 以 LED 规格书的脉冲额定值为准；
   芯片是 1/10 扫描，每颗灯的平均电流 = 设定值 ÷ 10（FILL 255 时每颗约 12.5mA）。
   想再亮就得提高峰值电流或改硬件方案。散热不足时把 MC 降档。

## 在代码里画图案

```c
set_led(3, 4, 255);                                   /* 点亮 (3,4)，亮度 255 */

uint8_t pts[][2] = {{1,1},{2,2},{3,3},{4,4}};         /* 对角线 */
set_leds(pts, 4, 255);
```

## FAULT：不用眼睛确认 LED 有没有导通

红外灯人眼看不见，很多手机摄像头也拍不到 940nm（IR-cut 滤掉了），
所以"我看不见光"不能证明灯没亮 —— 用芯片自带的检测电路拿电气证据。

- **LOD（开路）**：阈值 0.25V，扫到某行时若 `CSn` 电压连续 4 个子周期低于阈值 → 该路无电流
- **LSD（短路）**：阈值 `(VLED - 1)V`
- 结果寄存器：`0x64` Fault_state、`0x65~0x67` Dot_lod、`0x86~0x88` Dot_lsd；
  写 `0xA7`/`0xA8` 的 `0x0F` 清标志。检测要求 Mode 1/2 下 PWM ≥ 25。

### 为什么命令里要先做"阳性对照"

直接读 LOD 有歧义：读到 0 究竟是"真的没开路"，还是"检测电路压根没跑"？
所以 `FAULT` 先拿**幽灵点**（CS10~CS17，板上没接 LED）自证：把它们点亮后，
检测正常工作就**必然**报开路。

```
[对照] 幽灵点(CS10~CS17 无LED)标为开路的: 8/8  -> 检测电路工作正常
已用通道 CS0~CS9: CS0:-- CS1:-- ...            <- 全部 -- 表示导通正常
```

- 对照非 0 → 结论可信
- 对照 0/8 → **【无法判定】**，别把"全干净"当好消息

### 读结果时的坑

- **LOD / LSD 标志都是锁存的**：LOD 写 `0xA7`、LSD 写 `0xA8`（值 `0x0F`）才清。
  `FAULT` 命令内部两个都清；手动 `RREG` 读不会清 —— 所以手动只清了 `0xA7` 时，
  `0x64` 的 bit0（Global_LSD）可能还挂着旧值，别急着下"短路"的结论。
- **`FAULT` 会先把整屏点亮**（检测要求 PWM 达标），会覆盖你之前设的图案
- 想单独验证某一列：`CLEAR` → `PX` 点该列 → `WREG 0xA7 0xFF` + `WREG 0xA8 0xFF`
  → 等 0.2s → 读 `0x64/0x65`
- 检测只对 PWM 达标的通道生效，PWM=0 的通道不参与

### 看不到红外光怎么办（按可靠性排序）

1. **红外感应卡**（几块钱）—— 最直观、不挑波长
2. 红外摄像头 / 夜视监控 / 老式 USB 摄像头（很多没有 IR-cut）
3. 旧手机前置摄像头（前置 IR-cut 通常更弱）
4. **电视遥控器对照实验**：对着手机摄像头按遥控器，看不见就说明手机拍不到红外
5. 光电二极管 + 万用表 mV 档：红外 LED 当光电池用，被照到会有几十 mV

## OTA 升级（第二次烧录起不用串口线）

一键推送（推荐）：

```powershell
python esp32s3_led\tools\push_ota.py
python esp32s3_led\tools\push_ota.py -H 192.168.1.x    # 板子 IP 和默认值不同时
python esp32s3_led\tools\push_ota.py --expect v3.0     # 顺带核对版本号
```

脚本会自动：编译 → 起 HTTP 服务 → 发 `OTA <url>` → 等板子重启 → 用 `VERSION` 核验版本。

手动流程：
1. Build 得到 `build\led_matrix_wifi.bin`
2. 在 `build` 目录起 HTTP 服务：`python -m http.server 8000`
   （首次运行 Windows 防火墙会弹窗，选"允许"）
3. 连设备 8266 端口发：`OTA http://<电脑IP>:8000/led_matrix_wifi.bin`
4. 串口看进度（25%→50%→75%→100%），完成后自动重启

> 设备与电脑**跨网段也能跑通**（实测设备 `10.1.30.121` ← 电脑 `10.1.41.185`，
> 955KB 固件约 8.7 秒 ≈ 110 KB/s）。启动 HTTP 服务的终端别关。

### 升级失败会怎样

| 失败场景 | 结果 |
|---|---|
| 下载中断 / 404 / URL 写错 | `esp_ota_abort()`，**旧固件继续跑**，不会变砖 |
| 下载完整但镜像校验不过 | 不切换分区，**旧固件继续跑** |
| 新固件通过校验但运行崩溃 | ⚠️ **不会自动回退**，会一直重启进新分区，只能用串口重烧 |

最后一种是因为 `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` 目前关闭（IDF 默认）。
要启用自动回退需要两步：sdkconfig 打开该选项 + 在 `app_main()` 里 WiFi/TCP 就绪后调用
`esp_ota_mark_app_valid_cancel_rollback()`（漏掉这句会导致无限回滚）。
**升级期间不要断电。**

## 调试备忘

**客户端发的是 CRLF，不是 LF。** PowerShell、telnet、多数串口助手默认发 `\r\n`。
TCP 服务按 `\n` 切分，所以切出整行后会剪掉行尾的 `\r`/空格/Tab
（否则 `OTA <url>` 尾部多个 `\r` 会导致 HTTP 404）。

**Monitor 和 Flash 不能同时占串口。** 报 `Could not open COM3, the port is busy`
就是上一次的监视器终端还开着 —— 关掉再烧。判断依据：这个错是**还没握手**就失败；
真是下载模式问题会报 `No serial data received` / `Failed to connect`。

**心跳行每 15 秒打一次，顺手看资源余量：**

```
>>> 本机 IP: 10.1.30.121   TCP 端口: 8266   可用堆: xxxxx 字节 <<<
```

- **可用堆**：OTA 要 `malloc(4096)` + HTTP 客户端，太小就别指望 OTA 能成
- 上位机连上时还会打一行**本任务栈历史最低剩余字节**（接近 0 就该加栈；
  `tcp_srv` 的栈是 8192，之前用 4096 时栈溢出崩溃过）

**判断板上跑的是哪版固件 —— 认这两个：**

| 判据 | 位置 |
|---|---|
| `VERSION` 命令的应答 | 最直接（改代码时同步改 `tcp_server.c` 的 `FIRMWARE_VERSION`） |
| `app_init: Compile time:` | 串口日志，每次编译都会变 |

> ⚠️ 用过 OTA 之后 `boot: compile time` **会骗人**：OTA 只重写 app 分区，
> bootloader 还是最初串口烧的那版，编译时间永远停在那个时刻。

`tools/capture_log.py` 把这件事自动化了（先关掉 VSCode 监视器终端）：

```powershell
python esp32s3_led\tools\capture_log.py -p COM3 -t 20 --reset
```

抓 20 秒串口 → 日志存到 `tools/logs/` → 打印关键判据表 → 给结论（有没有栈溢出/掉电/崩溃、
是不是新固件）。依赖 `pyserial`：ESP-IDF 自带的那套 Python 环境里已经有了，用别的解释器跑
先 `pip install pyserial`。

**上位机连不上、但板子明明在线？** 先看电脑上有没有代理软件。
代理的 TUN 虚拟网卡会**假装接受所有 TCP 连接**（连得上但没有 banner、一发数据就断），
看起来像"设备没响应"。关掉代理即可。判断特征：**任意 IP 的同一端口都"开放"**。
