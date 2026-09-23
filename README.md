# Delta Infrared LED Controller

ESP32-S3 + LP5860 10x10 红外点阵的上位机、固件和图案库。上位机可连接一个或两个点阵控制板，手动画点、加载图案、控制亮度，并可用相机预览和识别点阵。

## 上位机

Linux/macOS（需要 Python 3.11+、Tkinter、`numpy`、`opencv-python`、`Pillow`）：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy opencv-python Pillow
./start.sh --check
./start.sh --ip 192.168.1.50
```

Windows 使用带 Tkinter 的 Python，运行 `python start.sh` 不适用，请运行：

```powershell
python software\led_gui.py --ip 192.168.1.50
```

默认设备地址是 `10.1.30.121` 和 `10.1.30.190`，端口 `8266`。启动前让电脑和 ESP32 连接同一 Wi-Fi；GUI 中也可以修改地址。

## 固件

工程位于 `firmware/esp32s3_led/`，使用 ESP-IDF v6.1。烧录前编辑 `main/wifi_app.c` 中的 `WIFI_SSID` 和 `WIFI_PASS`，然后在 ESP-IDF 环境执行：

```bash
cd firmware/esp32s3_led
idf.py set-target esp32s3
idf.py build flash monitor
```

硬件接线、电源要求、TCP 命令协议和 OTA 说明见 [固件说明](firmware/esp32s3_led/README.md)。

## 图案与坐标

- `software/patterns/`：可直接由 GUI 加载的 JSON 图案
- `patterns/LED_10x10_center_coordinates.csv`：10x10 点阵中心坐标（mm）
- `calibration/fisheye_intrinsics.json`：默认 4K/940nm 相机内参

上位机源码使用这些文件的仓库相对路径，不依赖原始开发机目录。

## 注意

此仓库不包含采集数据和实验报告。相机识别功能需要实际相机、对应镜头/滤光片和标定条件；仅控制 LED 时无需连接相机。
