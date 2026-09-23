#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-S3 串口日志抓取器 —— 一次性把「板子里到底跑的哪版固件」这件事钉死。

用途：
  1. 打开串口抓 N 秒日志，原样存到 tools/logs/ 下带时间戳的文件里
  2. 自动从日志里挑出关键判据（固件指纹 / 崩溃 / 服务状态）
  3. 自动判定「栈溢出有没有消失」「固件是不是刚编的那版」

用法（必须先关掉 VSCode 的监视器终端，否则串口被占）：
  C:\\Espressif\\tools\\python\\v6.1\\venv\\Scripts\\python.exe tools\\capture_log.py
  ... tools\\capture_log.py -p COM3 -t 20 --reset

参数：
  -p / --port   串口号，默认 COM3
  -b / --baud   波特率，默认 115200
  -t / --time   抓取秒数，默认 15
  --reset       先用 DTR/RTS 复位一次再抓（想看完整启动日志时用；
                注意本板 IO0 是手动飞线的，RTS 拉不动 IO0，所以只会复位 EN）
  -o / --out    指定输出文件路径，默认 tools/logs/日期-时间.txt
"""

import argparse
import datetime
import os
import re
import sys
import time

# pyserial 是 IDF venv 自带的（esptool 依赖它），一般不用装。
# 这里不直接 sys.exit —— 留一个 None，让本文件能被 import 去做纯文本的自测。
try:
    import serial
except ImportError:                                    # pragma: no cover
    serial = None


# ---------------------------------------------------------------- 关键判据
# 每项：(显示名, 正则, 说明)
MARKERS = [
    ("bootloader 编译时间", r"boot: compile time (.+?)\s*$"),
    ("app 编译时间",        r"app_init: Compile time:\s*(.+?)\s*$"),
    ("镜像 SHA256",         r"ELF file SHA256:\s*(\S+)"),
    ("LP5860 读回",         r"lp5860: .*读回校验(未通过|通过).*"),
    ("TCP 服务启动",        r"TCP 服务已启动.*"),
    ("上位机已连接",        r"上位机已连接.*"),
    ("栈溢出",              r"\*\*\*ERROR\*\*\* A stack overflow.*"),
    ("心跳行（含堆余量）",  r">>> 本机 IP:.*"),
    ("WiFi 拿到 IP",        r"WiFi 已连接，IP 地址:\s*(\S+)"),
    ("看门狗/重启",         r"(Brownout detector was triggered|Guru Meditation|rst:0x\S+ \((\w+)\))"),
]

# 判定规则：日志里出现这些就算「有问题」
BAD_PATTERNS = [
    (r"\*\*\*ERROR\*\*\* A stack overflow", "栈溢出仍然存在 → 板上还是旧固件"),
    (r"Brownout detector was triggered",    "电压跌落（brownout）→ 3.3V 供电带不动 WiFi 发射"),
    (r"Guru Meditation",                    "程序崩溃 → 看后面的 backtrace"),
]


def pick_markers(text):
    """从日志里挑出关键行，返回 [(显示名, 匹配内容)]"""
    found = []
    for name, pat in MARKERS:
        m = re.search(pat, text, re.M)
        if m:
            # 有捕获组就取捕获组，没有就取整行
            val = m.group(1) if m.groups() else m.group(0).strip()
            found.append((name, val.strip()))
    return found


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("-p", "--port", default="COM3")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-t", "--time", type=float, default=15.0)
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()

    if serial is None:
        print("缺少 pyserial。请用 IDF 自带的 Python 跑：")
        print(r"  C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe " + __file__)
        return 3

    # 默认存到脚本同级的 logs/ 目录
    here = os.path.dirname(os.path.abspath(__file__))
    out = args.out or os.path.join(
        here, "logs", datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".txt")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"[FAIL] 打不开 {args.port}：{e}")
        # 两种原因要分开说 —— 排查方向完全不同
        if isinstance(e, PermissionError) or "PermissionError" in str(e) \
                or "拒绝访问" in str(e):
            print("       → 端口被占用：VSCode 的监视器终端还开着，先关掉它再跑")
        elif isinstance(e, FileNotFoundError) or "FileNotFoundError" in str(e) \
                or "找不到" in str(e):
            print(f"       → 端口不存在：确认串口枚举（设备管理器里看真正的 COM 号），"
                  f"或拔插一次 USB 转串口")
        return 2

    if args.reset:
        # ESP32-S3 串口自动复位时序：EN 拉低 → 放开
        # （本板 IO0 是手动飞线，RTS 拉不动它，所以只复位、不进下载模式）
        ser.setDTR(False)
        ser.setRTS(True)
        time.sleep(0.1)
        ser.setRTS(False)
        time.sleep(0.05)

    print(f"[INFO] 抓取 {args.port} @ {args.baud}，{args.time:.0f} 秒 ...\n")
    print("-" * 60)

    buf = []
    t0 = time.time()
    # 串口里可能混进乱码字节。先让 stdout 用 replace 模式；万一控制台是 GBK 且
    # reconfigure 失败，_emit() 里还有一层兜底，保证不会因为一个乱码字符把抓取中断。
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    def _emit(s):
        try:
            print(s, end="", flush=True)
        except UnicodeEncodeError:
            enc = getattr(sys.stdout, "encoding", None) or "utf-8"
            sys.stdout.write(s.encode(enc, "replace").decode(enc, "replace"))
            sys.stdout.flush()

    while time.time() - t0 < args.time:
        chunk = ser.read(4096)
        if not chunk:
            continue
        s = chunk.decode("utf-8", errors="replace")
        buf.append(s)
        _emit(s)
    ser.close()

    text = "".join(buf)
    print("\n" + "-" * 60)

    with open(out, "w", encoding="utf-8", errors="replace") as f:
        f.write(text)
    print(f"[OK] 原始日志已存：{out}\n")

    # ---------------- 关键判据汇总 ----------------
    print("=" * 60)
    print("关键判据")
    print("=" * 60)
    found = pick_markers(text)
    if not found:
        print("(一个字节都没收到 —— 注意这**不是**脚本的问题，端口是打开成功的)")
        print("  按顺序查：")
        print("    1. 板子供电了吗？（3.3V 有没有）—— 没供电时串口就是全程静默")
        print("    2. USB 转串口的 TX→板子 RX、RX→板子 TX 有没有交叉接对")
        print("    3. 板子跟转串口共地了吗")
        print("    4. 加 --reset 强制复位一次，抓完整的开机 banner（最省事的排除法）")
    else:
        for name, val in found:
            print(f"  {name:<18}: {val}")

    # ---------------- 结论 ----------------
    print("\n" + "=" * 60)
    print("结论")
    print("=" * 60)

    if not text.strip():
        # 空日志不能报"没问题" —— 那是在骗人。没数据就只能说不知道。
        print("  [?] 无法判定：一个字节都没收到")
        print("      → 先确认板子供电、TX/RX 交叉接对、共地，再重跑")
        return 4

    problems = [(p, msg) for p, msg in BAD_PATTERNS if re.search(p, text)]
    if problems:
        for _, msg in problems:
            print(f"  [X] {msg}")
    else:
        print("  [V] 没有栈溢出 / 掉电 / 崩溃")

    if re.search(r">>> 本机 IP:.*可用堆", text):
        print("  [V] 心跳行含『可用堆』→ 板上是修复后的新固件")
    elif re.search(r">>> 本机 IP:", text):
        print("  [X] 心跳行没有『可用堆』→ 板上还是旧固件，需要重新烧录")
    elif re.search(r"TCP 服务已启动", text):
        print("  [?] 看到 TCP 服务启动了，但没等到心跳行（每 15 秒打一次，试试 -t 30）")
    else:
        print("  [?] 没抓到心跳行，也没看到 TCP 服务启动行 —— 确认日志是否被截断")

    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
