#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED 矩阵上位机（Tkinter 图形界面）
================================================================================
配套工程：ESP32-S3-WROOM-1 + LP5860 驱动 10x10 红外 LED 矩阵

运行（Windows，需要一个带 tkinter 的 Python；官方安装包和 miniconda 都自带）：
    python led_gui.py
    python led_gui.py --ip 192.168.1.50      # 板子 IP 和默认值不同时

界面用法
--------------------------------------------------------------------------------
  左键单击格子     切换该点：灭 -> 亮（用亮度滑块的值）-> 灭
  按住左键拖动     连续涂鸦：划过的格子按当前亮度点亮（不来回翻转）
  右键单击格子     强制熄灭该点（涂错了好擦）
  底部按钮         全灭 / 全亮 / 边框 / 反色 / 发送整帧
  快捷键           C=清空  F=全亮  B=边框  R=反色  S=发送整帧

通信协议（TCP，端口 8266，每条命令以 \\n 结尾，纯 ASCII）
--------------------------------------------------------------------------------
  发送  PX <x> <y> <v>      单点，x/y = 0~9，v = 亮度 0~255
        FRAME <v0> ... <v99>  整帧 100 个亮度，行优先：v[y*10 + x]
        CLEAR                 全灭
        FILL <v>              全屏填 v
        FAULT                 LED 开/短路检测（芯片自报电流有没有流过）
        VERSION               回固件版本号
  应答  HELLO <ip>           连上后下位机主动发第一条
        OK                   成功
        PONG                 PING 的应答
        ERR ...              出错（含中文提示，本程序按 UTF-8 解码）
        GRID ...             当前帧内容（调试）

坐标约定（与 ESP32 侧 ledmatrix.h 完全一致）
--------------------------------------------------------------------------------
  (0,0) 在左上角；x 向右 0~9（对应 LP5860 的 CS0~CS9 列），
  y 向下 0~9（对应 LP5860 的 SW0~SW9 扫描行）。
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import glob
import json
import os
from pathlib import Path
import queue
import socket
import sys
import threading
import tempfile
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, font as tkfont, ttk
import cv2
import numpy as np
from PIL import Image, ImageTk
from camera_capture import CameraCapture
from pattern_recognition import PatternRecognizer, pose_text, ROOT

# ============================== 可调参数 ==============================
DEFAULT_IP = "10.1.30.121"      # 开发环境里板子的 IP；换网络用 --ip 覆盖
DEFAULT_PORT = 8266             # 与 tcp_server.h 的 TCP_SERVER_PORT 一致
GRID = 10                       # 10x10 点阵
CELL = 26                       # 每个格子的边长（像素）
GAP = 2                         # 格子之间的缝隙
LOG_MAX_LINES = 300             # 日志区最多保留多少行

# ============================== 配色（蓝白极简） ==============================
C_BG = "#f5f7fa"        # 窗口底色
C_PANEL = "#ffffff"     # 面板底色
C_LINE = "#d0d7de"      # 边框线
C_ACCENT = "#1a73e8"    # 主蓝
C_ACCENT_DK = "#0b57d0"  # 深蓝（按下态）
C_TEXT = "#1f2328"      # 主文字
C_TEXT_DIM = "#6e7781"  # 次要文字
C_OFF = "#eef1f5"       # 熄灭格子的底色
C_ON_RGB = (0x1A, 0x73, 0xE8)   # 点亮格子的颜色（亮时）
C_OFF_RGB = (0xEE, 0xF1, 0xF5)  # 熄灭格子的颜色
C_OK = "#1a7f37"        # 状态-已连接
C_ERR = "#cf222e"       # 状态-错误

# 使用系统默认字体及中文回退，避免 Linux 缺少 Windows 字体时显示方框。
FONT = "TkDefaultFont"
INTRINSICS_PATH = (ROOT / "calibration" /
                   "decxin_v1_fisheye_940nm_4k" / "fisheye_intrinsics.json")
CAPTURE_ROOT = ROOT / "calibration" / "infrared_tracker_captures"


# ======================================================================
#  通信层：一条 TCP 连接 + 一个后台收包线程
# ======================================================================
class MatrixLink:
    """管理到 ESP32 的 TCP 连接。发送是主线程直接调用，接收放到后台线程。"""

    def __init__(self):
        self.sock = None
        self.rx_queue = queue.Queue()   # 收到的每一行丢这里，主线程定时取走
        self._rx_thread = None
        self._running = False

    @property
    def connected(self):
        return self.sock is not None

    def connect(self, ip, port, timeout=3.0):
        """建立连接。失败抛 OSError（调用方负责提示用户）。"""
        self.close()
        s = socket.create_connection((ip, port), timeout=timeout)
        s.settimeout(None)              # 之后交给收包线程阻塞读
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 小命令别攒包
        self.sock = s
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, args=(s,), daemon=True)
        self._rx_thread.start()

    def _rx_loop(self, sock):
        """后台线程：按行切分收到的字节流，投递到队列。"""
        buf = b""
        while self._running and self.sock is sock:
            try:
                data = sock.recv(1024)
            except OSError:
                break                    # 连接被关闭/出错，退出线程
            if not data:
                break                    # 对端断开
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                # 下位机的错误提示里有中文，按 UTF-8 解码（容错，不抛异常）
                self.rx_queue.put(line.decode("utf-8", "replace"))
        sock.close()
        if self.sock is sock:
            self.sock = None

    def send(self, cmd):
        """发一条命令（自动补换行）。未连接或发送失败抛 ConnectionError/OSError。"""
        if not self.sock:
            raise ConnectionError("尚未连接")
        self.sock.sendall((cmd + "\n").encode("ascii"))

    def close(self):
        self._running = False
        sock, self.sock = self.sock, None
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass


# ======================================================================
#  界面层
# ======================================================================
class LedApp:
    def __init__(self, root):
        self.root = root
        self.link = MatrixLink()
        self.frame = [0] * (GRID * GRID)   # 本地帧缓冲，行优先 frame[y*10+x]
        self.brightness_mask = [False] * (GRID * GRID)
        self.brightness_job = None
        self.pattern_dir = Path(__file__).resolve().parent / "patterns"
        self.cells = []                    # Canvas 里 100 个矩形的 id
        self.last_drag_cell = None         # 拖动时防重复触发
        self.need_full_redraw = True       # 首次绘制标记
        self.camera = None
        self.camera_running = False
        self.camera_job = None
        self.capture = None
        self.capture_preview_index = 0
        self.closing = False
        self.recognition_pool = None
        self.recognition_future = None
        self.recognition_job = None
        self.camera_generation = 0
        self.preview_geometry = None
        self.switching_device = False
        self.active_device = 0
        self.devices = [dict(ip=ip, imu=imu, link=self.link if i == 0 else MatrixLink(),
                             frame=[0]*100, mask=[False]*100, brightness=255, hz=2.,
                             blink=None, pattern="")
                        for i, (ip, imu) in enumerate(((DEFAULT_IP, False), ("10.1.30.190", True)))]
        self.camera_photo = None
        self.preview_window = None
        self.blink_job = None
        self.blink_frame = None
        self.blink_on = True
        intrinsics = json.loads(INTRINSICS_PATH.read_text(encoding="utf-8"))
        if intrinsics["camera_model"] != "opencv_fisheye":
            raise ValueError("内参必须使用 opencv_fisheye 模型")
        self.calibration_name = intrinsics["display_name"]
        self.camera_K = np.asarray(intrinsics["K"], dtype=np.float64)
        self.camera_D = np.asarray(intrinsics["D"], dtype=np.float64)
        self.calibration_size = (intrinsics["image_width"], intrinsics["image_height"])
        self.preview_map_size = None
        self.preview_maps = None
        self.var_undistort = tk.BooleanVar(value=True)
        self.var_intrinsics_status = tk.StringVar(value=f"{self.calibration_name} · 等待图像")
        self.var_cam_width = tk.IntVar(value=self.calibration_size[0])
        self.var_cam_height = tk.IntVar(value=self.calibration_size[1])
        self.var_cam_fps = tk.DoubleVar(value=30.0)
        self.var_cam_exposure = tk.DoubleVar(value=-6.0)
        self.var_cam_gain = tk.DoubleVar(value=0.0)
        self.var_auto_exposure = tk.BooleanVar(value=True)
        self.var_recognition = tk.BooleanVar(value=False)
        self.var_recognition_threshold = tk.IntVar(value=160)
        self.var_recognition_status = tk.StringVar(value="按当前本地图案匹配 · 请先核对灯板行列")
        self.var_pose_status = tk.StringVar(value="位姿：等待识别 · 原点为灯板中心")
        self.recognizer = PatternRecognizer(self.camera_K, self.camera_D, self.calibration_size)

        root.title("LED 双设备上位机 · 121 / 190（带 IMU）· 当前控制 121")
        root.configure(bg=C_BG)
        root.resizable(True, True)
        tkfont.nametofont(FONT).configure(size=14)
        tkfont.nametofont("TkTextFont").configure(size=14)
        root.option_add("*Font", FONT)
        ttk.Style(root).configure("TCombobox", font=FONT)
        root.geometry(f"{min(2000, root.winfo_screenwidth() - 80)}x"
                      f"{min(1200, root.winfo_screenheight() - 100)}")
        self._build_ui()
        self._log(f"已加载：{self.calibration_name}，{self.calibration_size[0]}×"
                  f"{self.calibration_size[1]}，RMS {intrinsics['rms_error_px']:.4f} px；"
                  f"适用固定焦点 {intrinsics['focus_absolute']}")
        self._redraw()
        self._bind_keys()

        # 定时取走接收队列里的行（Tkinter 只能在主线程操作控件）
        self.root.after(80, self._poll_rx)
        # 关窗时确保 socket 关掉，避免下位机那边一直以为还连着
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- 界面搭建 ----------------
    def _build_ui(self):
        # ---- 顶部：连接栏 ----
        top = tk.Frame(self.root, bg=C_BG)
        top.pack(fill="x", padx=14, pady=(14, 8))

        self.var_ip = tk.StringVar(value=DEFAULT_IP)
        self.var_port = tk.StringVar(value=str(DEFAULT_PORT))
        self.var_device = tk.IntVar(value=0)
        for index, device in enumerate(self.devices):
            card = tk.Frame(top, bg=C_PANEL, highlightthickness=1, highlightbackground=C_LINE)
            card.pack(side="left", fill="both", expand=True, padx=(0, 6))
            label = device['ip'] + (' · IMU' if device['imu'] else '')
            tk.Radiobutton(card, text=label, variable=self.var_device, value=index,
                           command=lambda i=index: self._select_device(i), bg=C_PANEL,
                           font=(tkfont.nametofont(FONT).actual('family'), 12)).pack(side="left")
            device['button'] = tk.Button(card, text="连接", width=4,
                font=(tkfont.nametofont(FONT).actual('family'), 12),
                command=lambda i=index: self._toggle_device_connection(i))
            device['button'].pack(side="left", padx=4)
            device['status'] = tk.Label(card, text="未连接", fg=C_TEXT_DIM, bg=C_PANEL,
                                       font=(tkfont.nametofont(FONT).actual('family'), 11))
            device['status'].pack(side="left", padx=3)
        self.btn_conn = self.devices[0]['button']
        self.lbl_status = self.devices[0]['status']
        patterns = tk.Frame(self.root, bg=C_BG)
        patterns.pack(fill="x", padx=14, pady=(0, 8))
        tk.Button(patterns, text="保存图案", command=self.act_save_pattern).pack(side="left", padx=(0, 8))
        tk.Label(patterns, text="已保存图案", bg=C_BG, fg=C_TEXT).pack(side="left")
        self.var_pattern = tk.StringVar()
        self.pattern_list = ttk.Combobox(patterns, textvariable=self.var_pattern, state="readonly", width=23)
        self.pattern_list.pack(side="left", padx=(10, 5))
        self.pattern_list.bind("<<ComboboxSelected>>", self._select_pattern)
        self._refresh_patterns()
        tk.Button(patterns, text="点亮选中", command=self._select_pattern).pack(side="left")
        tk.Button(patterns, text="导入图案", command=self.act_load_pattern).pack(side="left", padx=5)

        # ---- 主区：左侧点阵控制，右侧可伸缩相机预览 ----
        mid = tk.Frame(self.root, bg=C_BG)
        controls = tk.Frame(mid, bg=C_BG)
        controls.pack(side="left", fill="y", padx=(0, 14))

        boards = tk.Frame(controls, bg=C_BG)
        boards.pack(fill="x", pady=(0, 8))
        size = GAP + (CELL + GAP) * GRID
        for index, device in enumerate(self.devices):
            panel = tk.Frame(boards, bg=C_BG)
            panel.pack(side="left", padx=(0, 8 if index == 0 else 0))
            name = device['ip'] + (' · 带 IMU' if device['imu'] else ' · 点阵')
            device['heading'] = tk.Button(panel, text=name,
                font=(tkfont.nametofont(FONT).actual('family'), 11),
                command=lambda i=index: self._select_device(i), relief='flat', pady=2)
            device['heading'].pack(fill='x')
            device['canvas'] = tk.Canvas(panel, width=size, height=size, bg=C_PANEL,
                                          highlightthickness=2, highlightbackground=C_LINE)
            device['canvas'].pack()
            device['cells'] = self._build_grid(device['canvas'])
            for sequence, handler in (('<Button-1>', self._on_press),
                                      ('<B1-Motion>', self._on_drag),
                                      ('<ButtonRelease-1>', self._on_release),
                                      ('<Button-3>', self._on_right_click)):
                device['canvas'].bind(sequence, lambda event, i=index, action=handler:
                                       self._board_event(i, action, event))
        self.canvas, self.cells = self.devices[0]['canvas'], self.devices[0]['cells']
        self._highlight_device()

        ctrl = tk.Frame(controls, bg=C_BG)
        ctrl.pack(fill="x")

        # 亮度滑块
        brightness = tk.Frame(ctrl, bg=C_BG)
        brightness.grid(row=0, column=0, columnspan=2)
        tk.Label(brightness, text="亮度", font=FONT, bg=C_BG, fg=C_TEXT).pack(side="left")
        self.var_bri = tk.IntVar(value=255)
        self.var_bri_input = tk.StringVar(value="255")
        self.brightness_entry = tk.Spinbox(brightness, from_=0, to=255, width=4,
            textvariable=self.var_bri_input, validate="key",
            validatecommand=(self.root.register(lambda value: value == "" or (
                value.isascii() and value.isdigit() and 0 <= int(value) <= 255)), "%P"))
        self.brightness_entry.pack(side="left", padx=4)
        self.var_bri_input.trace_add("write", self._brightness_input_changed)
        self.scale_bri = tk.Scale(ctrl, from_=0, to=255, orient="horizontal",
                                  variable=self.var_bri, length=170,
                                  bg=C_BG, fg=C_TEXT, highlightthickness=0,
                                  troughcolor="#dbe3ec", activebackground=C_ACCENT,
                                  showvalue=False, font=FONT, command=self._brightness_slider_changed)
        self.scale_bri.grid(row=0, column=2, columnspan=2, pady=(0, 8))

        # 动作按钮
        for index, (text, cmd, tip) in enumerate([
            ("全灭", self.act_clear, "发 CLEAR，本地也清空"),
            ("全亮", self.act_fill, "发 FILL 255"),
            ("边框", self.act_border, "本地画边框后发整帧"),
            ("反色", self.act_invert, "本地反色后发整帧"),
            ("发送整帧", self.act_send_frame, "把本地 100 个亮度发成 FRAME"),
            ("清空本地", self.act_clear_local, "只清本地画面，不发命令"),
            ("读取当前帧", self.act_grid, "发 GRID，内容看下方日志"),
            ("LED 检测", self.act_fault,
             "发 FAULT：让 LP5860 自报这些 LED 有没有电流流过（会先把整屏点亮）"),
        ]):
            b = tk.Button(ctrl, text=text, font=FONT, anchor="w",
                          bg=C_PANEL, fg=C_TEXT, activebackground="#e8f0fe",
                          activeforeground=C_TEXT, relief="solid", bd=1,
                          cursor="hand2", padx=6, pady=3, command=cmd)
            b.grid(row=1 + index // 4, column=index % 4, padx=3, pady=3, sticky="ew")
            # 简易提示（鼠标悬停显示）
            self._tooltip(b, tip)

        extra = tk.Frame(self.root, bg=C_BG)
        extra.pack(fill="x", padx=14, pady=(0, 10))
        tk.Label(extra, text="闪烁 Hz", font=FONT, bg=C_BG, fg=C_TEXT).pack(side="left")
        self.var_hz = tk.DoubleVar(value=2.0)
        tk.Spinbox(extra, from_=0.5, to=20.0, increment=0.5, width=5,
                   textvariable=self.var_hz).pack(side="left", padx=5)
        self.btn_blink = tk.Button(extra, text="开始闪烁", command=self.toggle_blink)
        self.btn_blink.pack(side="left", padx=5)
        tk.Label(extra, text="相机", font=FONT, bg=C_BG, fg=C_TEXT).pack(side="left", padx=(14, 3))
        self.var_cam = tk.StringVar(value="自动")
        tk.Entry(extra, textvariable=self.var_cam, width=5).pack(side="left")
        self.btn_camera = tk.Button(extra, text="打开预览", command=self.toggle_camera)
        self.btn_camera.pack(side="left", padx=5)
        self.btn_cam_apply = tk.Button(extra, text="应用参数", command=self.apply_camera_params)
        self.btn_cam_apply.pack(side="left", padx=5)
        tk.Checkbutton(extra, text="去畸变", variable=self.var_undistort,
                       bg=C_BG).pack(side="left", padx=5)
        params = tk.Frame(self.root, bg=C_BG)
        params.pack(fill="x", padx=14, pady=(0, 8))
        tk.Label(params, text="分辨率", font=FONT, bg=C_BG, fg=C_TEXT).pack(side="left")
        ttk.Combobox(params, textvariable=self.var_cam_width, values=(640, 1280, 1920, 3840), width=5,
                     state="readonly").pack(side="left", padx=(4, 2))
        tk.Label(params, text="×", bg=C_BG, fg=C_TEXT).pack(side="left")
        ttk.Combobox(params, textvariable=self.var_cam_height, values=(480, 720, 1080, 2160), width=5,
                     state="readonly").pack(side="left", padx=(2, 10))
        tk.Label(params, text="帧率", font=FONT, bg=C_BG, fg=C_TEXT).pack(side="left")
        tk.Spinbox(params, from_=5, to=60, increment=5, textvariable=self.var_cam_fps, width=4).pack(side="left", padx=(4, 10))
        tk.Label(params, text="曝光", font=FONT, bg=C_BG, fg=C_TEXT).pack(side="left")
        tk.Spinbox(params, from_=-13, to=0, increment=1, textvariable=self.var_cam_exposure, width=4).pack(side="left", padx=(4, 10))
        tk.Label(params, text="增益", font=FONT, bg=C_BG, fg=C_TEXT).pack(side="left")
        tk.Spinbox(params, from_=0, to=100, increment=1, textvariable=self.var_cam_gain, width=4).pack(side="left", padx=(4, 8))
        tk.Checkbutton(params, text="自动曝光", variable=self.var_auto_exposure, bg=C_BG,
                       command=self.apply_camera_params).pack(side="left")
        preview_panel = tk.Frame(mid, bg=C_BG)
        preview_panel.pack(side="left", fill="both", expand=True)
        self.camera_label = tk.Canvas(preview_panel, width=640, height=360, bg="#111827",
                                      highlightthickness=1, highlightbackground="#475569")
        self.camera_text = self.camera_label.create_text(0, 0, text="相机未打开",
                                                         fill="white", font=FONT)
        self.camera_label.bind("<Configure>", lambda event: self.camera_label.coords(
            self.camera_text, event.width // 2, event.height // 2))
        tk.Label(self.root, textvariable=self.var_intrinsics_status, font=FONT,
                 bg=C_BG, fg=C_TEXT_DIM).pack(fill="x", padx=14, pady=(0, 6))
        record_bar = tk.Frame(self.root, bg=C_BG)
        record_bar.pack(fill="x", padx=14, pady=(0, 6))
        self.btn_capture = tk.Button(record_bar, text="开始采集 4K/30", command=self.toggle_capture)
        self.btn_capture.pack(side="left")
        self.var_capture_status = tk.StringVar(value="JPEG 原图＋逐帧时间戳 · 自动保存")
        tk.Label(record_bar, textvariable=self.var_capture_status, font=FONT,
                 bg=C_BG, fg=C_TEXT_DIM).pack(side="left", padx=12)
        self.camera_controls = list(params.winfo_children()) + [self.btn_camera, self.btn_cam_apply]

        recognition_bar = tk.Frame(preview_panel, bg=C_BG)
        recognition_bar.pack(fill="x", pady=(0, 4))
        tk.Checkbutton(recognition_bar, text="预览识别（4K 全图）", variable=self.var_recognition,
                       command=self._recognition_changed, bg=C_BG).pack(side="left")
        tk.Label(recognition_bar, text="阈值", bg=C_BG).pack(side="left", padx=(8, 3))
        tk.Spinbox(recognition_bar, from_=1, to=254, width=4,
                   textvariable=self.var_recognition_threshold).pack(side="left")
        self._tooltip(self.camera_label, "始终使用完整 3840×2160 原图识别；显示缩放不改变识别输入")
        recognition_status = tk.Label(preview_panel, textvariable=self.var_recognition_status,
                 font=(tkfont.nametofont(FONT).actual('family'), 12),
                 bg=C_BG, fg=C_TEXT_DIM, wraplength=400, anchor="w")
        recognition_status.pack(fill="x", pady=(0, 4))
        pose_status = tk.Label(preview_panel, textvariable=self.var_pose_status,
                               bg=C_BG, fg='#166534', anchor='w', justify='left')
        pose_status.pack(fill='x', pady=(0, 4))
        preview_panel.bind('<Configure>', lambda event: [label.configure(wraplength=max(100, event.width-10))
                                                         for label in (recognition_status, pose_status)])
        self.camera_label.pack(fill="both", expand=True)

        # ---- 底部：日志 ----
        logf = tk.Frame(self.root, bg=C_BG)
        logf.pack(side="bottom", fill="x", padx=14, pady=(0, 14))

        tk.Label(logf, text="通信日志", font=FONT, bg=C_BG, fg=C_TEXT_DIM).pack(anchor="w")
        box = tk.Frame(logf, bg=C_BG)
        box.pack(fill="both", expand=True, pady=(4, 0))

        self.log = tk.Text(box, width=1, height=3, font=FONT, bg=C_PANEL,
                           fg=C_TEXT, relief="solid", bd=1, highlightthickness=0,
                           wrap="none", state="disabled")
        self.log.pack(side="left", fill="both", expand=True)
        sb = tk.Scrollbar(box, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=sb.set)
        self.log.tag_config("tx", foreground=C_ACCENT_DK)
        self.log.tag_config("rx", foreground=C_OK)
        self.log.tag_config("err", foreground=C_ERR)
        mid.pack(fill="both", expand=True, padx=14, pady=(0, 10))

    def _build_grid(self, canvas):
        """在 Canvas 上画 10x10 个矩形，边框留一点缝，看起来像 LED 屏。"""
        cells = []
        step = CELL + GAP
        for y in range(GRID):
            for x in range(GRID):
                x0 = GAP + x * step
                y0 = GAP + y * step
                rect = canvas.create_rectangle(
                    x0, y0, x0 + CELL, y0 + CELL,
                    fill=C_OFF, outline=C_LINE, width=1)
                cells.append(rect)
        return cells

    @staticmethod
    def _tooltip(widget, text):
        """极简 tooltip：鼠标进入时在控件下方显示一行小字。"""
        tip = {"win": None}

        def show(_):
            if tip["win"]:
                return
            w = tk.Toplevel(widget)
            w.wm_overrideredirect(True)
            w.wm_geometry(f"+{widget.winfo_rootx() + 20}+{widget.winfo_rooty() + 24}")
            tk.Label(w, text=text, font=FONT, bg="#fffbe6",
                     fg=C_TEXT, relief="solid", bd=1, padx=6, pady=2).pack()
            tip["win"] = w

        def hide(_):
            if tip["win"]:
                tip["win"].destroy()
                tip["win"] = None

        widget.bind("<Enter>", show, add="+")
        widget.bind("<Leave>", hide, add="+")

    def _bind_keys(self):
        self.root.bind("<KeyPress>", self._on_shortcut)

    def _on_shortcut(self, event):
        if event.widget.winfo_class() in ("Entry", "TEntry", "Spinbox", "TSpinbox", "TCombobox", "Text"):
            return
        if event.state & 0xC:  # Ctrl / Alt 组合键不作为单字母操作。
            return
        action = {"c": self.act_clear, "f": self.act_fill, "b": self.act_border,
                  "r": self.act_invert, "s": self.act_send_frame}.get(event.keysym.lower())
        if action:
            action()
            return "break"

    # ---------------- 绘制 ----------------
    @staticmethod
    def _cell_rgb(v):
        """亮度 -> 颜色。用 gamma(0.7) 让低亮度也能看出差别。"""
        if v <= 0:
            return C_OFF_RGB
        t = (min(int(v), 255) / 255.0) ** 0.7
        return tuple(int(C_OFF_RGB[i] + (C_ON_RGB[i] - C_OFF_RGB[i]) * t) for i in range(3))

    def _redraw(self, preserve_brightness_mask=False):
        """按本地帧缓冲整屏重画。"""
        self._recognition_changed()
        if not preserve_brightness_mask:
            if self.brightness_job is not None:
                self.root.after_cancel(self.brightness_job)
                self.brightness_job = None
            self.brightness_mask = [v > 0 for v in self.frame]
        for idx, v in enumerate(self.frame):
            r, g, b = self._cell_rgb(v)
            self.canvas.itemconfig(self.cells[idx], fill=f"#{r:02x}{g:02x}{b:02x}")

    def _redraw_cell(self, x, y):
        """只重画一个格子（拖动时性能好）。"""
        self._recognition_changed()
        self.brightness_mask[y * GRID + x] = self.frame[y * GRID + x] > 0
        r, g, b = self._cell_rgb(self.frame[y * GRID + x])
        self.canvas.itemconfig(self.cells[y * GRID + x], fill=f"#{r:02x}{g:02x}{b:02x}")

    def _cell_at(self, px, py):
        """画布像素坐标 -> (x, y)；落在格子外返回 None。"""
        step = CELL + GAP
        x = int((px - GAP) // step)
        y = int((py - GAP) // step)
        if 0 <= x < GRID and 0 <= y < GRID:
            return x, y
        return None

    # ---------------- 鼠标交互 ----------------
    def _on_press(self, ev):
        self.canvas.focus_set()
        c = self._cell_at(ev.x, ev.y)
        if not c:
            return
        x, y = c
        self.last_drag_cell = c
        # 单击 = 切换：原来是亮的就熄灭，原来是灭的就按当前亮度点亮
        newv = 0 if self.frame[y * GRID + x] > 0 else self.var_bri.get()
        self._set_and_send(x, y, newv)

    def _on_drag(self, ev):
        c = self._cell_at(ev.x, ev.y)
        if not c or c == self.last_drag_cell:
            return                      # 还在同一格里，不重复发
        self.last_drag_cell = c
        x, y = c
        self._set_and_send(x, y, self.var_bri.get())   # 拖动 = 涂亮，不翻转

    def _on_release(self, _):
        self.last_drag_cell = None

    def _on_right_click(self, ev):
        c = self._cell_at(ev.x, ev.y)
        if c:
            self._set_and_send(c[0], c[1], 0)          # 右键 = 擦除

    # ---------------- 动作 ----------------
    def _brightness_slider_changed(self, value):
        value = str(int(float(value)))
        if self.var_bri_input.get() != value:
            self.var_bri_input.set(value)

    def _brightness_input_changed(self, *_):
        if self.switching_device:
            return
        try:
            value = int(self.var_bri_input.get())
        except ValueError:
            return  # 允许输入过程暂时清空；不发送无效亮度。
        if not 0 <= value <= 255:
            return
        self.var_bri.set(value)
        updated = [value if enabled else 0 for enabled in self.brightness_mask]
        if updated == self.frame:
            return
        self.frame = updated
        self._redraw(preserve_brightness_mask=True)
        # 连续拖动时每 30 ms 最多发送一次最新值，调到 0 仍保留原图案选点。
        if self.brightness_job is None:
            self.brightness_job = self.root.after(30, self._flush_brightness)

    def _flush_brightness(self):
        if self.brightness_job is not None:
            self.root.after_cancel(self.brightness_job)
            self.brightness_job = None
        blinking = self.blink_job is not None
        if self._send_frame() and blinking:
            self.toggle_blink()

    def _set_and_send(self, x, y, v):
        """更新本地画面并发单点命令。"""
        self.frame[y * GRID + x] = int(v)
        self._redraw_cell(x, y)
        self._send(f"PX {x} {y} {int(v)}")

    def act_clear(self):
        self.frame = [0] * (GRID * GRID)
        self._redraw()
        self._send("CLEAR")

    def act_fill(self):
        v = self.var_bri.get()
        self.frame = [v] * (GRID * GRID)
        self._redraw()
        self._send(f"FILL {v}")

    def act_border(self):
        v = self.var_bri.get()
        self.frame = [0] * (GRID * GRID)
        for i in range(GRID):
            self.frame[0 * GRID + i] = v          # 顶行
            self.frame[(GRID - 1) * GRID + i] = v  # 底行
            self.frame[i * GRID + 0] = v          # 左列
            self.frame[i * GRID + (GRID - 1)] = v  # 右列
        self._redraw()
        self._send_frame()

    def act_invert(self):
        self.frame = [0 if v > 0 else self.var_bri.get() for v in self.frame]
        self._redraw()
        self._send_frame()

    def act_clear_local(self):
        self.frame = [0] * (GRID * GRID)
        self._redraw()
        self._log("本地画面已清空（未发送命令）")

    def act_send_frame(self):
        self._send_frame()

    def _refresh_patterns(self):
        self.pattern_list.configure(values=sorted(
            (p.stem for p in self.pattern_dir.glob("*.json")), reverse=True))

    def act_save_pattern(self):
        name = simpledialog.askstring("保存图案", "图案名称：", parent=self.root,
                                      initialvalue=self.var_pattern.get())
        if name is None:
            return False
        name = name.strip()
        if (not name or name in (".", "..") or name.endswith(".")
                or any(c in '<>:"/\\|?*' or ord(c) < 32 for c in name)):
            self._log("名称不能为空，且不能包含路径或文件名特殊字符", "err")
            return False
        filename = self.pattern_dir / (name + ".json")
        if filename.exists() and not messagebox.askyesno(
                "覆盖图案", f"已存在“{name}”，是否覆盖？", parent=self.root):
            return False
        temporary = None
        try:
            self.pattern_dir.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8",
                    dir=Path(filename).parent, delete=False) as output:
                temporary = Path(output.name)
                json.dump({"width": GRID, "height": GRID, "frame": self.frame}, output, indent=2)
                output.write("\n")
            os.replace(temporary, filename)
        except OSError as exc:
            self._log(f"保存图案失败：{exc}", "err")
            return False
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        self._refresh_patterns()
        self.var_pattern.set(filename.stem)
        self._log(f"图案已保存：{filename}")
        return True

    def _select_pattern(self, event=None):
        name = self.var_pattern.get()
        if name in self.pattern_list["values"]:
            self.act_load_pattern(self.pattern_dir / (name + ".json"))

    def act_load_pattern(self, filename=None):
        importing = filename is None
        if importing:
            filename = filedialog.askopenfilename(parent=self.root, title="导入图案并点亮",
                        filetypes=[("图案文件", "*.json")])
        if not filename:
            return
        try:
            data = json.loads(Path(filename).read_text(encoding="utf-8"))
            if (not isinstance(data, dict) or data.get("width") != GRID
                    or data.get("height") != GRID):
                raise ValueError("图案必须为 10×10")
            frame = data.get("frame")
            if (not isinstance(frame, list) or len(frame) != GRID * GRID
                    or any(type(v) is not int or not 0 <= v <= 255 for v in frame)):
                raise ValueError("图案必须包含 100 个 0～255 的整数亮度")
        except (OSError, ValueError) as exc:
            self._log(f"加载图案失败：{exc}", "err")
            return
        self.frame = frame
        if not importing:
            self.var_pattern.set(Path(filename).stem)
        self._redraw()
        self._log(f"图案已加载：{filename}；未连接时可连接后按 S 发送整帧")
        self._send_frame()

    def toggle_blink(self):
        if self.blink_job is not None:
            self.blink_job = None
            self.btn_blink.configure(text="开始闪烁")
            self._send("BLINK STOP")
            self.blink_frame = None
            return
        try:
            hz = float(self.var_hz.get())
            if not 0.5 <= hz <= 20:
                raise ValueError
        except (TypeError, ValueError):
            self._log("闪烁频率必须在 0.5~20 Hz", "err")
            return
        self.blink_frame = self.frame[:]
        self.blink_job = True
        self.btn_blink.configure(text="停止闪烁")
        self._send(f"BLINK {hz:g}")

    def _blink_tick(self, half_ms):
        return

    def _blink_next(self, half_ms):
        self.blink_job = None
        self._blink_tick(half_ms)

    def toggle_camera(self):
        if self.capture is not None:
            self._log("采集期间请先停止采集，再关闭相机", "err")
            return
        self.camera_generation += 1
        self._recognition_changed()
        if self.camera_running:
            self.camera_running = False
            if self.camera_job is not None:
                self.root.after_cancel(self.camera_job)
                self.camera_job = None
            if self.camera:
                self.camera.release()
            self.camera = None
            self.camera_photo = None
            self.camera_label.delete("preview")
            self.camera_label.itemconfigure(self.camera_text, state="normal", text="相机未打开")
            self.btn_camera.configure(text="打开预览")
            return
        source = self.var_cam.get().strip()
        try:
            if not source or source in ("自动", "auto", "AUTO"):
                candidates = sorted(glob.glob("/dev/v4l/by-id/*")) + sorted(glob.glob("/dev/video*"))
                preferred = [p for p in candidates if "DECXIN" in p.upper() or "1BCF" in p.upper()]
                source = (preferred or candidates or [0])[0]
                self.var_cam.set(source)
            elif source.isdigit():
                source = int(source)
            self.camera = cv2.VideoCapture(source, cv2.CAP_V4L2) if isinstance(source, str) else cv2.VideoCapture(source)
            if not self.camera.isOpened():
                raise RuntimeError("无法打开相机")
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.apply_camera_params()
            for _ in range(10):
                self.camera.grab()
        except Exception as exc:
            self._log(f"相机打开失败：{exc}（请检查设备路径）", "err")
            if self.camera is not None:
                self.camera.release()
            self.camera = None
            return
        self.camera_running = True
        self.btn_camera.configure(text="关闭预览")
        self.camera_label.itemconfigure(self.camera_text, state="normal", text="等待相机画面…")
        self._camera_tick()

    def _camera_tick(self):
        if self.camera_job is not None:
            self.root.after_cancel(self.camera_job)
            self.camera_job = None
        if not self.camera_running or self.camera is None:
            return
        ok, image = self.camera.read()
        if ok:
            self._display_camera_image(image)
        else:
            self.camera_generation += 1  # Invalidate pending poses after a lost camera frame.
            self._recognition_changed()
            self.var_pose_status.set('位姿：等待相机画面')
            self.camera_label.itemconfigure(self.camera_text, state="normal", text="相机已打开，等待画面…")
        interval = max(10, int(1000 / max(1.0, self.var_cam_fps.get())))
        self.camera_job = self.root.after(interval, self._camera_tick)

    def _recognition_changed(self):
        self.camera_label.delete("recognition")
        self.var_pose_status.set('位姿：等待识别 · 原点为灯板中心' if self.var_recognition.get() else '位姿：识别关闭')
        self.var_recognition_status.set(self._device_name() + (' · 等待匹配 · 4K 全图' if self.var_recognition.get() else ' · 识别关闭'))

    def _recognition_key(self):
        return (tuple(self.frame), self.var_recognition_threshold.get(), self.camera_generation, self.active_device)

    def _display_camera_image(self, image):
        if not self.var_recognition.get():
            self._render_camera_image(image)
            return
        if self.recognition_future is not None:
            return  # One frame in flight; capture continues without accumulating a queue.
        try:
            key = self._recognition_key()
            if not 1 <= key[1] <= 254:
                raise ValueError
        except (ValueError, tk.TclError):
            self.var_recognition_status.set("识别阈值请输入 1～254")
            self._render_camera_image(image)
            return
        if self.recognition_pool is None:
            self.recognition_pool = ThreadPoolExecutor(max_workers=1)
        self.recognition_future = self.recognition_pool.submit(self.recognizer.detect, image, key[0], key[1])
        self.recognition_job = self.root.after(15, self._poll_recognition, image, key)

    def _poll_recognition(self, image, key):
        self.recognition_job = None
        if self.closing:
            return
        if not self.recognition_future.done():
            self.recognition_job = self.root.after(15, self._poll_recognition, image, key)
            return
        future, self.recognition_future = self.recognition_future, None
        try:
            result = future.result()
            if (not self.var_recognition.get() or key != self._recognition_key()
                    or not (self.camera_running or self.capture is not None)):
                return
            self.var_recognition_status.set(self._device_name() + ' · ' + result['status'])
            self._render_camera_image(image, result)
        except Exception as exc:
            self.var_recognition_status.set(f"识别失败：{exc}")
            self.var_pose_status.set('位姿：识别失败')
            self.camera_label.delete('recognition')

    def _render_camera_image(self, image, recognition=None):
        self.var_pose_status.set(pose_text(recognition) if recognition is not None else '位姿：等待识别')
        width = max(1, self.camera_label.winfo_width() - 2)
        height = max(1, self.camera_label.winfo_height() - 2)
        scale = min(width / image.shape[1], height / image.shape[0])
        size = (max(1, round(image.shape[1] * scale)), max(1, round(image.shape[0] * scale)))
        self.preview_geometry = (width // 2 + 1 - size[0] // 2,
                                 height // 2 + 1 - size[1] // 2, *size)
        image = self._prepare_preview(image, size)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.camera_photo = ImageTk.PhotoImage(Image.fromarray(image), master=self.root)
        self.camera_label.itemconfigure(self.camera_text, state="hidden")
        self.camera_label.delete("preview")
        self.camera_label.delete("recognition")
        self.camera_label.create_image(width // 2 + 1, height // 2 + 1,
                                      anchor="center", image=self.camera_photo, tags="preview")
        def screen_points(points):
            if self.var_undistort.get():
                K = self.camera_K.copy()
                K[0] *= size[0] / self.calibration_size[0]
                K[1] *= size[1] / self.calibration_size[1]
                points = cv2.fisheye.undistortPoints(points[:, None], self.camera_K, self.camera_D, P=K)[:, 0]
            else:
                points = points * np.array(size) / self.calibration_size
            return points + [width // 2 + 1 - size[0] // 2, height // 2 + 1 - size[1] // 2]
        if recognition is not None:
            if recognition.get('pose_axes_raw') is not None:
                axes = screen_points(recognition['pose_axes_raw'])
                if np.isfinite(axes).all() and np.abs(axes).max() < 1e6:
                    for label, color, endpoint in zip('XYZ', ('#ff5555', '#38ff72', '#5599ff'), axes[1:]):
                        self.camera_label.create_line(*axes[0], *endpoint, fill=color, width=3, arrow='last',
                                                      dash=(),
                                                      tags=('recognition', 'pose_axes'))
                        self.camera_label.create_text(*endpoint, text=label, fill=color, anchor='sw',
                                                      tags=('recognition', 'pose_axes'))
            if recognition.get('cluster_centers') is not None:
                for x, y in screen_points(recognition['cluster_centers']):
                    if np.isfinite([x, y]).all() and 0 <= x <= width and 0 <= y <= height:
                        self.camera_label.create_oval(x-9, y-9, x+9, y+9,
                                                     outline='#38ff72', width=2, tags='recognition')
            if recognition.get('outline') is not None:
                contour = screen_points(recognition['outline'])
                if np.isfinite(contour).all():
                    self.camera_label.create_polygon(*contour.ravel(), fill='', outline='#38ff72',
                                                     width=2, tags='recognition')
            for x, y in screen_points(recognition['candidates']) if len(recognition['candidates']) else []:
                if np.isfinite([x, y]).all() and 0 <= x <= width and 0 <= y <= height:
                    self.camera_label.create_oval(x-5, y-5, x+5, y+5, outline="#ffcc33", tags="recognition")
            if recognition['matched'] is not None:
                for led, (x, y) in zip(recognition['ids'], screen_points(recognition['matched'])):
                    if not (np.isfinite([x, y]).all() and 0 <= x <= width and 0 <= y <= height):
                        continue
                    self.camera_label.create_oval(x-7, y-7, x+7, y+7, outline="#38ff72", width=2, tags="recognition")
                    self.camera_label.create_text(x+9, y-9, text=f"{led//10+1}.{led%10+1}",
                                                  fill="#38ff72", anchor="sw", tags="recognition")

    def toggle_capture(self):
        if self.capture is not None:
            self.capture.stop_event.set()
            self.btn_capture.configure(state="disabled")
            self.var_capture_status.set("正在完成保存…")
            return
        self.var_cam_width.set(3840)
        self.var_cam_height.set(2160)
        self.var_cam_fps.set(30)
        if self.camera is None:
            self.toggle_camera()
        else:
            self.apply_camera_params()
        if self.camera is None:
            return
        if self.camera_job is not None:
            self.root.after_cancel(self.camera_job)
            self.camera_job = None
        self.var_recognition.set(False)
        self._recognition_changed()
        self._store_device()
        metadata = dict(led_devices=[dict(ip=d["ip"], has_imu=d["imu"], frame=d["frame"]) for d in self.devices],
                        active_led_device=self._device_name(), device=self.var_cam.get(), initial_led_frame=self.frame[:],
                        intrinsics=json.loads(INTRINSICS_PATH.read_text(encoding="utf-8")),
                        processing_mode='offline_full_4k', pattern_basis='initial_frame_only')
        self.capture = CameraCapture(self.camera, CAPTURE_ROOT, metadata)
        self.camera = None  # 采集线程独占相机；界面只读取最新 JPEG 供预览。
        self.camera_running = False
        self.capture_preview_index = 0
        for widget in self.camera_controls:
            widget.configure(state="disabled")
        self.btn_capture.configure(text="停止采集")
        self.var_capture_status.set("正在启动 4K/30 采集…")
        self._log(f"开始采集，数据自动保存到：{CAPTURE_ROOT}")
        self.capture.start()
        self.root.after(100, self._poll_capture)

    def _poll_capture(self):
        capture = self.capture
        if capture is None:
            return
        if not self.closing:
            latest = capture.latest
            if latest is not None and latest[0] != self.capture_preview_index:
                self.capture_preview_index = latest[0]
                image = cv2.imdecode(np.frombuffer(latest[1], dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is not None:
                    self._display_camera_image(image)
            self.var_capture_status.set(f"采集 {capture.frame_count} 帧 · 实际保存 {capture.measured_fps:.1f} fps")
        if capture.is_alive():
            self.root.after(100, self._poll_capture)
            return
        self.capture = None
        for widget in self.camera_controls:
            widget.configure(state="readonly" if isinstance(widget, ttk.Combobox) else "normal")
        self.btn_capture.configure(text="开始采集 4K/30", state="normal")
        summary = f"已保存 {capture.frame_count} 帧，实际 {capture.measured_fps:.1f} fps"
        self.var_capture_status.set("采集失败 · 已有数据保留" if capture.error else summary)
        self._log(f"{summary}；目录：{capture.directory}")
        if capture.error:
            self._log(capture.error, "err")
        if not self.closing:
            self.toggle_camera()

    def _prepare_preview(self, image, size):
        source_size = (image.shape[1], image.shape[0])
        if not self.var_undistort.get():
            self.var_intrinsics_status.set("原始画面 · 去畸变已关闭")
            return cv2.resize(image, size)
        if source_size != self.calibration_size:
            self.var_intrinsics_status.set(
                f"原始画面 · 实际 {source_size[0]}×{source_size[1]} 与 4K 内参不匹配，未去畸变")
            return cv2.resize(image, size)
        if self.preview_map_size != size:
            # 直接从标定分辨率映射到预览尺寸；只缩放输出投影，不假设相机低分辨率是纯缩放。
            preview_K = self.camera_K.copy()
            preview_K[0] *= size[0] / self.calibration_size[0]
            preview_K[1] *= size[1] / self.calibration_size[1]
            self.preview_maps = cv2.fisheye.initUndistortRectifyMap(
                self.camera_K, self.camera_D, np.eye(3), preview_K, size, cv2.CV_32FC1)
            self.preview_map_size = size
        self.var_intrinsics_status.set(f"{self.calibration_name} · 去畸变预览（视野与原图不同）")
        return cv2.remap(image, *self.preview_maps, interpolation=cv2.INTER_LINEAR)

    def apply_camera_params(self):
        if self.capture is not None:
            self._log("采集期间相机参数已锁定为 4K/30 fps", "err")
            return
        if self.camera is None:
            return
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.var_cam_width.get()))
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.var_cam_height.get()))
        self.camera.set(cv2.CAP_PROP_FPS, float(self.var_cam_fps.get()))
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if self.var_auto_exposure.get() else 1)
        if not self.var_auto_exposure.get():
            self.camera.set(cv2.CAP_PROP_EXPOSURE, float(self.var_cam_exposure.get()))
            self.camera.set(cv2.CAP_PROP_GAIN, float(self.var_cam_gain.get()))
        self._log(f"相机请求：{int(self.var_cam_width.get())}×{int(self.var_cam_height.get())}, "
                  f"{self.var_cam_fps.get():g} FPS；读回："
                  f"{int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
                  f"{int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))}, "
                  f"{self.camera.get(cv2.CAP_PROP_FPS):g} FPS（非实测帧率）")

    def act_grid(self):
        self._send("GRID")

    def act_fault(self):
        """LED 开/短路检测：让 LP5860 自己报告电流有没有流过（红外看不见时的电气证据）。

        注意：芯片要求 PWM 达标才触发检测，所以下位机会先把整屏点亮，
        这会覆盖当前图案，结果见下方日志。
        """
        self._send("FAULT")

    def _send_frame(self):
        """把本地 100 个亮度拼成一条 FRAME 命令发出去（行优先）。"""
        vals = " ".join(str(v) for v in self.frame)
        return self._send(f"FRAME {vals}")

    # ---------------- 连接管理 ----------------
    def _device_name(self):
        d = self.devices[self.active_device]
        return d['ip'] + ('（带 IMU）' if d['imu'] else '')

    def _highlight_device(self):
        for i, d in enumerate(self.devices):
            selected = i == self.active_device
            d['canvas'].configure(highlightbackground=C_ACCENT if selected else C_LINE)
            d['heading'].configure(bg=C_ACCENT if selected else C_PANEL,
                                   fg='white' if selected else C_TEXT)

    def _board_event(self, index, handler, event):
        self._select_device(index)
        handler(event)

    def _store_device(self):
        try:
            hz = self.var_hz.get()
        except tk.TclError:
            hz = self.devices[self.active_device]['hz']
        self.devices[self.active_device].update(link=self.link, frame=self.frame[:],
            mask=self.brightness_mask[:], brightness=self.var_bri.get(), hz=hz,
            blink=self.blink_job, pattern=self.var_pattern.get())

    def _select_device(self, index):
        if index == self.active_device:
            return
        if self.brightness_job is not None:
            self._flush_brightness()  # Finish on the original socket before switching.
        self._store_device()
        self.active_device = index
        self.var_device.set(index)
        d = self.devices[index]
        self.link, self.frame, self.brightness_mask = d['link'], d['frame'][:], d['mask'][:]
        self.btn_conn, self.lbl_status = d['button'], d['status']
        self.canvas, self.cells = d['canvas'], d['cells']
        self._highlight_device()
        self.var_ip.set(d['ip'])
        self.switching_device = True
        self.var_bri.set(d['brightness'])
        self.var_bri_input.set(str(d['brightness']))
        self.switching_device = False
        self.var_hz.set(d['hz'])
        self.var_pattern.set(d['pattern'])
        self.blink_job = d['blink']
        self.btn_blink.configure(text='停止闪烁' if self.blink_job else '开始闪烁')
        self._redraw(preserve_brightness_mask=True)
        self.root.title('LED 双设备上位机 · 当前控制 ' + self._device_name())
        self._log('当前控制与识别设备：' + self._device_name())

    def _toggle_connect(self):
        self._toggle_device_connection(self.active_device)

    def _toggle_device_connection(self, index):
        self._store_device()
        d = self.devices[index]
        link = d['link']
        if link.connected:
            link.close()
            d['status'].configure(text='未连接', fg=C_TEXT_DIM)
            d['button'].configure(text='连接')
            self._log(f"[{d['ip']}] 已断开")
            return
        ip = self.var_ip.get().strip() if index == self.active_device else d['ip']
        try:
            port = int(self.var_port.get())
            if not 1 <= port <= 65535:
                raise ValueError('端口必须为 1～65535')
            link.connect(ip, port)
        except (OSError, ValueError) as exc:
            d['status'].configure(text='连接失败', fg=C_ERR)
            self._log(f'[{ip}] 连接失败：{exc}', 'err')
            return
        d['ip'] = ip
        d['status'].configure(text='已连接', fg=C_OK)
        d['button'].configure(text='断开')
        self._log(f'[{ip}:{port}] 已连接，等待 HELLO', 'rx')

    def _set_status(self, ok, text):
        self.lbl_status.configure(text=text, fg=(C_OK if ok else C_TEXT_DIM))

    # ---------------- 收发与日志 ----------------
    def _send(self, cmd):
        try:
            self.link.send(cmd)
        except ConnectionError:
            self._log("发送失败：尚未连接，先点「连接」", "err")
            return
        except OSError as e:
            self._log(f"发送失败：{e}（连接可能已断开）", "err")
            self.link.close()
            self._set_status(False, "● 已断开")
            self.btn_conn.configure(text="连接")
            return
        # 命令可能很长（FRAME 全帧约 350 字符），日志里截断显示
        if cmd.split()[0] in ("CLEAR", "FILL", "PX", "ROW", "FRAME"):
            # 固件收到图案修改命令会停止闪烁，界面同步恢复。
            self.blink_job = None
            self.blink_frame = None
            self.btn_blink.configure(text="开始闪烁")
        shown = cmd if len(cmd) <= 70 else cmd[:67] + "..."
        self._log("[" + self._device_name() + "] → " + shown, "tx")
        return True

    def _log(self, text, tag=None):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n", tag or "")
        # 超过上限就删掉最老的行，防止内存一直涨
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > LOG_MAX_LINES:
            self.log.delete("1.0", f"{lines - LOG_MAX_LINES}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _poll_rx(self):
        """主线程定时把接收队列里的行搬进日志区。"""
        self.devices[self.active_device]['link'] = self.link
        for d in self.devices:
            try:
                for _ in range(100):  # Bound each tick so IMU traffic cannot starve the GUI.
                    self._log(f"[{d['ip']}] ← " + d['link'].rx_queue.get_nowait(), 'rx')
            except queue.Empty:
                pass
            if not d['link'].connected and d['button']['text'] == '断开':
                d['button'].configure(text='连接')
                d['status'].configure(text='已断开', fg=C_ERR)
        self.root.after(80, self._poll_rx)

    def _on_close(self):
        self.closing = True
        if self.recognition_job is not None:
            self.root.after_cancel(self.recognition_job)
            self.recognition_job = None
        if self.recognition_pool is not None:
            self.recognition_pool.shutdown(wait=False, cancel_futures=True)
            self.recognition_pool = None
        if self.capture is not None and self.capture.is_alive():
            self.capture.stop_event.set()
            self.var_capture_status.set("正在完成保存，请稍候…")
            self.root.after(100, self._on_close)
            return
        if self.brightness_job is not None:
            self.root.after_cancel(self.brightness_job)
        if self.camera_job is not None:
            self.root.after_cancel(self.camera_job)
        self.camera_running = False
        if self.camera:
            self.camera.release()
        self._store_device()
        for device in self.devices:
            device["link"].close()
        self.root.destroy()


# ======================================================================
#  入口
# ======================================================================
def main():
    ap = argparse.ArgumentParser(description="ESP32-S3 LED 矩阵上位机")
    ap.add_argument("--ip", default=DEFAULT_IP, help=f"设备 IP，默认 {DEFAULT_IP}")
    ap.add_argument("--check", action="store_true",
                    help="只检查运行环境（tkinter 是否可用）后退出，不开窗口")
    args = ap.parse_args()

    if args.check:
        # 不创建窗口，只验证依赖和配置，方便命令行自检
        print(f"Python   : {sys.version.split()[0]}  ({sys.executable})")
        print(f"tkinter  : Tk {tk.TkVersion}")
        print(f"默认目标 : {args.ip}:{DEFAULT_PORT} / 10.1.30.190:{DEFAULT_PORT}（带 IMU）")
        print(f"点阵     : {GRID}x{GRID}，格子 {CELL}px")
        intrinsics = json.loads(INTRINSICS_PATH.read_text(encoding="utf-8"))
        print(f"内参     : {intrinsics['display_name']}，RMS {intrinsics['rms_error_px']:.4f} px")
        print(f"内参路径 : {INTRINSICS_PATH}")
        print("环境检查通过，可以直接运行（不带 --check）打开界面")
        return

    root = tk.Tk()
    app = LedApp(root)
    app.var_ip.set(args.ip)
    # 开了就自动试连一次，省得每次手点（连不上只记日志，不弹窗打扰）
    root.after(300, app._toggle_connect)
    root.after(600, lambda: app._toggle_device_connection(1))
    root.after(800, app.toggle_camera)
    root.mainloop()


if __name__ == "__main__":
    main()
