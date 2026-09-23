#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
push_ota.py —— 一条命令完成 WiFi OTA 推送 + 重启后版本核验

为什么要有这个脚本
------------------
OTA 的踩坑点不在板子，而在"电脑这一侧"：
  1. HTTP 服务必须真的在监听（用 `( ... &)` 或 nohup 起的进程，在工具调用结束
     时会被回收，表现为板子报 ESP_ERR_HTTP_CONNECT —— 容易被误判成防火墙/跨网段）；
  2. 板子要访问的是**电脑的局域网 IP**，不是 127.0.0.1。多网卡时自己找容易找错；
  3. 推完必须等板子重启再发 VERSION，否则读到的是旧版本，会白高兴/白悲观一场。

这个脚本把这三件事都做了：自动探测出口 IP → 校验 HTTP 可达且文件大小对得上
→ 发 OTA → 轮询等重启 → 读 VERSION 确认版本真的换了。

用法
----
    python tools/push_ota.py                 # 默认 build/led_matrix_wifi.bin
    python tools/push_ota.py -H 192.168.1.50 # 指定板子 IP
    python tools/push_ota.py --expect v3.0   # 顺带核对版本号

返回码
    0 成功（观察到重启 / 版本变化 / 版本符合 --expect）
    2 板子连不上 / TCP 不通
    3 HTTP 侧不可达（服务没起、或文件不对）
    4 OTA 命令被拒（板子没回 "OTA START"）
    5 超时，或版本不符合 --expect
"""

import argparse
import os
import socket
import sys
import time
import urllib.request
import urllib.error

# ---------------------------------------------------------------- 默认参数
DEFAULT_BOARD = "10.1.30.121"       # 开发环境里板子的 IP；换网络用 -H 覆盖
DEFAULT_TCP_PORT = 8266             # 板子上的 TCP 指令端口（见 tcp_server.h: TCP_SERVER_PORT）
DEFAULT_HTTP_PORT = 8000            # 电脑上临时 HTTP 服务的端口
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
DEFAULT_BIN = os.path.join(PROJ, "build", "led_matrix_wifi.bin")


def log(tag, msg):
    print(f"[{tag}] {msg}", flush=True)


# ---------------------------------------------------------------- 网络小工具
def local_ip_towards(dst_host, dst_port=9):
    """探测"本机走哪张网卡能到板子"。

    做法：开一个 UDP socket connect 到目标地址。UDP 不会真的发包，但内核会
    据此选出路由和源地址，于是 getsockname() 就拿得到那张网卡的 IP。
    比解析 `ipconfig` 靠谱得多 —— 多网卡（有线 + WiFi + 虚拟机网卡）时不会选错。
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((dst_host, dst_port))
        return s.getsockname()[0]
    except OSError as e:
        log("!!", f"探测本机出口 IP 失败: {e}")
        return None
    finally:
        s.close()


def check_http(url, expect_size):
    """确认 HTTP 上真的能拿到固件，且大小和本地文件一致。"""
    try:
        req = urllib.request.Request(url, method="HEAD")
        with urllib.request.urlopen(req, timeout=8) as r:
            size = int(r.headers.get("Content-Length", -1))
    except urllib.error.URLError as e:
        return False, f"HTTP 打不开 {url} -> {e}"
    except Exception as e:                                  # noqa: BLE001
        return False, f"HTTP 探测异常: {e}"

    if size != expect_size:
        return False, (f"HTTP 上的固件大小 {size} != 本地 {expect_size}；"
                       "服务的是旧文件或目录不对（应在 build/ 下起服务）")
    return True, f"HTTP OK，Content-Length = {size}"


class Board:
    """到板子 TCP 指令端口的极简客户端（一次连接，逐行问答）。

    注意：板子每行以 '\\n' 结尾并返回一行响应；'\\r' 会被板子自行裁掉，
    但为了保险这里只发 '\\n'（历史上 CRLF 会把 URL 尾巴带上 \\r 导致 404）。
    """

    def __init__(self, host, port, timeout=5.0):
        self.host, self.port, self.timeout = host, port, timeout
        self.sock = None
        self.buf = b""
        self.banner = ""

    def __enter__(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        # 板子一 accept 就主动发一句 "HELLO <ip>"（见 tcp_server.c: snprintf(resp,"HELLO %s")）。
        # 如果不先把这行吃掉，后面每条命令读回来的都是这句问候语，
        # 看起来就像"命令没生效 / 版本一直没变"——白折腾一轮。
        self.banner = self.read_line()
        return self

    def __exit__(self, *exc):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        return False

    def read_line(self):
        """读一行（以 \\n 结尾），返回去掉换行的字符串。"""
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            self.buf += chunk
        if b"\n" not in self.buf:
            out, self.buf = self.buf, b""
            return out.decode("utf-8", "replace").strip()
        out, self.buf = self.buf.split(b"\n", 1)
        return out.decode("utf-8", "replace").strip()

    def cmd(self, line):
        """发一条指令，回一行响应（不带换行）。"""
        self.sock.sendall((line + "\n").encode())
        return self.read_line()

    def cmd_multi(self, line, idle=0.6):
        """发一条指令，收**多行**响应（GRID / FAULT 这类报告型命令）。

        板子这些命令一次 send 出好几行，而 cmd() 只 split 一次，会只拿到第一行 ——
        看起来像"命令只回了标题就没了"。这里改成：读到连续 idle 秒没有新数据为止。
        判断依据是 recv 超时（socket.timeout），不是行数，这样对任意长度的报告都成立。
        """
        self.sock.sendall((line + "\n").encode())
        lines = []
        old = self.sock.gettimeout()
        self.sock.settimeout(idle)
        try:
            while True:
                try:
                    chunk = self.sock.recv(4096)
                except socket.timeout:
                    break
                if not chunk:
                    break
                self.buf += chunk
        finally:
            self.sock.settimeout(old)
        text = self.buf.decode("utf-8", "replace")
        self.buf = b""
        for ln in text.split("\n"):
            if ln.strip():
                lines.append(ln.rstrip())
        return "\n".join(lines)


def wait_board(host, port, tries, delay=3.0, label=""):
    """轮询等板子的 TCP 端口可用（等重启时用）。"""
    for i in range(tries):
        try:
            with socket.create_connection((host, port), timeout=3.0):
                return True
        except OSError:
            if label:
                log("..", f"{label} 第 {i + 1}/{tries} 次未就绪")
            time.sleep(delay)
    return False


def get_version(host, port):
    try:
        with Board(host, port) as b:
            return b.cmd("VERSION")
    except OSError:
        return None


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="ESP32-S3 灯阵 一键 WiFi OTA 推送")
    ap.add_argument("-H", "--host", default=DEFAULT_BOARD, help="板子 IP")
    ap.add_argument("-p", "--port", type=int, default=DEFAULT_TCP_PORT, help="板子 TCP 指令端口")
    ap.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="电脑上 HTTP 服务端口")
    ap.add_argument("-f", "--file", default=DEFAULT_BIN, help="要推送的固件文件")
    ap.add_argument("--wait", type=int, default=90, help="等板子重启回来的最长时间（秒）")
    ap.add_argument("--expect", default="", help="期望的版本号子串，例如 v1.6（可选）")
    ap.add_argument("--skip-check", action="store_true", help="跳过 HTTP 预检（不推荐）")
    args = ap.parse_args()

    # --- 0. 本地固件 ---
    if not os.path.isfile(args.file):
        log("XX", f"找不到固件：{args.file}（先 idf.py build）")
        return 3
    size = os.path.getsize(args.file)
    log("--", f"本地固件 {args.file}（{size} 字节）")

    # --- 1. 板子在线吗 ---
    log("--", f"探测板子 {args.host}:{args.port} ...")
    if not wait_board(args.host, args.port, tries=3, delay=2.0, label="板子"):
        log("XX", f"连不上 {args.host}:{args.port} —— 板子没上电 / 没连上 WiFi / IP 变了？")
        return 2
    ver_before = get_version(args.host, args.port)
    log(">>", f"当前固件：{ver_before}")

    # --- 2. 电脑这一侧准备 ---
    myip = local_ip_towards(args.host)
    if not myip:
        return 3
    log("--", f"本机到板子的出口 IP = {myip}")
    url = f"http://{myip}:{args.http_port}/{os.path.basename(args.file)}"

    if not args.skip_check:
        ok, msg = check_http(url, size)
        log(">>" if ok else "XX", msg)
        if not ok:
            log("!!", f"先在 build 目录起服务再跑本脚本：")
            log("!!", f'   cd "{os.path.dirname(args.file)}" ; '
                      f'{sys.executable} -m http.server {args.http_port}')
            return 3
    else:
        log("!!", "已跳过 HTTP 预检 —— 这一步失败最容易误判成网络问题")

    # --- 3. 发 OTA ---
    log("--", f"发送 OTA {url}")
    t0 = time.time()
    try:
        with Board(args.host, args.port, timeout=10.0) as b:
            log("--", f"板子问候: {b.banner}")
            resp = b.cmd(f"OTA {url}")
    except OSError as e:
        log("XX", f"OTA 指令发送失败: {e}")
        return 2
    log("<<", resp or "(无响应)")

    if not resp.upper().startswith("OTA START"):
        log("XX", f"板子没有接受 OTA（期望回 'OTA START <url>'，实收 '{resp}'）")
        return 4

    # --- 4. 等重启 + 验版本 ---
    # 判据不能写成"版本必须变化" —— 推同一个版本两次时它就不会变，会误报超时。
    # 真正的地面真相是：板子**掉线过**（=真的重启了）。所以先等掉线，再等它回来。
    log("--", f"等板子重启（最多 {args.wait}s）...")
    deadline = t0 + args.wait
    went_down = False
    while time.time() < deadline:
        time.sleep(2)
        v = get_version(args.host, args.port)
        if v is None:
            if not went_down:
                log(">>", f"板子已掉线（重启中），耗时 {time.time() - t0:.1f}s")
                went_down = True
            continue

        elapsed = time.time() - t0
        ok_reason = None
        if went_down:
            ok_reason = "观察到重启"
        elif v != ver_before:
            ok_reason = f"版本已变化（{ver_before} -> {v}）"
        elif args.expect and args.expect in v and elapsed > 25:
            # 推同一个版本时版本号不会变，也抓不到掉线窗口（重启很快）。
            # 等足够久之后版本已符合预期，就认为成功 —— 但要说清这是"退而求其次"的判据。
            ok_reason = f"未抓到掉线窗口，但 {elapsed:.0f}s 后版本已符合预期"

        if not ok_reason:
            continue

        log("OK", f"耗时 {elapsed:.1f}s —— {ok_reason}")
        log("OK", f"当前固件：{v}（推送前 {ver_before}）")
        if args.expect and args.expect not in v:
            log("XX", f"版本不匹配预期 '{args.expect}'")
            return 5
        return 0

    v = get_version(args.host, args.port)
    log("XX", f"超时（{'观察到重启' if went_down else '未观察到重启'}）。当前版本 {v}")
    if not went_down:
        log("!!", "板子从头到尾没掉线 —— OTA 大概率没真正开始。先看串口日志确认下载是否成功：")
        log("!!", "   python tools\\capture_log.py -p COM3 -t 30")
    return 5


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        log("--", "用户中断")
        sys.exit(130)
