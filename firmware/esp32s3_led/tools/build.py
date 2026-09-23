#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build.py —— 不依赖 export.ps1 的一键编译（Windows / 任意 shell 都能用）

为什么需要它
------------
VSCode 里那个 `export.ps1` 在**非交互**会话里 dot-source 之后 IDF_PATH 仍然是空的，
直接跑 `idf.py` 会炸在 `Version.coerce(os.getenv('ESP_IDF_VERSION'))`。
手动设这一堆环境变量很容易漏（尤其是 IDF_TOOLS_PATH 少写一个 \\tools ——
症状是 `espidf.constraints.v6.1.txt doesn't exist`，看着像安装坏了，其实只是路径少了层）。

这个脚本把正确的一组值写死在代码里，谁调用都一样。实测通过的值见 DEFAULTS。

另外两个坑也一起处理了：
  * idf.py 的进度信息走 **stderr**，在 PowerShell 里会被包成 NativeCommandError，
    命令行看起来像"编译失败"，其实编译成功了。所以这里**只看 stdout 里的
    `Project build complete`**，并返回真实退出码。
  * Git Bash（MSys/Mingw）会被 idf.py 主动拒绝（"MSys/Mingw is no longer
    supported"），所以本脚本用 sys.executable 直接拉起 idf.py，绕开 shell 判定。

用法
----
    python tools/build.py            # 直接用任意 python 跑都行（脚本会自己切到 IDF venv）
    python tools/build.py -v         # 显示完整输出
    python tools/build.py --fullclean # 先全清再编

注意：idf.py 必须在 **IDF 自己的 venv** 里运行（它依赖 rich_click 等包），
用别的解释器跑会报 `No module named 'rich_click'`。本脚本会自动用 venv 的
python.exe 重新拉起自己，所以调用方用哪个 python 都无所谓。
"""

import argparse
import glob
import os
import re
import subprocess
import sys

# ---- IDF 环境路径：环境变量 > 自动探测 > 本机默认值 ----
# 开发机（小高这台）的实测值放在最后兜底。
# 换电脑 / 把工程发给别人时不必手改这个文件：只要那边跑过 ESP-IDF 的 export 脚本
# （或手动设过 IDF_PATH / IDF_TOOLS_PATH / IDF_PYTHON_ENV_PATH），就按环境变量走；
# 没设的话下面会去常见安装位置自动找一遍。
_DEFAULTS = {
    "IDF_PATH":       r"C:\esp\v6.1\esp-idf",
    "IDF_TOOLS_PATH": r"C:\Espressif\tools",   # ⚠️ 结尾要到 tools，不是 C:\Espressif
    "PY_ENV":         r"C:\Espressif\tools\python\v6.1\venv",
    "ROM_ELFS":       r"C:\Espressif\tools\esp-rom-elfs\20241011",
}


def _pick_dir(env_name, default, *patterns):
    """环境变量优先，其次按 glob 模式探测（版本号大的优先），最后回退默认值。"""
    v = os.environ.get(env_name)
    if v and os.path.isdir(v):
        return v
    for pat in patterns:
        for hit in sorted(glob.glob(pat), reverse=True):
            if os.path.isdir(hit):
                return hit
    return default


def _all_dirs(*patterns):
    """把多个 glob 模式展开成目录列表（用于拼 PATH）。"""
    out = []
    for pat in patterns:
        out += [h for h in glob.glob(pat) if os.path.isdir(h)]
    return out


IDF_PATH = _pick_dir(
    "IDF_PATH", _DEFAULTS["IDF_PATH"],
    r"C:\esp\*\esp-idf", r"C:\Espressif\frameworks\esp-idf-*",
    os.path.expanduser(r"~\esp\esp-idf"),
    os.path.expanduser(r"~\.espressif\frameworks\esp-idf-*"),
)
IDF_TOOLS_PATH = _pick_dir(
    "IDF_TOOLS_PATH", _DEFAULTS["IDF_TOOLS_PATH"],
    r"C:\Espressif\tools", os.path.expanduser(r"~\.espressif"),
)
PY_ENV = _pick_dir(
    "IDF_PYTHON_ENV_PATH", _DEFAULTS["PY_ENV"],
    os.path.join(IDF_TOOLS_PATH, "python", "*", "venv"),
)
ROM_ELFS = _pick_dir(
    "ESP_ROM_ELF_DIR", _DEFAULTS["ROM_ELFS"],
    os.path.join(IDF_TOOLS_PATH, "esp-rom-elfs", "*"),
)
# 工具链目录按实际安装的版本号通配，不再写死版本（升级 IDF 后不用改这个文件）
PATH_EXTRA = _all_dirs(
    os.path.join(IDF_TOOLS_PATH, "ccache", "*", "*"),
    os.path.join(IDF_TOOLS_PATH, "cmake", "*", "bin"),
    os.path.join(IDF_TOOLS_PATH, "ninja", "*"),
    os.path.join(IDF_TOOLS_PATH, "idf-exe", "*"),
    os.path.join(IDF_TOOLS_PATH, "xtensa-esp-elf", "*", "xtensa-esp-elf", "bin"),
    os.path.join(IDF_TOOLS_PATH, "riscv32-esp-elf", "*", "riscv32-esp-elf", "bin"),
) + [PY_ENV + r"\Scripts"]

VENV_PY = os.path.join(PY_ENV, "Scripts", "python.exe")


def ensure_venv_python():
    """若当前解释器不是 IDF venv 的 python，就用 venv 的 python 重新执行本脚本。

    idf.py 依赖 venv 里装的 rich_click 等包，拿别的解释器跑会直接报
    `No module named 'rich_click'`（看着像环境装坏了，其实只是解释器用错了）。
    """
    if os.path.abspath(sys.executable).lower() == os.path.abspath(VENV_PY).lower():
        return
    if not os.path.isfile(VENV_PY):
        print(f"[!!] 找不到 IDF venv 的 python：{VENV_PY}（继续用当前解释器试）")
        return
    sys.exit(subprocess.run([VENV_PY, os.path.abspath(__file__)] + sys.argv[1:]).returncode)

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)

# 关心的行：错误、警告、以及最终成果
KEY = re.compile(
    r"error:|错误|Error |ERROR|Project build complete|Successfully created|"
    r"binary size|Generated |warning: |\bFAILED\b",
    re.IGNORECASE,
)


def build_env():
    env = os.environ.copy()

    # ⚠️ 关键：idf.py 只要看到 MSYSTEM 就认为你在 MSys/Mingw 里跑，直接打印
    #   "MSys/Mingw is no longer supported" 然后什么都不干（而且退出码还是 0，
    #   看起来像"编译成功但没输出"）。从 Git Bash 调用时这个变量一定存在，必须剥掉。
    for k in ("MSYSTEM", "MSYSCON", "MINGW_PREFIX", "MINGW_CHOST",
              "MINGW_PACKAGE_PREFIX", "MSYS_NO_PATHCONV"):
        env.pop(k, None)

    env["IDF_PATH"] = IDF_PATH
    env["IDF_TOOLS_PATH"] = IDF_TOOLS_PATH
    env["IDF_PYTHON_ENV_PATH"] = PY_ENV
    env["ESP_IDF_VERSION"] = "6.1"      # 少了这个 -> Version.coerce(None) TypeError
    env["IDF_VERSION"] = "6.1.0"
    env["ESP_ROM_ELF_DIR"] = ROM_ELFS
    env["PATH"] = os.pathsep.join(PATH_EXTRA + [env.get("PATH", "")])
    # 关掉约束文件校验：CI / 非交互环境下没跑过 install 脚本时这一步会直接拦下来
    env["IDF_PYTHON_CHECK_CONSTRAINTS"] = "0"
    return env


def main():
    ensure_venv_python()          # 保证下面用的是 IDF venv 的解释器

    ap = argparse.ArgumentParser(description="ESP32-S3 灯阵 一键编译")
    ap.add_argument("args", nargs="*", default=["build"], help="传给 idf.py 的参数（默认 build）")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印完整输出")
    ap.add_argument("--fullclean", action="store_true", help="先 fullclean 再 build")
    a = ap.parse_args()

    idf_py = os.path.join(IDF_PATH, "tools", "idf.py")
    if not os.path.isfile(idf_py):
        print(f"[XX] 找不到 {idf_py} —— IDF_PATH 配错了？")
        return 2

    cmds = []
    if a.fullclean:
        cmds.append(["fullclean"])
    cmds.append(a.args or ["build"])

    for c in cmds:
        cmd = [sys.executable, idf_py] + c
        print(f"[--] {' '.join(c)}")
        p = subprocess.run(cmd, cwd=PROJ, env=build_env(),
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = p.stdout.decode("utf-8", "replace")
        lines = [l for l in out.splitlines() if l.strip()]

        if a.verbose:
            print(out)
        else:
            for l in lines:
                if KEY.search(l):
                    print("    " + l)

        # 判据只看 stdout 里的 Project build complete（stderr 的进度信息会骗人）
        done = "Project build complete" in out
        if c[0] == "fullclean":
            if p.returncode != 0:
                print(f"[XX] fullclean 失败 rc={p.returncode}")
                return p.returncode
            continue
        if done:
            print(f"[OK] 编译通过（rc={p.returncode}）")
            return 0
        print(f"[XX] 编译未完成 rc={p.returncode}，上面最后几行就是原因")
        if not a.verbose:
            print("[!!] 想看完整输出：加 -v")
        return p.returncode or 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
