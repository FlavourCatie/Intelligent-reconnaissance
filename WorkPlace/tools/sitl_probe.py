# -*- coding: utf-8 -*-
"""
sitl_probe.py —— 飞控链路探针：SITL 到底有没有连上飞控驱动？
================================================================================

它回答的是本轮最关键的一个问题
------------------------------
飞控驱动 `driver.exe` 每造一架飞机，都会拉起一个 ArduPilot SITL
（`bin/driver/bin/quadrotor_windows/model`，Cygwin 编的），然后用 dronekit
通过 **UDP** 跟它交换 MAVLink（HEARTBEAT）。用 Linux 版留档的命令行可以看清协议：

    model -S --model quadrotor --home 32,118,0,0 \\
          --defaults .../default.parm \\
          --base-port 28477 --serial0 udpclient:127.0.0.1:28477 -I0 --sysid 1

即：SITL 作为 UDP **客户端**，把 MAVLink 发到 `127.0.0.1:<base-port>`，
飞控驱动在那个端口上收心跳。

而 `driver.exe` 启动 SITL 时是把命令行拼成**一个字符串**（不是参数列表），
`--defaults <路径>` 没加引号。**工作区路径一带空格**，参数就被劈开，
后面的 `--serial0 udpclient:...` 整段丢失，SITL 退回默认的 TCP 5760：

    bind port 5760 for 0
    Serial port 0 on TCP port 5760
    Waiting for connection ....          ← 等一个永远不会来的连接

结果：dronekit 收不到心跳 → `UAV#n: Connect Timeout!` → 飞机没有动力学实例
→ `control_kinetic_simply()` 返回 200 但**零位移**。

本工具做两件事
--------------
1. 按"正确引号"（参数列表）拉起 SITL，监听 UDP，看有没有 HEARTBEAT —— 证明 SITL 本身没问题；
2. 按"天真拼串"（一个字符串、不加引号）再拉起一次 —— 如果路径带空格，这次必然拿不到心跳。

两次结果放一起，就能把"环境坏了"和"路径带空格"区分开。

用法
----
    python WorkPlace\\tools\\sitl_probe.py                  # 默认四旋翼，跑两种启动方式
    python WorkPlace\\tools\\sitl_probe.py --kind fixed_wing
    python WorkPlace\\tools\\sitl_probe.py --path D:\\some\\quadrotor_windows
    python WorkPlace\\tools\\sitl_probe.py --timeout 20 --keep
    python WorkPlace\\tools\\sitl_probe.py --dry-run        # 只打印命令，不启动

判读
----
    引号版 PASS / 拼串版 FAIL  → 路径带空格，按 docs/飞控调通_根因与修复.md 处理
    引号版 PASS / 拼串版 PASS  → 路径没问题，飞控另有故障（拿这段输出再查）
    引号版 FAIL                → SITL 本体起不来（缺 DLL / 被杀软拦 / 端口被占）

只依赖标准库，不 import 课程 SDK —— 所以**不需要仿真器、不需要 conda 环境**，
随便一个 Python 3 都能跑。
"""

from __future__ import print_function

import argparse
import os
import socket
import subprocess
import sys
import threading
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, WORKPLACE_DIR)

from core import env_paths  # noqa: E402

#: MAVLink 报文头魔数
MAGIC_V1 = 0xFE
MAGIC_V2 = 0xFD
#: HEARTBEAT 的 msgid
MSGID_HEARTBEAT = 0

#: SITL 的 home 点（与 driver 日志里一致：32N 118E）
HOME = "32.000000,118.000000,0.000000,0.000000"


# ---------------------------------------------------------------------------
#  MAVLink 最小解析（只要 msgid，不解 payload）
# ---------------------------------------------------------------------------
def parse_mavlink(buf):
    """返回 (msgid, sysid)；不是 MAVLink 帧返回 None。"""
    if len(buf) < 8:
        return None
    magic = buf[0] if isinstance(buf[0], int) else ord(buf[0])
    if magic == MAGIC_V1:
        return (buf[5], buf[3])
    if magic == MAGIC_V2:
        if len(buf) < 10:
            return None
        msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
        return (msgid, buf[5])
    return None


# ---------------------------------------------------------------------------
#  端口挑选
# ---------------------------------------------------------------------------
def _udp_free(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def _tcp_free(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def pick_base_port(start=28400, span=200):
    """挑一个 base-port：base 用 UDP，base+2/base+3 给 SITL 的 serial2/3 用 TCP。"""
    for base in range(start, start + span):
        if _udp_free(base) and _tcp_free(base + 2) and _tcp_free(base + 3):
            return base
    raise RuntimeError("找不到空闲端口（试过 %d~%d）" % (start, start + span))


# ---------------------------------------------------------------------------
#  命令行构造（两种拼法，故意都保留）
# ---------------------------------------------------------------------------
def build_args(sitl_dir, base_port, index=0):
    exe = os.path.join(sitl_dir, "model")
    parm = os.path.join(sitl_dir, "default.parm")
    return [
        exe, "-S", "--model", "quadrotor",
        "--home", HOME,
        "--defaults", parm,
        "--base-port", str(base_port),
        "--serial0", "udpclient:127.0.0.1:%d" % base_port,
        "-I", str(index),
        "--sysid", str(index + 1),
    ]


def build_string(args):
    """飞控驱动真正的做法：`" ".join(args)` —— 含空格的路径不会被加引号。"""
    return " ".join(args)


# ---------------------------------------------------------------------------
#  一次性探测
# ---------------------------------------------------------------------------
class Drain(threading.Thread):
    """把子进程 stdout 拉干（否则管道写满会把 SITL 卡死）。"""

    def __init__(self, handle):
        threading.Thread.__init__(self)
        self.daemon = True
        self.handle = handle
        self.lines = []

    def run(self):
        try:
            for raw in iter(self.handle.readline, b""):
                text = raw.decode("utf-8", "replace").rstrip()
                if text:
                    self.lines.append(text)
        except Exception:
            pass


def probe(sitl_dir, mode, timeout=15.0, keep=False, index=0):
    """mode: 'quoted'（参数列表）或 'naive'（拼接字符串）。返回 dict。"""
    base_port = pick_base_port()
    args = build_args(sitl_dir, base_port, index)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", base_port))
    sock.settimeout(0.5)

    cmd = args if mode == "quoted" else build_string(args)
    popen_kwargs = dict(stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if os.name != "nt":
        # POSIX 上 process group 方便整组回收
        popen_kwargs["preexec_fn"] = os.setsid

    started = time.time()
    proc = subprocess.Popen(cmd, cwd=sitl_dir, **popen_kwargs)
    drain = Drain(proc.stdout)
    drain.start()

    packets = 0
    heartbeats = 0
    first_hb_at = None
    deadline = started + timeout
    while time.time() < deadline:
        if proc.poll() is not None and packets == 0:
            # 进程已经死了，再等也没用
            break
        try:
            data, _addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        packets += 1
        parsed = parse_mavlink(data)
        if parsed and parsed[0] == MSGID_HEARTBEAT:
            heartbeats += 1
            if first_hb_at is None:
                first_hb_at = time.time() - started

    alive = proc.poll() is None
    if not keep:
        try:
            if os.name == "nt":
                subprocess.call(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                os.killpg(os.getpgid(proc.pid), 9)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
    sock.close()

    return {
        "mode": mode,
        "base_port": base_port,
        "command": cmd if isinstance(cmd, str) else " ".join(cmd),
        "argv": args if isinstance(cmd, list) else None,
        "packets": packets,
        "heartbeats": heartbeats,
        "first_heartbeat_s": first_hb_at,
        "alive": alive,
        "path_has_space": " " in os.path.abspath(sitl_dir),
        "output": drain.lines[:40],
        "ok": heartbeats > 0,
    }


# ---------------------------------------------------------------------------
#  输出
# ---------------------------------------------------------------------------
def show(result, title):
    print()
    print("-" * 78)
    print(title)
    print("-" * 78)
    print("  目录      : %s%s" % (result.get("dir", ""),
                                  "   [含空格]" if result.get("path_has_space") else ""))
    print("  base-port : %d" % result["base_port"])
    print("  命令      : %s" % result["command"])
    print("  存活      : %s" % ("是" if result["alive"] else "否（已退出）"))
    print("  收包      : %d 个（其中 HEARTBEAT %d 个）"
          % (result["packets"], result["heartbeats"]))
    if result["first_heartbeat_s"] is not None:
        print("  首个心跳  : 启动后 %.1f s" % result["first_heartbeat_s"])
    if result["output"]:
        print("  SITL 前几行:")
        for line in result["output"][:12]:
            print("      " + line)
    print("  判定      : %s" % ("PASS 有 MAVLink 心跳" if result["ok"]
                                else "FAIL 收不到心跳"))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="飞控链路探针：验证 SITL 能不能被拉起来并回 MAVLink 心跳")
    parser.add_argument("--path", help="直接指定 SITL 目录（含 model 的那层）")
    parser.add_argument("--kind", default="quadrotor",
                        choices=("quadrotor", "fixed_wing"),
                        help="机型，与 start.bat 的选项对应（默认四旋翼）")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="每种启动方式监听多少秒（默认 15）")
    parser.add_argument("--index", type=int, default=0,
                        help="实例号 -I（默认 0）")
    parser.add_argument("--only", choices=("quoted", "naive"),
                        help="只跑其中一种（默认两种都跑）")
    parser.add_argument("--keep", action="store_true", help="探测完不杀进程")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印将要执行的命令")
    parser.add_argument("--json", action="store_true", help="输出 JSON（给脚本用）")
    args = parser.parse_args(argv)

    sitl_dir = args.path or env_paths.find_sitl_dir(args.kind)
    if not sitl_dir or not os.path.isfile(os.path.join(sitl_dir, "model")):
        print("找不到 SITL 目录（model 二进制）。")
        print("  试过: %s" % (env_paths.find_sitl_dir(args.kind),))
        print("  用 --path 指定，例如：")
        print("    --path \"<工作区>\\environment_forclass\\environment\\"
              "simu_driver_windows\\bin\\driver\\bin\\quadrotor_windows\"")
        return 2

    print("SITL 目录 : %s" % sitl_dir)
    problems = env_paths.path_problems(sitl_dir)
    if problems:
        for code, msg in problems:
            print("  !! [%s] %s" % (code, msg))

    if args.dry_run:
        base = pick_base_port()
        args_list = build_args(sitl_dir, base, args.index)
        print()
        print("正确拼法（参数列表）:")
        print("  " + " ".join(args_list))
        print()
        print("飞控驱动的拼法（整串，路径带空格就会断）:")
        print("  " + build_string(args_list))
        return 0

    modes = [args.only] if args.only else ["quoted", "naive"]
    results = []
    for mode in modes:
        title = ("方式 A：参数列表（正确加引号）" if mode == "quoted"
                 else "方式 B：拼接字符串（driver.exe 的做法，不加引号）")
        print()
        print(">>> 运行 %s，监听 %.0f s ..." % (title, args.timeout))
        result = probe(sitl_dir, mode, timeout=args.timeout,
                       keep=args.keep, index=args.index)
        result["dir"] = sitl_dir
        results.append(result)
        show(result, title)

    # ---- 结论 ----
    print()
    print("=" * 78)
    by_mode = dict((r["mode"], r) for r in results)
    quoted = by_mode.get("quoted")
    naive = by_mode.get("naive")

    if quoted and quoted["ok"] and naive and not naive["ok"] and naive["path_has_space"]:
        print("结论：SITL 本身没问题，是**路径里的空格**把命令行劈开了。")
        print("      这就是飞控驱动日志里 `UAV#n: Connect Timeout!` 的直接原因。")
        print("      处理：把课程环境放到不含空格的路径（C:\\kyxz），")
        print("            或直接用  python WorkPlace\\tools\\sim_link.py --create")
        print("      详见：WorkPlace\\docs\\飞控调通_根因与修复.md")
    elif quoted and quoted["ok"] and (not naive or naive["ok"]):
        print("结论：SITL 起得来、心跳正常 —— 飞控链路本身没坏。")
        print("      如果实际跑的时候飞机还是不动，问题在别处：")
        print("        1) driver.exe 是否活着（tools\\env_guard.py 看进程与端口）")
        print("        2) config.yaml 的 Type 是不是 quadrotor（start.bat 选 2）")
        print("        3) 启动顺序：CarlaUE4 → start.bat → subject4.exe")
    elif quoted and not quoted["ok"]:
        print("结论：连正确拼法都收不到心跳 —— SITL 本体起不来。")
        print("      看上面的 SITL 输出：常见是 DLL 被杀软隔离、端口被占、")
        print("      或者 model 与当前 Windows 不兼容。")
    else:
        print("结论：见上面的明细。")

    if args.json:
        import json
        print()
        print(json.dumps(results, ensure_ascii=False, indent=2))
    return 0 if all(r["ok"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
