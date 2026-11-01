# -*- coding: utf-8 -*-
"""
uav_diag.py —— 无人机链路一体诊断（合并了原来 7 个脚本）
================================================================================

以前这些脚本是分开的，用起来要点来点去；现在合成一个，用子命令区分：

    info      打印机实际加载的 SDK 路径/哈希 + 关键接口返回值
    takeoff   单机爬升测试（30 秒判定飞控到底行不行）
    kinetic   控制指令 + launch_kinetic 全枚举（判断动力学通道是否可用）
    z         查看/修改场景里无人机的初始高度（vehicles.json）

典型用法
--------
    python WorkPlace\\tools\\uav_diag.py info
    python WorkPlace\\tools\\uav_diag.py takeoff
    python WorkPlace\\tools\\uav_diag.py kinetic --node 4
    python WorkPlace\\tools\\uav_diag.py z --show
    python WorkPlace\\tools\\uav_diag.py z --set 10

判读
----
* `takeoff` 高度上涨        → 飞控可用，`--uav-mode` 可切回 `kinetic`
* `kinetic` 出现非 None 数据 → 找到了动力学启动方式
* 两个都失败                → 交付 SDK 的飞控通道不可用，继续用 `virtual` 模式
"""

from __future__ import print_function

import argparse
import hashlib
import io
import json
import math
import os
import sys
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
PROJECT_DIR = os.path.dirname(WORKPLACE_DIR)
EASY_FOLLOW_DIR = os.path.join(PROJECT_DIR, "Materials", "practice", "easy_follow")
DEFAULT_SCENE = os.path.join(PROJECT_DIR, "Materials", "practice", "practice1_Scene")

sys.path.insert(0, EASY_FOLLOW_DIR)
sys.path.insert(0, WORKPLACE_DIR)

import numpy as np
if not hasattr(np, "mat"):
    np.mat = np.asmatrix


def log(msg):
    print("[uav] " + msg)
    sys.stdout.flush()


def now_ms():
    return int(time.time() * 1000)


def connect(host, port):
    from core.sdk_boot import make_client
    client = make_client(host, port)
    _, game, _ = client.get_game()
    return client, game


def register(client, node=4, name="UAV1"):
    _, n, code = client.register_node(node_type="四旋翼", node_name=name,
                                      node_no=node, frame_timestamp=now_ms())
    if n is None or code != 200:
        raise SystemExit("注册节点失败 code=%r" % (code,))
    return n


# ---------------------------------------------------------------------------
#  info
# ---------------------------------------------------------------------------
def cmd_info(args):
    print("=" * 72)
    print("1) Python 实际加载的 SDK")
    print("=" * 72)
    paths = {}
    for mod in ("swarmae", "swarmae_lib"):
        try:
            m = __import__(mod)
            p = getattr(m, "__file__", None) or getattr(m, "__path__", None)
            paths[mod] = str(p)
            log("%-12s %s" % (mod, p))
        except Exception as e:
            log("%-12s 导入失败: %r" % (mod, e))

    def head(path, n=16):
        try:
            h = hashlib.sha256()
            with open(path, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
            return h.hexdigest()[:n].upper()
        except Exception:
            return "?"

    for mod, p in paths.items():
        cur = os.path.abspath(str(p))
        for _ in range(12):
            if cur.endswith(".egg"):
                log("  %s  %d B  %s" % (cur, os.path.getsize(cur), head(cur)))
                break
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent

    print("")
    print("=" * 72)
    print("2) 关键接口实测")
    print("=" * 72)
    try:
        client, _ = connect(args.host, args.port)
        node = register(client, args.node)
    except SystemExit:
        raise
    except Exception as e:
        log("连不上仿真器: %r" % (e,))
        return 1
    for name, fn in (
        ("get_kinetic_data()", lambda: node.get_kinetic_data()),
        ("get_kinetic_data_simply()", lambda: node.get_kinetic_data_simply()),
        ("launch_kinetic(0,1,1,ts)", lambda: node.launch_kinetic(0, 1, 1, now_ms())),
        ("control_kinetic_simply_global(x,y,250,5,ts)",
         lambda: node.control_kinetic_simply_global(-1875.0, -521.9, 250.0, 5.0,
                                                    now_ms())),
    ):
        try:
            log("%-42s -> %r" % (name, fn()))
        except Exception as e:
            log("%-42s -> 异常 %r" % (name, e))
    return 0


# ---------------------------------------------------------------------------
#  takeoff
# ---------------------------------------------------------------------------
def cmd_takeoff(args):
    client, _ = connect(args.host, args.port)
    log("注册 UAV1（node_no=%d）..." % args.node)
    node = register(client, args.node)

    def pos():
        v = node.get_location()
        return float(v[0]), float(v[1]), float(v[2])

    x0, y0, z0 = pos()
    log("初始位置 (%.1f, %.1f, %.1f)" % (x0, y0, z0))
    log("静置 %.0f 秒..." % args.baseline)
    time.sleep(args.baseline)
    x1, y1, z1 = pos()
    log("静置后   (%.1f, %.1f, %.1f)  位移 %.2f m"
        % (x1, y1, z1, math.dist((x0, y0, z0), (x1, y1, z1))))

    log("命令爬升到 z=%.0f（原地）速度 %.1f，观察 %.0f 秒 ..."
        % (args.z, args.speed, args.seconds))
    hist = []
    t_end = time.time() + args.seconds
    try:
        while time.time() < t_end:
            cx, cy, cz = pos()
            node.control_kinetic_simply_global(cx, cy, args.z, args.speed, now_ms())
            hist.append(cz)
            print("\r    t=%5.1f s   z=%8.2f m" % (time.time() - (t_end - args.seconds), cz),
                  end="")
            sys.stdout.flush()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("")
    print("")

    if not hist:
        log("没采到数据")
        return 1
    dz = max(hist) - min(hist)
    log("高度：起始 %.1f → 结束 %.1f，变化幅度 %.1f m" % (hist[0], hist[-1], dz))
    if dz >= 5.0:
        log("结论：飞控**正常工作** —— 可以用 --uav-mode kinetic")
    else:
        log("结论：飞控**没有生效**（命令返回成功但不动）")
        log("      → 继续用 --uav-mode virtual；把这份输出发给组员核对交付文件")
    return 0


# ---------------------------------------------------------------------------
#  kinetic
# ---------------------------------------------------------------------------
def cmd_kinetic(args):
    client, _ = connect(args.host, args.port)
    node = register(client, args.node)

    def pos():
        v = node.get_location()
        return float(v[0]), float(v[1]), float(v[2])

    log("初始位置 (%.1f, %.1f, %.1f)" % pos())
    try:
        log("基线 get_kinetic_data() -> %r" % (node.get_kinetic_data(),))
    except Exception as e:
        log("基线读取异常 %r" % (e,))

    found = []

    def trial(label, fn, seconds):
        z_before = pos()[2]
        first = None
        t_end = time.time() + seconds
        while time.time() < t_end:
            x, y, z = pos()
            try:
                r = fn(x, y)
                if first is None:
                    first = r
            except Exception as e:
                log("  %-44s 异常 %r" % (label, e))
                return
            time.sleep(0.2)
        dz = pos()[2] - z_before
        try:
            full = node.get_kinetic_data()
        except Exception:
            full = "?"
        log("  %-44s Δz=%+7.2f m  数据=%r" % (label, dz, full))
        if abs(dz) > 1.0 or full is not None:
            found.append((label, dz, full))

    # 1) 几种控制调用形式
    log("")
    log("控制指令形式：")
    trial("control_kinetic_simply_global(x,y,z,v,ts)",
          lambda x, y: node.control_kinetic_simply_global(x, y, args.z, 5.0, now_ms()),
          args.trials)
    if hasattr(node, "control_kinetic_simply"):
        trial("control_kinetic_simply(x,y,z,v,ts)",
              lambda x, y: node.control_kinetic_simply(x, y, args.z, 5.0, now_ms()),
              args.trials)

    # 2) launch_kinetic 枚举
    launch = getattr(node, "launch_kinetic", None)
    if launch is not None:
        log("")
        log("launch_kinetic 枚举（sim_type 0~5 × count 1/3/6）：")
        for sim_type in range(6):
            for count in (1, 3, 6):
                try:
                    ret = launch(sim_type, count, 1, now_ms())
                except Exception as e:
                    log("  (%d,%d) 异常 %r" % (sim_type, count, e))
                    continue
                time.sleep(0.2)
                try:
                    full = node.get_kinetic_data()
                except Exception:
                    full = "?"
                z_before = pos()[2]
                t_end = time.time() + args.trials
                while time.time() < t_end:
                    x, y, z = pos()
                    node.control_kinetic_simply_global(x, y, args.z, 5.0, now_ms())
                    time.sleep(0.2)
                dz = pos()[2] - z_before
                log("  (%d,%d) 返回 %r  Δz=%+.2f m  数据=%r"
                    % (sim_type, count, ret, dz, full))
                if full is not None or abs(dz) > 1.0:
                    found.append(("launch(%d,%d)" % (sim_type, count), dz, full))
                try:
                    launch(sim_type, count, 0, now_ms())
                except Exception:
                    pass

    log("")
    log("=" * 68)
    if found:
        log("★ 有进展，把下面几行发我：")
        for label, dz, full in found:
            log("    %s  Δz=%+.2f  data=%r" % (label, dz, full))
    else:
        log("结论：控制指令与 launch_kinetic 都没有效果，get_kinetic_data() 恒为 None。")
        log("      这套 SDK 的飞控通道不可用，请继续使用 --uav-mode virtual")
    log("=" * 68)
    return 0


# ---------------------------------------------------------------------------
#  z
# ---------------------------------------------------------------------------
UAV_HINT_RADIUS = 20000      # detectRadius 明显更大的那组就是无人机


def cmd_z(args):
    path = os.path.join(args.scene, "vehicles.json")
    bak = path + ".bak"
    if not os.path.isfile(path):
        raise SystemExit("找不到 %s" % path)
    if args.restore:
        if not os.path.isfile(bak):
            raise SystemExit("没有备份 %s" % bak)
        import shutil
        shutil.copyfile(bak, path)
        log("已从备份还原 %s" % bak)
        return 0

    with io.open(path, encoding="utf-8") as f:
        data = json.load(f)

    def uav_items():
        for gi, grp in enumerate(data):
            for vi, v in enumerate(grp.get("vehicles", [])):
                r = v.get("detectRadius")
                if isinstance(r, (int, float)) and r >= UAV_HINT_RADIUS:
                    yield gi, vi, v

    items = list(uav_items())
    if not items:
        raise SystemExit("没识别出无人机组（detectRadius>=%d）" % UAV_HINT_RADIUS)

    log("当前无人机初始位置：")
    for gi, vi, v in items:
        print("    [组%d 条目%d] %s" % (gi, vi, v.get("position")))

    if args.show or args.set is None:
        return 0

    import shutil
    if not os.path.isfile(bak):
        shutil.copyfile(path, bak)
        log("已备份 %s" % bak)

    changed = 0
    for gi, vi, v in items:
        parts = [p.strip() for p in str(v.get("position")).split(",")]
        if len(parts) < 3:
            continue
        if abs(float(parts[2]) - args.set) < 1e-6:
            continue
        parts[2] = "%g" % args.set
        data[gi]["vehicles"][vi]["position"] = ", ".join(parts[:3])
        changed += 1

    if not changed:
        log("高度已经是 %.1f m，无需修改" % args.set)
        return 0
    with io.open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    log("已把 %d 架无人机的 z 改成 %g m" % (changed, args.set))
    log("!! 必须重启 subject4.exe 才生效")
    return 0


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="无人机链路一体诊断")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--node", type=int, default=4)
    sub = ap.add_subparsers(dest="cmd")

    sub.add_parser("info", help="打印 SDK 路径/哈希与接口返回值")

    p = sub.add_parser("takeoff", help="单机爬升测试（30 秒判飞控）")
    p.add_argument("--z", type=float, default=250.0)
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--baseline", type=float, default=2.0)

    p = sub.add_parser("kinetic", help="控制指令 + launch_kinetic 枚举")
    p.add_argument("--z", type=float, default=250.0)
    p.add_argument("--trials", type=float, default=3.0)

    p = sub.add_parser("z", help="查看/修改无人机初始高度")
    p.add_argument("--scene", default=DEFAULT_SCENE)
    p.add_argument("--show", action="store_true")
    p.add_argument("--set", type=float, default=None)
    p.add_argument("--restore", action="store_true")

    args = ap.parse_args()
    if not args.cmd:
        args.cmd = "info"
    fn = {"info": cmd_info, "takeoff": cmd_takeoff,
          "kinetic": cmd_kinetic, "z": cmd_z}[args.cmd]
    return fn(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n中断")
        sys.exit(130)
