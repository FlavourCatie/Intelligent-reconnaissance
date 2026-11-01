# -*- coding: utf-8 -*-
"""
reset_scene.py —— 重置侦察流程（杀场景进程 / 还原配置 / 重新拉起）
====================================================================

演示时最常见的需求：**把这一遍跑完的场景恢复成初始状态，再来一遍。**

SDK 没有"重置"接口，所以重置 = 三步：
    1. 杀掉正在跑的 subject4.exe（场景进程）
    2. 把 targets.json / vehicles.json 还原成基准备份
    3. 重新启动 subject4.exe

基准备份怎么来
--------------
文件名为 `<原名>.orig`，由本脚本的 `--snapshot` 生成（**建议在做任何修改前先跑一次**）。
`tools/spawn_targets.py` 写 targets.json 前也会自动建这个备份（只建一次，不覆盖）。

用法
----
    # 0) 【重要】第一次使用前，先把当前场景存成基准
    python WorkPlace\\tools\\reset_scene.py --snapshot

    # 1) 完整重置：杀进程 -> 还原敌人和车 -> 重启场景
    python WorkPlace\\tools\\reset_scene.py

    # 2) 用 spawn_targets 生成的新敌人，只想重启场景生效（不还原敌人）
    python WorkPlace\\tools\\reset_scene.py --keep-enemies

    # 3) 只还原文件，不重启（例如还没开 CarlaUE4）
    python WorkPlace\\tools\\reset_scene.py --no-relaunch

    # 4) 看看现在有什么、备份在不在
    python WorkPlace\\tools\\reset_scene.py --list

注意
----
* 本脚本**不会**重启 CarlaUE4 和 start.bat（飞控驱动），只动场景进程。
  要连 Carla 一起重启加 `--kill-carla`。
* 如果场景进程是手动双击启动的，杀完之后由本脚本重新拉起，会继承当前控制台的
  输入输出；想让它独立窗口运行可以加 `--new-window`。
"""

from __future__ import print_function

import argparse
import os
import shutil
import subprocess
import sys
import time

WORKPLACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(WORKPLACE)
sys.path.insert(0, WORKPLACE)

DEFAULT_SCENE = os.path.join(PROJECT, "Materials", "practice", "practice1_Scene")
MANAGED = ("targets.json", "vehicles.json", "elements.json", "game.json")

IS_WIN = (os.name == "nt")


# ---------------------------------------------------------------------------
def sh(cmd):
    """跑一条系统命令，返回 (returncode, 输出)。不抛异常。"""
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             shell=False)
        out, _ = p.communicate()
        return p.returncode, out.decode("gbk", "replace") if out else ""
    except Exception as e:
        return -1, repr(e)


def proc_alive(exe_name):
    """进程是否还在（Windows 用 tasklist，其它平台用 pgrep）。"""
    if IS_WIN:
        _, out = sh(["tasklist", "/FI", "IMAGENAME eq %s" % exe_name, "/NH"])
        return exe_name.lower() in out.lower()
    rc, _ = sh(["pgrep", "-f", exe_name])
    return rc == 0


def kill_proc(exe_name):
    if IS_WIN:
        rc, out = sh(["taskkill", "/F", "/IM", exe_name])
        return rc == 0, out.strip()
    rc, out = sh(["pkill", "-f", exe_name])
    return rc == 0, out.strip()


def find_exe(scene_dir, prefer=None):
    """在场景目录里找场景进程的 exe。"""
    cands = [f for f in os.listdir(scene_dir)
             if f.lower().endswith(".exe")]
    if prefer:
        if prefer in cands:
            return prefer
        for c in cands:
            if prefer.lower() in c.lower():
                return c
    if "subject4.exe" in [c.lower() for c in cands]:
        return "subject4.exe"
    return cands[0] if cands else None


# ---------------------------------------------------------------------------
def cmd_list(scene_dir, exe):
    print("场景目录: %s" % scene_dir)
    print("场景进程: %s  运行中: %s"
          % (exe or "(未找到)", "是" if (exe and proc_alive(exe)) else "否"))
    print("\n%-16s %-10s %-10s" % ("文件", "当前", "基准备份"))
    print("-" * 44)
    for name in MANAGED:
        p = os.path.join(scene_dir, name)
        b = p + ".orig"
        cur = "有" if os.path.isfile(p) else "缺"
        bak = "有" if os.path.isfile(b) else "-"
        print("%-16s %-10s %-10s" % (name, cur, bak))
    no_bak = [n for n in MANAGED if not os.path.isfile(os.path.join(scene_dir, n + ".orig"))]
    if no_bak:
        print("\n提示：%s 还没有基准备份。先跑一次 `--snapshot` 再修改场景。"
              % ", ".join(no_bak))


def cmd_snapshot(scene_dir):
    n = 0
    for name in MANAGED:
        p = os.path.join(scene_dir, name)
        if not os.path.isfile(p):
            continue
        b = p + ".orig"
        if os.path.isfile(b):
            print("  已存在，跳过: %s" % os.path.basename(b))
            continue
        shutil.copy2(p, b)
        print("  已备份: %-16s -> %s" % (name, os.path.basename(b)))
        n += 1
    print("完成，新建 %d 个备份。" % n)


def cmd_restore(scene_dir, keep):
    n = 0
    for name in MANAGED:
        if name in keep:
            print("  保留（不还原）: %s" % name)
            continue
        p = os.path.join(scene_dir, name)
        b = p + ".orig"
        if not os.path.isfile(b):
            print("  跳过（无备份）: %s" % name)
            continue
        shutil.copy2(b, p)
        print("  已还原: %-16s <- %s" % (name, os.path.basename(b)))
        n += 1
    print("完成，还原 %d 个文件。" % n)


def main():
    ap = argparse.ArgumentParser(
        description="重置侦察流程：杀场景进程 / 还原配置 / 重新拉起",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default=DEFAULT_SCENE, help="场景目录（含 subject4.exe）")
    ap.add_argument("--exe", default=None, help="场景进程名（默认自动找 subject4.exe）")
    ap.add_argument("--keep-enemies", action="store_true", help="不还原 targets.json")
    ap.add_argument("--keep-vehicles", action="store_true", help="不还原 vehicles.json")
    ap.add_argument("--keep-all", action="store_true", help="一个文件都不还原，只重启")
    ap.add_argument("--no-relaunch", action="store_true", help="杀完不重启")
    ap.add_argument("--kill-carla", action="store_true", help="连 CarlaUE4 一起杀")
    ap.add_argument("--new-window", action="store_true", help="用独立窗口启动场景进程")
    ap.add_argument("--wait", type=float, default=6.0, help="重启后等待秒数（默认6）")
    ap.add_argument("--snapshot", action="store_true", help="只把当前场景存成基准备份")
    ap.add_argument("--list", action="store_true", help="只看状态，不做任何改动")
    args = ap.parse_args()

    scene_dir = os.path.abspath(args.scene)
    if not os.path.isdir(scene_dir):
        raise SystemExit("场景目录不存在: %s" % scene_dir)

    exe = find_exe(scene_dir, args.exe)

    if args.list:
        cmd_list(scene_dir, exe)
        return

    if args.snapshot:
        print("把当前场景存成基准备份 …")
        cmd_snapshot(scene_dir)
        return

    print("=" * 60)
    print("重置场景: %s" % scene_dir)
    print("=" * 60)

    # ---- 先算出"哪些文件要保留"（杀进程失败时也要用到）----
    keep = set()
    if args.keep_all:
        keep = set(MANAGED)
    if args.keep_enemies:
        keep.add("targets.json")
    if args.keep_vehicles:
        keep.add("vehicles.json")

    # ---- 1. 杀进程 ----
    if exe and proc_alive(exe):
        print("[1/3] 结束场景进程 %s …" % exe)
        ok, out = kill_proc(exe)
        time.sleep(0.6)
        if ok and not proc_alive(exe):
            print("      已结束")
        else:
            # 关键：杀不掉就**绝对不能**再拉起一个。
            # 实测后果：两个 subject4 抢同一个 CARLA 世界，新实例报
            # "INTERNAL ERROR: cannot create temporary directory!" 直接挂，
            # 之后 SDK 一路 "streaming client: connection failed: 拒绝连接"。
            print("")
            print("!" * 66)
            print("!! 无法结束 %s —— 拒绝访问" % exe)
            print("!! 常见原因：场景进程是以【管理员权限】启动的，当前终端权限不足。")
            print("!! 原始输出：%s" % out[:200].replace("\n", " "))
            print("")
            print("!! 处理办法（任选其一）：")
            print("!!   1) 用【管理员身份】重开 PowerShell 再跑")
            print("!!   2) 手动关掉 subject4.exe 窗口后重跑")
            print("!!")
            print("!! 已跳过重启 —— 否则会多开一个实例把仿真器弄坏。")
            print("!" * 66)
            print("")
            print("[2/3] 仍按参数处理文件 …")
            cmd_restore(scene_dir, keep)
            print("[3/3] 跳过重启")
            return 2
    else:
        print("[1/3] 场景进程未在运行（%s）" % (exe or "未找到 exe"))

    if args.kill_carla:
        print("      顺带结束 CarlaUE4 …")
        kill_proc("CarlaUE4.exe")
        kill_proc("CarlaUE4-Win64-Shipping.exe")

    # ---- 2. 还原文件 ----
    print("[2/3] 还原配置文件 …")
    cmd_restore(scene_dir, keep)

    # ---- 3. 重启 ----
    if args.no_relaunch:
        print("[3/3] --no-relaunch：不重启")
        return
    if not exe:
        print("[3/3] 场景目录里没找到 .exe，无法重启")
        return

    exe_path = os.path.join(scene_dir, exe)
    # 关键：taskkill /F 是硬杀，CARLA 那边销毁残留 actor 是异步的。
    # 实测 2 s 不够——旧的车还没清掉，新实例再生成 vehicle.* 就撞上
    # UE4 断言 "VehicleAnim != nullptr"（CarlaWheeledVehicle.cpp:557），
    # UE4 直接崩，subject4 的 init_vehicles 随之拿到 None 退出。
    # 这里给 8 s，让 CARLA 把上一个客户端的 actor 清干净。
    print("[3/3] 等待 8.0 s 让 CARLA 清理旧实例残留的 actor …")
    time.sleep(8.0)
    print("      启动场景进程: %s" % exe_path)
    try:
        if IS_WIN and args.new_window:
            subprocess.Popen([exe_path], cwd=scene_dir,
                             creationflags=subprocess.CREATE_NEW_CONSOLE)
        else:
            subprocess.Popen([exe_path], cwd=scene_dir)
    except Exception as e:
        print("      启动失败: %r" % (e,))
        return 2

    print("      等待 %.1f s …" % args.wait)
    time.sleep(args.wait)

    alive = proc_alive(exe)
    print("      运行中: %s" % ("是" if alive else "否（启动失败）"))
    if not alive:
        print("")
        print("!" * 66)
        print("!! 场景进程启动后立刻退出了")
        print("!! 实测最常见原因：UE4 断言 VehicleAnim != nullptr")
        print("!!   （CarlaWheeledVehicle.cpp:557）——上一个实例强行结束时，")
        print("!!   它生成的车 actor 还没清干净，新实例再生成车辆就撞上")
        print("!!   \"动画实例为空\"，UE4 崩溃，subject4 的 init_vehicles 拿到")
        print("!!   None 后报 add_to_team 失败并退出。")
        print("!!")
        print("!! 单独重启 subject4 这条路不可靠。建议整套重起：")
        print("!!   全部关掉 → CarlaUE4 → start.bat(选2 旋翼) → subject4.exe")
        print("!" * 66)
        return 2

    print("\n完成。接着跑：")
    print("    python WorkPlace\\missions\\full_mission.py --uav-mode kinematic")


if __name__ == "__main__":
    main()
