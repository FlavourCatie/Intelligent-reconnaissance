# -*- coding: utf-8 -*-
"""
preflight.py —— 启动前置检查（飞控驱动是否在跑、启动顺序对不对）
================================================================================

为什么需要这个
--------------
四旋翼的运动由 `simu_driver_windows` 里 `start.bat` 启动的**外部飞控驱动**
（`main-release.exe` + `driver.exe`）计算后回写 CARLA。

驱动没跑、或者**比 CarlaUE4 先启动**（此时世界还不存在，绑不上实例）时，
`control_kinetic_simply_global()` 会**返回成功但完全不动**，且没有任何报错。
无人车不受影响（走 apply_vehicle_control 直接控制）。

课程文档对顺序的说法不一致：
    Materials/readme.txt             : start.bat → CarlaUE4 → subject4.exe
    environment_forclass/environment/kyxz.md : 仿真器 → start.bat → subject4.exe   ← D4 说明

2026-10-28 实测：按前者启动（驱动比 CarlaUE4 早 32 秒），三架 M10 高度恒为 100.0 m，
82 秒 1650 帧纹丝不动。因此**按 kyxz.md 的顺序**做检查提示。

本模块只做**只读检查**，不改任何状态；任何一步查不到就安静跳过。
"""

from __future__ import print_function

import os
import subprocess

DRIVERS = ("main-release.exe", "driver.exe")
SIM = "CarlaUE4.exe"
SCENE = "subject4.exe"

ORDER_HINT = (
    "正确顺序（依据 environment_forclass/environment/kyxz.md 对 D4 组别的说明）：\n"
    "    1) CarlaUE4\n"
    "    2) Materials\\simu_driver_windows\\start.bat  → 选 2（旋翼）\n"
    "    3) Materials\\practice\\practice1_Scene\\subject4.exe\n"
    "    4) 再跑算法脚本\n"
    "  start.bat 要一直开着；它若比 CarlaUE4 先启动，世界还不存在，飞控绑不上飞机。"
)


# ---------------------------------------------------------------------------
#  进程枚举（纯 Win32 Toolhelp 快照，不经子进程）
# ---------------------------------------------------------------------------
TH32CS_SNAPPROCESS = 0x00000002
MAX_PATH = 260


def _process_table():
    """返回 {进程名小写: [pid, ...]}。

    **失败返回 None**（区别于"查到了但没这个进程"）。
    不用 `tasklist`：某些环境/权限下子进程会被拒，或者没有 tasklist，
    那会被误判成"驱动没跑"——这种误报比不检查更糟。
    """
    if os.name != "nt":
        return None
    try:
        import ctypes
        from ctypes import wintypes

        class PROCESSENTRY32(ctypes.Structure):
            _fields_ = [
                ("dwSize", wintypes.DWORD),
                ("cntUsage", wintypes.DWORD),
                ("th32ProcessID", wintypes.DWORD),
                ("th32DefaultHeapID", ctypes.c_void_p),
                ("th32ModuleID", wintypes.DWORD),
                ("cntThreads", wintypes.DWORD),
                ("th32ParentProcessID", wintypes.DWORD),
                ("pcPriClassBase", ctypes.c_long),
                ("dwFlags", wintypes.DWORD),
                ("szExeFile", ctypes.c_char * MAX_PATH),
            ]

        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
        snapshot = k32.CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0)
        if not snapshot or snapshot == wintypes.HANDLE(-1).value:
            return None
        table = {}
        try:
            entry = PROCESSENTRY32()
            entry.dwSize = ctypes.sizeof(PROCESSENTRY32)
            ok = k32.Process32First(snapshot, ctypes.byref(entry))
            while ok:
                name = entry.szExeFile.decode("mbcs", "replace").lower()
                table.setdefault(name, []).append(int(entry.th32ProcessID))
                ok = k32.Process32Next(snapshot, ctypes.byref(entry))
        finally:
            k32.CloseHandle(snapshot)
        return table
    except Exception:
        return None


def _filetime_to_epoch(ft):
    """Windows FILETIME(100ns since 1601) -> unix 秒。"""
    return ft / 1e7 - 11644473600.0


def _proc_start(pid):
    """取进程启动时间（epoch 秒）；失败返回 None。"""
    if os.name != "nt":
        return None
    try:
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
        handle = k32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, int(pid))
        if not handle:
            return None
        try:
            created = wintypes.FILETIME()
            exited = wintypes.FILETIME()
            kernel = wintypes.FILETIME()
            user = wintypes.FILETIME()
            ok = k32.GetProcessTimes(handle, ctypes.byref(created),
                                     ctypes.byref(exited),
                                     ctypes.byref(kernel), ctypes.byref(user))
            if not ok:
                return None
            ft = (created.dwHighDateTime << 32) | created.dwLowDateTime
            return _filetime_to_epoch(ft)
        finally:
            k32.CloseHandle(handle)
    except Exception:
        return None


# ---------------------------------------------------------------------------
#  对外：一次检查
# ---------------------------------------------------------------------------
def check():
    """返回 dict：
        available  检查本身是否成功（False 时调用方应当什么都不提示）
        running    {名字: bool}
        starts     {名字: epoch 或 None}
        order_ok   True/False/None
        problems   [提示文字, ...]
    """
    table = _process_table()
    result = {"available": table is not None, "running": {}, "starts": {},
              "order_ok": None, "problems": []}
    if table is None:
        return result

    for exe in (SIM,) + DRIVERS + (SCENE,):
        pids = table.get(exe.lower(), [])
        result["running"][exe] = bool(pids)
        result["starts"][exe] = _proc_start(pids[0]) if pids else None

    # 驱动是否在跑
    missing = [e for e in DRIVERS if not result["running"][e]]
    if missing:
        result["problems"].append(
            "飞控驱动未运行：%s —— 无人机不会动。" % ", ".join(missing))

    # 启动顺序：驱动应晚于 CarlaUE4
    sim_t = result["starts"].get(SIM)
    drv_t = [result["starts"][e] for e in DRIVERS if result["starts"][e]]
    if sim_t is not None and drv_t:
        if min(drv_t) < sim_t:
            result["order_ok"] = False
            result["problems"].append(
                "启动顺序疑似反了：飞控驱动比 CarlaUE4 早 %.0f 秒启动。"
                % (sim_t - min(drv_t)))
        else:
            result["order_ok"] = True
    return result


def report(res):
    """把 check() 的结果格式化成若干行文字。"""
    lines = []
    for exe in (SIM, "main-release.exe", "driver.exe", SCENE):
        state = "运行中" if res["running"].get(exe) else "未运行"
        lines.append("  %-16s %s" % (exe, state))
    if res["order_ok"] is False:
        lines.append("  !! 启动顺序：飞控早于 CarlaUE4（这是错的）")
    elif res["order_ok"] is True:
        lines.append("  启动顺序：CarlaUE4 先于飞控驱动（正确）")
    for p in res["problems"]:
        lines.append("  !! " + p)
    return lines


if __name__ == "__main__":
    r = check()
    print("前置检查：")
    for line in report(r):
        print(line)
    if r["problems"]:
        print()
        print(ORDER_HINT)
    else:
        print("\n未发现问题。")
