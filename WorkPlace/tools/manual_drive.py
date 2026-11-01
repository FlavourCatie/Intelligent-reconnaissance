# -*- coding: utf-8 -*-
"""
manual_drive.py —— 无人车手动驾驶测试工具（多车切换，方向键操作）
================================================================================

用途
----
联调前的"手感测试"：不跑任何算法，纯粹用键盘开车，确认
    ① 车能正确响应油门/转向；② 多台车切换不会串控；③ 换车后旧车能停住。

参考了 `environment_forclass/environment/scripts_test/manual_control.py` 的思路
（方向键开车 + 数字键切车），但做了三点改动：

    · **不用 pygame**：原脚本要开一个 200×200 的假窗口才能收键事件；
      这里用 Windows 自带的 `msvcrt` 直接读控制台按键，零依赖、无窗口。
    · **只注册无人车**（默认 3 台），不碰无人机。
    · **切车自动收油**：原脚本换车时旧车会保持上一帧的油门继续跑，
      这里切走时会把旧车显式归零。

控制方式
--------
    ↑ / ↓      油门 +/－（松开后缓慢回落）
    ← / →      左转 / 右转（松开后自动回正）
    空格        刹车到停（按住即刹）
    x          手刹开关
    1 / 2 / 3   切换到第 N 台车
    Tab / n     切到下一台
    p           打印所有车的当前位置
    h           显示/隐藏帮助
    q / Esc     退出（所有车停车 + 手刹）

用法
----
    # 前置：CarlaUE4 + start.bat + subject4.exe 都已启动
    python WorkPlace\\tools\\manual_drive.py

    # 先直接开 2 号车
    python WorkPlace\\tools\\manual_drive.py --start 2

    # 如果场景里不止 3 台车
    python WorkPlace\\tools\\manual_drive.py --count 3

    # 没有仿真器时，只检查按键解析和油门积分逻辑（离线自测）
    python WorkPlace\\tools\\manual_drive.py --selftest

注意
----
* 必须用 **Python 3.6 + swarmae** 环境（要连 SDK）；
  `--selftest` 不需要仿真器，但仍需能导入本文件所在包的依赖。
* 车号（node_no）必须是 1/2/3……，和 `vehicles.json` 里的顺序一致：
  前三台是 `vehicle.imv.rb`（无人车），后面才是无人机。
"""

from __future__ import print_function

import argparse
import math
import os
import sys
import time
import traceback

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
PROJECT_DIR = os.path.dirname(WORKPLACE_DIR)
EASY_FOLLOW_DIR = os.path.join(PROJECT_DIR, "Materials", "practice", "easy_follow")

for _p in (EASY_FOLLOW_DIR, WORKPLACE_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

# NumPy 2.x 兼容（lib/*.py 的类型注解里用了 np.mat）
if not hasattr(np, "mat"):
    np.mat = np.asmatrix


# ===========================================================================
#  按键解析（Windows 控制台，无窗口）
# ===========================================================================
#: 方向键是两字节序列：先 '\xe0' 或 '\x00'，再跟一个字母
_ARROW = {
    "H": "up",
    "P": "down",
    "K": "left",
    "M": "right",
}

#: 普通字符键的别名（WASD 也支持）
_CHAR = {
    "\t": "next",
    " ": "brake",
    "\x1b": "quit",
    "w": "up",
    "s": "down",
    "a": "left",
    "d": "right",
    "W": "up",
    "S": "down",
    "A": "left",
    "D": "right",
    "x": "handbrake",
    "X": "handbrake",
    "1": "1",
    "2": "2",
    "3": "3",
    "4": "4",
    "5": "5",
    "p": "positions",
    "P": "positions",
    "h": "help",
    "H": "help",
    "q": "quit",
    "Q": "quit",
    "n": "next",
    "N": "next",
}


class ConsoleKeys(object):
    """把 msvcrt 的原始字节流解析成按键名列表。不依赖任何第三方库。"""

    def __init__(self):
        self.available = True
        self._carry = ""
        try:
            import msvcrt  # noqa: F401
        except Exception:
            self.available = False

    def poll(self):
        """返回这一轮按下的键名列表（可能为空）。"""
        if not self.available:
            return []
        import msvcrt
        buf = self._carry
        self._carry = ""
        while msvcrt.kbhit():
            buf += msvcrt.getwch()

        out = []
        i = 0
        while i < len(buf):
            ch = buf[i]
            if ch in ("\x00", "\xe0"):
                if i + 1 >= len(buf):
                    self._carry = ch       # 半个序列，下一轮再接
                    break
                out.append(_ARROW.get(buf[i + 1], "unknown"))
                i += 2
                continue
            name = _CHAR.get(ch)
            if name:
                out.append(name)
            i += 1
        return out


# ===========================================================================
#  驾驶状态机（与按键/网络解耦，便于离线自测）
# ===========================================================================
class DriveState(object):
    """把"这一帧按了哪些键"积分成油门/转向/刹车。

    松开后的回落是有意为之：
        · 油门松开 -> 缓慢回落到 0（滑行减速，而不是瞬间断油）
        · 转向松开 -> 自动回正
    """

    def __init__(self, throttle_step=0.08, steer_step=0.08,
                 throttle_decay=0.90, steer_decay=0.85,
                 max_throttle=1.0, max_steer=0.8):
        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0
        self.hand_brake = False
        self.throttle_step = throttle_step
        self.steer_step = steer_step
        self.throttle_decay = throttle_decay
        self.steer_decay = steer_decay
        self.max_throttle = max_throttle
        self.max_steer = max_steer

    def reset(self):
        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0
        self.hand_brake = False

    def update(self, keys):
        keys = set(keys)

        # ---- 油门 ----
        if "up" in keys:
            self.throttle = min(self.max_throttle, self.throttle + self.throttle_step)
        elif "down" in keys:
            self.throttle = max(-self.max_throttle, self.throttle - self.throttle_step)
        else:
            self.throttle *= self.throttle_decay
            if abs(self.throttle) < 0.01:
                self.throttle = 0.0

        # ---- 转向 ----
        if "left" in keys:
            self.steer = max(-self.max_steer, self.steer - self.steer_step)
        elif "right" in keys:
            self.steer = min(self.max_steer, self.steer + self.steer_step)
        else:
            self.steer *= self.steer_decay
            if abs(self.steer) < 0.01:
                self.steer = 0.0

        # ---- 刹车 / 手刹 ----
        if "brake" in keys:
            self.brake = 1.0
            self.throttle = 0.0
        else:
            self.brake = 0.0
        if "handbrake" in keys:
            self.hand_brake = not self.hand_brake

        return self.command()

    def command(self):
        return dict(steer=self.steer, throttle=self.throttle,
                    brake=self.brake, hand_brake=self.hand_brake)


# ===========================================================================
#  自测（不需要仿真器）
# ===========================================================================
def _install_fake_msvcrt(chars):
    """把假 msvcrt 塞进 sys.modules，用来离线测试 ConsoleKeys 的真实解析逻辑。"""
    import types
    mod = types.ModuleType("msvcrt")
    queue = list(chars)

    def kbhit():
        return bool(queue)

    def getwch():
        return queue.pop(0) if queue else ""

    mod.kbhit = kbhit
    mod.getwch = getwch
    sys.modules["msvcrt"] = mod
    return mod


def selftest():
    print("=" * 68)
    print("离线自测：按键解析 + 油门/转向积分")
    print("=" * 68)

    real_msvcrt = sys.modules.get("msvcrt", "ABSENT")

    print("\n[1] ConsoleKeys 按键解析")
    cases = [
        ("\xe0H", ["up"]),
        ("\xe0P", ["down"]),
        ("\xe0K", ["left"]),
        ("\xe0M", ["right"]),
        ("\t", ["next"]),
        ("123", ["1", "2", "3"]),
        (" ", ["brake"]),
        ("w", ["up"]),
        ("WASD", ["up", "left", "down", "right"]),
        ("\xe0H\xe0M", ["up", "right"]),
        ("z", []),
        ("\xe0H1", ["up", "1"]),
    ]
    ok = True
    for raw, want in cases:
        _install_fake_msvcrt(raw)
        keys = ConsoleKeys()
        got = keys.poll()
        good = (got == want)
        ok = ok and good
        print("   %-14r -> %-24r %s"
              % (raw, got, "OK" if good else "FAIL 期望 %r" % (want,)))
    # 半个方向键序列要能留到下一轮（\xe0 单独到达时不能丢）
    _install_fake_msvcrt("\xe0")
    keys = ConsoleKeys()
    half = keys.poll()
    _install_fake_msvcrt("H")          # 下一轮才来后半截
    rest = keys.poll()
    got = rest
    good = (half == [] and rest == ["up"])
    ok = ok and good
    print("   %-14s -> 首轮 %r，次轮 %r  %s"
          % ("半个序列", half, rest, "OK" if good else "FAIL"))
    assert ok, "按键解析用例失败"

    # 还原真实 msvcrt
    if real_msvcrt == "ABSENT":
        sys.modules.pop("msvcrt", None)
    else:
        sys.modules["msvcrt"] = real_msvcrt

    print("\n[2] 油门：按住 ↑ 逐步加大，松开缓慢回落")
    d = DriveState()
    seq = []
    for _ in range(5):
        d.update({"up"})
        seq.append(round(d.throttle, 3))
    print("   按住 5 帧 ->", seq)
    assert seq == sorted(seq) and seq[-1] > seq[0]
    for _ in range(40):
        d.update(set())
    print("   松开 40 帧 -> %.4f（应≈0）" % d.throttle)
    assert d.throttle == 0.0

    print("\n[3] 转向：按住 → 打到上限，松开回正")
    d.reset()
    for _ in range(30):
        d.update({"right"})
    print("   按住右 30 帧 -> steer=%.2f" % d.steer)
    assert abs(d.steer - d.max_steer) < 1e-9
    for _ in range(40):
        d.update(set())
    print("   松开 40 帧 -> steer=%.4f" % d.steer)
    assert d.steer == 0.0

    print("\n[4] 刹车优先：刹车时油门清零")
    d.reset()
    d.update({"up"})
    d.update({"up"})
    print("   加速后 throttle=%.2f" % d.throttle)
    d.update({"brake"})
    print("   踩刹车后 throttle=%.2f brake=%.2f" % (d.throttle, d.brake))
    assert d.throttle == 0.0 and d.brake == 1.0

    print("\n[5] 手刹是开关")
    d.reset()
    d.update({"handbrake"})
    print("   按一次 -> hand_brake=%s" % d.hand_brake)
    assert d.hand_brake is True
    d.update({"handbrake"})
    print("   再按一次 -> hand_brake=%s" % d.hand_brake)
    assert d.hand_brake is False

    print("\n全部通过。\n")


# ===========================================================================
#  主流程
# ===========================================================================
HELP = ("↑/↓ 油门   ←/→ 转向   空格 刹车   x 手刹   "
        "1/2/3 切车   Tab 下一台   p 位置   h 帮助   q 退出")


def fmt_pos(v):
    st = v.get_state()
    return ("(%.1f, %.1f, %.1f) yaw=%6.1f° v=%5.2f"
            % (st.pose_.x_, st.pose_.y_, st.pose_.z_,
               math.degrees(st.pose_.theta_), st.v_))


def main():
    ap = argparse.ArgumentParser(
        description="无人车手动驾驶测试（多车切换，方向键操作）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--count", type=int, default=3, help="注册几台无人车（默认3）")
    ap.add_argument("--start", type=int, default=1, help="启动时控制第几台（默认1）")
    ap.add_argument("--rate", type=float, default=20.0, help="控制频率 Hz（默认20）")
    ap.add_argument("--selftest", action="store_true",
                    help="只跑离线自测（不需要仿真器）然后退出")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    from swarmae.SwarmAEClient import SwarmAEClient
    import lib.swarm_api as api

    log = lambda m: print("[manual_drive] " + m)

    log("连接 %s:%d ..." % (args.ip, args.port))
    client = SwarmAEClient(ue_ip=args.ip, ue_port=args.port)
    _, game, _ = client.get_game()

    # ---- 只注册无人车 ----
    vehicles = []
    for i in range(1, args.count + 1):
        v = api.Vehicle(client, vehicle_name="UGV%d" % i,
                        vehicle_no=i, vehicle_type="ugv")
        vehicles.append(v)
        log("已注册 UGV%d（node_no=%d）" % (i, i))
    if not vehicles:
        raise SystemExit("没有注册任何车，检查 --count")

    keys = ConsoleKeys()
    if not keys.available:
        raise SystemExit("当前环境读不到控制台按键（需要 Windows 的 msvcrt）")

    idx = max(0, min(args.start - 1, len(vehicles) - 1))
    drive = DriveState()
    dt = 1.0 / max(1.0, args.rate)
    tick = 0

    # 所有车先归零，避免场景里残留上次的控制量
    for v in vehicles:
        v.apply_control(0.0, 0.0, 0.0, False)

    log("控制第 %d 台（共 %d 台）。%s" % (idx + 1, len(vehicles), HELP))
    log("按 q 退出；切车时旧车会自动收油。")

    try:
        while True:
            t0 = time.time()
            pressed = keys.poll()
            events = set(pressed)

            # ---- 退出 ----
            if "quit" in events:
                log("退出")
                break

            # ---- 切车 ----
            new_idx = idx
            for k in pressed:
                if k in ("1", "2", "3", "4", "5"):
                    n = int(k) - 1
                    if 0 <= n < len(vehicles):
                        new_idx = n
                elif k == "next":
                    new_idx = (idx + 1) % len(vehicles)
            if new_idx != idx:
                # 先把旧车收油停住，再把新车的控制状态清零
                vehicles[idx].apply_control(0.0, 0.0, 0.5, False)
                idx = new_idx
                drive.reset()
                log("切换到 UGV%d  %s" % (idx + 1, fmt_pos(vehicles[idx])))

            # ---- 帮助 / 位置 ----
            if "help" in events:
                print()
                print("   " + HELP)
            if "positions" in events:
                print()
                for i, v in enumerate(vehicles):
                    mark = "->" if i == idx else "  "
                    print("   %s UGV%d %s" % (mark, i + 1, fmt_pos(v)))

            # ---- 积分控制量并下发 ----
            cmd = drive.update(events & {"up", "down", "left", "right",
                                         "brake", "handbrake"})
            vehicles[idx].apply_control(cmd["steer"], cmd["throttle"],
                                        cmd["brake"], cmd["hand_brake"])

            # ---- 状态行 ----
            tick += 1
            if tick % 4 == 0:      # 每 4 帧刷一次，避免刷屏太快
                st = vehicles[idx].get_state()
                line = ("\r[%d/%d] UGV%d  steer=%+.2f  thr=%+.2f  brake=%.0f  hb=%d  "
                        "v=%5.2f  pos=(%7.1f,%7.1f)  yaw=%6.1f°   "
                        % (idx + 1, len(vehicles), idx + 1,
                           cmd["steer"], cmd["throttle"], cmd["brake"],
                           1 if cmd["hand_brake"] else 0,
                           st.v_, st.pose_.x_, st.pose_.y_,
                           math.degrees(st.pose_.theta_)))
                sys.stdout.write(line)
                sys.stdout.flush()

            # ---- 保持频率 ----
            sleep = dt - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print()
        log("用户中断（Ctrl+C）")
    finally:
        print()
        # 停车：先刹，再拉手刹
        for v in vehicles:
            try:
                v.apply_control(0.0, 0.0, 1.0, False)
            except Exception:
                pass
        time.sleep(0.3)
        for v in vehicles:
            try:
                v.apply_control(0.0, 0.0, 0.0, True)
            except Exception:
                pass
        log("已对所有车下发刹车 + 手刹")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
