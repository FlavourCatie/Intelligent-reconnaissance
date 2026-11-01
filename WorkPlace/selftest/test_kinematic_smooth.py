# -*- coding: utf-8 -*-
"""数值验证运动学播放的平滑性（不靠肉眼）。

检查三件事：
  1. 位置连续 —— 每帧位移应 ≈ 速度 × dt，绝不出现瞬移
  2. 偏航连续 —— 每帧转角逐不超过 YAW_RATE × dt（+少量容差），不会突然掉头
  3. 帧率抖动时 —— dt 忽大忽小时位移随之缩放，但单帧位移有上限（不抽搐）
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HERE, "fakesdk"))
os.environ["FAKE_PROJECT_ROOT"] = ROOT

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.pause = lambda *a, **k: None

sys.path.insert(0, os.path.join(ROOT, "WorkPlace"))
sys.path.insert(0, os.path.join(ROOT, "WorkPlace", "missions"))
sys.path.insert(0, os.path.join(ROOT, "Materials", "practice", "easy_follow"))

import control_uav as cu
import uav_runner

# ---- 造一个和真实航线形状类似的折线（含直角与斜角）----
ROUTE = [(-1775.0, -517.0), (-1400.0, -420.0), (-900.0, -420.0),
         (-200.0, -700.0), (600.0, -300.0), (1400.0, -860.0),
         (1900.0, -300.0), (1900.0, -60.0)]


class FakeVehicle(object):
    def __init__(self):
        self.actor = None

    def position(self):
        return (0.0, 0.0, 250.0)


class FakeActor(object):
    def __init__(self):
        self.log = []

    def set_transform(self, tf):
        self.log.append((tf.location.x, tf.location.y, tf.rotation.yaw))


class FakeCarla(object):
    class Location(object):
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class Rotation(object):
        def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
            self.pitch, self.yaw, self.roll = pitch, yaw, roll

    class Transform(object):
        def __init__(self, location, rotation):
            self.location = location
            self.rotation = rotation


def make_runner():
    routes = dict((n, list(ROUTE)) for n in cu.UAV_NAMES)
    vehicles = dict((n, FakeVehicle()) for n in cu.UAV_NAMES)
    actors = dict((n, FakeActor()) for n in cu.UAV_NAMES)
    for n in cu.UAV_NAMES:
        vehicles[n].actor = actors[n]

    class Log(object):
        def warning(self, *a, **k): pass
        def info(self, *a, **k): pass
        def exception(self, *a, **k): pass

    r = uav_runner.KinematicUavRunner(vehicles, routes, Log(), speed=15.0,
                                      dt=0.05, altitude=250.0)
    r._carla = FakeCarla
    return r, actors


def run(steps, dt_seq):
    """用给定 dt 序列跑，返回 (最大位移, 最大转角, 末位置, 轨迹点)。"""
    clock = [1000.0]

    def fake_clock():
        return clock[0]

    routes = dict((n, list(ROUTE)) for n in cu.UAV_NAMES)
    vehicles = dict((n, FakeVehicle()) for n in cu.UAV_NAMES)
    actors = {}
    for n in cu.UAV_NAMES:
        a = FakeActor()
        vehicles[n].actor = a
        actors[n] = a

    class Log(object):
        def warning(self, *a, **k): pass
        def info(self, *a, **k): pass
        def exception(self, *a, **k): pass

    r = uav_runner.KinematicUavRunner(vehicles, routes, Log(), speed=15.0,
                                      dt=0.05, altitude=250.0, clock=fake_clock)
    r._carla = FakeCarla
    r.start()

    max_step = 0.0
    max_turn = 0.0
    prev = None
    for i in range(steps):
        clock[0] += dt_seq[i % len(dt_seq)]
        r.step()
        a = actors["UAV1"].log[-1]
        if prev is not None:
            d = math.hypot(a[0] - prev[0], a[1] - prev[1])
            max_step = max(max_step, d)
            dyaw = abs((a[2] - prev[2] + 180.0) % 360.0 - 180.0)
            max_turn = max(max_turn, dyaw)
        prev = a
    return max_step, max_turn, prev, r


print("=" * 72)
print("1) 稳定帧率（dt=0.05 s，1500 帧）")
print("=" * 72)
step_m, turn_m, last, r = run(1500, [0.05])
print("   单帧最大位移 = %.4f m   （期望 ≈ 速度×dt = %.3f）" % (step_m, 15.0 * 0.05))
print("   单帧最大转角 = %.3f deg（上限 %.1f）" % (turn_m, r.YAW_RATE * 0.05))
print("   已飞 %.1f / %.1f m" % (r.dist["UAV1"], r.total["UAV1"]))
assert step_m <= 15.0 * 0.05 + 0.02, "出现了瞬移"
assert turn_m <= r.YAW_RATE * 0.05 + 0.01, "偏航角突变"

print()
print("=" * 72)
print("2) 帧率抖动（dt 在 0.02~0.30 s 之间乱跳，模拟小地图卡顿）")
print("=" * 72)
jitter = [0.02, 0.30, 0.05, 0.12, 0.03, 0.25, 0.05, 0.08]
step_j, turn_j, last_j, rj = run(1500, jitter)
cap = 15.0 * rj.MAX_DT
print("   单帧最大位移 = %.4f m   （上限 速度×MAX_DT = %.3f）" % (step_j, cap))
print("   单帧最大转角 = %.3f deg（上限 %.1f）" % (turn_j, rj.YAW_RATE * rj.MAX_DT))
assert step_j <= cap + 0.02, "卡顿时发生瞬移"
assert turn_j <= rj.YAW_RATE * rj.MAX_DT + 0.01, "卡顿时偏航角突变"
print("   → 卡顿被钳制在 %.2f m/帧以内，不会突然跳一段" % cap)

print()
print("=" * 72)
print("3) 转角圆弧是否生效（对比原始折线）")
print("=" * 72)
raw = sum(cu.distance(a, b) for a, b in zip(ROUTE, ROUTE[1:]))
print("   原始折线长度 = %.1f m" % raw)
print("   平滑后长度   = %.1f m（圆弧抄近路，略短属正常）" % r.total["UAV1"])
print("   路径点数     = %d（等距重采样，间距 %.1f m）"
      % (len(r.path["UAV1"]), r.SAMPLE_SPACING))
assert r.total["UAV1"] > raw * 0.9, "平滑后路径短了太多，可能抄到不该走的地方"

print()
print("=" * 72)
print("全部通过：位移连续、偏航连续、卡顿有上限")
print("=" * 72)
