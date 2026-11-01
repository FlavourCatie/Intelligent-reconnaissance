# -*- coding: utf-8 -*-
"""
UGV1 从当前位置继续直行, 开到下一个路口(分岔口)并停车。

与 ugv1_straight.py 的区别: 目标路口是相对"当前车辆位置"的下一个东侧路口,
而不是相对出生点; 并在临近路口时减速, 尽量停在路口处。
"""
import math
import os
import time
import numpy as np
from PIL import Image

from swarmae.SwarmAEClient import SwarmAEClient

import lib.swarm_api as api
import lib.image_process as imp
import lib.frenet_optimal_planner as fop
import lib.controller as control
import lib.data_struct as struct
import lib.utils as utils

TARGET_V      = 10.0   # 巡航车速 (m/s)
SLOW_RANGE    = 50.0   # 距路口多少米开始减速 (m)
STOP_DIST     = 3.0    # 距路口多近算"到达" (m)
MAX_RUNTIME   = 120.0  # 最多跑多久 (s)


def find_crossings_on_row(binary, row):
    """找某一行上被南北向路穿过的列 (路口 x 坐标, 已按分组取中心)。"""
    W = binary.shape[1]
    cols = [x for x in range(1, W - 1)
            if binary[row, x] and binary[row - 3, x] and binary[row + 3, x]]
    groups = []
    if cols:
        start = prev = cols[0]
        for x in cols[1:]:
            if x - prev > 3:
                groups.append((start + prev) // 2)
                start = x
            prev = x
        groups.append((start + prev) // 2)
    return groups


def build_straight_ref(p0, p1, n=20, resolution=2.0):
    """在 p0 -> p1 之间均匀取 n 个点, 拟合参考路径 (需 >=4 点)。"""
    xs = np.linspace(p0[0], p1[0], n)
    ys = np.linspace(p0[1], p1[1], n)
    return fop.ReferencePath(np.mat(xs).T, np.mat(ys).T, resolution)


def calculate_target_pose(vehicle_pose, ref_path):
    """计算参考路径上的预瞄点。"""
    front_x = vehicle_pose.x_ + 10 * math.cos(vehicle_pose.theta_)
    front_y = vehicle_pose.y_ + 10 * math.sin(vehicle_pose.theta_)
    dx = np.ravel(ref_path.interp_x_) - front_x
    dy = np.ravel(ref_path.interp_y_) - front_y
    dist = np.abs(dx) + np.abs(dy)
    idx = int(np.argmin(dist))
    return struct.Transform(ref_path.interp_x_[idx, 0],
                            ref_path.interp_y_[idx, 0],
                            ref_path.interp_theta_[idx, 0])


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()

    game.stage_start('plan_start')
    game.stage_start('reconnaissance_start')
    game.stage_start('vau_reconnaissance_start')

    ugv = api.Vehicle(AeClient, "UGV1", 1, "ugv")

    # 路网骨架 (直接加载预生成的骨架图, 避免在线 skeletonize 内存占用)
    img, scale, world_offset, _, _ = game.get_road_network()
    skl_png = "skeletonized_image.png"
    img_skl = Image.open(skl_png) if os.path.exists(skl_png) else imp.skeleton_refinement(img)
    binary = np.all(np.array(img_skl) >= 200, axis=-1)

    pose = ugv.get_transform()
    cur_px = (int(scale * (pose.x_ - world_offset[0])),
              int(scale * (pose.y_ - world_offset[1])))
    row = cur_px[1]
    crossings = find_crossings_on_row(binary, row)
    east = [c for c in crossings if c > cur_px[0]]
    print("当前位置: 世界 (%.1f, %.1f)  像素 %s" % (pose.x_, pose.y_, cur_px))
    print("该车道东侧路口(像素x):", east)

    if not east:
        print("当前位置东侧没有路口了, 无需行驶。")
        return

    nxt = east[0]
    goal_world = (nxt * scale + world_offset[0], pose.y_)
    print("下一个路口(分岔口) x=%.1f, 终点 (%.1f, %.1f)"
          % (nxt * scale + world_offset[0], goal_world[0], goal_world[1]))

    ref_path = build_straight_ref((pose.x_, pose.y_), goal_world)
    stanley = control.Stanley(k=1)
    pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)

    initial_time = time.time()
    prev_time = time.time()
    dt = 0.05

    print("\n开始直行 ...\n")
    while True:
        pose = ugv.get_transform()
        imu = ugv.get_imu_data()
        state = struct.State(pose, abs(imu.v), imu.a)

        dist2goal = utils.distance(pose.x_, pose.y_, goal_world[0], goal_world[1])
        if dist2goal < STOP_DIST or (time.time() - initial_time) > MAX_RUNTIME:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n停车。距路口 %.1f m, 位置 (%.1f, %.1f)"
                  % (dist2goal, pose.x_, pose.y_))
            break

        # 临近路口减速
        target_v = TARGET_V
        if dist2goal < SLOW_RANGE:
            target_v = max(TARGET_V * dist2goal / SLOW_RANGE, 1.0)

        target_pose = calculate_target_pose(pose, ref_path)
        delta = stanley.update_control(target_pose, state)
        accel = pid.update_control(target_v, state.v_, dt)

        steer = delta / (math.pi / 6)
        if accel > 0:
            throttle, brake = min(accel, 1.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(accel) * 0.5, 1.0)

        ugv.apply_control(steer, throttle, brake, False)

        if int(time.time() * 2) % 4 == 0:
            print("  pos=(%.1f, %.1f) v=%.1f 距路口 %.0f m"
                  % (pose.x_, pose.y_, state.v_, dist2goal))

        dt = time.time() - prev_time
        prev_time = time.time()
        time.sleep(0.05)


if __name__ == '__main__':
    main()
