# -*- coding: utf-8 -*-
"""
无人车1 (UGV1) 从出生点复位, 沿车道向东直行, 直到探测到 100m 内的路障后停车。

流程:
  1. 连仿真器, 注册 UGV1, 开启阶段。
  2. 通过 SDK 内部 carla actor 的 set_transform 把 UGV1 复位到出生点。
  3. 沿当前车道向东直行 (Stanley + PID), 每步调用 detect_fun()。
  4. 一旦探测到路障 (Type == "Other" 或含 "Barricade"), 立即刹停并打印。
"""
import math
import time
import json
import numpy as np

from swarmae import SwarmAENode as SN
from swarmae.SwarmAEClient import SwarmAEClient

import lib.swarm_api as api
import lib.frenet_optimal_planner as fop
import lib.controller as control
import lib.data_struct as struct
import lib.utils as utils

TARGET_V    = 10.0    # 巡航车速 (m/s)
GOAL_FAR    = 2000.0  # 终点放在前方很远, 实际靠"探测到路障"触发停车
MAX_RUNTIME = 100.0   # 最多跑多久 (s)
SPAWN       = (-1829.6, -517.7, 2.1)  # vehicles.json 中 UGV1 出生点


def build_straight_ref(p0, p1, n=20, resolution=2.0):
    """在 p0 -> p1 之间均匀取 n 个点拟合参考路径 (需 >=4 点)。"""
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


def is_barricade(type_name):
    """探测结果里的路障: 已确认的 BarricadeHard 表现为 Type='Other'。"""
    t = (type_name or "").lower()
    return t == "other" or "barricade" in t


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()
    game.stage_start('plan_start')
    game.stage_start('reconnaissance_start')
    game.stage_start('vau_reconnaissance_start')

    ugv = api.Vehicle(AeClient, "UGV1", 1, "ugv")

    # --- 复位到出生点 (通过 SDK 内部 carla actor 直接 set_transform) ---
    carla = SN.carla
    actor = ugv.Vehicle_._SwarmAENode__actor
    actor.set_transform(carla.Transform(
        carla.Location(SPAWN[0], SPAWN[1], SPAWN[2]),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)))
    time.sleep(1.5)                      # 等车辆落地
    ugv.apply_control(0, 0, 0, True)     # 短暂手刹清掉残余速度
    time.sleep(0.3)
    ugv.apply_control(0, 0, 0, False)

    pose = ugv.get_transform()
    if utils.distance(pose.x_, pose.y_, SPAWN[0], SPAWN[1]) > 5.0:
        print("ERROR: 复位失败, 当前位置 (%.1f, %.1f) 不在出生点附近, 中止。"
              % (pose.x_, pose.y_))
        return
    print("已复位到出生点: (%.1f, %.1f)  朝向 %.3f rad"
          % (pose.x_, pose.y_, pose.theta_))

    # --- 沿当前车道向东的参考直线 ---
    goal_world = (pose.x_ + GOAL_FAR, pose.y_)
    ref_path = build_straight_ref((pose.x_, pose.y_), goal_world)
    stanley = control.Stanley(k=1)
    pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)

    initial_time = time.time()
    prev_time = time.time()
    dt = 0.05

    print("开始直行, 每步探测 100m 内路障 ...\n")
    while True:
        pose = ugv.get_transform()
        imu = ugv.get_imu_data()
        state = struct.State(pose, abs(imu.v), imu.a)

        # 探测, 找路障
        ob = ugv.detect_fun()
        barricades = []
        for i in range(len(ob.Type)):
            if is_barricade(ob.Type[i]):
                d = utils.distance(pose.x_, pose.y_, ob.x_ue[i], ob.y_ue[i])
                barricades.append((ob.Type[i], ob.x_ue[i], ob.y_ue[i], d))

        if barricades:
            barricades.sort(key=lambda b: b[3])
            t, bx, by, d = barricades[0]
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n>>> 探测到路障, 停车! 类型=%s 位置=(%.1f, %.1f) 距 %.1f m"
                  % (t, bx, by, d))
            print("    停车位置 (%.1f, %.1f)" % (pose.x_, pose.y_))
            try:  # 停车时取一次原始返回, 打印 Name 便于确认是哪个路障
                _, _, raw = ugv.Vehicle_.detect_situation()
                print("    原始返回:", raw)
            except Exception as e:
                print("    (取原始返回失败: %s)" % e)
            break

        if (time.time() - initial_time) > MAX_RUNTIME:
            ugv.apply_control(0, 0, 0.8, False)
            time.sleep(1)
            ugv.apply_control(0, 0, 0, True)
            print("\n超时停车, 未探测到路障。位置 (%.1f, %.1f)"
                  % (pose.x_, pose.y_))
            break

        target_pose = calculate_target_pose(pose, ref_path)
        delta = stanley.update_control(target_pose, state)
        accel = pid.update_control(TARGET_V, state.v_, dt)

        steer = delta / (math.pi / 6)
        if accel > 0:
            throttle, brake = min(accel, 1.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(accel) * 0.5, 1.0)

        ugv.apply_control(steer, throttle, brake, False)

        if int(time.time() * 2) % 4 == 0:
            det_str = ",".join(ob.Type) if ob.Type else "-"
            print("  pos=(%.1f, %.1f) v=%.1f  detect=[%s]"
                  % (pose.x_, pose.y_, state.v_, det_str))

        dt = time.time() - prev_time
        prev_time = time.time()
        time.sleep(0.05)


if __name__ == '__main__':
    main()
