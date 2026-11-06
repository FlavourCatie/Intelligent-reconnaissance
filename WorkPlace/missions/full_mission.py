# -*- coding: utf-8 -*-
"""
full_mission.py —— 一条命令跑完整链路：无人机侦查 → 无人车打击 + 小地图
================================================================================

整合内容
--------
    missions/control_uav.py   无人机侦查（组员交付，未改动）
    missions/uav_runner.py    把它的阻塞循环改成可单步驱动（适配层）
    missions/control_ugv.py   无人车打击（组员交付，仅改资源图路径）
    missions/live_track.py    小地图显示（我的）

流程
----
    ① 解锁侦察阶段（否则 get_road_network() 会拒绝）
    ② 三架无人机起飞 → 按规划航线扫描东侧区域（Z=250，边飞边探测）
    ③ 无人机每发现一个目标就发布到 config/detected.json（组员实现）
    ④ 本脚本把发现的坐标吸附到路网，贪心分配给三台无人车
    ⑤ 三台无人车各自 A* → 行驶 → 途中遇障重规划 → 到达
    ⑥ 全过程在 live_track 小地图上实时显示：环境 / 平台轨迹 / 敌人空心→实心 / 目标星标

设计要点
--------
* **单进程、单 client、一次注册**：不出现"显示端和控制端重复注册同名节点"的问题。
* **主循环统一节拍**：UAV 一帧 → UGV 一帧 → 显示一帧，全部 20 Hz。
* 组员的两个模块保持原样，`control_uav.py` 仍然可以单独跑。

用法
----
    # 前置：CarlaUE4 → start.bat → subject4.exe（三车三机场景）已启动
    python WorkPlace\\missions\\full_mission.py

    # 只看无人机侦查，不起无人车
    python WorkPlace\\missions\\full_mission.py --no-ugv

    # 不弹小地图（只跑逻辑、看日志）
    python WorkPlace\\missions\\full_mission.py --no-view

    # 侦查超时后强制用预设目标点（联调用）
    python WorkPlace\\missions\\full_mission.py --use-default-enemies

注意
----
* 必须 Python 3.6 + swarmae。
* 无人机固定 Z=250 m、15 m/s，这是组员实测"能看见目标"的高度，别改。
* 无人车的目标必须落在路网上：本脚本会把无人机给的坐标吸附到最近的路网像素。
"""

from __future__ import print_function

import argparse
import itertools
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import traceback

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
PROJECT_DIR = os.path.dirname(WORKPLACE_DIR)
EASY_FOLLOW_DIR = os.path.join(PROJECT_DIR, "Materials", "practice", "easy_follow")

for _p in (THIS_DIR, EASY_FOLLOW_DIR, WORKPLACE_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

import lib.swarm_api as api
import control_uav as cu
import control_ugv as ugv
import live_track as lt
from uav_runner import build_uav_vehicles, UavRunner
from core.fleet import Pose

UGV_NAMES = ("UGV1", "UGV2", "UGV3")


def log(msg):
    print("[mission] " + msg)


# ---------------------------------------------------------------------------
#  阶段解锁
# ---------------------------------------------------------------------------
def stage_unlock(game):
    """按无人车组员的实测结论：不先跑完侦察阶段，get_road_network() 会返回失败。"""
    seq = [
        ("start", "plan_start"),
        ("start", "reconnaissance_start"),
        ("complete", "reconnaissance_end"),
        ("start", "vau_reconnaissance_start"),
        ("complete", "vau_reconnaissance_end"),
    ]
    for kind, name in seq:
        try:
            if kind == "start":
                game.stage_start(name)
            else:
                game.stage_complete(name)
        except Exception as e:
            log("阶段 %s(%s) 调用异常（继续）: %r" % (kind, name, e))
    log("侦察阶段已解锁")


# ---------------------------------------------------------------------------
#  飞控驱动提示（检查逻辑在 core/preflight.py）
# ---------------------------------------------------------------------------
KINEMATIC_TICK = 0.02      # 运动学模式的循环周期（50 Hz）
YIELD_GAP = 14.0           # 两车小于这个距离就触发让行 m
YIELD_HOLD = 3.0           # 让行持续秒数（避免来回抖动）


DRIVER_HINT = (
    "正确顺序（依据 environment_forclass/environment/kyxz.md 对 D4 组别）：\n"
    "    1) CarlaUE4\n"
    "    2) Materials\\simu_driver_windows\\start.bat  → 选 2（旋翼）\n"
    "    3) Materials\\practice\\practice1_Scene\\subject4.exe\n"
    "    4) 再跑本脚本\n"
    "  start.bat 要一直开着；它若比 CarlaUE4 先启动，世界还不存在，飞控绑不上飞机。")


# ---------------------------------------------------------------------------
#  位姿读取（UAV 用 UavVehicle.Vehicle_，UGV 用 api.Vehicle.Vehicle_，统一处理）
# ---------------------------------------------------------------------------
def pose_of_virtual(name, runner):
    """虚拟模式下的 UAV 位姿（不读真实 actor）。

    速度按"阶段"给：爬升/巡航时给巡航速度（小地图上图标是有颜色的），
    飞完航线（hold）或损毁（dead）给 0（图标自动变灰）。
    这样观众一眼能看出"哪架还在侦查、哪架已经结束"。
    """
    x, y, z, yaw = runner.virtual_pose(name)
    try:
        phase = runner.phases_snapshot().get(name, "cruise")
    except Exception:
        phase = "cruise"
    speed = 0.0 if phase in ("hold", "dead") else float(
        getattr(runner, "speed", 15.0) or 15.0)
    return Pose(name, "uav", float(x), float(y), float(z),
                math.radians(float(yaw)), speed)


def pose_of(name, vtype, node):
    loc = node.get_location()
    att = node.get_attitude()
    vel = node.get_velocity()
    return Pose(name, vtype,
                float(loc[0]), float(loc[1]), float(loc[2]),
                math.radians(float(att[0])),
                float(vel[3]))


# ---------------------------------------------------------------------------
#  目标点处理：吸附到路网 + 分配
# ---------------------------------------------------------------------------
def snap_to_road(points, binary, scale, off, search=40):
    """把无人机给的敌人坐标吸附到最近的路网像素（否则车开不到/停不下来）。"""
    out = []
    for p in points:
        px, py = ugv.w2p(float(p[0]), float(p[1]), scale, off)
        pt = ugv.nearest_road(binary, px, py, search=search)
        if pt is None:
            log("目标 (%.1f, %.1f) 附近 %.0f 像素内没有路网，跳过"
                % (p[0], p[1], search))
            continue
        wx, wy = ugv.p2w(pt[0], pt[1], scale, off)
        out.append((wx, wy))
    # 去掉吸附后重合的点
    uniq = []
    for p in out:
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 8.0 for q in uniq):
            uniq.append(p)
    return uniq


def assign_targets(points, starts, n=3, headings=None, trails=None):
    """把 n 个目标点分给 n 台车，使总代价最小（暴力枚举，n=3 时最多几百次）。

    headings 给出时，代价里加上"掉头/大角度转向"的惩罚：
    这条路的通行难度不只看直线距离——出生点就要掉头的车极容易和别的车挤死
    （实测 UX）。惩罚项 ≈ 车身需要转过的角度 × 45 m 等效距离。
    """
    if not points:
        return []
    if len(points) < n:
        # 目标"不够"才循环分配（多余的车间距靠超车逻辑处理）；
        # 恰好相等时必须走下面的最优匹配，否则会出现"两车同一目标"。
        return [points[i % len(points)] for i in range(n)]

    def _seg_dist(px, py, a, b):
        """点到线段距离（用来判断"这段路是不是又走一遍"）。"""
        ax, ay = a; bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 <= 1e-9:
            return math.hypot(px - ax, py - ay)
        tt = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        return math.hypot(px - (ax + tt * dx), py - (ay + tt * dy))

    def cost_of(start, heading, goal, trail=None):
        d = math.hypot(goal[0] - start[0], goal[1] - start[1])
        if heading is None:
            return d
        bearing = math.atan2(goal[1] - start[1], goal[0] - start[0])
        delta = abs((bearing - heading + math.pi) % (2 * math.pi) - math.pi)
        return d + 45.0 * (delta / math.pi)      # 掉头约 +45 m 等效

    best, best_cost = None, None
    for combo in itertools.combinations(points, n):
        for perm in itertools.permutations(combo, n):
            cost = sum(cost_of(starts[i], headings[i] if headings else None, perm[i],
                            (trails[i] if trails and i < len(trails) else None))
                       for i in range(n))
            if best_cost is None or cost < best_cost:
                best, best_cost = perm, cost
    return list(best)


try:
    from tactics import slots as _tactics_slots
except Exception:
    _tactics_slots = None

_TACTICS_MASK = None
_TACTICS_ENCIRCLE = {}   # tid -> (x, y, need_slots)


def _tactics_mask():
    """道路掩膜（懒加载），用于路口判定与阵位吸附。"""
    global _TACTICS_MASK
    if _TACTICS_MASK is not None:
        return _TACTICS_MASK
    if _tactics_slots is None:
        return None
    try:
        _TACTICS_MASK = _tactics_slots.RoadMask.from_png(
            os.path.join(WORKPLACE_DIR, "assets", "roadnet_raw.png"),
            meta_path=os.path.join(WORKPLACE_DIR, "config", "roadnet_meta.json"))
    except Exception:
        _TACTICS_MASK = None
    return _TACTICS_MASK


# tid -> (x, y, 需要几个阵位才算"到齐")
_TACTICS_ENCIRCLE = {}


def tactics_decide(tac, now, free_idx, pool, ugvs, uav_vehicles, radius,
                   binary=None, scale=1.0, off=(0.0, 0.0)):
    """调用战术层：喂覆盖 + 车辆 + 敌人，收回「每台车去哪」。

    返回 {vid: (x, y)}；空 dict 表示没有可用计划（调用方退回贪心）。
    """
    # 1) 覆盖：无人机当前位置记一笔
    for n in cu.UAV_NAMES:
        try:
            p = uav_vehicles[n].position()
            tac.mark_coverage(p[0], p[1], radius)
        except Exception:
            pass
    # 2) 车辆
    vehicles = []
    for k in free_idx:
        pose = ugvs[k].get_state().pose_
        vehicles.append({"vid": UGV_NAMES[k],
                         "pos": (float(pose.x_), float(pose.y_)),
                         "commit": "L0"})
    # 3) 敌人（先用普通目标占位，路口阵位由 P2 的 slots 模块接管）
    enemies = []
    _mask = _tactics_mask()
    _TACTICS_ENCIRCLE.clear()   # 每轮重建，避免已消灭目标残留黄圈
    for i, p in enumerate(pool):
        # 只处理任务区内的敌人：区外目标会被路网裁剪挡住，派车必卡死
        if not tac.zone.contains(float(p[0]), float(p[1]), margin=20.0):
            continue
        # 目标 id 用坐标生成（原来用 pool 下标，敌人减少后会错位，黄圈就跑到别人身上）
        tid = "E%d_%d" % (int(round(float(p[0]))), int(round(float(p[1]))))
        kind, deg, slots_ = "road", 1, None
        if _mask is not None:
            try:
                kind, _branches = _tactics_slots.classify(
                    _mask, (float(p[0]), float(p[1])))
                if kind in ("junction", "tee") and len(_branches) >= 2:
                    _ss = _tactics_slots.build_slots(
                        _mask, (float(p[0]), float(p[1])), _branches)
                    if _ss:
                        slots_ = [{"sid": s["sid"], "pos": s["pos"]} for s in _ss]
                        deg = len(_branches)
                        _TACTICS_ENCIRCLE[tid] = (float(p[0]), float(p[1]),
                                                  len(_ss))
                else:
                    kind = "road"
            except Exception:
                kind, deg, slots_ = "road", 1, None
        try:
            log("[分类] %s (%.0f,%.0f) -> %s 分支=%d 阵位=%d"
                % (tid, p[0], p[1], kind, deg, len(slots_ or [])))
        except Exception:
            pass
        enemies.append((tid, (float(p[0]), float(p[1])), kind, deg, float(now), slots_))
    # 4) 前沿候选：路网格点抽样（每 25 像素一个）
    road_pts = None
    if binary is not None:
        road_pts = []
        step = 25
        for py_ in range(0, binary.shape[0], step):
            for px_ in range(0, binary.shape[1], step):
                if binary[py_, px_]:
                    wx, wy = ugv.p2w(px_, py_, scale, off)
                    road_pts.append((float(wx), float(wy)))
        if len(road_pts) > 4000:
            road_pts = road_pts[::max(1, len(road_pts) // 4000)]
    plan = tac.update(now, vehicles, enemies, road_points=road_pts)
    if not plan:
        return {}
    out = {}
    for g in plan.get("groups", []):
        for s in g.get("slots", []):
            out[s["vid"]] = (s["pos"][0], s["pos"][1])
    for it in plan.get("idle", []):
        out.setdefault(it["vid"], (it["frontier"][0], it["frontier"][1]))
    return out


def freeze_flight_driver():
    """结束飞控驱动，免得它在 5 Hz 写自己的位姿、和我们的 set_transform 抢高度。

    只在运动学模式下有意义（真实飞行本来就需要它）。
    实测：不结束它时，无人机在 UE4 里会小幅上下抖，而我们的读数却是恒定的 ——
    说明是驱动在中间改写了 actor 位姿。
    """
    killed = []
    for exe in ("main-release.exe", "driver.exe"):
        try:
            out = subprocess.run(["taskkill", "/F", "/IM", exe],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if out.returncode == 0:
                killed.append(exe)
        except Exception:
            pass
    if killed:
        log("已结束飞控驱动以稳定无人机位姿：%s" % ", ".join(killed))
        log("  （真实飞行不可用了；要恢复请重开 Materials/simu_driver_windows/start.bat）")
    else:
        log("--freeze-driver：没找到/没能结束飞控驱动（可能已经不在运行）")
    return killed


def merge_goals_atomic(updates, path=None):
    """把 UGV 目标并进 goals.json（原子替换，保留队友写的其它字段）。

    注意：不能用 `control_uav.sync_goals()`——它按设计**只允许写 UAV1/2/3**，
    传 UGV 名字会直接抛 SafetyError。
    """
    path = path or cu.GOALS_PATH
    data = {}
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                log("goals.json 不是对象，放弃写入以免覆盖队友内容")
                return False
        except Exception as e:
            log("goals.json 解析失败，放弃写入（不覆盖队友内容）: %r" % e)
            return False
    for k, v in updates.items():
        data[k] = [float(v[0]), float(v[1])]
    d = os.path.dirname(path) or "."
    try:
        fd, tmp = tempfile.mkstemp(dir=d, suffix=".json")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        os.replace(tmp, path)
        return True
    except Exception as e:
        log("写 goals.json 失败: %r" % e)
        return False


# ---------------------------------------------------------------------------
#  无人车复位到出生点（组员脚本里的做法；SDK 不支持时跳过）
# ---------------------------------------------------------------------------
def teleport_ugvs(vehicles, spawns):
    try:
        from swarmae import SwarmAENode as SN
        carla = SN.carla
    except Exception as e:
        log("拿不到 carla 模块，跳过复位（车本来就该在出生点）: %r" % e)
        return False
    try:
        for v, (sx, sy, sz) in zip(vehicles, spawns):
            actor = getattr(v.Vehicle_, "_SwarmAENode__actor", None)
            if actor is None:
                log("拿不到 actor 句柄，跳过复位")
                return False
            actor.set_transform(carla.Transform(
                carla.Location(sx, sy, sz),
                carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)))
        time.sleep(1.5)
        for v in vehicles:
            v.apply_control(0, 0, 0, True)
        time.sleep(0.3)
        for v in vehicles:
            v.apply_control(0, 0, 0, False)
        log("三台无人车已复位到出生点")
        return True
    except Exception as e:
        log("复位失败（继续，车保持在当前位置）: %r" % e)
        return False


def face_targets(vehicles, agents):
    """发车前把每台车转向它自己的目标，避免开局掉头。

    为什么必须做：三台车出生在同一个丁字路口、`vehicles.json` 里 yaw 都是 0，
    于是朝向完全一致。当敌人出现在丁字口的两个侧支路时，车必须原地掉头或倒车，
    而车体运动学（前轮转向 + 最小转弯半径）在路口根本没有空间完成这个动作 ——
    表现就是"卡住/反复倒车"。

    直接把车身朝向转到目标方向后，车一起步就是"顺着路走"，不再需要掉头。
    """
    try:
        from swarmae import SwarmAENode as SN
        carla = SN.carla
    except Exception as e:
        log("拿不到 carla 模块，跳过转向: %r" % (e,))
        return 0
    turned = 0
    for a, v in zip(agents, vehicles):
        try:
            st = v.get_state().pose_
            x, y, z = float(st.x_), float(st.y_), float(st.z_)
            tx, ty = float(a.target[0]), float(a.target[1])
            if abs(tx - x) < 1e-6 and abs(ty - y) < 1e-6:
                continue
            yaw = math.degrees(math.atan2(ty - y, tx - x))
            actor = getattr(v.Vehicle_, "_SwarmAENode__actor", None)
            if actor is None:
                continue
            actor.set_transform(carla.Transform(
                carla.Location(x, y, z),
                carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)))
            log("%s 起步朝向 -> %.0f°（指向目标 %.0f, %.0f）"
                % (a.name, yaw, tx, ty))
            turned += 1
        except Exception as e:
            log("%s 转向失败（忽略）: %r" % (a.name, e))
    if turned:
        time.sleep(0.5)
        for v in vehicles:
            try:
                v.apply_control(0.0, 0.0, 1.0, False)   # 先刹住，别带着旧速度冲出去
            except Exception:
                pass
    return turned


def compute_search_roi(uav_pos, ugv_pos, markers, args):
    """算出一个"任务区域"矩形 (cx, cy, half)，扫描区域和局部地图都用它。

    取「无人机出生点 + 无人车出生点 + 近处敌人」的包围盒：
    既保证覆盖当前敌情，又不会把视角撑回全城。
    """
    xs, ys = [], []
    for p in uav_pos:
        xs.append(float(p[0]))
        ys.append(float(p[1]))
    for p in ugv_pos:
        xs.append(float(p[0]))
        ys.append(float(p[1]))
    if not xs or not ys:
        return None

    base_x = sum(xs) / float(len(xs))
    base_y = sum(ys) / float(len(ys))
    near = [m for m in markers
            if math.hypot(m.x - base_x, m.y - base_y) <= args.view_roi_range]
    if len(near) < len(markers):
        log("任务区域只统计出生点 %.0f m 内的 %d 个目标（共 %d 个）"
            % (args.view_roi_range, len(near), len(markers)))
    for m in near:
        xs.append(float(m.x))
        ys.append(float(m.y))

    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0
    # 不做上限截断：搜索区域与局部地图必须是同一个矩形，
    # 一旦截断，敌人环带的边缘就会落在扫描范围之外。
    half = max(args.view_roi_min, half + args.view_roi_margin)
    return (cx, cy, half)


def _all_uavs_at_altitude(uav_vehicles, tol=5.0):
    """三架机是不是都已经到巡航高度（用于判断能否改航）。"""
    try:
        for name in cu.UAV_NAMES:
            if uav_vehicles[name].position()[2] < cu.ALTITUDE - tol:
                return False
        return True
    except Exception:
        return False


def parse_goto_points(text):
    """把 "x,y;x,y;x,y" 解析成 [(x,y), ...]。分号和竖线都当分隔符。"""
    if not text:
        return []
    points = []
    for chunk in text.replace("|", ";").replace("，", ",").split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p for p in chunk.replace(" ", ",").split(",") if p.strip()]
        if len(parts) < 2:
            raise SystemExit("--goto 片段格式不对：%r（应为 x,y）" % chunk)
        points.append((float(parts[0]), float(parts[1])))
    return points


def build_goto_routes(planner, starts, points, tol=None):
    """把 --goto 给的坐标变成「起飞 → 直飞目标点」的两点航线。

    points 少于三架机时按距离就近分配（可能多架飞同一个点）。
    落在禁飞区/安全余量内的点会被丢弃并记日志——宁可不去，也不能撞禁飞区。
    """
    names = list(cu.UAV_NAMES)
    if not points:
        return None
    safe = []
    for (x, y) in points:
        if planner.point_safe((x, y), planning=True):
            safe.append((float(x), float(y)))
        else:
            log("goto 目标 (%.1f, %.1f) 在禁飞区/安全余量内，已丢弃" % (x, y))
    if not safe:
        return None

    routes = {}
    for i, name in enumerate(names):
        start = starts[name]
        if len(safe) >= len(names):
            target = safe[i]
            routes[name] = [start, target]
        else:
            target = min(safe, key=lambda p: math.hypot(p[0] - start[0], p[1] - start[1]))
            routes[name] = [start, target]
        if not planner.segment_safe(start, routes[name][-1]):
            log("%s 的 goto 直线段不安全（穿禁飞区），改用绕行航点" % name)
            via = planner.detour(start, routes[name][-1]) if hasattr(planner, "detour") else None
            if via:
                routes[name] = [start] + list(via) + [routes[name][-1]]
    return routes


def build_enemy_routes(planner, starts, points, per_uav=2):
    """按敌人实际位置给三架机排航线（就近分配 + 就近串联 + 走安全规划）。

    为什么要这个：`cu.search_routes()` 用的是 game.json 里**固定的条带图案**，
    和敌人无关，导致每次都是同样 7/7/9 个航点、绕三四公里，侦查极慢。
    直接飞向敌人，几公里能压到一两百米。

    每一段都过 `planner.route()`，所以依然绕开禁飞区（不会为了快而穿禁区）。
    """
    remaining = [tuple(p[:2]) for p in points]

    # ---- 分配策略：先保证每架机各拿一个，再谈"多串几个" ----
    # 之前的写法是"第一架机先把最近的两个都拿走"，导致第三架机没有目标、
    # 航线退化成 [起点] 原地不动（观感上就是"有一架没出动"）。
    groups = dict((n, []) for n in cu.UAV_NAMES)
    pool = list(remaining)
    for name in cu.UAV_NAMES:
        if not pool:
            break
        start = tuple(starts[name][:2])
        pool.sort(key=lambda p: cu.distance(start, p))
        groups[name].append(pool.pop(0))
    # 还有剩余目标就补给"离它最近且数量最少"的那架
    while pool and any(len(g) < per_uav for g in groups.values()):
        best = None
        for name in cu.UAV_NAMES:
            if len(groups[name]) >= per_uav:
                continue
            last = groups[name][-1] if groups[name] else tuple(starts[name][:2])
            cand = min(pool, key=lambda p: cu.distance(last, p))
            d = cu.distance(last, cand)
            if best is None or d < best[0]:
                best = (d, name, cand)
        if best is None:
            break
        _, name, cand = best
        groups[name].append(cand)
        pool.remove(cand)

    routes = {}
    for name in cu.UAV_NAMES:
        start = tuple(starts[name][:2])
        if not groups[name]:
            routes[name] = [start]
            log("  [航线] %s 没有分到目标（目标数少于机数），原地待命" % name)
            continue
        legs = [start] + groups[name]

        pts = [legs[0]]
        for a, b in zip(legs, legs[1:]):
            try:
                seg = planner.route(a, b)
            except cu.SafetyError as e:
                log("  [航线] %s 到 (%.0f,%.0f) 无安全路径，跳过: %s"
                    % (name, b[0], b[1], e))
                continue
            pts.extend(seg[1:] if pts else seg)
        routes[name] = pts if len(pts) >= 2 else [start]
    return routes


# ---------------------------------------------------------------------------
#  参数
# ---------------------------------------------------------------------------
def _fix_leading_dash(argv, value_opts):
    out, i = [], 0
    while i < len(argv):
        a = argv[i]
        if a in value_opts and i + 1 < len(argv):
            out.append("%s=%s" % (a, argv[i + 1]))
            i += 2
            continue
        out.append(a)
        i += 1
    return out


def parse_args():
    ap = argparse.ArgumentParser(
        description="完整链路：无人机侦查 → 无人车打击 + 小地图显示",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--scene", default=os.path.join(PROJECT_DIR, "Materials",
                                                    "practice", "practice1_Scene"),
                    help="场景目录（含 game.json / vehicles.json / targets.json）")
    ap.add_argument("--node-nos", nargs=3, type=int, default=None,
                    metavar=("UAV1", "UAV2", "UAV3"),
                    help="无人机节点号；默认按 vehicles.json 顺序推断")

    g = ap.add_argument_group("流程参数")
    g.add_argument("--min-targets", type=int, default=3,
                   help="侦察到几个目标就派无人车（默认3）")
    g.add_argument("--recon-timeout", type=float, default=300.0,
                   help="侦查阶段最长等待秒数（默认300）")
    g.add_argument("--use-default-enemies", action="store_true",
                   help="侦查超时后用组员的预设目标点兜底（联调用）")
    g.add_argument("--no-ugv", action="store_true", help="只跑无人机侦查，不起无人车")
    g.add_argument("--no-teleport", action="store_true", help="不把无人车复位到出生点")
    g.add_argument("--duration", type=float, default=1800.0, help="总运行上限秒数")
    g.add_argument("--ugv-after", type=float, default=0.0,
                   help="发现目标后立即派车；>0 表示再等这么多秒")

    g = ap.add_argument_group("无人机模式")
    g.add_argument("--uav-mode", choices=("kinetic", "kinematic", "virtual"),
                   default="kinetic",
                   help="kinetic=真实飞控动力学；kinematic=set_transform 沿航线搬；"
                        "virtual=完全不碰物理，位置与探测全用数学算（最稳，画面里飞机不动）")
    g.add_argument("--uav-route", choices=("sweep", "enemies", "strips", "goto"),
                   default="sweep",
                   help="sweep=牛耕式覆盖扫描（默认，敌人位置未知时的正确做法）；"
                        "enemies=按已知敌人位置排航线；"
                        "goto=直接飞向 --goto 给出的坐标（侦察/打击指定点）；"
                        "strips=game.json 里的固定条带图案")
    g.add_argument("--goto", default=None,
                   help="--uav-route goto 的目标坐标，形如 \"x,y;x,y;x,y\"（世界坐标，m）。"
                        "可以只给 1~2 个点，其余飞机就近分配；也可以每架给一个")
    g.add_argument("--goto-z", type=float, default=None,
                   help="goto 模式的到达高度（默认用 --uav-altitude）")
    g.add_argument("--goto-tol", type=float, default=8.0,
                   help="goto 判到点的水平误差阈值 m（默认8）")
    g.add_argument("--attack", dest="attack", action="store_true", default=True,
                   help="（默认开启）无人车到达后自动开火消灭附近敌人")
    g.add_argument("--no-attack", dest="attack", action="store_false",
                   help="只开车不打仗（调试用）")
    g.add_argument("--ugv-waves", dest="ugv_waves", action="store_true", default=True,
                   help="（默认开启）车打完一批后，若还有敌人就再派一批")
    g.add_argument("--attack-range", type=float, default=120.0,
                   help="无人车开火的最大距离 m（默认120）")
    g.add_argument("--roi-with-markers", action="store_true",
                   help="任务区把已知敌人也算进包围盒（演示用；真实任务不需要知道敌人位置）")
    g.add_argument("--shot-at", type=float, default=0.0,
                   help="运行到第 N 秒自动保存一张小地图 PNG 到 WorkPlace/figs/")
    g.add_argument("--area", default="auto",
                   help="【统一区域】侦查区域 = 小地图局部区域 = \"x0,y0,x1,y1\"（世界坐标 m）。"
                        "填 auto 则自动算。用这一个参数保证『扫的就是小地图上看到的那一块』，"
                        "不会再出现『小地图范围比扫描范围大、敌人扫不到』")
    g.add_argument("--sweep-radius", type=float, default=300.0,
                   help="覆盖扫描的探测半径 R（默认300），条带间距自动取 2R")
    g.add_argument("--sweep-half", type=float, default=None,
                   help="扫描区域半边长 m；默认=None 表示**直接采用局部地图那个任务区域**，"
                        "这样扫的就是看到的那一块，不会多扫")
    g.add_argument("--sweep-partition", choices=("rects", "strips"), default="rects",
                   help="rects=把区域切成 3 块矩形，每架机各扫一块（默认）；"
                        "strips=全局条带再按航程均衡")
    g.add_argument("--sweep-region", default="auto",
                   help="扫描区域 x0,y0,x1,y1；填 auto 表示"
                        "以无人机出生点为中心、半边长 = R+150 m 的方块"
                        "（演示用，比整张任务区快得多）；默认取 game.json 任务区")
    g.add_argument("--uav-strike", action="store_true", default=False,
                   help="（默认开启）发现足够目标后，把三架无人机也改飞过去（一机一个目标）")
    g.add_argument("--no-uav-strike", dest="uav_strike", action="store_false", default=False,
                   help="只派无人车，无人机继续悬停")
    g.add_argument("--uav-per-target", type=int, default=2,
                   help="每架机串几个目标点（默认2）")
    g.add_argument("--keep-driver", action="store_true",
                   help="【不建议】运动学模式下保留飞控驱动。实测评测：驱动一直在跑"
                        "零推力的动力学，会把无人机往下拽，导致抖动/撞墙/坠毁")
    g.add_argument("--freeze-driver", action="store_true",
                   help="（默认已开启）运动学模式开始前结束 driver.exe / main-release.exe")
    g.add_argument("--uav-speed", type=float, default=None,
                   help="运动学/虚拟模式的巡航速度 m/s（默认15；演示可调快，如 25）")
    g.add_argument("--uav-altitude", type=float, default=250.0,
                   help="飞行高度 m（默认250，kinetic/kinematic/virtual 都生效）。"
                        "注意 SITL 爬升率约 2.5 m/s，250 m 从地面起飞要约 100 s；"
                        "演示建议 120~150。别低于 100 m：会被楼挡住，撞楼掉血归零后 actor 失效")
    g.add_argument("--land", dest="land", action="store_true", default=True,
                   help="（默认开启）收尾时把三架机降回地面再退出")
    g.add_argument("--no-land", dest="land", action="store_false",
                   help="收尾时让飞机停在巡航高度（保持画面）")

    g = ap.add_argument_group("显示")
    g.add_argument("--no-view", action="store_true", help="不弹小地图（只跑逻辑）")
    g.add_argument("--view-interval", type=float, default=1.2,
                   help="小地图刷新间隔秒（默认1.2s）。按时间而不是按帧，"
                        "帧率不稳时也不会画得过勤，更省内存与CPU")
    g.add_argument("--view-every", type=int, default=None,
                   help="（可选）另外再按帧数节流；一般不需要")
    g.add_argument("--focus", default="UAV1", help="小地图启动聚焦哪台；0=总览")
    g.add_argument("--view-roi", choices=("auto", "none"), default="auto",
                   help="小地图右侧面板：auto=锁定任务区域（默认，演示更清晰）；"
                        "none=跟随 focus 的 ±zoom 视角")
    g.add_argument("--view-roi-margin", type=float, default=160.0,
                   help="自动 ROI 在包围盒外再留多少米（默认160）")
    g.add_argument("--view-roi-min", type=float, default=420.0,
                   help="自动 ROI 的最小半径 m（默认350，太近看不清）")
    g.add_argument("--view-roi-max", type=float, default=0.0,
                   help="自动 ROI 的最大半径 m（默认0=不限制）；设 500 就固定只看 1 km 见方")
    g.add_argument("--view-roi-range", type=float, default=1500.0,
                   help="只把距无人机出生点这个范围内的敌人算进 ROI（默认1500）；"
                        "场景里可能还留着远处的旧靶标，不筛的话视角会被撑到全城")
    g.add_argument("--match-tol", type=float, default=20.0,
                   help="发现坐标与 targets.json 已知目标判为同一个的距离阈值 m")
    g.add_argument("--no-face-target", action="store_true",
                   help="不把车转向目标（默认会转）。三台车出生朝向相同，"
                        "不转的话敌人位于侧支路时车必须掉头，路口空间不够会卡死")
    g.add_argument("--tactics", action="store_true", default=False,
                   help="启用战术层（编队/前沿待机/计划文件派车）；默认关闭，保持原有贪心派车")
    g.add_argument("--zone-clip", action="store_true", default=False,
                   help="把路网裁剪到任务区内（默认关：裁剪会切断边界道路，把车挡在外面）")
    g.add_argument("--ugv-start", default=None,
                   help="开局把三台无人车整体传送到 x,y（仿真里 UGV 出生点写死，改 json 无效）")
    g.add_argument("--zone", default=None,
                   help="战术层任务区 x0,y0,x1,y1（--tactics 时必填，无默认值）")
    g.add_argument("--plan-file", default=None,
                   help="strike_plan.json 路径（默认 WorkPlace/config/strike_plan.json）")
    g.add_argument("--dispatch-gap", type=float, default=4.0,
                   help="三台车错开发车的间隔秒数（默认4）。同时出发容易在同一个路口"
                        "挤成一团，错开能显著减少卡死")
    g.add_argument("--ugv-deadline", type=float, default=300.0,
                   help="无人车出发后最多跑多少秒（默认300）；超时直接收尾，"
                        "避免卡在个别车一直 drive 上不结束")
    g.add_argument("--console-every", type=float, default=5.0,
                   help="控制台心跳间隔秒数（默认5；0=关闭）")
    g.add_argument("--takeoff-timeout", type=float, default=240.0,
                   help="起飞阶段告警秒数（默认240）。SITL 上电要先等 GPS home + EKF 收敛"
                        "才 arm（实测 20~60 s），这段时间飞机不动是正常的，不是飞控坏了")
    g.add_argument("--ignore-driver-check", action="store_true",
                   help="跳过飞控驱动进程检查")

    value_opts = ("--scene", "--node-nos", "--focus", "--sweep-region", "--goto", "--area", "--zone", "--plan-file", "--ugv-start")
    args = ap.parse_args(_fix_leading_dash(sys.argv[1:], value_opts))
    return args


# ---------------------------------------------------------------------------
#  主流程
# ---------------------------------------------------------------------------
def main():
    args = parse_args()

    # 场景里到底放了多少个敌人（targets.json 的 count 之和）——
    # 用于"全歼才结束"的判定：只打光"已发现"的就收工，会在还有 5 个没被发现时误结束。
    # ---- 战术层（可选）----
    tactics = None
    if getattr(args, "tactics", False):
        try:
            from tactics.zones import Zone
            from tactics.planner import Tactics

            _zone = Zone.parse(args.zone, name="tactics_zone")   # 缺 --zone 直接抛错
            _plan_path = args.plan_file or os.path.join(
                WORKPLACE_DIR, "config", "strike_plan.json")
            tactics = Tactics(_zone, _plan_path)
            log("战术层已启用：zone=%s  plan=%s" % (_zone.as_arg(), _plan_path))
        except Exception as _e:
            raise SystemExit("战术层初始化失败：%s" % (_e,))

    expected_enemies = 0
    try:
        _tp = os.path.join(args.scene, "targets.json")
        with open(_tp, "r", encoding="utf-8-sig") as _fp:
            _groups = json.load(_fp)
        for _g in _groups:
            _types = _g.get("types") or []
            if any(t in ("dynamic.prop.soldier", "dynamic.prop.stryker") for t in _types):
                expected_enemies += int(_g.get("count") or len(_g.get("points") or []))
    except Exception:
        expected_enemies = 0

    # 飞行高度对 kinetic/kinematic/virtual 三种模式统一生效
    try:
        cu.set_altitude(args.uav_altitude)
    except cu.SafetyError as e:
        raise SystemExit("--uav-altitude 不合法：%s" % e)

    # ---------------- 连接 ----------------
    # 注意：SDK 的 SwarmAEClient.__init__ 会自己解析 sys.argv（只认 -i/-p/-s/-n/-t），
    # 直接构造会把我们的 --uav-mode 之类当成非法参数而报错。
    # make_client 在构造期间把 argv 收起来，构造完还原。
    from core.sdk_boot import make_client
    log("连接 %s:%d ..." % (args.host, args.port))
    client = make_client(args.host, args.port)
    _, game, _ = client.get_game()

    stage_unlock(game)

    # ---------------- 小地图场景 ----------------
    img, scale, offset, items = lt.parse_road_network(game.get_road_network())
    if img is None:
        raise SystemExit("拿不到路网图，先确认 subject4.exe 已启动")
    scene = lt.Scene(img, scale, offset)

    env = None
    game_json = os.path.join(args.scene, "game.json")
    if os.path.isfile(game_json):
        from core import scenefile
        env = lt.Environment(scenefile.load_environment(game_json))
        log("环境：" + env.summary())

    markers = lt.load_known_markers(args.scene)
    log("targets.json 已知目标 %d 个（初始全部空心）" % len(markers))

    # ---------------- 飞控驱动预检 ----------------
    if not args.ignore_driver_check:
        from core import preflight
        pf = preflight.check()
        if pf.get("available"):
            log("启动前置检查：")
            for line in preflight.report(pf):
                log(line)
            if pf["problems"]:
                log("=" * 62)
                for p in pf["problems"]:
                    log("!! " + p)
                log(preflight.ORDER_HINT)
                log("   无人车不受影响（走 CARLA 直接控制），可以先只验证车。")
                log("=" * 62)
        else:
            log("（读不到进程列表，跳过启动前置检查）")

    # ---------------- 无人车（先注册、复位，但先不动） ----------------
    ugvs = []
    for i, name in enumerate(UGV_NAMES):
        v = api.Vehicle(client, vehicle_name=name, vehicle_no=i + 1,
                        vehicle_type="ugv")
        ugvs.append(v)
    log("已注册 3 台无人车（node_no 1..3）")
    if not args.no_teleport:
        teleport_ugvs(ugvs, ugv.SPAWNS)

    # ---------------- 无人机 ----------------
    entries = cu.scene_uavs(args.scene)
    node_nos = args.node_nos or [e[0] for e in entries]
    log("无人机节点号：%s（来自 %s）" % (node_nos, os.path.basename(args.scene)))

    logger, log_path = cu.make_logger()
    log("无人机日志：%s" % log_path)

    uav_vehicles = build_uav_vehicles(client, node_nos, logger)
    planner = cu.SafePlanner(cu.read_json(game_json))
    if args.uav_mode == "kinetic" and not getattr(args, "no_spread", False):
        # 三架机出生点只隔 2~3 m，直接一起爬升会互相撞；先在同高度水平错开
        try:
            cu.spread_uavs(uav_vehicles, logger)
        except Exception as e:
            log("无人机散开失败（继续按原位置起飞）：%r" % (e,))
    starts = dict((n, uav_vehicles[n].position()) for n in cu.UAV_NAMES)

    # ---- 先算任务区域（扫描区域与局部地图共用同一个矩形）----
    search_roi = None
    try:
        uav_pos = [starts[n] for n in cu.UAV_NAMES]
        ugv_pos = []
        for v in ugvs:
            st = v.get_state().pose_
            ugv_pos.append((float(st.x_), float(st.y_)))
        # 【真实任务】无人机不知道敌人位置：任务区只按"出生点"算，不把 markers 算进去。
        # 想看"按敌情撑开"的旧行为，加 --roi-with-markers。
        roi_markers = markers if getattr(args, "roi_with_markers", False) else []
        search_roi = compute_search_roi(uav_pos, ugv_pos, roi_markers, args)
        if search_roi:
            log("任务区域：中心 (%.0f, %.0f)  ±%.0f m（%.0f×%.0f）"
                % (search_roi[0], search_roi[1], search_roi[2],
                   2 * search_roi[2], 2 * search_roi[2]))
    except Exception as e:
        log("任务区域计算失败（用出生点默认范围）: %r" % (e,))

    # ---- 统一区域：--area 同时决定「扫描区域」和「小地图区域」 ----
    # 以前这两块是各算各的：小地图 ROII 由 bbox 撑大、扫描区域另给，
    # 结果小地图比实际扫描范围大，地图上看到的敌人根本扫不到。
    if args.area and str(args.area).strip().lower() != "auto":
        try:
            x0, y0, x1, y1 = [float(v) for v in
                              str(args.area).replace("，", ",").split(",")[:4]]
        except Exception:
            raise SystemExit("--area 格式应为 x0,y0,x1,y1（世界坐标米）")
        if x1 <= x0 or y1 <= y0:
            raise SystemExit("--area 要求 x1>x0 且 y1>y0")
        # 把区域向里收 35 m：任务区常常正好压在 arena 边界上，'边上的整条扫描线'会因为
        # 安全余量被判为不可用 → 整块被判 dead → 触发"全局条带"回退，三机互相重复扫描。
        INSET = 35.0
        x0, y0, x1, y1 = x0 + INSET, y0 + INSET, x1 - INSET, y1 - INSET
        half = max(x1 - x0, y1 - y0) / 2.0
        search_roi = ((x0 + x1) / 2.0, (y0 + y1) / 2.0, half)
        args.sweep_region = "%.1f,%.1f,%.1f,%.1f" % (x0, y0, x1, y1)
        args.sweep_half = half
        log("统一区域：x[%.0f, %.0f] y[%.0f, %.0f]（%.0f×%.0f m）"
            " —— 扫描区域 = 小地图区域"
            % (x0, x1, y0, y1, x1 - x0, y1 - y0))

    routes = None
    if args.uav_route == "sweep":
        try:
            from sweep_plan import build as build_sweep, load_region
            if args.sweep_region and args.sweep_region.strip().lower() != "auto":
                region = tuple(float(v) for v in
                               args.sweep_region.replace("，", ",").split(",")[:4])
                assert len(region) == 4
            elif args.sweep_region and args.sweep_region.strip().lower() == "auto":
                if args.sweep_half is not None:
                    xs = [float(starts[n][0]) for n in cu.UAV_NAMES]
                    ys = [float(starts[n][1]) for n in cu.UAV_NAMES]
                    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
                    half = args.sweep_half
                    log("扫描区域 = 手动指定（出生点为中心，半边长 %.0f m）" % half)
                elif search_roi is not None:
                    cx, cy, half = search_roi
                    log("扫描区域 = 局部地图任务区域（±%.0f m，%.0f×%.0f）"
                        % (half, 2 * half, 2 * half))
                else:
                    xs = [float(starts[n][0]) for n in cu.UAV_NAMES]
                    ys = [float(starts[n][1]) for n in cu.UAV_NAMES]
                    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
                    half = 700.0
                    log("扫描区域 = 兜底（出生点为中心，半边长 %.0f m）" % half)
                region = (cx - half, cy - half, cx + half, cy + half)
            else:
                region = load_region(args.scene)
            routes, strips, groups, step = build_sweep(
                region, len(cu.UAV_NAMES), args.sweep_radius,
                starts=starts, planner=planner,
                partition=args.sweep_partition)
            if args.sweep_partition == "rects":
                try:
                    from sweep_plan import partition_rects
                    for i, r in enumerate(partition_rects(region, len(cu.UAV_NAMES))):
                        log("  子区域%d: x[%.0f, %.0f]  y[%.0f, %.0f]"
                            % (i + 1, r[0], r[2], r[1], r[3]))
                except Exception:
                    pass
            log("牛耕式覆盖扫描：R=%.0f m，条带间距 2R=%.0f m（实际 %.1f m），共 %d 条扫描线"
                % (args.sweep_radius, 2 * args.sweep_radius, step, len(strips)))
            for nm in cu.UAV_NAMES:
                r = routes.get(nm, [])
                L = sum(cu.distance(a, b) for a, b in zip(r, r[1:]))
                log("  %s：%d 个航点，航程 %.0f m，预计 %.0f s"
                    % (nm, len(r), L, L / max(1.0, cu.CRUISE_SPEED)))
        except Exception as e:
            log("生成扫描航线失败，退回敌人位置航线：%r" % (e,))
            routes = None

    if args.uav_route == "goto":
        pts = parse_goto_points(args.goto)
        if not pts:
            raise SystemExit('--uav-route goto 需要 --goto "x,y;x,y" 形式的目标坐标')
        if args.goto_tol:
            cu.set_waypoint_tol(args.goto_tol)
        routes = build_goto_routes(planner, starts, pts)
        if routes is None:
            raise SystemExit("所有 goto 目标都落在禁飞区/安全余量内，放弃本次飞行")
        log("航线 = 直奔给定坐标（%d 个目标点，到点判定 %.0f m）" % (len(pts), cu.WAYPOINT_TOL))
        for nm in cu.UAV_NAMES:
            log("  %s -> (%.1f, %.1f) 高度 %.0f m"
                % (nm, routes[nm][-1][0], routes[nm][-1][1], cu.ALTITUDE))

    if routes is None and args.uav_route == "enemies":
        try:
            tp = os.path.join(args.scene, "targets.json")
            known = [x for x in scenefile.load_targets(tp) if not x.assoc]
            pts = [(x.x, x.y) for x in known]
        except Exception as e:
            log("读 targets.json 失败，退回固定条带航线: %r" % (e,))
            pts = []
        # 落在禁飞区/安全余量里的目标不能作为航点，否则每段都会规划失败
        safe_pts = [p for p in pts if planner.point_safe(p, planning=True)]
        if len(safe_pts) < len(pts):
            log("有 %d 个目标在禁飞区/安全余量内，不作为航点"
                % (len(pts) - len(safe_pts)))
        pts = safe_pts
        if len(pts) >= len(cu.UAV_NAMES):
            routes = build_enemy_routes(planner, starts, pts,
                                        per_uav=max(1, args.uav_per_target))
            log("航线按敌人位置生成（共 %d 个已知目标，每架最多串 %d 个）"
                % (len(pts), max(1, args.uav_per_target)))
        else:
            log("已知目标只有 %d 个，退回固定条带航线" % len(pts))
    if routes is None:
        routes = cu.search_routes(planner, starts)
    if args.uav_mode in ("kinematic", "virtual"):
        from uav_runner import KinematicUavRunner
        # 默认就停掉飞控驱动：它在跑零推力动力学，会把我们摆好的飞机往下拽，
        # 结果是抖动 → 撞进建筑 → 掉血归零。实测两架机 6 秒内就 dead 了。
        if not args.keep_driver:
            log("运动学模式：先停掉飞控驱动（否则它会和 set_transform 抢位姿）")
            freeze_flight_driver()
        else:
            log("--keep-driver：保留飞控驱动（会和不动的 set_transform 抢位姿）")
        virtual = (args.uav_mode == "virtual")
        known = []
        try:
            tp = os.path.join(args.scene, "targets.json")
            known = [(x.x, x.y) for x in scenefile.load_targets(tp) if not x.assoc]
        except Exception:
            pass
        runner = KinematicUavRunner(uav_vehicles, routes, logger,
                                    speed=(args.uav_speed or cu.CRUISE_SPEED),
                                    dt=cu.DT,
                                    altitude=args.uav_altitude,
                                    virtual=virtual, known_targets=known,
                                    detect_radius=300.0)
        if virtual:
            log("!! 无人机采用【虚拟模式】：完全绕开物理引擎，位置与探测用数学计算。")
            log("   仿真画面里的飞机不会移动（因为它的物理关不掉，会一直下坠）。")
            log("   侦查/发现/派车链路完全真实，小地图上的航迹是准确的。")
        log("!! 无人机采用【运动学播放】模式（set_transform 沿航线搬）——"
            "非真实飞行，仅用于飞控动力学不可用时把流程跑通")
    else:
        runner = UavRunner(uav_vehicles, planner, routes, logger,
                           use_default=False)
    # ---- 航段绕行：连接段穿禁飞区时，用规划器的 route() 补绕行点 ----
    # 牛耕只保证"扫描线自身"安全；两条线之间的斜向连接段可能直接穿过禁飞区，
    # 实测 UAV2 就是这样越界后触发安全边界的。
    try:
        fixed = {}
        inserted = 0
        for _n, _pts in routes.items():
            _out = [_pts[0]]
            for _i in range(1, len(_pts)):
                _a, _b = _out[-1], _pts[_i]
                if not planner.segment_safe(_a, _b, planning=True):
                    try:
                        _mid = planner.route(_a, _b)
                    except Exception:
                        _mid = None
                    if _mid and len(_mid) > 2:
                        for _m in _mid[1:-1]:
                            _out.append((float(_m[0]), float(_m[1])))
                            inserted += 1
                        log("%s 连接段穿禁飞区，插入 %d 个绕行点" % (_n, len(_mid) - 2))
                _out.append(_b)
            fixed[_n] = _out
        if inserted:
            log("绕行点合计 %d 个" % inserted)
        routes = fixed
    except Exception as _e:
        log("航段绕行处理失败（继续用原航线）：%r" % (_e,))

    log("无人机航线已规划：%s" % dict((n, len(routes[n])) for n in cu.UAV_NAMES))

    # ---------------- 无人车路网 ----------------
    binary, raw_bin, ugv_scale, ugv_off = ugv.load_binary(game)
    log("无人车路网已加载（骨架 + 真实路宽）")

    # 战术层启用时：把任务区以外的路网全部置为不可通行，
    # 这样 A* 只会在局部地图内找路，车不会开出作战区域。
    if tactics is not None and getattr(args, "zone_clip", False):
        try:
            import numpy as _np
            _z = tactics.zone
            _h, _w = binary.shape
            _xs = _np.arange(_w) * ugv_scale + ugv_off[0]
            _ys = _np.arange(_h) * ugv_scale + ugv_off[1]
            _m = 30.0     # 边界放宽 30 m：避免把贴着边界的道路切断导致路网断开
            _inside = ((_xs[None, :] >= _z.x0 - _m) & (_xs[None, :] <= _z.x1 + _m)
                       & (_ys[:, None] >= _z.y0 - _m) & (_ys[:, None] <= _z.y1 + _m))
            binary = binary & _inside
            log("路网已裁剪到任务区内：x[%.0f,%.0f] y[%.0f,%.0f]"
                % (_z.x0, _z.x1, _z.y0, _z.y1))
        except Exception as _e:
            log("任务区裁剪失败（继续用全图）：%r" % (_e,))

    # --ugv-start x,y：仿真里 UGV 出生点是写死的（改 vehicles.json 不生效，实测），
    # 开局把三台车整体传送到任务区附近，保持相对队形与朝向；落点会用路网吸附，
    # 避免像第一次那样落进绿化带（虽然能开出来，但很容易卡住）。
    if getattr(args, "ugv_start", None):
        try:
            import control_ugv as _cu2
            _carla = _cu2.SN.carla      # carla 由 swarmae SDK 提供，不是独立包
            _tx, _ty = [float(s) for s in args.ugv_start.split(",")[:2]]
            # 落点必须离任务区边界 ≥80 m：之前停在 x=-770（区域西边界）时，
            # 两车被挤在边界道路上互相顶死，路径存在但一步也走不动。
            if tactics is not None:
                _z = tactics.zone
                _mgn = 80.0
                _tx = min(max(_tx, _z.x0 + _mgn), _z.x1 - _mgn)
                _ty = min(max(_ty, _z.y0 + _mgn), _z.y1 - _mgn)
            _px = [v.get_state().pose_.x_ for v in ugvs]
            _py = [v.get_state().pose_.y_ for v in ugvs]
            _dx = _tx - sum(_px) / len(_px)
            _dy = _ty - sum(_py) / len(_py)
            _snapped = 0
            for _v in ugvs:
                _p = _v.get_state().pose_
                _gx, _gy = _p.x_ + _dx, _p.y_ + _dy
                try:
                    _ppx, _ppy = ugv.w2p(_gx, _gy, ugv_scale, ugv_off)
                    _nx, _ny = ugv.nearest_road(binary, _ppx, _ppy, search=40)
                    _gx, _gy = ugv.p2w(_nx, _ny, ugv_scale, ugv_off)
                    _snapped += 1
                except Exception:
                    pass
                _actor = _v.Vehicle_._SwarmAENode__actor
                _actor.set_transform(_carla.Transform(
                    _carla.Location(_gx, _gy, float(_p.z_)),   # 用原高度，别再写死 2.1（会插进路面/建筑）
                    _carla.Rotation(pitch=0.0, yaw=float(_p.theta_), roll=0.0)))
            log("已把无人车平移到 (%.0f, %.0f)（Δ=%.0f, %.0f，%d 台已吸附到路面）"
                % (_tx, _ty, _dx, _dy, _snapped))
            time.sleep(1.5)
            for _v in ugvs:
                _v.apply_control(0, 0, 0, True)
        except Exception as _e:
            log("无人车起始平移失败（继续用默认出生点）：%r" % (_e,))

    try:
        _sp = []
        for _v in ugvs:
            _p = _v.get_state().pose_
            _sp.append("%s(%.0f,%.0f)" % (getattr(_v, "name", "?"), _p.x_, _p.y_))
        log("无人车初始位置：%s" % "  ".join(_sp))
    except Exception as _e:
        log("读取无人车初始位置失败：%r" % (_e,))

    # ---------------- 小地图 ----------------
    names = list(UGV_NAMES) + list(cu.UAV_NAMES)
    types = dict([(n, "ugv") for n in UGV_NAMES] + [(n, "uav") for n in cu.UAV_NAMES])
    view = None
    if not args.no_view:
        focus = None if args.focus == "0" else args.focus

        roi = search_roi if args.view_roi == "auto" else None
        if args.view_roi == "auto" and roi is None:
            xs, ys = [], []
            for n in cu.UAV_NAMES:
                try:
                    p = uav_vehicles[n].position()
                    xs.append(float(p[0]))
                    ys.append(float(p[1]))
                except Exception:
                    pass
            # 只用"离无人机出生点足够近"的敌人：场景里往往还残留远处的旧靶标，
            # 全都算进去的话视角会被撑回全城，看不到细节。
            # 无人车出生点也纳入，否则车的起点可能被裁在视野外
            try:
                for v in ugvs:
                    st = v.get_state().pose_
                    xs.append(float(st.x_))
                    ys.append(float(st.y_))
            except Exception:
                pass

            if xs and ys:
                base_x = sum(xs) / float(len(xs))
                base_y = sum(ys) / float(len(ys))
                near = [m for m in markers
                        if math.hypot(m.x - base_x, m.y - base_y) <= args.view_roi_range]
                if len(near) < len(markers):
                    log("ROI 只统计出生点 %.0f m 内的 %d 个目标（共 %d 个）"
                        % (args.view_roi_range, len(near), len(markers)))
                for m in near:
                    xs.append(float(m.x))
                    ys.append(float(m.y))
            if xs and ys:
                cx = (min(xs) + max(xs)) / 2.0
                cy = (min(ys) + max(ys)) / 2.0
                half = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0
                half = max(args.view_roi_min, half + args.view_roi_margin)
                if args.view_roi_max > 0:
                    half = min(half, args.view_roi_max)
                # 视野别越出地图（否则一半是黑边）
                try:
                    ax, ay = scene.to_world(0, 0)
                    bx, by = scene.to_world(scene.w, scene.h)
                    lo_x, hi_x = min(ax, bx), max(ax, bx)
                    lo_y, hi_y = min(ay, by), max(ay, by)
                    if hi_x - lo_x > 2 * half:
                        cx = min(max(cx, lo_x + half), hi_x - half)
                    if hi_y - lo_y > 2 * half:
                        cy = min(max(cy, lo_y + half), hi_y - half)
                except Exception:
                    pass
                roi = (cx, cy, half)
                log("小地图锁定任务区域：中心 (%.0f, %.0f)  ±%.0f m"
                    % (cx, cy, half))

        view = lt.LiveView(scene, env, names, types, zoom=320.0, focus=focus,
                           title="full_mission - recon -> strike", roi=roi)
        view.set_markers(markers)
        try:
            view.scan_radius_px = float(args.sweep_radius)
        except Exception:
            pass
        # 把本轮真正的搜索区域画出来（青绿实线），与场景边界区分
        # 战术层启用时，小地图框直接取任务区（Zone），保证车不会"跑出画面"
        if tactics is not None:
            try:
                _z = tactics.zone
                _cx, _cy = _z.center()
                _half = max(_z.size()) / 2.0
                search_roi = (_cx, _cy, _half)
            except Exception:
                pass
        if search_roi is not None:
            view.set_search_area(*search_roi)
            log("小地图：青绿实线框 = 本轮搜索区域 ±%.0f m" % search_roi[2])
        log("小地图已打开：1-6 聚焦 / 0 总览 / +/- 缩放 / s 存图")

    # ---------------- 状态 ----------------
    ugv_agents = []
    ugv_targets = {}
    yield_until = {}           # 车名 -> 让行截止时间
    go_at = {}                 # 车名 -> 允许起步的时间（错峰发车）

    ugv_started_at = None
    recon_started_at = time.time()
    last_view = 0.0
    frame = 0
    uav_ok = True
    uav_strike_done = False
    uav_strike_diag = False
    shot_done = False
    last_attack = 0.0
    last_contact = 0.0
    attacked_pts = []
    ugv_trails = dict((n, []) for n in UGV_NAMES)
    killed_pts = []
    last_log_encircle = 0.0
    last_watchdog = 0.0
    last_dbg = 0.0
    ugv_stuck = {}
    prev_uav_pos = None
    last_progress = time.time()
    engaged_pts = []
    last_seen = []
    exit_reason = "运行到 --duration 上限"

    log("开始：无人机起飞侦查（Z=%.0f，%.0f m/s，模式 %s）"
        % (cu.ALTITUDE, cu.CRUISE_SPEED, args.uav_mode))
    last_beat = 0.0
    takeoff_warned = False

    try:
        while True:
            t0 = time.time()
            frame += 1
            el = t0 - recon_started_at

            if el > args.duration:
                break

            # ---- 0. 控制台心跳（没有它，一切正常时控制台看起来是"卡住"的）----
            if args.console_every > 0 and (t0 - last_beat) >= args.console_every:
                last_beat = t0
                if getattr(runner, "virtual", False):
                    zs = [round(runner.virtual_pose(n)[2], 1) for n in cu.UAV_NAMES]
                    vs = [0.0] * len(cu.UAV_NAMES)
                else:
                    try:
                        zs = [round(uav_vehicles[n].position()[2], 1)
                              for n in cu.UAV_NAMES]
                    except Exception:
                        zs = ["?"] * len(cu.UAV_NAMES)
                    try:
                        vs = [round(uav_vehicles[n].Vehicle_.get_velocity()[3], 1)
                              for n in cu.UAV_NAMES]
                    except Exception:
                        vs = ["?"] * len(cu.UAV_NAMES)
                phases = runner.phases_snapshot()
                log("t=%5.0fs  机高=%s/%.0f  速度=%s  阶段=%s  已发现=%d  车=%s"
                    % (el, zs, cu.ALTITUDE, vs,
                       ",".join(phases[n][:4] for n in cu.UAV_NAMES),
                       len(runner.enemies()),
                       ",".join("%s:%s" % (a.name, a.status) for a in ugv_agents)
                       or "待命"))

            # ---- 0b. 起飞超时自检：飞控驱动没跑时最常见的症状 ----
            if (not takeoff_warned) and uav_ok and el > args.takeoff_timeout \
                    and runner.all_takeoff():
                takeoff_warned = True
                try:
                    zs = [round(uav_vehicles[n].position()[2], 1)
                          for n in cu.UAV_NAMES]
                except Exception:
                    zs = "?"
                log("=" * 62)
                log(".. 无人机 %.0fs 仍在起飞阶段，高度 %s（目标 %.0f m）"
                    % (el, zs, cu.ALTITUDE))
                log("   先别急着怀疑飞控：SITL 上电后要先等 GPS home + EKF 收敛才 arm，")
                log("   实测 20~60 s，这段时间命令返回 200 但飞机就是不动，属正常。")
                log("   判断依据在驱动日志里（bin/driver/log/*.log）：出现 'Arming motors' 就说明")
                log("   已经起飞；出现 'PreArm: GPS: waiting for home' 就再等一会。")
                log("   若超过 4 分钟仍为 0 位移，再查：" + DRIVER_HINT)
                log("=" * 62)
            if takeoff_warned and uav_ok and el > 3.0 * args.takeoff_timeout \
                    and runner.all_takeoff():
                log("无人机长时间无法起飞，停止飞行（若已有目标，后续流程照常）")
                uav_ok = False
                try:
                    runner.stop()
                except Exception:
                    pass

            # ---- 1. 无人机一帧 ----
            if uav_ok:
                try:
                    runner.step()
                except cu.SafetyError as e:
                    uav_ok = False
                    log("无人机触发安全边界，停止飞行：%s" % e)
                    runner.stop()

            # ---- 2. 同步"已发现"到小地图 ----
            enemies = runner.enemies()
            # 【修正】飞机还在爬升时就别"官宣发现目标"：SDK 在地面也能报出附近敌人，
            # 结果出现"无人机还在起飞、无人车已经出发"的怪象。
            airborne = _all_uavs_at_altitude(uav_vehicles, tol=30.0)
            if enemies and markers and airborne:
                det = [{"x": p[0], "y": p[1]} for p in enemies]
                # 【修正】只用 20 m 阈值匹配经常一个都对不上（SDK 报回的坐标
                # 与 targets.json 有偏差），结果"任务已经发现 3 个"但小地图还是空心圈。
                # 这里放宽到 80 m，并且对"没匹配上的探测"退化为"标记最近的未发现目标"，
                # 保证小地图计数与任务计数一致。
                found, _ = lt.match_detections(det, markers, tol=max(args.match_tol, 80.0))
                if not found:
                    for d in det:
                        best_i, best_d = -1, None
                        for i, m in enumerate(markers):
                            if m.discovered or m.extra:
                                continue
                            dd = math.hypot(m.x - d["x"], m.y - d["y"])
                            if best_d is None or dd < best_d:
                                best_i, best_d = i, dd
                        if best_i >= 0:
                            found.add(best_i)
                changed = False
                for i in found:
                    if not markers[i].discovered:
                        markers[i].discovered = True
                        changed = True
                        log("发现目标 #%d %s (%.1f, %.1f)"
                            % (i, markers[i].cn, markers[i].x, markers[i].y))
                        # 【分类前移】发现即判定路口，黄圈不用等到派车才出现
                        if tactics is not None:
                            try:
                                _mk = _tactics_mask()
                                if _mk is not None and _tactics_slots is not None:
                                    _kd, _br = _tactics_slots.classify(
                                        _mk, (markers[i].x, markers[i].y))
                                    if _kd in ("junction", "tee") and len(_br) >= 2:
                                        _sl = _tactics_slots.build_slots(
                                            _mk, (markers[i].x, markers[i].y), _br)
                                        if len(_sl) >= 2:
                                            _key = "%d_%d" % (int(markers[i].x),
                                                              int(markers[i].y))
                                            _TACTICS_ENCIRCLE[_key] = (
                                                markers[i].x, markers[i].y, len(_sl))
                                            log("[分类] (%.0f,%.0f) -> %s 阵位=%d"
                                                % (markers[i].x, markers[i].y,
                                                   _kd, len(_sl)))
                            except Exception:
                                pass
                if changed and view is not None:
                    view.known_override = len(enemies)
            view.discovered_override = len(enemies)
            view.set_markers(markers)
            if _TACTICS_ENCIRCLE:
                try:
                    view.set_encircle([(v[0], v[1]) for v in _TACTICS_ENCIRCLE.values() if v[2] >= 2])
                except Exception:
                    pass

            # 记录每台车走过的点，供"重复路径代价"使用（每 15 m 一个采样点）
            try:
                for _i, _v in enumerate(ugvs):
                    _p = _v.get_state().pose_
                    _tr = ugv_trails[UGV_NAMES[_i]]
                    if (not _tr) or math.hypot(_p.x_ - _tr[-1][0], _p.y_ - _tr[-1][1]) > 15.0:
                        _tr.append((float(_p.x_), float(_p.y_)))
                        if len(_tr) > 400:
                            del _tr[0:100]
            except Exception:
                pass
            # 【进度判定】只要还有无人机在移动（>3 m），就认为任务在推进，不触发"侦查超时"
            try:
                _cur = [uav_vehicles[n].position() for n in cu.UAV_NAMES]
                if prev_uav_pos is None or any(
                        math.hypot(c[0] - p[0], c[1] - p[1]) > 0.05
                        for c, p in zip(_cur, prev_uav_pos)):
                    last_progress = t0
                prev_uav_pos = _cur
            except Exception:
                pass
            # ---- 3. 无人车持续分配（一辆打完立刻接下一个目标）----
            # 结束的车把车辆释放出来，供下一个目标使用
            for a in list(ugv_agents):
                if a.status in ("arrived", "no_path", "stuck", "max_replan"):
                    ugv_agents.remove(a)
                    continue
                # 【新增】目标已经被打死了就立即就地释放，不必再开到那个点。
                # 判定收紧到 12 m：之前 40 m 太宽，1 号车打死一个敌人时，
                # 会把旁边路过的 2、3 号车也一起"释放"（表现为突然转向后停死）。
                if any(math.hypot(a.target[0] - q[0], a.target[1] - q[1]) < 12.0
                       for q in killed_pts):
                    try:
                        a.finish("arrived", "目标已被消灭，就地待命")
                    except Exception:
                        pass
                    uw = ugv_targets.pop(a.name, None)   # 清掉残留目标，避免小地图显示假目标
                    log("%s 的目标已被消灭，就地转为空闲" % a.name)
                    ugv_agents.remove(a)
            # ---- 调试：每 5 s 打印三车状态（定位卡死用）----
            if tactics is not None and (t0 - last_dbg) > 5.0:
                last_dbg = t0
                _dbg = []
                for _a in ugv_agents:
                    try:
                        _p = _a.ugv.get_state().pose_
                        _dbg.append("%s[%s] p(%.0f,%.0f) t(%.0f,%.0f) n=%d"
                                    % (_a.name, _a.status, _p.x_, _p.y_,
                                       _a.target[0], _a.target[1],
                                       len(getattr(_a, "path", []) or [])))
                    except Exception:
                        pass
                if _dbg:
                    log("[状态] " + " | ".join(_dbg))
            free_idx = [i for i, n in enumerate(UGV_NAMES)
                        if all(a.name != n for a in ugv_agents)]
            # ---- 看门狗：空闲且掉到任务区外的车，拉回路网（防止整轮卡死）----
            if tactics is not None and (t0 - last_watchdog) > 3.0:
                last_watchdog = t0
                for _k, _nm in enumerate(UGV_NAMES):
                    if any(_a.name == _nm for _a in ugv_agents):
                        continue
                    try:
                        _pose = ugvs[_k].get_state().pose_
                        if tactics.zone.contains(_pose.x_, _pose.y_, margin=5.0):
                            continue
                        _ppx, _ppy = ugv.w2p(_pose.x_, _pose.y_, ugv_scale, ugv_off)
                        _nx, _ny = ugv.nearest_road(binary, _ppx, _ppy, search=80)
                        _wx, _wy = ugv.p2w(_nx, _ny, ugv_scale, ugv_off)
                        if tactics.zone.contains(_wx, _wy, margin=0.0):
                            import control_ugv as _cu3
                            _c3 = _cu3.SN.carla
                            _act = ugvs[_k].Vehicle_._SwarmAENode__actor
                            _act.set_transform(_c3.Transform(
                                _c3.Location(_wx, _wy, 2.1),
                                _c3.Rotation(pitch=0.0, yaw=float(_pose.theta_), roll=0.0)))
                            ugvs[_k].apply_control(0, 0, 0, True)
                            log("%s 在区域外，已拉回 (%.0f,%.0f)" % (_nm, _wx, _wy))
                    except Exception:
                        pass
            try:
                scanned = all(p == "hold" for p in runner.phases_snapshot().values())
            except Exception:
                scanned = False
            if (not args.no_ugv) and free_idx:
                # 【修正】候选池只排除"已打死"和"正在被某车打"的点。
                # 原来用 attacked_pts 永久排除：车派过之后（哪怕卡住没打成），这个敌人
                # 就再也不会被派第二次 —— 实测出现"已发现 5、只打死 3、车全部待命"。
                busy_now = [a.target for a in ugv_agents]
                pool = [p for p in enemies
                        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 40.0
                               for q in killed_pts)
                        and all(math.hypot(p[0] - q[0], p[1] - q[1]) > 40.0
                                for q in busy_now)]
                # 【未来更优解】若某辆"还在路上"的车，目标离该敌人很近（<150 m），先别把这个
                # 敌人派给远处的空闲车——等那辆车到了顺手打掉，避免"A 车跑远路、B 车就在旁边"。
                if pool and ugv_agents:
                    busy_targets = [a.target for a in ugv_agents]
                    keep = [e for e in pool
                            if not any(math.hypot(t[0] - e[0], t[1] - e[1]) < 150.0
                                       for t in busy_targets)]
                    if keep:
                        pool = keep
                # 【逐敌派车】只要有"未分配敌人 + 空闲车 + 飞机已在空中"就派，
                # 不再等凑够 3 个、也不等扫描结束；否则一辆车打完要干等到扫完才能接下一个。
                ready = bool(pool) and _all_uavs_at_altitude(uav_vehicles, tol=30.0)

                # 无人机全死了/全飞完了，就别再空等 —— 有几个目标派几个
                if (not ready) and len(pool) > 0:
                    try:
                        # 【修正】all_searching() 的含义是"三架都在空中"(cruise/hold)，
                        # 不是"扫完了"。原写法导致起飞阶段就判定"没更多目标"，
                        # 一发现 1 个敌人就急着派车。现在要求三架都进入 hold 才算扫完。
                        uav_done = runner.all_searching() and all(
                            p == "hold" for p in runner.phases_snapshot().values())
                        dead = getattr(runner, "dead", set())
                        if dead and len(dead) >= len(cu.UAV_NAMES):
                            uav_done = True
                    except Exception:
                        uav_done = False
                    if uav_done:
                        log("无人机已无更多可侦查目标，现有 %d 个目标就派车"
                            "（不足 --min-targets %d）" % (len(pool), args.min_targets))
                        ready = True

                # 【改】超时判据 = 连续 recon_timeout 秒"既没发现新目标、无人机也没动"
                # 否则长航时巡查（半图要 10+ 分钟）会被误判成卡死
                if ((not ready) and (not pool) and (not scanned)
                        and (t0 - last_progress) > args.recon_timeout
                        and runner.all_takeoff()):   # 只有"从没起飞成功"才判超时
                    if args.use_default_enemies:
                        log("侦查超时（%.0fs），补入组员的预设目标点" % el)
                        for p in cu.DEFAULT_ENEMIES:
                            if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 20.0
                                   for q in pool):
                                pool.append(tuple(p))
                        ready = len(pool) >= args.min_targets
                    else:
                        log("侦查超时（%.0fs）且只发现 %d 个目标（要 %d 个），退出。"
                            "可加 --use-default-enemies 兜底"
                            % (el, len(pool), args.min_targets))
                        exit_reason = "侦查超时"
                        break

                if ready and el >= args.ugv_after:
                    pts = snap_to_road(pool, binary, ugv_scale, ugv_off)
                    if not pts:
                        log("目标都吸附不到路网，暂不派车")
                    else:
                        starts_xy = [(ugvs[k].get_state().pose_.x_,
                                      ugvs[k].get_state().pose_.y_) for k in free_idx]
                        headings = [ugvs[k].get_state().pose_.theta_ for k in free_idx]
                        # ---- 战术层（可选）：由 planner 决定每台车去哪 ----
                        plan_targets = None
                        if tactics is not None:
                            try:
                                plan_targets = tactics_decide(
                                    tactics, t0, free_idx, pool, ugvs,
                                    uav_vehicles, args.sweep_radius,
                                    binary=binary, scale=ugv_scale, off=ugv_off)
                            except Exception as _e:
                                log("战术层求解失败，退回贪心：%r" % (_e,))
                                plan_targets = None
                        if plan_targets:
                            chosen = [plan_targets.get(UGV_NAMES[k]) for k in free_idx]
                            chosen = [p for p in chosen if p is not None]
                        else:
                            chosen = assign_targets(pts, starts_xy, len(free_idx),
                                                    trails=[ugv_trails[UGV_NAMES[k]] for k in free_idx],
                                                    headings=headings)
                        # 【修正】目标点少于车数时，不要几台车挤同一个点（会互相挡死）。
                        # 去重后"有几处目标就派几台车"，剩下的车原地待命。
                        uniq = []
                        for p in chosen:
                            if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 25.0
                                   for q in uniq):
                                uniq.append(p)
                        if len(uniq) < len(chosen):
                            log("目标点去重：%d -> %d 处，只派 %d 台车（其余待命）"
                                % (len(chosen), len(uniq), len(uniq)))
                        for slot, k in enumerate(free_idx):
                            if slot >= len(uniq):
                                break
                            name = UGV_NAMES[k]
                            # （A* 预检曾在此，因对未吸附起点会误判而移除）
                            ugv_targets[name] = uniq[slot]
                            attacked_pts.append(uniq[slot])
                            a = ugv.UgvAgent(ugvs[k], name, uniq[slot],
                                             binary, raw_bin, ugv_scale, ugv_off)
                            a.start()
                            ugv_agents.append(a)
                        ugv_started_at = time.time()
                        for i, nm in enumerate(UGV_NAMES):
                            go_at[nm] = ugv_started_at + i * max(0.0, args.dispatch_gap)
                        if args.dispatch_gap > 0:
                            log("错峰发车：每台间隔 %.0f s" % args.dispatch_gap)
                        if not args.no_face_target:
                            face_targets(ugvs, ugv_agents)
                        # 同步写进 goals.json，小地图星标与实际目标一致
                        merge_goals_atomic(ugv_targets)
                        log("已派车：%s" % dict(
                            (k, (round(v[0], 1), round(v[1], 1)))
                            for k, v in ugv_targets.items()))
                        if len(pts) < len(UGV_NAMES):
                            log("注意：只有 %d 个可用目标点，部分无人车会被派到同一处"
                                % len(pts))
                        # 无人机改飞由 3b 段统一处理（与是否派车无关）

            # ---- 3b. 无人机改飞已发现目标（与派车无关，单独触发）----
            if args.uav_strike and args.uav_mode == "kinetic" and not uav_strike_done:
                gate_num = len(enemies) >= args.min_targets
                gate_alt = _all_uavs_at_altitude(uav_vehicles) or runner.all_searching()
                if not uav_strike_diag:
                    uav_strike_diag = True
                    log("[strike] 门限诊断：已发现=%d（需 %d） 全部到巡航高度=%s 阶段=%s"
                        % (len(enemies), args.min_targets, gate_alt,
                           runner.phases_snapshot()))
                if gate_num and gate_alt:
                    try:
                        # 一个目标只派一架机：目标少于飞机时，其余飞机继续自己的巡查，
                        # 绝不能三架挤同一个点（实测到点瞬间互相撞、速度飙到 23 m/s）。
                        remaining = [tuple(p[:2]) for p in enemies]
                        plan = {}
                        for name in cu.UAV_NAMES:
                            if not remaining:
                                break
                            here = uav_vehicles[name].position()
                            best = min(remaining,
                                       key=lambda p: math.hypot(p[0] - here[0],
                                                                p[1] - here[1]))
                            remaining.remove(best)
                            plan[name] = [best]
                        if plan:
                            cu.request_retarget(plan)
                            log("无人机改飞目标：%s" % dict(
                                (n, (round(p[0][0], 1), round(p[0][1], 1)))
                                for n, p in plan.items()))
                        uav_strike_done = True
                    except Exception as e:
                        log("无人机改飞失败：%r" % (e,))

            # ---- 4. 无人车一帧（含会车让行）----
            if ugv_agents:
                paused = set()
                # 组员的避障算法只处理"前方有障碍"，没考虑**两车相向而行**：
                # 这时双方都判定"路被挡"，一个倒车、一个前进，就会撞上。
                # 这里在调度层加一条让行规则：两车进入 YIELD_GAP 内，
                # 让"离自己目标更远"的那台刹停 YIELD_HOLD 秒，另一台先过。
                try:
                    pos = {}
                    for a in ugv_agents:
                        st = a.ugv.get_state().pose_
                        pos[a.name] = (float(st.x_), float(st.y_))
                    now_t = time.time()
                    for i in range(len(ugv_agents)):
                        for j in range(i + 1, len(ugv_agents)):
                            a, b = ugv_agents[i], ugv_agents[j]
                            # 【修正】已经到点/上报的车不会再动，别让还在跑的车给它让行。
                            # 原来不管对方状态，只要距离 < YIELD_GAP 就判让行，
                            # 实测 UGV3 一直给已经停在旁边的 UGV2 让行，永远走不了。
                            if a.status != "drive" or b.status != "drive":
                                continue
                            if a.name not in pos or b.name not in pos:
                                continue
                            d = math.hypot(pos[a.name][0] - pos[b.name][0],
                                           pos[a.name][1] - pos[b.name][1])
                            if d >= YIELD_GAP:
                                continue
                            ra = math.hypot(a.target[0] - pos[a.name][0],
                                            a.target[1] - pos[a.name][1])
                            rb = math.hypot(b.target[0] - pos[b.name][0],
                                            b.target[1] - pos[b.name][1])
                            loser = a if ra > rb else b
                            if yield_until.get(loser.name, 0.0) < now_t:
                                yield_until[loser.name] = now_t + YIELD_HOLD
                                log("%s 与 %s 相距 %.0f m，%s 让行 %.0f s"
                                    % (a.name, b.name, d, loser.name, YIELD_HOLD))
                    for name, until in list(yield_until.items()):
                        if until > now_t:
                            paused.add(name)
                        else:
                            yield_until.pop(name, None)
                except Exception as e:
                    log("让行判定异常（忽略，继续行驶）：%r" % (e,))

                t_now = time.time()
                for a in ugv_agents:
                    if t_now < go_at.get(a.name, 0.0):
                        continue          # 还没轮到它出发
                    if a.name in paused:
                        try:
                            a.ugv.apply_control(0.0, 0.0, 1.0, False)   # 刹停
                        except Exception:
                            pass
                        continue
                    try:
                        a.step()
                    except Exception as e:
                        log("%s 步进异常：%r" % (a.name, e))

            # ---- 5. 小地图 ----
            # 按"时间间隔"刷新：帧率抖动时也不会画得过勤
            _due = (t0 - last_view) >= max(0.05, args.view_interval)
            if args.view_every is not None:
                _due = _due and (frame % max(1, args.view_every) == 0)
            if view is not None and _due:
                snap = {}
                for name, v in zip(UGV_NAMES, ugvs):
                    try:
                        snap[name] = pose_of(name, "ugv", v.Vehicle_)
                    except Exception:
                        pass
                for name in cu.UAV_NAMES:
                    try:
                        if getattr(runner, "virtual", False):
                            snap[name] = pose_of_virtual(name, runner)
                        else:
                            snap[name] = pose_of(name, "uav",
                                                 uav_vehicles[name].Vehicle_)
                    except Exception:
                        pass
                goals = {}
                # 两种 runner 都提供 last_goals；用 getattr 兜底，避免显示层因
                # 缺属性把整条链路带崩（运动学 runner 就曾漏过这个字段）
                uav_goals = getattr(runner, "last_goals", {}) or {}
                for n in cu.UAV_NAMES:
                    g = uav_goals.get(n)
                    if g:
                        goals[n] = (g[0], g[1])
                goals.update(ugv_targets)
                srcs = dict((n, "file" if n in ugv_targets else "nearest")
                            for n in goals)
                view.set_goals(goals, srcs)

                status = []
                for a in ugv_agents:
                    status.append("%s:%s" % (a.name, a.status))
                extra = "UAV:%s 已发现 %d | %s" % (
                    "OK" if uav_ok else "STOP",
                    len(enemies),
                    " ".join(status) if status else "等待侦查")
                view.update(snap, el, extra)
                # 到点自动存一张小地图（不用手按 s，也不用屏幕截图）
                if args.shot_at > 0 and el >= args.shot_at and not shot_done:
                    try:
                        path = view.save_snapshot()
                        shot_done = True
                        log("已保存小地图快照: %s" % path)
                    except Exception as e:
                        shot_done = True
                        log("小地图快照保存失败: %r" % (e,))
                last_view = t0

            # ---- 5a. 判定"已消灭"：打过、且现在探测不到了 ----
            if engaged_pts and markers and last_seen:
                alive = list(last_seen)
                still = []
                for q in engaged_pts:
                    near = any(math.hypot(q[0] - a0, q[1] - a1) < 40.0
                               for (a0, a1) in alive)
                    if near:
                        still.append(q)
                    else:
                        for m in markers:
                            if (not m.extra
                                    and math.hypot(m.x - q[0], m.y - q[1]) < 40.0):
                                m.extra = True
                                log("已消灭：%s (%.1f, %.1f)" % (m.cn, m.x, m.y))
                engaged_pts = still
                if view is not None:
                    view.set_markers(markers)
            # ---- 5b. 到点的无人车开火 ----
            if args.attack and ugv_agents and t0 - last_attack > 2.0:
                last_attack = t0
                seen_pts = []
                for a in ugv_agents:
                    if a.status not in ("arrived", "drive", "reverse"):
                        continue
                    try:
                        ob = a.ugv.detect_fun()
                    except Exception:
                        continue
                    try:
                        for kk in range(len(list(getattr(ob, "Type", []) or []))):
                            seen_pts.append((ob.x_ue[kk], ob.y_ue[kk]))
                    except Exception:
                        pass
                    types = list(getattr(ob, "Type", []) or [])
                    if not types:
                        continue
                    # 【重要】detect_fun 会把"自己人"也算成障碍物：HRT-M10 是我们自己的无人机、
                    # RaiderBuggy 是队友车。不过滤的话车会对着头顶的无人机打（实测就是这个现象）。
                    HOSTILE = ("Soldier", "Stryker", "Barricade_BP", "Pillbox", "Building")
                    cand = [k for k in range(len(types)) if types[k] in HOSTILE]
                    # 障碍物（沙袋/工事/建筑）挡路时，按"离车当前距离"选，不是按目标点距离
                    try:
                        _here = a.ugv.get_state().pose_
                        _hx, _hy = float(_here.x_), float(_here.y_)
                    except Exception:
                        _hx = _hy = None
                    if not cand:
                        continue
                    best, bd = -1, None
                    for k in cand:
                        try:
                            if (types[k] in ("Barricade_BP", "Pillbox", "Building")
                                    and _hx is not None):
                                d = math.hypot(ob.x_ue[k] - _hx, ob.y_ue[k] - _hy)
                            else:
                                d = math.hypot(ob.x_ue[k] - a.target[0],
                                               ob.y_ue[k] - a.target[1])
                        except Exception:
                            continue
                        if bd is None or d < bd:
                            best, bd = k, d
                    if best < 0 or bd is None or bd > args.attack_range:
                        continue
                    # ---- 合围规则：路口目标必须"到齐"才允许开火 ----
                    if tactics is not None and _TACTICS_ENCIRCLE:
                        try:
                            _need = None
                            for _tid, _ev in _TACTICS_ENCIRCLE.items():
                                if math.hypot(ob.x_ue[best] - _ev[0],
                                              ob.y_ue[best] - _ev[1]) < 60.0:
                                    _need = _ev[2]
                                    break
                            if _need:
                                _ready = 0
                                for _a in ugv_agents:
                                    try:
                                        _pose = _a.ugv.get_state().pose_
                                        _dp = math.hypot(_pose.x_ - _a.target[0],
                                                         _pose.y_ - _a.target[1])
                                        _dt = math.hypot(_a.target[0] - ob.x_ue[best],
                                                         _a.target[1] - ob.y_ue[best])
                                        if _dp < 30.0 and _dt < 60.0:
                                            _ready += 1
                                    except Exception:
                                        pass
                                if _ready < _need:
                                    if t0 - last_log_encircle > 5.0:
                                        last_log_encircle = t0
                                        log("%s 等待合围：就位 %d/%d，暂不开火"
                                            % (a.name, _ready, _need))
                                    continue
                        except Exception:
                            pass
                    try:
                        a.ugv.set_weapon(ob, best)
                        last_contact = t0
                        # 记下"打过这个点"，等它从探测列表消失再判死
                        # （开火当帧血量往往还没归零，原来的判据几乎永远不成立）
                        engaged_pts.append((ob.x_ue[best], ob.y_ue[best]))
                        try:
                            hp = list(getattr(ob, "Health", []) or [])
                            if best < len(hp) and float(hp[best]) <= 0:
                                for m in markers:
                                    if (not m.extra
                                            and math.hypot(m.x - ob.x_ue[best],
                                                           m.y - ob.y_ue[best]) < 40.0):
                                        m.extra = True
                                if view is not None:
                                    view.set_markers(markers)
                        except Exception:
                            pass
                        log("%s 开火 -> %s (%.1f, %.1f) 距离 %.0f m"
                            % (a.name, types[best], ob.x_ue[best], ob.y_ue[best], bd))
                    except Exception as e:
                        log("%s 开火失败：%r" % (a.name, e))

                last_seen = seen_pts     # 供 5a 判死使用
                if engaged_pts:
                    alive = list(last_seen)
                    still2 = []
                    for q in engaged_pts:
                        if any(math.hypot(q[0] - a0, q[1] - a1) < 40.0 for (a0, a1) in alive):
                            still2.append(q)
                        else:
                            for m in markers:
                                if (not m.extra and math.hypot(m.x - q[0], m.y - q[1]) < 40.0):
                                    m.extra = True
                                    killed_pts.append((m.x, m.y))
                                    log("已消灭：%s (%.1f, %.1f)" % (m.cn, m.x, m.y))
                    engaged_pts = still2
                    if view is not None:
                        view.set_markers(markers)
            # ---- 6. 结束判定 ----
            # 【新增】只要"已发现的敌人全部被打死"，就不必再等扫描收尾，直接结束。
            # 【关键】必须打光场景里的全部敌人才算完成；只打光"已发现"的，会在还剩 5 个没找到时误结束。
            _enough = (expected_enemies <= 0) or (len(killed_pts) >= expected_enemies)
            all_killed = _enough and bool(enemies) and all(
                any(math.hypot(p[0] - q[0], p[1] - q[1]) < 80.0 for q in killed_pts)
                for p in enemies)
            if all_killed and (t0 - last_contact) > 5.0:
                exit_reason = ("任务完成：已发现的 %d 个敌人全部消灭"
                               "（最近一次交火 %.0f s 前，未等扫描收尾）"
                               % (len(enemies), t0 - last_contact))
                break
            if ugv_agents and all(a.status in ("arrived", "no_path", "stuck",
                                               "max_replan") for a in ugv_agents):
                ugv_idle = True
            else:
                ugv_idle = False
            uav_scanned = all(p in ("hold", "dead")
                              for p in runner.phases_snapshot().values())
            if ugv_idle and uav_scanned and (t0 - last_contact) > 10.0:
                exit_reason = ("任务完成：全区域扫描完毕，已发现目标均已消灭"
                               "（最近一次交火 %.0f s 前）" % (t0 - last_contact))
                break
            if ugv_agents and args.ugv_deadline > 0:
                ran = t0 - (ugv_started_at or t0)
                if ran > args.ugv_deadline:
                    exit_reason = ("无人车超过 --ugv-deadline %.0f s，收尾"
                                   % args.ugv_deadline)
                    log(exit_reason)
                    break

            # ---- 节拍 ----
            # 运动学模式下位姿靠 set_transform 一帧帧"搬"，帧率越高越顺，
            # 所以用更短的 tick；真机模式仍按协议 50 ms 走。
            tick = KINEMATIC_TICK if args.uav_mode in ("kinematic", "virtual") else cu.DT
            sleep = tick - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        exit_reason = "用户中断（Ctrl+C）"
    except Exception:
        exit_reason = "异常退出"
        traceback.print_exc()
    finally:
        log("结束原因：%s" % exit_reason)
        # 先停掉控制线程（它退出时会发一次"悬停在巡航高度"），再降落；
        # 顺序反过来的话，悬停指令会把刚降下去的飞机又拉回 250 m。
        try:
            runner.stop()
        except Exception:
            pass
        if args.land and args.uav_mode == "kinetic" and uav_ok:
            try:
                time.sleep(1.5)
                log("开始降落（Z %.0f -> 低空 %.0f m）..." % (cu.ALTITUDE, cu.SAFE_LAND_ALT))
                if cu.land_uavs(uav_vehicles, logger):
                    log("三架机已降到安全低空并稳住")
                else:
                    log("降落未在超时内完成，请看无人机日志")
            except Exception as e:
                log("降落失败：%r" % (e,))
        for v in ugvs:
            try:
                v.apply_control(0.0, 0.0, 1.0, False)
            except Exception:
                pass
        if ugv_agents:
            log("无人车汇总：")
            for a in ugv_agents:
                try:
                    p = a.ugv.get_transform()
                    log("  %s 状态=%s 位置=(%.1f, %.1f) 重规划=%d"
                        % (a.name, a.status, p.x_, p.y_, a.replan_count))
                except Exception:
                    pass
        log("已发现目标 %d 个：%s" % (len(runner.enemies()),
                                    [(round(p[0], 1), round(p[1], 1))
                                     for p in runner.enemies()]))
        log("无人机日志：%s" % log_path)
        if view is not None:
            try:
                view.close()
            except Exception:
                pass


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
