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
_LAST_SNAP_LOG = 0.0


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
YIELD_GAP = 22.0           # 两车小于这个距离就触发让行 m（正面相遇要提前反应）
YIELD_HOLD = 8.0           # 让行持续秒数（避免来回抖动）
YIELD_HOLD_HEADON = 12.0   # 正面相遇时让行更久，让对方完成借道超车
ENEMY_STOP_R = 20.0        # 任何车距活敌小于这个距离必须刹停（禁止撞上去）
ENCIRCLE_TIMEOUT = 420.0  # 路口合围等多久还凑不齐就降级（绕行到另一侧可能要 5~7 min）
ENCIRCLE_MAX_SLOTS = 2    # 最多几面同时围；两面对侧翼足够，第三面常要绕 2 km


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
    global _LAST_SNAP_LOG
    out = []
    for p in points:
        px, py = ugv.w2p(float(p[0]), float(p[1]), scale, off)
        pt = ugv.nearest_road(binary, px, py, search=search)
        if pt is None:
            # 节流：吸附失败会在主循环里被反复调用，不节流会每帧刷一行日志
            _now = time.time()
            if _now - _LAST_SNAP_LOG > 5.0:
                _LAST_SNAP_LOG = _now
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

try:
    from tactics import deploy as _tactics_deploy
except Exception:
    _tactics_deploy = None

try:
    from tactics.convoy import ConvoyController as _ConvoyController
except Exception:
    _ConvoyController = None

_TACTICS_MASK = None
_TACTICS_ENCIRCLE = {}   # tid -> (x, y, need_slots, t_first)
_TACTICS_ASSAULT_SINCE = {}  # tid -> 首次被派车/开始合围的时刻（超时降级用）
_TACTICS_WAIT_VIDS = set()   # 本轮到路口阵位、应停车等合围的车
_TACTICS_ENEMY_BY_VID = {}   # 本轮合围车 -> 敌人坐标（用于路径禁止穿过敌人）


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


def road_path(binary, scale, off, start, goal, search_start=40, search_goal=20):
    """A* 世界坐标路径（找不到路点/无解返回 None）。"""
    try:
        sp = ugv.nearest_road(binary, *ugv.w2p(float(start[0]), float(start[1]),
                                               scale, off), search=search_start)
        gp = ugv.nearest_road(binary, *ugv.w2p(float(goal[0]), float(goal[1]),
                                               scale, off), search=search_goal)
        if sp is None or gp is None:
            return None
        pp = ugv.astar(binary, sp, gp)
        if pp is None:
            return None
        return [ugv.p2w(px, py, scale, off) for (px, py) in pp]
    except Exception:
        return None


def route_hits_other(path, others, clearance=15.0):
    """路径是否贴近其它活敌；返回敌人坐标或 None。"""
    if not path or not others:
        return None
    for p in path[:-6]:
        for op in others:
            ox, oy = float(op[0]), float(op[1])
            if math.hypot(p[0] - ox, p[1] - oy) < clearance:
                return (ox, oy)
    return None


def route_violation(path, enemy, slot, clearance=12.0, approach_tol_deg=70.0,
                    max_detour=2.2, detour_slack=250.0, others=None,
                    other_clearance=15.0):
    """检查「去阵位的路线」是否真的从该分支逼近。

    返回 None=合格；否则返回中文原因。判据：
      1. 路径长度不能离谱（绕 1 km 到 200 m 外不算合围，算绕路）；
      2. 路径不能在到达阵位前穿过敌人（clearance 12 m）；
      3. **不能穿过/贴脸其它活敌**（other_clearance 15 m）；
      4. 最后 ~20 m 的走向必须与「从该分支外侧向敌人逼近」一致（±70°）。
    """
    if not path or len(path) < 2:
        return "无路径"
    ex, ey = float(enemy[0]), float(enemy[1])
    sx, sy = float(slot[0]), float(slot[1])
    direct = math.hypot(sx - float(path[0][0]), sy - float(path[0][1]))
    length = 0.0
    for a, b in zip(path, path[1:]):
        length += math.hypot(float(b[0]) - float(a[0]),
                             float(b[1]) - float(a[1]))
    if length > max(max_detour * max(direct, 1.0), direct + detour_slack):
        return "绕行过远（%.0f m / 直线 %.0f m）" % (length, direct)
    # 穿过敌人：最后 6 个点（≈6 m）是阵位本身，排除
    for p in path[:-6]:
        if math.hypot(p[0] - ex, p[1] - ey) < clearance:
            return "路线会穿过敌人"
        if others:
            for op in others:
                ox, oy = float(op[0]), float(op[1])
                if math.hypot(ox - ex, oy - ey) < 30.0:
                    continue          # 同一个目标点
                if math.hypot(p[0] - ox, p[1] - oy) < other_clearance:
                    return "路线经过其他敌人 (%.0f,%.0f)" % (ox, oy)
    end = path[-1]
    ref = path[max(0, len(path) - 20)]
    bear = math.atan2(float(end[1]) - float(ref[1]),
                      float(end[0]) - float(ref[0]))
    slot_dir = math.atan2(sy - ey, sx - ex)
    expected = slot_dir + math.pi
    diff = abs((bear - expected + math.pi) % (2 * math.pi) - math.pi)
    if diff > math.radians(approach_tol_deg):
        return "接近方向不符（%.0f° / 期望 %.0f°）" % (
            math.degrees(diff), math.degrees(expected) % 360.0)
    return None


def _path_length(path):
    if not path or len(path) < 2:
        return 0.0
    return sum(math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
               for a, b in zip(path, path[1:]))


def _initial_turn_penalty(heading, path, weight=450.0):
    """起步大转角惩罚（等效米）。

    实测南侧车被派到北侧阵位时，face_targets 原地转 180° 会卡在路沿
    （速度<0.3 持续 8 s）。分配时把「当前朝向 → 路径首段方向」的转角
    折算成距离，优先选不用掉头的车。
    """
    if heading is None or not path or len(path) < 2:
        return 0.0
    j = min(10, len(path) - 1)
    bear = math.atan2(float(path[j][1]) - float(path[0][1]),
                      float(path[j][0]) - float(path[0][0]))
    diff = abs((float(heading) - bear + math.pi) % (2 * math.pi) - math.pi)
    return float(weight) * diff / math.pi


def best_encircle_match(vids, slots, matrix, max_slots=None):
    """在「车 × 阵位」可行矩阵上选最大匹配，再取总路程最短的。

    matrix[(vid, sid)] = (violation_or_None, path_len)
    优先保证同时占的阵位最多（真多面），其次总路程最短。
    """
    if not vids or not slots:
        return []
    best, best_key = [], None
    maxk = min(len(vids), len(slots))
    if max_slots:
        maxk = min(maxk, int(max_slots))
    for k in range(maxk, 0, -1):
        for vsel in itertools.combinations(vids, k):
            for ssel in itertools.permutations(slots, k):
                ok = True
                total = 0.0
                for vid, s in zip(vsel, ssel):
                    why, length = matrix.get((vid, s.get("sid")),
                                             ("无路径", 0.0))
                    if why is not None:
                        ok = False
                        break
                    total += length
                if not ok:
                    continue
                if best_key is None or total < best_key:
                    best_key = total
                    best = list(zip(vsel, ssel))
        if best:
            return best
    return best


def tactics_decide(tac, now, free_idx, pool, ugvs, uav_vehicles, radius,
                   binary=None, scale=1.0, off=(0.0, 0.0), road_points=None,
                   all_enemies=None):
    """调用战术层：喂覆盖 + 车辆 + 敌人，收回「每台车去哪」。

    返回 {vid: (x, y)}：**只包含通过路线校验的打击任务**。
      * None  = 这一轮没有新计划（调用方可退回贪心）；
      * {}    = 有战术计划但阵位都不可达（调用方应等待，不要贪心硬派）；
      * {...} = 可执行的打击目标。
    """
    _TACTICS_WAIT_VIDS.clear()
    _TACTICS_ENEMY_BY_VID.clear()
    _live_points = [(float(p[0]), float(p[1]))
                    for p in (all_enemies if all_enemies is not None else pool)]
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
                         "heading": float(getattr(pose, "theta_", 0.0)),
                         "commit": "L0"})
    # 3) 敌人（先用普通目标占位，路口阵位由 P2 的 slots 模块接管）
    enemies = []
    _mask = _tactics_mask()
    # 注意：不要整体 clear——每轮只有"未分配"的敌人会重新登记，clear 会把正在合围的条目也抹掉，
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
                        slots_ = [{"sid": s["sid"], "pos": s["pos"], "dir": s.get("dir")} for s in _ss]
                        deg = len(_branches)
                        _old = _TACTICS_ENCIRCLE.get(tid)
                        _t_first = (_old[3] if _old and len(_old) > 3 else float(now))
                        _TACTICS_ENCIRCLE[tid] = (float(p[0]), float(p[1]),
                                                  len(_ss), _t_first)
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
    if road_points is None and binary is not None:
        road_points = build_road_points(binary, scale, off)
    plan = tac.update(now, vehicles, enemies, road_points=road_points)
    if not plan:
        # 防抖（改进<20%）会把「新出现的敌人」一直挡在外面。每 2 s 强制重算一次，
        # 保证无人车自己发现、无人机没扫到的目标也能拿到阵位，而不是退回贪心直冲。
        try:
            _last_force = float(getattr(tac, "_last_force", -999.0))
            if float(now) - _last_force > 2.0:
                tac._last_force = float(now)
                plan = tac.update(now, vehicles, enemies, road_points=road_points,
                                  force=True)
        except Exception:
            plan = None
    if not plan:
        return None
    # 5) 路线感知的阵位重匹配：
    #    分配层只看 ETA，会为了省几秒把西侧的车派到东侧阵位，再让它穿过敌人。
    #    这里对每个路口/丁字目标，用「车 × 阵位」可行矩阵重新做最大匹配：
    #      优先多占阵位（真多面），其次总路程最短；不可行的配对直接剔除。
    enemy_by_tid = {}
    slots_by_tid = {}
    for _tid, _p, _k, _d, _t, _s in enemies:
        enemy_by_tid[_tid] = (float(_p[0]), float(_p[1]))
        slots_by_tid[_tid] = list(_s or [])
    vpos = dict((v["vid"], v["pos"]) for v in vehicles)
    vhead = dict((v["vid"], v.get("heading")) for v in vehicles)
    out = {}
    changed = False
    used_vids = set()
    # 路口/丁字优先，且阵位多的目标先分；路段敌人最后拿剩下的车。
    _groups = sorted(plan.get("groups", []),
                     key=lambda g: (0, -len(g.get("slots", [])))
                     if g.get("kind") in ("junction", "tee") else (1, 0))
    for g in _groups:
        tid = g.get("target_id")
        kind = g.get("kind")
        enemy = enemy_by_tid.get(tid)
        if kind in ("junction", "tee"):
            all_slots = slots_by_tid.get(tid) or []
            if enemy is None or not all_slots:
                g["slots"] = []
                changed = True
                continue
            group_vids = [s["vid"] for s in g.get("slots", [])]
            # 路口优先：未被占用的车都可以补阵位（包括原本要去打路段的）
            extras = [v["vid"] for v in vehicles
                      if v["vid"] not in used_vids and v["vid"] not in group_vids]
            cand_vids = group_vids + extras
            _others = [p for p in _live_points
                       if math.hypot(float(p[0]) - enemy[0],
                                     float(p[1]) - enemy[1]) >= 30.0]
            # 如果直着开过去一定要穿过敌人，就试一次“绕到该分支外侧”的路径：
            # 在本车临时网格里把敌人挖掉，A* 会走别的路绕到阵位。
            _blocked = None
            if binary is not None:
                _blocked = binary.copy()
                _epx, _epy = ugv.w2p(enemy[0], enemy[1], scale, off)
                ugv.block_disk(_blocked, _epx, _epy, 13.0 / max(1e-6, scale))
            matrix = {}
            for vid in cand_vids:
                start = vpos.get(vid)
                if start is None:
                    continue
                for slot in all_slots:
                    path = (road_path(binary, scale, off, start, slot["pos"])
                            if binary is not None else None)
                    why = route_violation(path, enemy, slot["pos"],
                                          others=_others)
                    if why is not None and _blocked is not None:
                        path2 = road_path(_blocked, scale, off, start, slot["pos"])
                        why2 = route_violation(
                            path2, enemy, slot["pos"],
                            max_detour=4.5, detour_slack=1500.0,
                            others=_others)
                        if why2 is None:
                            path, why = path2, None
                    # 起步大转角惩罚：避免把“南边的车”派到北侧再让它原地掉头，
                    # 实测原地 180° 掉头会卡在路沿（速度<0.3 持续 8 s）。
                    turn = _initial_turn_penalty(vhead.get(vid), path)
                    matrix[(vid, slot["sid"])] = (why, _path_length(path) + turn)
            chosen = best_encircle_match(cand_vids, all_slots, matrix,
                                         max_slots=ENCIRCLE_MAX_SLOTS)
            if not chosen:
                changed = True
                log("[合围] %s (%s) 没有车辆能合法逼近，暂不派车"
                    % (tid, g.get("kind")))
                g["slots"] = []
                continue
            for vid, slot in chosen:
                used_vids.add(vid)
            new_slots = []
            for vid, slot in chosen:
                why, _ln = matrix.get((vid, slot["sid"]), (None, 0.0))
                if why is not None:
                    continue
                new_slots.append({"vid": vid, "branch": slot.get("sid"),
                                  "pos": list(slot["pos"]), "hold": None,
                                  "eta": round(_ln / 8.0, 1)})
                out[vid] = (slot["pos"][0], slot["pos"][1])
                _TACTICS_WAIT_VIDS.add(vid)
                _TACTICS_ENEMY_BY_VID[vid] = (float(enemy[0]), float(enemy[1]))
            _old_sig = sorted((s["vid"], s.get("branch"))
                              for s in g.get("slots", []))
            _new_sig = sorted((s["vid"], s["branch"]) for s in new_slots)
            if new_slots:
                _TACTICS_ASSAULT_SINCE.setdefault(tid, float(now))
            if _old_sig != _new_sig:
                changed = True
                log("[合围] %s 重匹配：%d 个阵位 -> %s"
                    % (tid, len(new_slots),
                       "/".join("%s@%s" % (s["vid"], s["branch"])
                                for s in new_slots) or "无"))
            g["slots"] = new_slots
        else:
            # 路段敌人：单车抵近，不要求多面；但路线不能穿过其它活敌，
            # 否则会“越过一个还没合围的路口敌人”跑到另一侧。
            _tpos = enemy or (g["slots"][0]["pos"] if g.get("slots") else None)
            _others = []
            if _tpos is not None:
                _others = [p for p in _live_points
                           if math.hypot(float(p[0]) - _tpos[0],
                                         float(p[1]) - _tpos[1]) >= 30.0]
            kept = []
            for s in g.get("slots", []):
                if s["vid"] in used_vids:
                    continue
                _sp = vpos.get(s["vid"])
                _path = (road_path(binary, scale, off, _sp, s["pos"])
                         if (binary is not None and _sp is not None) else None)
                _hit = route_hits_other(_path, _others)
                if _hit is not None:
                    log("[路段] %s 去 (%.0f,%.0f) 的路线经过其他敌人 "
                        "(%.0f,%.0f)，暂不派车"
                        % (s["vid"], s["pos"][0], s["pos"][1],
                           _hit[0], _hit[1]))
                    continue
                kept.append(s)
                out[s["vid"]] = (s["pos"][0], s["pos"][1])
                used_vids.add(s["vid"])
            if not kept and g.get("slots") and _tpos is not None:
                _cand = [v for v in vehicles if v["vid"] not in used_vids]
                _cand.sort(key=lambda v: math.hypot(
                    v["pos"][0] - _tpos[0], v["pos"][1] - _tpos[1]))
                for _v in _cand:
                    _path = road_path(binary, scale, off, _v["pos"], _tpos)
                    if route_hits_other(_path, _others) is not None:
                        continue
                    _s = {"vid": _v["vid"], "branch": "center",
                          "pos": list(_tpos), "hold": None, "eta": None}
                    kept.append(_s)
                    out[_v["vid"]] = (_tpos[0], _tpos[1])
                    used_vids.add(_v["vid"])
                    break
            g["slots"] = kept
    plan["groups"] = sorted([g for g in _groups if g.get("slots")],
                            key=lambda g: 0 if g.get("kind") in ("junction", "tee") else 1)
    if changed:
        try:
            tac.store.write(plan)
        except Exception:
            pass
    return out


def build_road_points(binary, scale, off, step=25, cap=4000):
    """把路网二值图抽样成世界坐标点，供前沿点/待机点选择用。

    注意要按「路面像素序列」抽样，不能按 `binary[::step, ::step]` 网格抽样：
    骨架只有 1 px 宽，网格抽样会漏掉绝大部分道路（实测只剩 34 个点，
    导致前沿待机点永远选不出来）。
    """
    ys, xs = np.nonzero(binary)
    n = len(xs)
    if n == 0:
        return []
    stride = max(1, int(step))
    if n > cap * stride:
        stride = max(stride, n // cap)
    pts = []
    for i in range(0, n, stride):
        wx, wy = ugv.p2w(int(xs[i]), int(ys[i]), scale, off)
        pts.append((float(wx), float(wy)))
    if len(pts) > cap:
        pts = pts[::max(1, len(pts) // cap)]
    return pts


def uav_xy(name, runner, uav_vehicles):
    """取无人机水平位置（虚拟模式读数学位姿，否则读 actor）。"""
    try:
        if getattr(runner, "virtual", False):
            p = runner.virtual_pose(name)
            return (float(p[0]), float(p[1]))
        p = uav_vehicles[name].position()
        return (float(p[0]), float(p[1]))
    except Exception:
        return None


def mark_contacts_on_markers(markers, contacts, tol=80.0):
    """把车载 contact 对齐到 targets.json 的 marker，让 UI 真的显示“已发现”。

    以前车载传感器发现了目标，但 marker 没被点亮：UGV 已经动了，UI 还是全空心。
    """
    if not markers or not contacts:
        return 0
    found = 0
    for c in contacts.values():
        x, y = c["pos"]
        best_i, best_d = -1, None
        for i, m in enumerate(markers):
            if m.extra:
                continue
            d = math.hypot(m.x - x, m.y - y)
            if best_d is None or d < best_d:
                best_i, best_d = i, d
        if best_i >= 0 and best_d is not None and best_d <= tol:
            m = markers[best_i]
            if not m.discovered:
                m.discovered = True
                found += 1
                log("车载发现目标 #%d %s (%.1f, %.1f)"
                    % (best_i, m.cn, m.x, m.y))
    return found


def live_enemy_points(enemies, contacts, killed_pts, merge=20.0):
    """合并 UAV 目标 + 车载 contact，去掉疑似已击杀的，供避障/防撞共用。"""
    out = []
    src = list(enemies) + [c["pos"] for c in (contacts or {}).values()]
    for p in src:
        try:
            x, y = float(p[0]), float(p[1])
        except Exception:
            continue
        if any(math.hypot(x - q[0], y - q[1]) < merge for q in killed_pts):
            continue
        if any(math.hypot(x - q[0], y - q[1]) < merge for q in out):
            continue
        out.append((x, y))
    return out


def mark_uav_coverage(tac, runner, uav_vehicles, radius):
    """把三架机当前位置记进覆盖栅格（编队待机点的来源）。"""
    for n in cu.UAV_NAMES:
        p = uav_xy(n, runner, uav_vehicles)
        if p is not None:
            try:
                tac.mark_coverage(p[0], p[1], radius)
            except Exception:
                pass


def scan_ugv_contacts(agents, ugvs, now, store, zone, max_age=30.0,
                      merge_dist=25.0):
    """把无人车自己传感器看到的敌人并入战术层视野。

    关键：扫描**所有已注册车辆**，而不是只扫已有 UgvAgent 的车。
    否则开局三车还没接任务时，近处路口敌人要等到派车后才被发现，
    就会先被派去远处路段，等发现路口时已经没有第二辆车可合围。
    """
    HOSTILE = ("Soldier", "Stryker")
    targets = list(ugvs) if ugvs else [a.ugv for a in agents]
    for v in targets:
        try:
            ob = v.detect_fun()
        except Exception:
            continue
        types = list(getattr(ob, "Type", []) or [])
        xs = list(getattr(ob, "x_ue", []) or [])
        ys = list(getattr(ob, "y_ue", []) or [])
        hps = list(getattr(ob, "Health", []) or [])
        for i, t in enumerate(types):
            if t not in HOSTILE or i >= len(xs) or i >= len(ys):
                continue
            if i < len(hps):
                try:
                    if float(hps[i]) <= 0.0:
                        continue
                except Exception:
                    pass
            x, y = float(xs[i]), float(ys[i])
            if not zone.contains(x, y, margin=0.0):
                continue
            key = None
            for k, c in store.items():
                if math.hypot(c["pos"][0] - x, c["pos"][1] - y) < merge_dist:
                    key = k
                    break
            if key is None:
                key = (int(round(x / 10.0)), int(round(y / 10.0)))
                store[key] = {"pos": (x, y), "t": now}
            else:
                store[key]["pos"] = (x, y)
                store[key]["t"] = now
    for k in list(store.keys()):
        if now - store[k]["t"] > max_age:
            store.pop(k, None)
    return store


def choose_staging_point(tac, binary, scale, off, road_points):
    """选前沿待机点：取「已扫/未扫交界带」上离区域中心最近的路点。

    没有前沿（无人机还没扫到任务区、或已经全扫完）时返回 None —— 不猜一个
    「区域中心」让车盲目前出到未侦查区域。
    """
    if road_points:
        cands = []
        for p in road_points:
            try:
                if tac.cov.inner_ok(p[0], p[1]) and tac.cov.on_frontier(p[0], p[1]):
                    cands.append(p)
            except Exception:
                pass
        if cands:
            cx, cy = tac.zone.center()
            return min(cands, key=lambda p: math.hypot(p[0] - cx, p[1] - cy))
    return None


class FormationRuntime(object):
    """编队接入主循环的运行时状态机。

    职责：
      1. 没接敌时组织「前沿待机」：一辆领航、其余纵队跟到待机点；
      2. 接敌后领航车变成打击车，僚车沿它的路径跟到距目标 rally 米处停；
      3. 新目标下达时僚车直接分裂成打击车（由主循环调用 on_split）。

    只操作 UgvAgent 对象；不直接连 SDK。
    """

    def __init__(self, args, tactics, binary, raw, scale, off, road_points):
        self.args = args
        self.tactics = tactics
        self.binary = binary
        self.raw = raw
        self.scale = scale
        self.off = off
        self.road_points = road_points
        self.convoy = None
        if _ConvoyController is not None:
            self.convoy = _ConvoyController(
                kind=args.formation, spacing=args.formation_spacing,
                lateral=args.formation_lateral, rally=args.formation_rally)
        self.staging_done = False
        self.staging_name = None
        self.staging_target = None
        self.staging_started_at = None
        self.last_stage_try = 0.0
        self.last_stage_wait_log = 0.0
        self.last_update = 0.0
        self.last_log = 0.0
        self.last_msg = ""

    # ---------------- 工具 ----------------
    def _project(self, x, y):
        try:
            px, py = ugv.w2p(x, y, self.scale, self.off)
            pt = ugv.nearest_road(self.binary, px, py, search=12)
            if pt is None:
                return None
            return ugv.p2w(pt[0], pt[1], self.scale, self.off)
        except Exception:
            return None

    def _road_width(self, pos, heading):
        if self.raw is None or _tactics_deploy is None:
            return None
        try:
            return _tactics_deploy.road_width_at(
                self.raw, pos, heading, scale=self.scale, off=self.off)
        except Exception:
            return None

    @staticmethod
    def _alive(a):
        return a is not None and a.status in ("init", "drive", "reverse")

    def _agent_of(self, agents, name):
        for a in agents:
            if a.name == name and a.status not in ("arrived", "no_path",
                                                   "stuck", "max_replan"):
                return a
        return None

    def _log(self, msg, now, force=False):
        # 编队状态每拍都可能因为成员顺序变化而产生不同文案；这里按时间节流，
        # 只在换领航车/开始待机时强制打印，避免 1 Hz 刷屏。
        if force or (now - self.last_log) > 15.0:
            log(msg)
            self.last_msg = msg
            self.last_log = now

    # ---------------- 主入口 ----------------
    def update(self, now, el, agents, ugvs, enemies, cooldown=None,
               contacts=None, uav_ready=True):
        if self.convoy is None:
            return
        # 无人机没完全升空/没进入巡航之前，无人车一律不动。
        if not uav_ready:
            return
        # 没有 UAV 目标、也没有车载传感器目标：保持初始位置待命，
        # 不再“接敌前先往前沿待机点开”。
        if not enemies and not contacts:
            return
        if enemies:
            # 只有「任务区内」的敌人出现才停止前沿待机；区外旧靶标不影响
            try:
                in_zone = any(self.tactics.zone.contains(float(p[0]), float(p[1]),
                                                        margin=50.0)
                              for p in enemies)
            except Exception:
                in_zone = True
            if in_zone:
                # 接敌后不再往前沿待机点走；编队转为跟随打击车
                self.staging_done = True

        def _cooling(nm):
            return (cooldown is not None
                    and (now - cooldown.get(nm, -999.0)) < 20.0)

        # 1) 找领航车：打击车优先，其次在前沿待机的车
        strike = [a for a in agents
                  if a.kind == "strike" and self._alive(a)]
        stage = [a for a in agents
                 if a.kind == "staging" and self._alive(a)]
        leader = None
        if strike:
            leader = min(strike, key=lambda a: math.hypot(
                a.target[0] - a.ugv.get_state().pose_.x_,
                a.target[1] - a.ugv.get_state().pose_.y_))
        elif stage and not self.staging_done:
            leader = stage[0]
        elif (not self.staging_done) and (not getattr(self.args, "no_staging", False)) \
                and el >= 3.0 and (now - self.last_stage_try) > 5.0:
            self.last_stage_try = now
            # 2) 组织前沿待机：选一辆离待机点最近的车当领航
            target = choose_staging_point(self.tactics, self.binary, self.scale,
                                          self.off, self.road_points)
            if target is None:
                if (now - self.last_stage_wait_log) > 30.0:
                    self.last_stage_wait_log = now
                    log("[编队] 还没有扫到任务区（无前沿待机点），编队原地待命")
            if target is not None:
                name, k = None, None
                for i, nm in enumerate(UGV_NAMES):
                    if any(a.name == nm and a.kind == "strike" for a in agents):
                        continue
                    if _cooling(nm):
                        continue
                    p = ugvs[i].get_state().pose_
                    if name is None or math.hypot(p.x_ - target[0],
                                                  p.y_ - target[1]) < math.hypot(
                            ugvs[k].get_state().pose_.x_ - target[0],
                            ugvs[k].get_state().pose_.y_ - target[1]):
                        name, k = nm, i
                if name is not None:
                    a = self._agent_of(agents, name)
                    if a is not None:
                        ok = a.retarget(target, role="lead", kind="staging")
                    else:
                        a = ugv.UgvAgent(ugvs[k], name, target, self.binary,
                                         self.raw, self.scale, self.off,
                                         role="lead", kind="staging")
                        a.start()
                        agents.append(a)
                        ok = a.status == "drive"
                    if ok:
                        self.staging_name = name
                        self.staging_target = target
                        self.staging_started_at = now
                        leader = a
                        self._log("[编队] 前沿待机：%s 领航 -> (%.0f,%.0f)，"
                                  "其余纵队跟随（间距 %.0f m）"
                                  % (name, target[0], target[1],
                                     self.args.formation_spacing), now, force=True)
                    else:
                        log("[编队] %s 去前沿待机点无路可达，取消待机" % name)
                        self.staging_done = True
        if leader is None:
            return

        # 3) 僚车集合：没有打击任务、也不是领航车的车
        # 注意用「所有 kind=strike 的车」而不是只算在途的：刚结束、还没被清理的
        # 打击车如果被当成空闲，会在这里被重新拉去跟随，造成同一台车两个 agent。
        strike_names = set(a.name for a in agents if a.kind == "strike")
        members = []
        for i, nm in enumerate(UGV_NAMES):
            if nm == leader.name or nm in strike_names or _cooling(nm):
                continue
            # 接敌后原来的待机车也降级为僚车（跟着打击车走），别让它继续往前沿跑
            if (leader.kind != "strike"
                    and any(a.name == nm and a.kind == "staging" and self._alive(a)
                            for a in agents)):
                continue
            try:
                p = ugvs[i].get_state().pose_
                members.append({"name": nm, "pos": (float(p.x_), float(p.y_))})
            except Exception:
                pass
        if not members:
            return

        # 4) 滞后下发（控制器内部还会按 12 m / 1 s 再过滤）
        if (now - self.last_update) < 0.5:
            return
        self.last_update = now
        try:
            pose = leader.ugv.get_state().pose_
            lpos = (float(pose.x_), float(pose.y_))
            heading = float(getattr(pose, "theta_", 0.0))
        except Exception:
            return
        prev_leader = getattr(self.convoy, "_leader", None)
        targets = self.convoy.update(
            now,
            {"name": leader.name, "pos": lpos,
             "path": leader.path or [], "kind": leader.kind},
            members, project=self._project,
            road_width=self._road_width(lpos, heading))
        if not targets:
            return
        changed = []
        for nm, pt in targets.items():
            k = UGV_NAMES.index(nm)
            a = self._agent_of(agents, nm)
            if a is not None:
                if a.kind == "strike":
                    continue
                ok = a.retarget(pt, role="follow", kind="follow")
            else:
                a = ugv.UgvAgent(ugvs[k], nm, pt, self.binary, self.raw,
                                 self.scale, self.off, role="follow",
                                 kind="follow")
                a.start()
                agents.append(a)
                ok = a.status in ("drive", "hold")
            if ok:
                changed.append(nm)
        if changed:
            self._log("[编队] %s 领航，%s 跟随（%s，间距 %.0f m，"
                      "僚车止于目标前 %.0f m）"
                      % (leader.name, "/".join(changed),
                         self.convoy.kind or self.args.formation,
                         self.args.formation_spacing,
                         self.args.formation_rally), now,
                      force=(prev_leader != leader.name))

    def on_split(self, name):
        """僚车转成打击车时调用，清掉它的跟随历史，避免分裂后又被拉回旧点。"""
        if self.convoy is not None:
            self.convoy.reset(name)

    def on_agent_gone(self, name):
        if name == self.staging_name:
            self.staging_done = True
            self.staging_name = None


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


def place_ugvs_at_starts(vehicles, starts):
    """把无人车摆到开局展开点（保留原高度，按道路切线设朝向）。

    与 --ugv-start 的整体平移不同：每台车的落点独立、都已吸附在路面上，
    不会出现"三台车挤在同一条路/区域边界"的情况。
    """
    try:
        from swarmae import SwarmAENode as SN
        carla = SN.carla
    except Exception as e:
        log("拿不到 carla 模块，跳过开局展开: %r" % e)
        return False
    moved = 0
    try:
        for v, s in zip(vehicles, starts):
            pose = v.get_state().pose_
            actor = getattr(v.Vehicle_, "_SwarmAENode__actor", None)
            if actor is None:
                continue
            # 用原高度：写死 2.1 会把车插进路面/建筑（交接文档问题 #6）
            actor.set_transform(carla.Transform(
                carla.Location(float(s["pos"][0]), float(s["pos"][1]),
                               float(pose.z_)),
                carla.Rotation(pitch=0.0, yaw=float(s.get("yaw", 0.0)),
                               roll=0.0)))
            moved += 1
        time.sleep(1.5)
        for v in vehicles:
            v.apply_control(0, 0, 0, True)
        time.sleep(0.3)
        for v in vehicles:
            v.apply_control(0, 0, 0, False)
        log("无人车已摆到开局展开点（%d 台）" % moved)
        return moved > 0
    except Exception as e:
        log("开局展开失败（继续用默认出生点）: %r" % e)
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
    # 按车名配对，不能直接 zip(agents, vehicles)：编队分裂时只会有 1~2 台新车，
    # 直接 zip 会把 UGV2 的目标算到 UGV1 的车上，转错方向。
    pairs = []
    for i, nm in enumerate(UGV_NAMES):
        for a in agents:
            if a.name == nm and i < len(vehicles):
                pairs.append((a, vehicles[i]))
                break
    for a, v in pairs:
        try:
            st = v.get_state().pose_
            x, y, z = float(st.x_), float(st.y_), float(st.z_)
            # 朝向沿 A* 路径起步，而不是直指目标：目标在另一条路上时，
            # 直接朝目标会先在路口掉头（实测 UGV1 因此多绕了 100 多米）。
            tx, ty = float(a.target[0]), float(a.target[1])
            if a.path and len(a.path) >= 2:
                j = min(12, len(a.path) - 1)
                tx, ty = float(a.path[j][0]), float(a.path[j][1])
            if abs(tx - x) < 1e-6 and abs(ty - y) < 1e-6:
                continue
            yaw = math.degrees(math.atan2(ty - y, tx - x))
            actor = getattr(v.Vehicle_, "_SwarmAENode__actor", None)
            if actor is None:
                continue
            actor.set_transform(carla.Transform(
                carla.Location(x, y, z),
                carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)))
            log("%s 起步朝向 -> %.0f°（沿路径指向 %.0f, %.0f）"
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


def uav_recon_ready(runner, uav_vehicles):
    """无人机是否已经**完全升空并进入巡航**。

    不能在“还在爬升”时派车：之前只要 z>=120（tol=30）就判定 ready，
    三车会先于无人机升空/侦测出发。这里要求：
      * 没有任何一架处于 take/climb；
      * 全部进入 crui/cruise/hold（或全部 dead）。
    """
    try:
        phases = runner.phases_snapshot()
    except Exception:
        phases = {}
    vals = [str(v).lower() for v in phases.values()]
    if any(v in ("take", "climb", "takeoff") for v in vals):
        return False
    if vals and all(v in ("crui", "cruise", "hold", "dead") for v in vals):
        return True
    if getattr(runner, "virtual", False):
        return False
    return _all_uavs_at_altitude(uav_vehicles, tol=5.0)


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
    g.add_argument("--no-spread", action="store_true",
                   help="不把三架无人机水平散开（--no-teleport 的配套项，"
                        "保持 vehicles.json 里的原始出生位置）")
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
    g.add_argument("--ugv-deploy", dest="ugv_deploy", action="store_true",
                   default=True,
                   help="（默认开启，仅 --tactics）按覆盖最优自动选三车的开局展开点")
    g.add_argument("--no-ugv-deploy", dest="ugv_deploy", action="store_false",
                   help="不自动选开局展开点（保留场景出生点或 --ugv-start）")
    g.add_argument("--deploy-margin", type=float, default=150.0,
                   help="开局展开点离任务区边界的最小距离 m（默认150）")
    g.add_argument("--deploy-min-sep", type=float, default=180.0,
                   help="三台车开局两两最小间距 m（默认180）")
    g.add_argument("--formation", choices=("off", "column", "wedge", "auto"),
                   default="column",
                   help="（仅 --tactics）编队队形：column=纵队（默认，窄路稳）；"
                        "wedge=楔形；auto=按路宽自适应；off=关闭编队")
    g.add_argument("--formation-spacing", type=float, default=12.0,
                   help="编队车间距 m（默认12）")
    g.add_argument("--formation-lateral", type=float, default=4.0,
                   help="楔形横向偏移 m（默认4）")
    g.add_argument("--formation-rally", type=float, default=200.0,
                   help="领航车打敌人时，僚车最多跟到距目标多少米（默认200，视野外）")
    g.add_argument("--no-staging", action="store_true",
                   help="关闭接敌前的「前沿待机」推进（车留在开局展开点）")
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
    # 卡死诊断：每 60 s 把调用栈打到 stderr，定位"日志停在某一时刻"的问题
    try:
        import faulthandler
        faulthandler.dump_traceback_later(60, repeat=True)
    except Exception:
        pass
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

    # ---------------- 无人车路网（提前加载：开局展开点要用） ----------------
    binary, raw_bin, ugv_scale, ugv_off = ugv.load_binary(game)
    log("无人车路网已加载（骨架 + 真实路宽）")

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

    # ---- 开局位置优化：按「任务区路网覆盖」选 3 个展开点 ----
    # 仿真出生点固定在场地西侧，离半图任务区 1 km；之前用 --ugv-start 整体平移，
    # 三台车又挤成一个点、在区域边界互相顶死。这里在路网上做 k-center 选点：
    # 覆盖半径最小、两两间距 ≥180 m、离边界 ≥150 m、朝向沿道路切线。
    deployed = False
    if (tactics is not None and getattr(args, "ugv_deploy", True)
            and not args.ugv_start and not args.no_ugv
            and not args.no_teleport):
        if _tactics_deploy is None:
            log("开局展开点模块不可用（tactics/deploy.py 导入失败），保留默认出生点")
        else:
            try:
                _starts = _tactics_deploy.choose_starts(
                    binary, tactics.zone, n=len(UGV_NAMES),
                    margin=args.deploy_margin, min_sep=args.deploy_min_sep,
                    raw=raw_bin, scale=ugv_scale, off=ugv_off)
                if _starts:
                    log("开局位置优化：")
                    for _line in _tactics_deploy.format_starts(_starts).splitlines():
                        log("  " + _line)
                    deployed = place_ugvs_at_starts(ugvs, _starts)
                else:
                    log("开局展开点没选出来（区域里没有可用路网），保留默认出生点")
            except Exception as _e:
                log("开局位置优化失败（继续用默认出生点）: %r" % (_e,))
    elif tactics is not None and args.no_teleport:
        log("--no-teleport：保持场景 vehicles.json 的出生位置，不做开局展开/平移")
    elif tactics is not None and args.ugv_start:
        log("已给 --ugv-start，跳过自动开局展开点（整体平移优先）")


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

    # 注意：路网在无人车注册后已提前加载（开局展开点需要它），这里不再重复读图。
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

    # ---------------- 战术层：编队接入主循环 ----------------
    # 路网点抽样一次，供「前沿待机点」与 planner 复用（之前每次派车都重扫全图）。
    road_points = None
    if tactics is not None:
        road_points = build_road_points(binary, ugv_scale, ugv_off)
        log("路网候选点：%d 个（前沿待机/分配共用）" % len(road_points))
    formation_rt = None
    if tactics is not None and args.formation != "off" and not args.no_ugv:
        if _ConvoyController is None:
            log("编队控制器不可用（tactics/convoy.py 导入失败），编队关闭")
        else:
            formation_rt = FormationRuntime(
                args, tactics, binary, raw_bin, ugv_scale, ugv_off, road_points)
            log("编队已接入主循环：队形 %s，间距 %.0f m，僚车止于目标前 %.0f m%s"
                % (args.formation, args.formation_spacing, args.formation_rally,
                   "，接敌前先前沿待机" if not args.no_staging else "，不做前沿待机"))
    elif tactics is not None:
        log("--formation off：编队关闭，空闲车原地待命")

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
    engaged_once = False   # 是否真的开过火（避免"从没交火"就按 10 s 无交火收尾）
    attacked_pts = []
    ugv_trails = dict((n, []) for n in UGV_NAMES)
    killed_pts = []
    last_log_encircle = 0.0
    last_watchdog = 0.0
    last_dbg = 0.0
    hold_clear_since = {}   # 合围车 hold 后目标已不存在的计时
    last_route_wait_log = 0.0
    ugv_contacts = {}       # 无人车自己看到的敌人（补无人机视野）
    last_ugv_scan = 0.0
    last_enemy_stop_log = {}   # 每台车“距敌刹停”日志节流
    ugv_stuck = {}
    ugv_cooldown = {}   # P3：释放后的改派冷却，避免刚停稳就被重新打舵
    last_cool_log = 0.0
    prev_uav_pos = None
    last_progress = time.time()
    last_cov = 0.0
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

            # ---- 1b. 覆盖记账（编队前沿待机点的来源）----
            if tactics is not None and (t0 - last_cov) > 2.0:
                last_cov = t0
                mark_uav_coverage(tactics, runner, uav_vehicles, args.sweep_radius)

            # ---- 2. 同步"已发现"到小地图 ----
            uav_ready = uav_recon_ready(runner, uav_vehicles)
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
                                                markers[i].x, markers[i].y,
                                                len(_sl), t0)
                                            log("[分类] (%.0f,%.0f) -> %s 阵位=%d"
                                                % (markers[i].x, markers[i].y,
                                                   _kd, len(_sl)))
                                        # P2 修正：不再在 1 px 宽的骨架上硬挖 14 m 洞。
                                        # 硬封锁会把整个路口从路网里剪断（实测绕行 1 km），
                                        # 改为派车时做「路线校验」：路径若穿过敌人就剔除该阵位。
                                        log("[合围] 路口 (%.0f,%.0f) 不做硬封锁，"
                                            "改由路线校验保证不穿过敌人"
                                            % (markers[i].x, markers[i].y))
                            except Exception:
                                pass
                if changed and view is not None:
                    view.known_override = sum(1 for m in markers if not m.extra)
            if view is not None:
                view.discovered_override = sum(
                    1 for m in markers if m.discovered and not m.extra)
                view.set_markers(markers)
            # 只保留还活着的合围目标（30 m 内有已击杀点则移除）
            try:
                for _k in list(_TACTICS_ENCIRCLE.keys()):
                    _v = _TACTICS_ENCIRCLE[_k]
                    if any(math.hypot(_v[0] - q[0], _v[1] - q[1]) < 30.0 for q in killed_pts):
                        _TACTICS_ENCIRCLE.pop(_k, None)
                        _TACTICS_ASSAULT_SINCE.pop(_k, None)
            except Exception:
                pass
            if _TACTICS_ENCIRCLE and view is not None:
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
                    ugv_cooldown[a.name] = t0
                    if formation_rt is not None:
                        formation_rt.on_agent_gone(a.name)
                    continue
                # 【新增】目标已经被打死了就立即就地释放，不必再开到那个点。
                # 判定收紧到 12 m：之前 40 m 太宽，1 号车打死一个敌人时，
                # 会把旁边路过的 2、3 号车也一起"释放"（表现为突然转向后停死）。
                # 只对「打击车」生效：编队僚车的 target 是跟随点，不能被击杀点误伤。
                # 路口阵位的车停在离敌人 18 m 的封锁位，击杀判定要放宽到 30 m，
                # 否则目标死了它还一直 hold 在原位。
                # 路口阵位现在离敌人 30 m，击杀释放半径要覆盖阵位（45 m）。
                _rel_r = 45.0 if getattr(a, "wait_at_target", False) else 12.0
                if a.kind == "strike" and any(
                        math.hypot(a.target[0] - q[0], a.target[1] - q[1]) < _rel_r
                        for q in killed_pts):
                    try:
                        a.finish("arrived", "目标已被消灭，就地待命")
                    except Exception:
                        pass
                    uw = ugv_targets.pop(a.name, None)   # 清掉残留目标，避免小地图显示假目标
                    log("%s 的目标已被消灭，就地转为空闲" % a.name)
                    ugv_agents.remove(a)
                    ugv_cooldown[a.name] = t0
                    if formation_rt is not None:
                        formation_rt.on_agent_gone(a.name)
            # 【解冻】合围车 hold 后，如果目标已经不在任何活敌列表里，
            # 8 s 后释放；避免“打完了还钉在阵位上，下一批目标不接”。
            _live_now = live_enemy_points(enemies, ugv_contacts, killed_pts)
            for a in list(ugv_agents):
                if not (getattr(a, "wait_at_target", False)
                        and a.kind == "strike" and a.status == "hold"):
                    hold_clear_since.pop(a.name, None)
                    continue
                _tgt = getattr(a, "avoid_enemy", None) or a.target
                _still = any(math.hypot(_tgt[0] - q[0], _tgt[1] - q[1]) < 45.0
                             for q in _live_now)
                if _still:
                    hold_clear_since.pop(a.name, None)
                    continue
                _since = hold_clear_since.setdefault(a.name, t0)
                if t0 - _since > 8.0:
                    try:
                        a.finish("arrived", "目标已不存在，释放去接下一批")
                    except Exception:
                        pass
                    ugv_targets.pop(a.name, None)
                    ugv_agents.remove(a)
                    ugv_cooldown[a.name] = t0
                    hold_clear_since.pop(a.name, None)
                    log("%s 合围目标已不存在，释放（8 s 超时）" % a.name)
                    if formation_rt is not None:
                        formation_rt.on_agent_gone(a.name)
            # ---- 调试：每 5 s 打印三车状态（定位卡死用）----
            if tactics is not None and (t0 - last_dbg) > 5.0:
                last_dbg = t0
                _dbg = []
                for _a in ugv_agents:
                    try:
                        _p = _a.ugv.get_state().pose_
                        _dbg.append("%s[%s/%s] p(%.0f,%.0f) t(%.0f,%.0f) n=%d"
                                    % (_a.name, _a.status, getattr(_a, "kind", "?"),
                                       _p.x_, _p.y_,
                                       _a.target[0], _a.target[1],
                                       len(getattr(_a, "path", []) or [])))
                    except Exception:
                        pass
                if _dbg:
                    log("[状态] " + " | ".join(_dbg))
            # 空闲 = 没有「打击任务」在身。编队僚车（follow）随时可以被拉去分裂打击。
            _strike_busy = set()
            for _a in ugv_agents:
                if (getattr(_a, "kind", "strike") == "strike"
                        and _a.status in ("init", "drive", "reverse", "hold")):
                    _strike_busy.add(_a.name)
            free_idx = [i for i, n in enumerate(UGV_NAMES)
                        if n not in _strike_busy]
            # P3：刚被释放的车先冷却 20 s 再接新任务（避免刚停稳就被重新打舵 -> 抽搐）
            _cool = [k for k, n in enumerate(UGV_NAMES)
                     if k in free_idx and (t0 - ugv_cooldown.get(n, -999.0)) < 20.0]
            if _cool:
                free_idx = [k for k in free_idx if k not in _cool]
                if t0 - last_cool_log > 10.0:
                    last_cool_log = t0
                    log("冷却中（%.0f s 后可再派）：%s"
                        % (20.0, ",".join(UGV_NAMES[k] for k in _cool)))
            # ---- 车载传感器补盲：无人机没扫到的敌人也要进战术层 ----
            if tactics is not None and (t0 - last_ugv_scan) > 2.0:
                last_ugv_scan = t0
                _n0 = len(ugv_contacts)
                scan_ugv_contacts(ugv_agents, ugvs, t0, ugv_contacts,
                                  tactics.zone)
                if len(ugv_contacts) > _n0:
                    log("车载传感器补充发现 %d 个目标（无人机视野外）"
                        % (len(ugv_contacts) - _n0))
                _new_marks = mark_contacts_on_markers(markers, ugv_contacts)
                if _new_marks and view is not None:
                    view.set_markers(markers)
                    view.discovered_override = sum(
                        1 for m in markers if m.discovered and not m.extra)
                    view.known_override = sum(1 for m in markers if not m.extra)
            # ---- 看门狗：空闲且掉到任务区外的车，拉回路网（防止整轮卡死）----
            if (tactics is not None and not args.no_teleport
                    and (t0 - last_watchdog) > 3.0):
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
                                _c3.Location(_wx, _wy, float(_pose.z_)),
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
                # 只把「打击车」的目标算作占用：编队僚车/待机车不能被当成正在打某个敌人
                busy_now = [getattr(a, "avoid_enemy", None) or a.target
                            for a in ugv_agents
                            if getattr(a, "kind", "strike") == "strike"]
                # 已指派的目标也留在池里：最后一个路口敌人可能先只到了一台车，
                # 后面空闲车必须还能加入另一侧（以前把 busy 点排除，第二台永远接不上）。
                pool = [p for p in enemies
                        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 40.0
                               for q in killed_pts)]
                # 车载传感器看到的敌人也进入候选池：它们可能 UAV 没扫到，
                # 但仍需要分类（是不是路口）和分配，而不是等车开到面前直接开火。
                for _c in ugv_contacts.values():
                    _p = _c["pos"]
                    if any(math.hypot(_p[0] - q[0], _p[1] - q[1]) < 40.0
                           for q in killed_pts):
                        continue
                    if any(math.hypot(_p[0] - q[0], _p[1] - q[1]) < 30.0
                           for q in pool):
                        continue
                    pool.append(_p)
                # 不再按 busy_now 做“别抢”过滤：路口第二个侧翼正是要抢同一个点。
                # 【逐敌派车】只要有"未分配敌人 + 空闲车 + 飞机已在空中"就派，
                # 不再等凑够 3 个、也不等扫描结束；否则一辆车打完要干等到扫完才能接下一个。
                ready = bool(pool) and uav_ready

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
                                _all_live = (list(enemies)
                                             + [c["pos"] for c in ugv_contacts.values()]
                                             + [(m.x, m.y) for m in markers
                                                if getattr(m, "discovered", False)
                                                and not getattr(m, "extra", False)])
                                plan_targets = tactics_decide(
                                    tactics, t0, free_idx, pool, ugvs,
                                    uav_vehicles, args.sweep_radius,
                                    binary=binary, scale=ugv_scale, off=ugv_off,
                                    road_points=road_points,
                                    all_enemies=_all_live)
                            except Exception as _e:
                                log("战术层求解失败，退回贪心：%r" % (_e,))
                                plan_targets = None
                        # 把「车 -> 目标」配对保存下来。战术层只返回真正有打击任务的车，
                        # 不能像以前那样按 free_idx 顺序 zip（否则会错位：空闲车拿到别人的目标）。
                        pairs = []
                        if plan_targets is None:
                            # 战术层这一轮没有新计划：退回贪心（原逻辑）
                            chosen = assign_targets(
                                pts, starts_xy, len(free_idx),
                                trails=[ugv_trails[UGV_NAMES[k]] for k in free_idx],
                                headings=headings)
                            pairs = list(zip(free_idx, chosen))
                        elif plan_targets:
                            for k in free_idx:
                                p = plan_targets.get(UGV_NAMES[k])
                                if p is not None:
                                    pairs.append((k, p))
                        else:
                            # 有战术计划，但所有阵位都通不过路线校验：
                            # 宁可等/换位，也不能贪心硬派过去穿过敌人。
                            if t0 - last_route_wait_log > 5.0:
                                last_route_wait_log = t0
                                log("战术层无可行合围路线，暂停派车（等待换位/新阵位）")
                        # 【修正】目标点少于车数时，不要几台车挤同一个点（会互相挡死）。
                        # 去重后"有几处目标就派几台车"，剩下的车由编队接管。
                        uniq_pairs = []
                        for k, p in pairs:
                            if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 25.0
                                   for _kk, q in uniq_pairs):
                                uniq_pairs.append((k, p))
                        if len(uniq_pairs) < len(pairs):
                            log("目标点去重：%d -> %d 处，只派 %d 台车（其余编队待命）"
                                % (len(pairs), len(uniq_pairs), len(uniq_pairs)))
                        new_agents = []
                        _enemy_map = _TACTICS_ENEMY_BY_VID if plan_targets else {}
                        for k, p in uniq_pairs:
                            name = UGV_NAMES[k]
                            _enemy = _enemy_map.get(name)
                            existing = None
                            for _a in ugv_agents:
                                if (_a.name == name
                                        and _a.status not in ("arrived", "no_path",
                                                              "stuck", "max_replan")):
                                    existing = _a
                                    break
                            if existing is not None:
                                # 编队成员直接分裂成打击车：保留 agent，只换目标/角色，
                                # 不 finish+start（避免原地掉头抽搐）
                                if existing.kind == "strike":
                                    continue
                                existing.avoid_enemy = _enemy
                                if existing.retarget(p, role="strike", kind="strike"):
                                    existing.wait_at_target = (name in _TACTICS_WAIT_VIDS)
                                    if formation_rt is not None:
                                        formation_rt.on_split(name)
                                    log("[分裂] %s 退出编队 -> 打击目标 (%.0f,%.0f)"
                                        % (name, p[0], p[1]))
                                else:
                                    existing.avoid_enemy = None
                                    existing.avoid_enemies = []
                                    log("%s 改派打击目标失败（保持跟随）" % name)
                                    continue
                            else:
                                # （A* 预检曾在此，因对未吸附起点会误判而移除）
                                a = ugv.UgvAgent(ugvs[k], name, p,
                                                 binary, raw_bin, ugv_scale, ugv_off,
                                                 role="strike", kind="strike")
                                a.wait_at_target = (name in _TACTICS_WAIT_VIDS)
                                a.avoid_enemy = _enemy
                                a.start()
                                ugv_agents.append(a)
                                if a.status in ("drive", "init"):
                                    new_agents.append(a)
                            ugv_targets[name] = p
                            attacked_pts.append(p)
                        if uniq_pairs:
                            ugv_started_at = time.time()
                            for a in new_agents:
                                kk = UGV_NAMES.index(a.name)
                                go_at[a.name] = ugv_started_at + kk * max(
                                    0.0, args.dispatch_gap)
                            if args.dispatch_gap > 0 and new_agents:
                                log("错峰发车：每台间隔 %.0f s" % args.dispatch_gap)
                            if new_agents and not args.no_face_target:
                                face_targets(ugvs, new_agents)
                            # 同步写进 goals.json，小地图星标与实际目标一致
                            merge_goals_atomic(ugv_targets)
                            log("已派车：%s" % dict(
                                (k, (round(v[0], 1), round(v[1], 1)))
                                for k, v in ugv_targets.items()))
                        if len(pts) < len(UGV_NAMES):
                            log("注意：只有 %d 个可用目标点，部分无人车会被派到同一处"
                                % len(pts))
                        # 无人机改飞由 3b 段统一处理（与是否派车无关）

            # ---- 3c. 编队：接敌前推进到前沿待机；接敌后僚车跟随打击车 ----
            # 放在派车之后：新目标优先把僚车「分裂」成打击车，剩下的才继续排队形。
            if formation_rt is not None:
                try:
                    formation_rt.update(t0, el, ugv_agents, ugvs, enemies,
                                        cooldown=ugv_cooldown,
                                        contacts=ugv_contacts,
                                        uav_ready=uav_ready)
                except Exception as _e:
                    log("编队更新异常（本拍跳过）：%r" % (_e,))

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
                            # 【编队】同向的领航/僚车不适用"让行"：让行会让领航车给
                            # 后方僚车让路，僚车又被 front_gap 挡住 → 两车对死。
                            # 编队间距交给 UgvAgent 的 front_gap（僚车 8 m）处理。
                            if (getattr(a, "kind", "strike") != "strike"
                                    or getattr(b, "kind", "strike") != "strike"):
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
                            # 两台车“想走的方向”夹角接近 180° = 正面相遇。
                            # 正面相遇时让行要停得更久，让对方完成借道超车；
                            # 只停 3 s 的话，双方会同时起步再顶死。
                            _da = math.atan2(a.target[1] - pos[a.name][1],
                                             a.target[0] - pos[a.name][0])
                            _db = math.atan2(b.target[1] - pos[b.name][1],
                                             b.target[0] - pos[b.name][0])
                            _head_on = math.cos(_da - _db) < -0.35
                            _hold = YIELD_HOLD_HEADON if _head_on else YIELD_HOLD
                            if yield_until.get(loser.name, 0.0) < now_t:
                                yield_until[loser.name] = now_t + _hold
                                log("%s 与 %s 相距 %.0f m%s，%s 让行 %.0f s"
                                    % (a.name, b.name, d,
                                       "（正面相遇）" if _head_on else "",
                                       loser.name, _hold))
                    for name, until in list(yield_until.items()):
                        if until > now_t:
                            paused.add(name)
                        else:
                            yield_until.pop(name, None)
                except Exception as e:
                    log("让行判定异常（忽略，继续行驶）：%r" % (e,))

                # 活敌列表（UAV + 车载 contact），用于“禁止撞敌”的物理门限。
                _live_enemies = live_enemy_points(enemies, ugv_contacts, killed_pts)

                t_now = time.time()
                for a in ugv_agents:
                    if t_now < go_at.get(a.name, 0.0):
                        continue          # 还没轮到它出发
                    if a.status in ("drive", "reverse") and _live_enemies:
                        _stop_hit = None
                        _stop_own = False
                        try:
                            _pose = a.ugv.get_state().pose_
                            _own = getattr(a, "avoid_enemy", None)
                            for _p in _live_enemies:
                                _d = math.hypot(_p[0] - _pose.x_, _p[1] - _pose.y_)
                                # 自己的合围目标允许靠到 12 m 内（阵位在 30 m，
                                # 且 avoid_enemy 已在路径里挖了 13 m 洞）；
                                # 其它敌人（路过/穿行）保持 20 m 安全距离。
                                _is_own = (_own is not None
                                           and math.hypot(_p[0] - _own[0],
                                                          _p[1] - _own[1]) < 5.0)
                                _r = 12.0 if _is_own else ENEMY_STOP_R
                                if _d < _r:
                                    _stop_hit = (_d, _r)
                                    _stop_own = _is_own
                                    break
                        except Exception:
                            _stop_hit = None
                        if _stop_hit is not None:
                            try:
                                a.ugv.apply_control(0.0, 0.0, 1.0, False)
                            except Exception:
                                pass
                            _last = last_enemy_stop_log.get(a.name, 0.0)
                            if t_now - _last > 5.0:
                                last_enemy_stop_log[a.name] = t_now
                                log("%s 距敌 %.0f m，刹停（禁止撞击/等待合围）"
                                    % (a.name, _stop_hit[0]))
                            continue
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
                    if a.status not in ("arrived", "drive", "reverse", "hold"):
                        continue
                    # 只有被战术层指派了打击任务的车才开火；编队僚车/前沿待机车
                    # 只负责机动，不抢打路过的敌人（否则 70 m 外就替别人开火，
                    # 合围还没成形目标就没了）。
                    if getattr(a, "kind", "strike") != "strike":
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
                    # 打击车只打“自己这一路的目标”，不能因为别的敌人更近就转火；
                    # 否则合围车会隔着 90 m 把另一侧的敌人先打死。
                    if cand and a.kind == "strike":
                        _lock = getattr(a, "avoid_enemy", None)
                        if _lock is None and not getattr(a, "wait_at_target", False):
                            _lock = a.target
                        if _lock is not None:
                            cand = [k for k in cand
                                    if math.hypot(ob.x_ue[k] - _lock[0],
                                                  ob.y_ue[k] - _lock[1]) < 45.0]
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
                    # ---- 合围规则：路口目标必须「到齐」且来自不同分支才开火 ----
                    # 关键：不能只凭 _TACTICS_ENCIRCLE 里有没有这个坐标，
                    # 无人车自己传感器先看到的敌人也要现场分类（之前会直接开火）。
                    if (tactics is not None
                            and types[best] in ("Soldier", "Stryker")):
                        try:
                            _ex, _ey = float(ob.x_ue[best]), float(ob.y_ue[best])
                            _kd, _br = None, []
                            _mk = _tactics_mask()
                            if _mk is not None and _tactics_slots is not None:
                                _kd, _br = _tactics_slots.classify(_mk, (_ex, _ey))
                            _entry = None
                            if _kd in ("junction", "tee") and len(_br or []) >= 2:
                                for _tid, _ev in _TACTICS_ENCIRCLE.items():
                                    if math.hypot(_ex - _ev[0], _ey - _ev[1]) < 30.0:
                                        _entry = _ev
                                        break
                                if _entry is None:
                                    _ss = _tactics_slots.build_slots(_mk, (_ex, _ey), _br)
                                    if len(_ss) >= 2:
                                        _tid = "E%d_%d" % (int(round(_ex)),
                                                           int(round(_ey)))
                                        _entry = (_ex, _ey, len(_ss), t0)
                                        _TACTICS_ENCIRCLE[_tid] = _entry
                                        log("[合围] 车载识别路口 (%.0f,%.0f) -> %s "
                                            "阵位=%d，先合围再开火"
                                            % (_ex, _ey, _kd, len(_ss)))
                            if _entry is not None:
                                _need_slots = min(int(_entry[2]),
                                                  ENCIRCLE_MAX_SLOTS)
                                # 超时从「第一次派车/开始合围」算，而不是从发现目标算，
                                # 否则车开 100 s 才到，就永远不会等第二辆了。
                                _assault_t = _TACTICS_ASSAULT_SINCE.setdefault(
                                    _tid, t0)
                                _age = t0 - _assault_t
                                _ready = 0
                                _assigned = 0
                                _dirs = []
                                for _a in ugv_agents:
                                    try:
                                        if getattr(_a, "kind", "strike") != "strike":
                                            continue
                                        _pose = _a.ugv.get_state().pose_
                                        _dp = math.hypot(_pose.x_ - _a.target[0],
                                                         _pose.y_ - _a.target[1])
                                        _dt = math.hypot(_a.target[0] - _ex,
                                                         _a.target[1] - _ey)
                                        if _dt < 60.0:
                                            _assigned += 1
                                            if _dp < 30.0:
                                                _ready += 1
                                                # 用车辆**实际位置**相对敌人的方位，
                                                # 而不是它被指派的目标点：防止“目标在对面、
                                                # 车实际还在同侧/已穿过敌人”也算一个分支。
                                                _dirs.append(math.degrees(math.atan2(
                                                    _pose.y_ - _ey,
                                                    _pose.x_ - _ex)) % 360.0)
                                    except Exception:
                                        pass
                                # 就位车辆必须分布在 >=2 个不同方向（夹角 >45°）
                                _uniq_dirs = []
                                for _d in _dirs:
                                    if all(abs((_d - _u + 180.0) % 360.0 - 180.0) > 45.0
                                           for _u in _uniq_dirs):
                                        _uniq_dirs.append(_d)
                                _branches = len(_uniq_dirs)
                                _need = _need_slots
                                if _age >= ENCIRCLE_TIMEOUT:
                                    # 长时间还凑不齐：按实际就位/指派数降级，
                                    # 否则单个敌人会把整辆车永远钉在原地。
                                    # 90 s 是为了覆盖另一辆车从远处开过来的时间，
                                    # 15 s 会在第二辆车还在路上时就先开火。
                                    _need = max(1, min(_need_slots,
                                                       max(1, _assigned),
                                                       max(1, _ready)))
                                    if _branches < min(2, _need_slots):
                                        _need = 1
                                if _need >= 2 and _branches < 2:
                                    if t0 - last_log_encircle > 5.0:
                                        last_log_encircle = t0
                                        log("%s 等待合围：就位分支 %d/%d，暂不开火"
                                            % (a.name, _branches, _need))
                                    continue
                                if _ready < _need:
                                    if t0 - last_log_encircle > 5.0:
                                        last_log_encircle = t0
                                        log("%s 等待合围：就位 %d/%d（分支 %d），暂不开火"
                                            % (a.name, _ready, _need, _branches))
                                    continue
                        except Exception:
                            pass
                    try:
                        a.ugv.set_weapon(ob, best)
                        last_contact = t0
                        engaged_once = True
                        # 记下"打过这个点"，等它从探测列表消失再判死
                        # （开火当帧血量往往还没归零，原来的判据几乎永远不成立）
                        engaged_pts.append((ob.x_ue[best], ob.y_ue[best]))
                        try:
                            hp = list(getattr(ob, "Health", []) or [])
                            if best < len(hp) and float(hp[best]) <= 0:
                                _kx = float(ob.x_ue[best])
                                _ky = float(ob.y_ue[best])
                                _hit_marker = False
                                for m in markers:
                                    if (not m.extra
                                            and math.hypot(m.x - _kx,
                                                           m.y - _ky) < 40.0):
                                        m.extra = True
                                        _hit_marker = True
                                        if not any(math.hypot(m.x - q[0],
                                                              m.y - q[1]) < 20.0
                                                   for q in killed_pts):
                                            killed_pts.append((m.x, m.y))
                                            log("已消灭：%s (%.1f, %.1f)"
                                                % (m.cn, m.x, m.y))
                                if not _hit_marker:
                                    killed_pts.append((_kx, _ky))
                                    log("已消灭：目标 (%.1f, %.1f)" % (_kx, _ky))
                                if view is not None:
                                    view.set_markers(markers)
                        except Exception:
                            pass
                        # 士兵一发 5 连发就打光；不等“从探测列表消失”（尸体会一直
                        # 被车载/无人机看到，之前就是这里导致永远不释放）。
                        if types[best] == "Soldier":
                            _kx = float(ob.x_ue[best])
                            _ky = float(ob.y_ue[best])
                            _marked = False
                            for m in markers:
                                if (not m.extra
                                        and math.hypot(m.x - _kx,
                                                       m.y - _ky) < 40.0):
                                    m.extra = True
                                    _marked = True
                                    if not any(math.hypot(m.x - q[0],
                                                          m.y - q[1]) < 20.0
                                               for q in killed_pts):
                                        killed_pts.append((m.x, m.y))
                                        log("已消灭：%s (%.1f, %.1f)"
                                            % (m.cn, m.x, m.y))
                            if not _marked and not any(
                                    math.hypot(_kx - q[0], _ky - q[1]) < 20.0
                                    for q in killed_pts):
                                killed_pts.append((_kx, _ky))
                                log("已消灭：目标 (%.1f, %.1f)" % (_kx, _ky))
                            if view is not None:
                                view.set_markers(markers)
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
            if ugv_idle and uav_scanned and engaged_once and (t0 - last_contact) > 10.0:
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
