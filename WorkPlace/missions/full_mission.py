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


def assign_targets(points, starts, n=3):
    """把 n 个目标点分给 n 台车，使总直线距离最短（暴力枚举，n=3 时最多几百次）。"""
    if not points:
        return []
    if len(points) < n:
        # 目标"不够"才循环分配（多余的车间距靠超车逻辑处理）；
        # 恰好相等时必须走下面的最优匹配，否则会出现"两车同一目标"。
        return [points[i % len(points)] for i in range(n)]

    best, best_cost = None, None
    for combo in itertools.combinations(points, n):
        for perm in itertools.permutations(combo, n):
            cost = sum(math.hypot(perm[i][0] - starts[i][0],
                                  perm[i][1] - starts[i][1]) for i in range(n))
            if best_cost is None or cost < best_cost:
                best, best_cost = perm, cost
    return list(best)


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
    g.add_argument("--uav-route", choices=("sweep", "enemies", "strips"),
                   default="sweep",
                   help="sweep=牛耕式覆盖扫描（默认，敌人位置未知时的正确做法）；"
                        "enemies=按已知敌人位置排航线；"
                        "strips=game.json 里的固定条带图案")
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
                   help="运动学模式下的飞行高度 m（默认250）。"
                        "别调太低：100 m 会被楼挡住，撞楼掉血归零后 actor 失效，"
                        "探测全部失效（表现为机高显示 ? 且一个目标都发现不了）")

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
    g.add_argument("--dispatch-gap", type=float, default=4.0,
                   help="三台车错开发车的间隔秒数（默认4）。同时出发容易在同一个路口"
                        "挤成一团，错开能显著减少卡死")
    g.add_argument("--ugv-deadline", type=float, default=300.0,
                   help="无人车出发后最多跑多少秒（默认300）；超时直接收尾，"
                        "避免卡在个别车一直 drive 上不结束")
    g.add_argument("--console-every", type=float, default=5.0,
                   help="控制台心跳间隔秒数（默认5；0=关闭）")
    g.add_argument("--takeoff-timeout", type=float, default=60.0,
                   help="无人机起飞超时秒数（默认60；超时会大声提示飞控驱动问题）")
    g.add_argument("--ignore-driver-check", action="store_true",
                   help="跳过飞控驱动进程检查")

    value_opts = ("--scene", "--node-nos", "--focus", "--sweep-region")
    args = ap.parse_args(_fix_leading_dash(sys.argv[1:], value_opts))
    return args


# ---------------------------------------------------------------------------
#  主流程
# ---------------------------------------------------------------------------
def main():
    args = parse_args()

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
    starts = dict((n, uav_vehicles[n].position()) for n in cu.UAV_NAMES)

    # ---- 先算任务区域（扫描区域与局部地图共用同一个矩形）----
    search_roi = None
    try:
        uav_pos = [starts[n] for n in cu.UAV_NAMES]
        ugv_pos = []
        for v in ugvs:
            st = v.get_state().pose_
            ugv_pos.append((float(st.x_), float(st.y_)))
        search_roi = compute_search_roi(uav_pos, ugv_pos, markers, args)
        if search_roi:
            log("任务区域：中心 (%.0f, %.0f)  ±%.0f m（%.0f×%.0f）"
                % (search_roi[0], search_roi[1], search_roi[2],
                   2 * search_roi[2], 2 * search_roi[2]))
    except Exception as e:
        log("任务区域计算失败（用出生点默认范围）: %r" % (e,))

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
    log("无人机航线已规划：%s" % dict((n, len(routes[n])) for n in cu.UAV_NAMES))

    # ---------------- 无人车路网 ----------------
    binary, raw_bin, ugv_scale, ugv_off = ugv.load_binary(game)
    log("无人车路网已加载（骨架 + 真实路宽）")

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
        # 把本轮真正的搜索区域画出来（青绿实线），与场景边界区分
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
    exit_reason = "运行到 --duration 上限"

    log("开始：无人机起飞侦查（Z=250，15 m/s）")
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
                log("!! 无人机 %.0fs 仍停在起飞阶段，高度 %s（目标 %.0f m）"
                    % (el, zs, cu.ALTITUDE))
                log("   命令下发返回成功、但高度一点没变 —— 基本可以确定是飞控驱动的问题。")
                log("   " + DRIVER_HINT)
                log("   日志: %s" % log_path)
                log("=" * 62)
            if takeoff_warned and uav_ok and el > 2.0 * args.takeoff_timeout \
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
            if enemies and markers:
                det = [{"x": p[0], "y": p[1]} for p in enemies]
                found, _ = lt.match_detections(det, markers, tol=args.match_tol)
                changed = False
                for i in found:
                    if not markers[i].discovered:
                        markers[i].discovered = True
                        changed = True
                        log("发现目标 #%d %s (%.1f, %.1f)"
                            % (i, markers[i].cn, markers[i].x, markers[i].y))
                if changed and view is not None:
                    view.set_markers(markers)

            # ---- 3. 是否派出无人车 ----
            if (not args.no_ugv) and not ugv_agents:
                pool = list(enemies)
                ready = len(pool) >= args.min_targets

                # 无人机全死了/全飞完了，就别再空等 —— 有几个目标派几个
                if (not ready) and len(pool) > 0:
                    try:
                        uav_done = runner.all_searching() or (
                            getattr(runner, "dead", set())
                            and len(getattr(runner, "dead", set())) >= len(cu.UAV_NAMES))
                    except Exception:
                        uav_done = False
                    if uav_done:
                        log("无人机已无更多可侦查目标，现有 %d 个目标就派车"
                            "（不足 --min-targets %d）" % (len(pool), args.min_targets))
                        ready = True

                if not ready and el > args.recon_timeout:
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
                        starts_xy = [(v.get_state().pose_.x_,
                                      v.get_state().pose_.y_) for v in ugvs]
                        chosen = assign_targets(pts, starts_xy, len(UGV_NAMES))
                        for i, name in enumerate(UGV_NAMES):
                            ugv_targets[name] = chosen[i]
                            a = ugv.UgvAgent(ugvs[i], name, chosen[i],
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
                last_view = t0

            # ---- 6. 结束判定 ----
            if ugv_agents and all(a.status in ("arrived", "no_path", "stuck",
                                               "max_replan") for a in ugv_agents):
                exit_reason = "无人车全部结束（到达或上报）"
                break
            # 派车后跑太久就收尾：个别车会一直 drive（绕路/被堵），
            # 不设上限的话整轮会拖到 --duration（900 s），看起来像卡死。
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
        try:
            runner.stop()
        except Exception:
            pass
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
