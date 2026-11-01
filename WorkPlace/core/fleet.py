# -*- coding: utf-8 -*-
"""
fleet.py —— 无人平台编队接口层（位置查询 / 目的地下发 / 分车规划 / 分车控制）
================================================================================

这是给"上层任务脚本"用的 API，回答三个问题：

    1. 现在每台车/机在哪？        ->  fleet.snapshot()
    2. 我要让某台去哪？            ->  fleet.set_target("UGV2", x, y)
    3. 让它真的开过去。            ->  fleet.plan(...) 然后 loop: fleet.step(dt)

设计要点
--------
* **每台平台有自己的目标**。不再是一个全局 --goal 让所有车往一个点挤。
* **车走道路，机走直线**：
    - 无人车：A* 在路网骨架上规划，Stanley + PID 闭环跟踪；
    - 无人机：直接飞向目标点（越障能力由高度保证），按航点推进。
  这样 UAV 才不会被强行拴在道路上——现实里它们不需要。
* **同目标的车自动错开**：多台车分到同一个目标点时，按 --spacing 给每台加
  沿路径法向的偏移，避免叠在一起。
* **A* 结果缓存**：同一个目标点只算一次 A*，其余车复用再偏移。全图 4794² 的
  A* 很慢，这一步能把启动时间从「6 次」压到「1 次」。

用法
----
    from swarmae.SwarmAEClient import SwarmAEClient
    from core.fleet import Fleet

    client = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    fleet = Fleet(client)                       # 注册 3 车 3 机

    print(fleet.snapshot()["UGV1"])             # 实时位置
    fleet.set_target("UGV1", 1200, -400)        # 指定目的地
    fleet.set_target("UAV1", 1200, -400)
    fleet.plan(scene, img_skl)                  # 规划（车 A*，机直线）

    import time
    while True:
        dt = 0.05
        fleet.step(dt)
        print(fleet.status())
        time.sleep(dt)

也可以完全不碰 Python：把目的地写进 JSON，脚本轮询即可（演示时最方便）

    # config/deploy.json
    { "UGV1": [1200, -400], "UAV1": {"x": 1200, "y": -400, "alt": 70} }

    fleet.poll_deploy_json("WorkPlace/config/deploy.json")   # 文件一改，车就转向
"""

from __future__ import print_function

import json
import math
import os
import sys
import time

import numpy as np

# 复用 easy_follow 里的算法与封装
_HERE = os.path.dirname(os.path.abspath(__file__))
_WORKPLACE = os.path.dirname(_HERE)
_PROJECT = os.path.dirname(_WORKPLACE)
_EASY_FOLLOW = os.path.join(_PROJECT, "Materials", "practice", "easy_follow")

if _EASY_FOLLOW not in sys.path:
    sys.path.insert(0, _EASY_FOLLOW)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# NumPy 2.x 兼容：lib/*.py 在类型注解里用了 np.mat（2.0 已移除）
if not hasattr(np, "mat"):
    np.mat = np.asmatrix

import lib.swarm_api as api
import lib.controller as control
import lib.data_struct as struct
import lib.frenet_optimal_planner as fop
import lib.global_planner as gp
import lib.image_process as imp

try:
    from lib.formation_calculate import FormationToolkit
    _HAS_FORMATION = True
except Exception:
    _HAS_FORMATION = False


# ---------------------------------------------------------------------------
#  数据结构
# ---------------------------------------------------------------------------
class Pose(object):
    """一次快照里的单台平台状态（全部是"米 / 弧度"）。"""
    __slots__ = ("name", "type", "x", "y", "z", "yaw", "v", "moving", "target", "arrived")

    def __init__(self, name, vtype, x, y, z, yaw, v):
        self.name = name
        self.type = vtype
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.v = v
        self.moving = abs(v) > 0.5
        self.target = None
        self.arrived = False

    def to_dict(self):
        return dict(name=self.name, type=self.type,
                    x=self.x, y=self.y, z=self.z,
                    yaw=self.yaw, yaw_deg=math.degrees(self.yaw), v=self.v,
                    moving=self.moving,
                    target=self.target, arrived=self.arrived)

    def __repr__(self):
        return ("<Pose %s (%.1f, %.1f, %.1f) v=%.1f%s>"
                % (self.name, self.x, self.y, self.z, self.v,
                   " MOVING" if self.moving else ""))


# ---------------------------------------------------------------------------
#  队形偏移小工具（同目标多车错开用）
# ---------------------------------------------------------------------------
def lateral_offsets(n, spacing):
    """把 n 台车在路径法向上均匀摊开，返回 [(lat, lon), ...]。"""
    if n <= 1:
        return [(0.0, 0.0)]
    mid = (n - 1) / 2.0
    return [((mid - i) * spacing, 0.0) for i in range(n)]


def _offset_path(base, lat, lon):
    """沿法向/弧长偏移参考线；失败就退回原线（不能让规划整个崩掉）。"""
    if abs(lat) < 1e-6 and abs(lon) < 1e-6:
        return base
    if not _HAS_FORMATION:
        return base
    try:
        return FormationToolkit().offset_path(base, lateral_offset=lat,
                                              longitudinal_offset=lon)
    except Exception:
        return base


def _straight_path(x1, y1, x2, y2, resolution=2.0):
    """两点直线参考线（无人机用）。距离太近就补两个点，避免样条退化。"""
    d = math.hypot(x2 - x1, y2 - y1)
    if d < 1e-6:
        x2, y2, d = x1 + 0.5, y1, 0.5
    n = max(2, int(d / max(resolution, 0.5)) + 1)
    xs = np.linspace(x1, x2, n)
    ys = np.linspace(y1, y2, n)
    return fop.ReferencePath(np.mat(xs).T, np.mat(ys).T, resolution=resolution)


# ---------------------------------------------------------------------------
#  单台平台的控制器
# ---------------------------------------------------------------------------
class UgvCtrl(object):
    """Stanley 横向 + PID 纵向，结构与 practice/plan_control.py 同源。"""

    def __init__(self, k_stanley=1.0, kp=0.25, ki=0.0, kd=0.0, lookahead=10.0):
        self.stanley = control.Stanley(k=k_stanley)
        self.pid = control.PID(Kp=kp, Ki=ki, Kd=kd)
        self.lookahead = float(lookahead)

    def command(self, st, ref, v_target, dt, remaining):
        fx = st.pose_.x_ + self.lookahead * math.cos(st.pose_.theta_)
        fy = st.pose_.y_ + self.lookahead * math.sin(st.pose_.theta_)
        rx = np.ravel(ref.interp_x_)
        ry = np.ravel(ref.interp_y_)
        idx = int(np.argmin(np.abs(rx - fx) + np.abs(ry - fy)))

        tgt = struct.Transform(float(rx[idx]), float(ry[idx]),
                               float(np.ravel(ref.interp_theta_)[idx]))
        delta = self.stanley.update_control(tgt, st)
        steer = delta / (math.pi / 6.0)

        # 临近终点按剩余距离压速度，避免冲过头
        v_allow = math.sqrt(max(0.0, 2.0 * 1.5 * remaining)) + 0.6
        v_cmd = min(v_target, v_allow)
        accel = self.pid.update_control(v_cmd, st.v_, dt)
        if accel > 0:
            return float(np.clip(steer, -1, 1)), float(np.clip(accel, 0, 1)), 0.0, False, v_cmd
        return (float(np.clip(steer, -1, 1)), 0.0,
                float(np.clip(abs(accel) * 0.5, 0, 1)), False, v_cmd)


class UavCtrl(object):
    """无人机是开环"给点+给速"飞控，所以只需要航点推进 + 到达判定。"""

    def __init__(self, lookahead=60.0, tol=8.0):
        self.lookahead = float(lookahead)
        self.tol = float(tol)

    def command(self, st, ref, alt, v_target, idx_hint=0):
        xs = np.ravel(ref.interp_x_)
        ys = np.ravel(ref.interp_y_)
        n = len(xs)
        res = float(ref.sample_resolution_)

        # 只在当前索引附近找投影，避免路径自身折返时"跳回去"
        lo = max(0, idx_hint - 40)
        hi = min(n, idx_hint + 600)
        d2 = (xs[lo:hi] - st.pose_.x_) ** 2 + (ys[lo:hi] - st.pose_.y_) ** 2
        idx = lo + int(np.argmin(d2)) if hi > lo else n - 1

        ahead = int(round(self.lookahead / max(res, 1e-3)))
        j = min(n - 1, idx + ahead)
        done = math.hypot(xs[-1] - st.pose_.x_, ys[-1] - st.pose_.y_) < self.tol
        return float(xs[j]), float(ys[j]), float(alt), float(v_target), done, idx


# ---------------------------------------------------------------------------
#  Fleet
# ---------------------------------------------------------------------------
class Fleet(object):
    TARGET_TOL_UGV = 8.0        # 车到点判定（米）
    TARGET_TOL_UAV = 10.0

    def __init__(self, client, ugv_names=("UGV1", "UGV2", "UGV3"),
                 uav_names=("UAV1", "UAV2", "UAV3"),
                 veh_type_map=None, verbose=True):
        self.client = client
        self.verbose = verbose
        self._v = {}            # name -> api.Vehicle
        self._type = {}         # name -> "ugv"/"uav"
        self._goal = {}         # name -> (x, y)
        self._alt = {}          # name -> 期望高度
        self._ref = {}          # name -> ReferencePath
        self._ctrl = {}
        self._idx = {}          # uav 航点索引记忆
        self._arrived = {}
        self._astar_cache = {}  # (gx,gy) -> path_px
        self._deploy_mtime = None
        self.move_uav = True    # 置 False 则无人机只悬停不追航点

        self.ugv_names = list(ugv_names)
        self.uav_names = list(uav_names)

        for i, nm in enumerate(self.ugv_names):
            self._v[nm] = api.Vehicle(client, vehicle_name=nm, vehicle_no=i + 1,
                                      vehicle_type="ugv")
            self._type[nm] = "ugv"
            self._ctrl[nm] = UgvCtrl()
            self._arrived[nm] = False
        for i, nm in enumerate(self.uav_names):
            self._v[nm] = api.Vehicle(client, vehicle_name=nm, vehicle_no=4 + i,
                                      vehicle_type="uav")
            self._type[nm] = "uav"
            self._ctrl[nm] = UavCtrl()
            self._arrived[nm] = False

        if self.verbose:
            print("[fleet] 已注册 %d 车 %d 机: %s"
                  % (len(self.ugv_names), len(self.uav_names),
                     ", ".join(self.ugv_names + self.uav_names)))

    # ---------------- 基本属性 ----------------
    def names(self):
        return self.ugv_names + self.uav_names

    def vehicle(self, name):
        return self._v[name]

    def vehicles(self, vtype=None):
        return [self._v[n] for n in self.names()
                if vtype is None or self._type[n] == vtype]

    def is_ugv(self, name):
        return self._type[name] == "ugv"

    # ---------------- 接口 1：实时位置 ----------------
    def snapshot(self):
        """返回 {name: Pose}。这是"实时判断该去哪"的数据源。"""
        out = {}
        for nm in self.names():
            v = self._v[nm]
            st = v.get_state()
            p = Pose(nm, self._type[nm],
                     st.pose_.x_, st.pose_.y_, st.pose_.z_,
                     st.pose_.theta_, st.v_)
            p.target = self._goal.get(nm)
            p.arrived = self._arrived.get(nm, False)
            out[nm] = p
        return out

    def positions(self):
        """只要坐标的轻量版本：{name: (x, y, z)}。"""
        return {nm: (p.x, p.y, p.z) for nm, p in self.snapshot().items()}

    def status(self):
        """带上目标/剩余距离/是否到达，适合直接打印或塞进 HUD。"""
        snap = self.snapshot()
        out = {}
        for nm, p in snap.items():
            g = self._goal.get(nm)
            rem = None if g is None else math.hypot(g[0] - p.x, g[1] - p.y)
            out[nm] = dict(type=p.type, x=p.x, y=p.y, z=p.z,
                           v=p.v, moving=p.moving,
                           target=g, remaining=rem,
                           arrived=self._arrived.get(nm, False))
        return out

    # ---------------- 接口 2：下发目的地 ----------------
    def set_target(self, name, x, y, alt=None):
        if name not in self._v:
            raise KeyError("未知平台: %s（可用: %s）" % (name, ", ".join(self.names())))
        self._goal[name] = (float(x), float(y))
        if alt is not None:
            self._alt[name] = float(alt)
        self._arrived[name] = False
        self._ref.pop(name, None)      # 目标变了，旧参考线作废
        if self.verbose:
            print("[fleet] %-5s -> (%.1f, %.1f)%s"
                  % (name, x, y, "" if alt is None else "  alt=%.0f" % alt))

    def set_targets(self, mapping):
        """mapping: {name: (x,y)} 或 {name: {"x":..,"y":..,"alt":..}}"""
        for name, val in mapping.items():
            if isinstance(val, dict):
                self.set_target(name, val["x"], val["y"], val.get("alt"))
            elif isinstance(val, (list, tuple)) and len(val) >= 2:
                self.set_target(name, val[0], val[1])
            else:
                raise ValueError("无法解析 %s 的目标: %r" % (name, val))

    def clear_targets(self, names=None):
        for nm in (names or self.names()):
            self._goal.pop(nm, None)
            self._ref.pop(nm, None)
            self._arrived[nm] = False

    def targets(self):
        return dict(self._goal)

    def set_altitude(self, alt, names=None):
        for nm in (names or self.uav_names):
            self._alt[nm] = float(alt)

    def detect_targets(self, name="UGV1"):
        """借用某台车的探测能力，返回 [{'type','x','y','z','health','kind'}]。

        单位判别：任务区在 ±2300 米内，原始值出现 |v|>10000 说明是厘米。
        """
        node = self._v[name].Vehicle_
        try:
            ret = node.detect_situation()
        except Exception as e:
            if self.verbose:
                print("[fleet] detect_situation 失败: %r" % (e,))
            return []
        if not isinstance(ret, (tuple, list)) or len(ret) < 3:
            return []
        try:
            objs = json.loads(ret[2])
        except Exception:
            return []
        if not objs:
            return []
        mx = 0.0
        for o in objs:
            try:
                mx = max(mx, abs(float(o["Position"]["X"])), abs(float(o["Position"]["Y"])))
            except Exception:
                pass
        k = 0.01 if mx > 10000 else 1.0
        out = []
        for o in objs:
            try:
                p = o["Position"]
                out.append(dict(type=o.get("Type_Name") or o.get("Type") or "Unknown",
                                x=float(p["X"]) * k, y=float(p["Y"]) * k,
                                z=float(p.get("Z", 0)) * k,
                                health=o.get("HealthPoint", 0) or 0,
                                kind=o.get("Type", "")))
            except Exception:
                continue
        return out

    # ---------------- 接口 3：规划 ----------------
    def plan(self, scene, img_skl, spacing=15.0, uav_alt=60.0,
             astar_scale=1, verbose=True):
        """给每台有目标的平台生成参考线。

        车：A* 走道路（同一目标只算一次，其余车法向错开）
        机：直线飞向目标
        """
        # ---- 按目标分组，决定谁跑 A* ----
        groups = {}
        for nm, g in self._goal.items():
            if self.is_ugv(nm):
                key = (round(g[0], 1), round(g[1], 1))
                groups.setdefault(key, []).append(nm)

        astar_runs = 0
        for key, members in groups.items():
            gx, gy = key
            leader = members[0]
            cache_key = key
            if cache_key in self._astar_cache:
                base = self._astar_cache[cache_key]
            else:
                start = self._v[leader].get_state().pose_
                sx, sy = scene.to_px(start.x_, start.y_)
                px, py = scene.to_px(gx, gy)
                sx, sy, px, py = imp.skeleton_point(img_skl, sx, sy, px, py)
                t0 = time.time()
                base = gp.AStar(int(sx), int(sy), int(px), int(py),
                                img_skl, scale=astar_scale).update_planning()
                astar_runs += 1
                if not base:
                    if verbose:
                        print("[fleet] A* 失败: %s -> (%.1f, %.1f)，跳过" % (leader, gx, gy))
                    continue
                arr = np.array(base, dtype=float)
                wx = arr[:, 0] / scene.scale + scene.offset[0]
                wy = arr[:, 1] / scene.scale + scene.offset[1]
                base = fop.ReferencePath(np.mat(wx).T, np.mat(wy).T, resolution=2)
                self._astar_cache[cache_key] = base
                if verbose:
                    print("[fleet] A* %-5s -> (%.1f,%.1f)  %.1fs  %.0f m  %d pts"
                          % (leader, gx, gy, time.time() - t0,
                             base.total_length_, base.interp_size_))

            # 同目标的几台车沿法向错开
            offs = lateral_offsets(len(members), spacing)
            for nm, (lat, lon) in zip(members, offs):
                self._ref[nm] = _offset_path(base, lat, lon) if len(members) > 1 else base

        # ---- 无人机：直线 ----
        for nm in self.uav_names:
            g = self._goal.get(nm)
            if g is None:
                continue
            st = self._v[nm].get_state().pose_
            self._ref[nm] = _straight_path(st.x_, st.y_, g[0], g[1])
            self._idx[nm] = 0

        if verbose:
            print("[fleet] 规划完成：%d 次 A*，%d 条参考线"
                  % (astar_runs, len(self._ref)))
        return self._ref

    # ---------------- 接口 4：控制一步 ----------------
    def step(self, dt, speed=12.0, uav_speed=14.0, uav_alt=60.0):
        """下发一次控制。返回 {name: dict(cmd=..., arrived=...)}"""
        report = {}
        snap = self.snapshot()

        # 无人车：按目标分组保持车距
        ugv_with_goal = [nm for nm in self.ugv_names if nm in self._ref]
        for nm in ugv_with_goal:
            v = self._v[nm]
            p = snap[nm]
            ref = self._ref[nm]
            gx = float(np.ravel(ref.interp_x_)[-1])
            gy = float(np.ravel(ref.interp_y_)[-1])
            remaining = math.hypot(gx - p.x, gy - p.y)

            if remaining < self.TARGET_TOL_UGV or self._arrived[nm]:
                self._arrived[nm] = True
                v.apply_control(0.0, 0.0, 1.0, False)
                report[nm] = dict(arrived=True, remaining=remaining)
                continue

            v_t = float(speed)
            # 跟车：与前一台同目标车保持间距
            for other in ugv_with_goal:
                if other == nm:
                    continue
                po = snap[other]
                if po.arrived:
                    continue
                gap = math.hypot(po.x - p.x, po.y - p.y)
                ahead = (np.ravel(self._ref[other].interp_x_)[-1] -
                         np.ravel(ref.interp_x_)[-1]) ** 2
                if gap < 20.0:
                    v_t = min(v_t, max(2.0, speed * (gap / 20.0)))

            steer, thr, brk, hb, v_cmd = self._ctrl[nm].command(
                self._as_state(nm, p), ref, v_t, dt, remaining)
            v.apply_control(steer, thr, brk, hb)
            report[nm] = dict(arrived=False, remaining=remaining, v_cmd=v_cmd)

        # 无人机：推到下一个航点
        for nm in self.uav_names:
            if nm not in self._ref:
                continue
            if not self.move_uav:
                p = snap[nm]
                self._v[nm].target_point_control(p.x, p.y, self._alt.get(nm, uav_alt), 0.0)
                report[nm] = dict(arrived=True, hovering=True)
                continue
            v = self._v[nm]
            p = snap[nm]
            alt = self._alt.get(nm, uav_alt)
            tx, ty, tz, tv, done, idx = self._ctrl[nm].command(
                self._as_state(nm, p), self._ref[nm], alt, uav_speed,
                self._idx.get(nm, 0))
            self._idx[nm] = idx
            if done or self._arrived[nm]:
                self._arrived[nm] = True
                v.target_point_control(p.x, p.y, alt, 0.0)
                report[nm] = dict(arrived=True)
            else:
                v.target_point_control(tx, ty, tz, tv)
                report[nm] = dict(arrived=False)

        return report

    def _as_state(self, nm, pose):
        return struct.State(struct.Transform(pose.x, pose.y, pose.yaw, pose.z), pose.v, 0.0)

    def arrived(self, name=None):
        if name:
            return self._arrived.get(name, False)
        return all(self._arrived.get(n, False) for n in self._goal)

    def stop_all(self):
        for nm in self.ugv_names:
            try:
                self._v[nm].apply_control(0.0, 0.0, 1.0, False)
            except Exception:
                pass
        for nm in self.uav_names:
            try:
                p = self.snapshot()[nm]
                self._v[nm].target_point_control(p.x, p.y, self._alt.get(nm, 60.0), 0.0)
            except Exception:
                pass

    # ---------------- 接口 5：从 JSON 下发（演示最方便） ----------------
    def prime_deploy(self, path):
        """启动时先记下文件当前时间戳，避免"一启动就被文件里的旧目标覆盖"。

        这样只有**运行中真正改过** deploy.json 才会触发重新规划。
        """
        try:
            self._deploy_mtime = os.path.getmtime(path) if path else None
        except OSError:
            self._deploy_mtime = None
        return self._deploy_mtime

    def poll_deploy_json(self, path, reload_tol=0.05):
        """目标文件一变就重新下发。返回 True 表示这次读到了新目标。"""
        if not path or not os.path.isfile(path):
            return False
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return False
        if self._deploy_mtime is not None and abs(mtime - self._deploy_mtime) < reload_tol:
            return False
        self._deploy_mtime = mtime
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print("[fleet] 读取 %s 失败: %r" % (path, e))
            return False

        if not isinstance(data, dict):
            print("[fleet] %s 应是一个 JSON 对象 {平台名: [x, y]}" % path)
            return False

        if data.get("__clear__") or data.get("__清空__"):
            self.clear_targets()
        payload = {k: v for k, v in data.items() if not k.startswith("__")}
        if payload:
            self.set_targets(payload)
            print("[fleet] 从 %s 收到 %d 条新目标，需要重新 plan()" % (path, len(payload)))
        return True
