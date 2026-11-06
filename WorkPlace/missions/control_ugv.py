# -*- coding: utf-8 -*-
"""Step 5: 3 台 UGV 各自驶向一个目标 + 无路可走兜底。

每台 UGV 独立走 step4 的完整闭环: 边开边探测路障 → 封格子 → 重规划 → 死路倒车退出 → 继续。
A* 无解 / 重规划超限 / 撞停 → 停车上报(无路可走)。

三台在同一个主循环里轮流控制(单线程, 避免 SDK 并发), 倒车做成非阻塞状态机。
"""
import os
import time
import math
import heapq
import itertools
import numpy as np
from PIL import Image

from swarmae import SwarmAENode as SN
from swarmae.SwarmAEClient import SwarmAEClient

import lib.swarm_api as api
import lib.controller as control
import lib.data_struct as struct
import lib.utils as utils

_HERE = os.path.dirname(os.path.abspath(__file__))
# 资源图统一放到 WorkPlace/assets/（原脚本假设图片与 .py 同目录）
_ASSETS = os.path.join(os.path.dirname(_HERE), "assets")
SKL_PNG = os.path.join(_ASSETS, "skeletonized_image.png")
RAW_PNG = os.path.join(_ASSETS, "roadnet_raw.png")

# ---- 参数 ----
TARGET_V     = 5.0     # 直道巡航 (m/s)
SLOW_V       = 2.5     # 临近拐弯 (m/s)
SLOW_RANGE   = 20.0    # 距拐弯多少米减速 (m)
LOOKAHEAD    = 8.0     # 预瞄距离 (m)
ARRIVE_DIST  = 20.0    # 距路网终点多近算到达 (m)
CLEARANCE    = 10.0    # 路障中心离路径多近算挡路 (m)
MAX_RUNTIME  = 600.0   # 每台最长行驶 (s)
MAX_REPLAN   = 5       # 每台最多重规划次数
BLOCK_R      = 10      # 封格子半径 (px≈m)
REVERSE_OVERSHOOT = 12.0   # 倒车退出路口时多倒 (m)
REVERSE_THROTTLE  = -0.7   # 倒车油门
DEV_WARN     = 5.0     # 距路中心线超过多少算偏离 (m)
FRONT_GAP    = 18.0    # 正前方多近有队友(其它UGV)就刹停等待, 防追尾 (m)

# ---- 超车参数 ----
OVERTAKE_TIMEOUT = 3.0    # 前方队友连续停多久(秒)判定为停死, 触发超车
OVERTAKE_OFFSET  = 5.0    # 超车最大横向偏移 (m)
OVERTAKE_LEAD    = 12.0   # 超车 S 形侧偏的进入/退出距离 (m)
OVERTAKE_HOLD    = 8.0    # 越过挡路车后继续保持偏移多远 (m)
OVERTAKE_MARGIN  = 1.0    # 距挡路车保留的安全余量 (m)
CAR_HALF_W       = 1.5    # 车半宽 (m)

SPAWNS  = [(-1829.6, -517.7, 2.1),
           (-1859.6, -517.4, 2.1),
           (-1887.4, -517.4, 2.1)]
TARGETS = [(-1000.0, -520.0),   # UGV1: 东(过路障, 唯一绕行者)
           (-1500.0, -217.0),   # UGV2: 南(在x=-1488.5拐南)
           (-1200.0, -870.0)]   # UGV3: 北(在x=-1488.5拐北, 目标落在北路y≈-870上)


def load_binary(game):
    _, scale, world_offset, _, _ = game.get_road_network()
    skl = np.array(Image.open(SKL_PNG))
    binary = np.all(skl >= 200, axis=-1) if skl.ndim == 3 else (skl >= 200)
    raw = np.array(Image.open(RAW_PNG).convert('L'))
    raw_bin = raw >= 128
    return binary, raw_bin, scale, world_offset


def w2p(x, y, scale, off):
    return int(scale * (x - off[0])), int(scale * (y - off[1]))


def p2w(px, py, scale, off):
    return px / scale + off[0], py / scale + off[1]


def nearest_road(binary, px, py, search=60):
    H, W = binary.shape
    best, best_pt = None, None
    for dy in range(-search, search + 1):
        for dx in range(-search, search + 1):
            nx, ny = px + dx, py + dy
            if 0 <= nx < W and 0 <= ny < H and binary[ny, nx]:
                d = dx * dx + dy * dy
                if best is None or d < best:
                    best, best_pt = d, (nx, ny)
    return best_pt


def astar(binary, start, goal):
    H, W = binary.shape

    def h(p):
        return math.hypot(p[0] - goal[0], p[1] - goal[1])

    g = {start: 0.0}
    parent = {start: None}
    cnt = itertools.count(1)
    heap = [(h(start), 0, start)]
    closed = set()
    moves = [(1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        cx, cy = cur
        for dx, dy in moves:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if not binary[ny, nx]:
                continue
            nxt = (nx, ny)
            if nxt in closed:
                continue
            ng = g[cur] + (1.414 if dx and dy else 1.0)
            if ng < g.get(nxt, 1e18):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(heap, (ng + h(nxt), next(cnt), nxt))
    return None


def block_disk(grid, cx, cy, r):
    H, W = grid.shape
    yy, xx = np.ogrid[:H, :W]
    grid[(xx - cx) ** 2 + (yy - cy) ** 2 <= r * r] = False


def nearest_idx(path, x, y):
    best_i, best_d = 0, 1e18
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def lookahead_pose(path, x, y, L, span=5):
    n = len(path)
    i = nearest_idx(path, x, y)
    acc = 0.0
    for k in range(i, n - 1):
        x1, y1 = path[k]
        x2, y2 = path[k + 1]
        seg = math.hypot(x2 - x1, y2 - y1)
        if acc + seg >= L:
            frac = (L - acc) / seg if seg > 0 else 0.0
            tx = x1 + frac * (x2 - x1)
            ty = y1 + frac * (y2 - y1)
            j = min(k + span, n - 1)
            theta = math.atan2(path[j][1] - path[k][1], path[j][0] - path[k][0])
            return struct.Transform(tx, ty, theta)
        acc += seg
    j = n - 1
    i0 = max(0, j - span)
    theta = math.atan2(path[j][1] - path[i0][1], path[j][0] - path[i0][0])
    return struct.Transform(path[j][0], path[j][1], theta)


def detect_corners(path, span=6, min_angle_deg=30):
    corners = []
    n = len(path)
    for i in range(span, n - span):
        xa, ya = path[i - span]
        xb, yb = path[i]
        xc, yc = path[i + span]
        th1 = math.atan2(yb - ya, xb - xa)
        th2 = math.atan2(yc - yb, xc - xb)
        if abs(utils.normalize_angle(th2 - th1)) > math.radians(min_angle_deg):
            corners.append(i)
    return corners


def is_barricade(t):
    t = (t or "").lower()
    return t == "other" or "barricade" in t


def is_teammate_ugv(t):
    """探测到的其它 UGV (RaiderBuggy) —— 不包含无人机 HRT-M10。"""
    t = (t or "").lower()
    return "raider" in t or "buggy" in t


def overtake_envelope(s, lead, hold):
    """超车横向偏移包络: s 为相对挡路车的沿路径带符号距离(负=后方)。"""
    if s <= -lead:
        return 0.0
    if s < 0.0:
        return 0.5 * (1.0 - math.cos(math.pi * (-s) / lead))
    if s < hold:
        return 1.0
    if s < hold + lead:
        return 0.5 * (1.0 + math.cos(math.pi * (s - hold) / lead))
    return 0.0


def needs_reverse(path, pose, look=6):
    if len(path) < look + 1:
        return False
    x0, y0 = path[0]
    x1, y1 = path[look]
    dx, dy = x1 - x0, y1 - y0
    if dx == 0 and dy == 0:
        return False
    fwd = (math.cos(pose.theta_), math.sin(pose.theta_))
    return (dx * fwd[0] + dy * fwd[1]) < 0


class UgvAgent:
    def __init__(self, ugv, name, target, binary, raw, scale, off):
        self.ugv = ugv
        self.name = name
        self.target = target
        self.scale = scale
        self.off = off
        self.raw = raw
        self.blocked_grid = binary.copy()
        self.blocked_obstacles = set()
        self.path = None
        self.corners = []
        self.replan_count = 0
        self.status = 'init'   # init/drive/reverse/arrived/no_path/stuck/max_replan
        self.stanley = control.Stanley(k=1)
        self.pid = control.PID(Kp=0.1, Ki=0.0, Kd=0.0)
        self.t0 = time.time()
        self.prev = time.time()
        self.max_dev = 0.0
        self.last_dev = 0.0
        self.stuck_since = None
        self.rev_target = 0.0
        self.rev_traveled = 0.0
        self.rev_t0 = 0.0
        self.rev_prev = 0.0
        self.wait_since = None       # 开始在前方队友后面等待的时刻
        self.wait_blocker = None     # 等待开始时挡路队友的位置 (世界坐标)
        self.overtaking = False      # 是否正在超车
        self.overtake_blocker = None # 正在超的挡路队友世界坐标 (bx, by)

    # ---- 规划 ----
    def plan_from(self, pose):
        sp = nearest_road(self.blocked_grid, *w2p(pose.x_, pose.y_, self.scale, self.off))
        gp = nearest_road(self.blocked_grid, *w2p(*self.target, self.scale, self.off))
        if sp is None or gp is None:
            return None
        pp = astar(self.blocked_grid, sp, gp)
        if pp is None:
            return None
        return [p2w(px, py, self.scale, self.off) for (px, py) in pp]

    # ---- 终态 ----
    def finish(self, status, msg):
        self.status = status
        self.ugv.apply_control(0, 0, 0.8, False)
        time.sleep(0.3)
        self.ugv.apply_control(0, 0, 0, True)
        print("    [%s] %s" % (self.name, msg))

    # ---- 启动 ----
    def start(self):
        pose = self.ugv.get_transform()
        self.path = self.plan_from(pose)
        if self.path is None:
            self.finish('no_path', "初始就无路可走, 停车上报。")
            return
        self.corners = detect_corners(self.path)
        self.status = 'drive'
        print("    [%s] 初始路径 %d 点 目标 %s" % (self.name, len(self.path), self.target))

    # ---- 单步 ----
    def step(self):
        if self.status in ('arrived', 'no_path', 'stuck', 'max_replan'):
            return
        if self.status == 'reverse':
            self._step_reverse()
        elif self.status == 'drive':
            self._step_drive()

    # ---- 倒车(非阻塞) ----
    def _step_reverse(self):
        imu = self.ugv.get_imu_data()
        dt = time.time() - self.rev_prev
        self.rev_prev = time.time()
        self.rev_traveled += abs(imu.v) * dt
        self.ugv.apply_control(0, REVERSE_THROTTLE, 0, False)
        if self.rev_traveled >= self.rev_target or (time.time() - self.rev_t0) > 150.0:
            self.ugv.apply_control(0, 0, 0.8, False)
            time.sleep(0.5)
            self.ugv.apply_control(0, 0, 0, True)
            time.sleep(0.2)
            pose = self.ugv.get_transform()
            self.replan_count += 1
            new_path = self.plan_from(pose)
            if new_path is None:
                self.finish('no_path', "倒车后无路可走, 停车上报。")
            else:
                self.path = new_path
                self.corners = detect_corners(new_path)
                self.status = 'drive'
                self.prev = time.time()
                self.stuck_since = None
                print("    [%s] 倒车 %.1f m 完成, 重规划 %d 点" % (self.name, self.rev_traveled, len(new_path)))

    # ---- 超车 ----
    def _clear_overtake(self):
        self.overtaking = False
        self.overtake_blocker = None
        self.wait_since = None
        self.wait_blocker = None

    def _side_width(self, cx, cy, dx, dy):
        """从中心线点 (cx,cy) 沿单位方向 (dx,dy) 数连续路面像素, 返回米数。"""
        H, W = self.raw.shape
        x, y = float(cx) + dx, float(cy) + dy
        n = 0
        while 0 <= int(x) < W and 0 <= int(y) < H and self.raw[int(y), int(x)]:
            x += dx
            y += dy
            n += 1
        return n * self.scale

    def _pick_overtake_side(self, bx, by):
        """在挡路车处沿垂直切向两侧量路宽, 选较宽一侧超车。
        返回 (perp_x, perp_y, off_m) 或 None(两侧都不够宽)。"""
        i = nearest_idx(self.path, bx, by)
        n = len(self.path)
        i0 = max(0, i - 5)
        i1 = min(n - 1, i + 5)
        tx = self.path[i1][0] - self.path[i0][0]
        ty = self.path[i1][1] - self.path[i0][1]
        L = math.hypot(tx, ty) or 1.0
        tx, ty = tx / L, ty / L
        cx, cy = w2p(self.path[i][0], self.path[i][1], self.scale, self.off)
        w_plus = self._side_width(cx, cy, -ty, tx)
        w_minus = self._side_width(cx, cy, ty, -tx)
        if w_plus >= w_minus:
            w_side, perp_x, perp_y = w_plus, -ty, tx
        else:
            w_side, perp_x, perp_y = w_minus, ty, -tx
        need_off = 2.0 * CAR_HALF_W + OVERTAKE_MARGIN   # 最小安全偏移(错开两车+余量)
        max_off = w_side - CAR_HALF_W                   # 不压路沿的最大偏移
        if max_off < need_off:
            return None
        return perp_x, perp_y, min(OVERTAKE_OFFSET, max_off)

    def _build_overtake_path(self, bx, by, perp_x, perp_y, off_m):
        """在挡路车附近给中心线路径加横向偏移, 生成超车路径(世界坐标)。"""
        i = nearest_idx(self.path, bx, by)
        n = len(self.path)
        s = [0.0] * n
        for j in range(i + 1, n):
            s[j] = s[j - 1] + math.hypot(self.path[j][0] - self.path[j - 1][0],
                                         self.path[j][1] - self.path[j - 1][1])
        for j in range(i - 1, -1, -1):
            s[j] = s[j + 1] - math.hypot(self.path[j + 1][0] - self.path[j][0],
                                         self.path[j + 1][1] - self.path[j][1])
        out = []
        for j, (x, y) in enumerate(self.path):
            e = overtake_envelope(s[j], OVERTAKE_LEAD, OVERTAKE_HOLD)
            out.append((x + perp_x * off_m * e, y + perp_y * off_m * e))
        return out

    def _try_overtake(self, blocker):
        """为挡路队友生成超车路径; 成功返回 True 并替换 self.path。"""
        bx, by = blocker
        res = self._pick_overtake_side(bx, by)
        if res is None:
            return False
        perp_x, perp_y, off_m = res
        self.path = self._build_overtake_path(bx, by, perp_x, perp_y, off_m)
        self.corners = detect_corners(self.path)
        self.overtake_blocker = (bx, by)
        self.prev = time.time()
        self.stuck_since = None
        print("    [%s] 前方队友停死, 超车: 侧偏 %.1f m (垂向 %.2f,%.2f)"
              % (self.name, off_m, perp_x, perp_y))
        return True

    def _overtake_done(self, pose):
        """超车中判断是否已越过挡路车(挡路车落到身后一个 lead 距离)。"""
        if self.overtake_blocker is None:
            return True
        bx, by = self.overtake_blocker
        fwd = (math.cos(pose.theta_), math.sin(pose.theta_))
        behind = (bx - pose.x_) * fwd[0] + (by - pose.y_) * fwd[1]
        if behind < -OVERTAKE_LEAD:
            self.overtake_blocker = None
            print("    [%s] 超车完成, 回到中心线" % self.name)
            return True
        return False

    # ---- 前进跟车 ----
    def _step_drive(self):
        pose = self.ugv.get_transform()
        imu = self.ugv.get_imu_data()
        state = struct.State(pose, abs(imu.v), imu.a)

        # 到达 (比路径终点, 已吸附到路)
        # 【修正】只比"路网终点"会误判：目标离路 14 m 时，车已经开到目标旁边
        # （实测距目标 1.1 m），但离路网终点还 >20 m，于是永远不判到达，
        # 停在那儿 8 s 后被判成"疑似撞停 → stuck"。这里两个条件满足其一即到达。
        gx, gy = self.path[-1]
        near_path_end = utils.distance(pose.x_, pose.y_, gx, gy) < ARRIVE_DIST
        near_target = (self.target is not None and utils.distance(
            pose.x_, pose.y_, self.target[0], self.target[1]) < ARRIVE_DIST)
        if near_path_end or near_target:
            self.finish('arrived', "到达目标附近, 停车。位置(%.1f,%.1f) max_dev=%.1f m"
                        % (pose.x_, pose.y_, self.max_dev))
            return

        if (time.time() - self.t0) > MAX_RUNTIME:
            self.finish('stuck', "超时停车。位置(%.1f,%.1f)" % (pose.x_, pose.y_))
            return

        # 探测路障
        ob = self.ugv.detect_fun()

        # 队友避让 / 超车: 正前方近距离有其它 UGV(RaiderBuggy) → 先刹停等待;
        # 连续停死超过 OVERTAKE_TIMEOUT → 侧偏超车。
        if self.overtaking:
            if self._overtake_done(pose):
                self.overtaking = False
        elif len(ob.Type):
            fwd = (math.cos(pose.theta_), math.sin(pose.theta_))
            blocker = None
            for i in range(len(ob.Type)):
                if not is_teammate_ugv(ob.Type[i]):
                    continue
                dx = ob.x_ue[i] - pose.x_
                dy = ob.y_ue[i] - pose.y_
                d = math.hypot(dx, dy)
                if 0.0 < d < FRONT_GAP and (dx * fwd[0] + dy * fwd[1]) > 0.3 * d:
                    blocker = (ob.x_ue[i], ob.y_ue[i])
                    break
            if blocker is not None:
                now = time.time()
                if self.wait_since is None:
                    self.wait_since = now
                    self.wait_blocker = blocker
                elif self.wait_blocker is not None:
                    moved = math.hypot(blocker[0] - self.wait_blocker[0],
                                       blocker[1] - self.wait_blocker[1])
                    if moved > 1.0:        # 还在动 → 重置等待
                        self.wait_since = now
                        self.wait_blocker = blocker
                if now - self.wait_since > OVERTAKE_TIMEOUT:
                    self.wait_since = None
                    if self._try_overtake(blocker):
                        self.overtaking = True
                    else:
                        self.finish('stuck', "前方队友停死且无空间超车, 停车上报。位置(%.1f,%.1f)"
                                    % (pose.x_, pose.y_))
                        return
                else:
                    self.ugv.apply_control(0, 0, 0.8, False)
                    return
            else:
                self.wait_since = None
                self.wait_blocker = None

        blocking = None
        if len(ob.Type):
            imin = nearest_idx(self.path, pose.x_, pose.y_)
            for i in range(len(ob.Type)):
                if not is_barricade(ob.Type[i]):
                    continue
                bx, by = ob.x_ue[i], ob.y_ue[i]
                key = (round(bx), round(by))
                if key in self.blocked_obstacles:
                    continue
                md = min(utils.distance(bx, by, px, py) for (px, py) in self.path[imin:])
                if md < CLEARANCE:
                    blocking = (ob.Type[i], bx, by, md)
                    break

        if blocking:
            self._handle_blocking(blocking, ob, pose)
            return

        # 临近拐弯减速
        imin = nearest_idx(self.path, pose.x_, pose.y_)
        near_corner = False
        for ci in self.corners:
            if ci > imin and utils.distance(pose.x_, pose.y_, self.path[ci][0], self.path[ci][1]) < SLOW_RANGE:
                near_corner = True
                break
        target_v = SLOW_V if near_corner else TARGET_V

        dev = math.hypot(pose.x_ - self.path[imin][0], pose.y_ - self.path[imin][1])
        self.max_dev = max(self.max_dev, dev)
        self.last_dev = dev

        # 撞停检测
        if state.v_ < 0.3 and target_v > 0.5:
            if self.stuck_since is None:
                self.stuck_since = time.time()
            elif time.time() - self.stuck_since > 8.0:
                # 停在目标旁边也算完成，别报 stuck（终判是给"真卡住"用的）
                if (self.target is not None and utils.distance(
                        pose.x_, pose.y_, self.target[0],
                        self.target[1]) < ARRIVE_DIST * 1.5):
                    self.finish('arrived', "已在目标旁停稳。位置(%.1f,%.1f)"
                                % (pose.x_, pose.y_))
                    return
                self.finish('stuck', "疑似撞停(速度<0.3 持续 8s)。位置(%.1f,%.1f) dev=%.1f"
                            % (pose.x_, pose.y_, dev))
                return
        else:
            self.stuck_since = None

        target = lookahead_pose(self.path, pose.x_, pose.y_, LOOKAHEAD)
        delta = self.stanley.update_control(target, state)
        dt = time.time() - self.prev
        self.prev = time.time()
        accel = self.pid.update_control(target_v, state.v_, dt)
        steer = delta / (math.pi / 6)
        if accel > 0:
            throttle, brake = min(accel, 1.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(accel) * 0.5, 1.0)
        self.ugv.apply_control(steer, throttle, brake, False)

    # ---- 路障挡路: 刹停→封格→重规划→(倒车) ----
    def _handle_blocking(self, blocking, ob, pose):
        self._clear_overtake()
        t, bx, by, md = blocking
        self.ugv.apply_control(0, 0, 0.8, False)
        time.sleep(0.5)
        self.ugv.apply_control(0, 0, 0, True)
        print("    [%s] 路障挡路 类型=%s 路障=(%.1f,%.1f) 距路径 %.1f m"
              % (self.name, t, bx, by, md))

        # 把当前探测到的所有路障封进路网
        for i in range(len(ob.Type)):
            if not is_barricade(ob.Type[i]):
                continue
            bpx, bpy = w2p(ob.x_ue[i], ob.y_ue[i], self.scale, self.off)
            block_disk(self.blocked_grid, bpx, bpy, BLOCK_R)
            self.blocked_obstacles.add((round(ob.x_ue[i]), round(ob.y_ue[i])))

        self.replan_count += 1
        if self.replan_count > MAX_REPLAN:
            self.finish('max_replan', "重规划次数超限, 停车上报。位置(%.1f,%.1f)" % (pose.x_, pose.y_))
            return

        new_path = self.plan_from(pose)
        if new_path is None:
            self.finish('no_path', "无路可走(兜底), 停车上报。位置(%.1f,%.1f)" % (pose.x_, pose.y_))
            return

        # 若绕行开头要往回走, 先倒车退出死路(转非阻塞状态)
        if needs_reverse(new_path, pose):
            c = detect_corners(new_path)
            if not c:
                self.finish('no_path', "绕行路径无拐点, 无法倒车退出, 停车上报。")
                return
            corner = new_path[c[0]]
            d0 = utils.distance(pose.x_, pose.y_, corner[0], corner[1])
            self.rev_target = d0 + REVERSE_OVERSHOOT
            self.rev_traveled = 0.0
            self.rev_t0 = time.time()
            self.rev_prev = time.time()
            self.status = 'reverse'
            print("    [%s] 绕行需倒车 %.1f m 退出死路" % (self.name, self.rev_target))
            return

        self.path = new_path
        self.corners = detect_corners(new_path)
        self.prev = time.time()
        self.stuck_since = None
        print("    [%s] 重规划完成 %d 点" % (self.name, len(new_path)))


def main():
    AeClient = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    _, game, _ = AeClient.get_game()
    game.stage_start('plan_start')
    game.stage_start('reconnaissance_start')
    game.stage_complete('reconnaissance_end')
    game.stage_start('vau_reconnaissance_start')
    game.stage_complete('vau_reconnaissance_end')

    binary, raw, scale, off = load_binary(game)

    # 创建 3 台 UGV 并复位到各自出生点
    carla = SN.carla
    ugvs = []
    for i in range(3):
        ugv = api.Vehicle(AeClient, "UGV%d" % (i + 1), i + 1, "ugv")
        actor = ugv.Vehicle_._SwarmAENode__actor
        sx, sy, sz = SPAWNS[i]
        actor.set_transform(carla.Transform(
            carla.Location(sx, sy, sz),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)))
        ugvs.append(ugv)
    time.sleep(1.5)
    for ugv in ugvs:
        ugv.apply_control(0, 0, 0, True)
    time.sleep(0.3)
    for ugv in ugvs:
        ugv.apply_control(0, 0, 0, False)

    # 每台一个 agent + 一个目标
    agents = []
    for i in range(3):
        a = UgvAgent(ugvs[i], "UGV%d" % (i + 1), TARGETS[i], binary, raw, scale, off)
        a.start()
        agents.append(a)

    print("start 3 UGVs ...\n")
    last_print = 0.0
    while True:
        for a in agents:
            a.step()

        if time.time() - last_print > 5.0:
            last_print = time.time()
            for a in agents:
                p = a.ugv.get_transform()
                print("  [%s] %-10s pos=(%.1f,%.1f) dev=%.1f replan=%d"
                      % (a.name, a.status, p.x_, p.y_, a.last_dev, a.replan_count))

        if all(a.status in ('arrived', 'no_path', 'stuck', 'max_replan') for a in agents):
            break
        time.sleep(0.05)

    print("\n===== 汇总 =====")
    for a in agents:
        p = a.ugv.get_transform()
        print("  [%s] 状态=%s 终点=(%.1f,%.1f) max_dev=%.1f m replan=%d"
              % (a.name, a.status, p.x_, p.y_, a.max_dev, a.replan_count))


if __name__ == "__main__":
    main()
