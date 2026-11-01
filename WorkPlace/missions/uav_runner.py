# -*- coding: utf-8 -*-
"""
uav_runner.py —— 把组员的无人机控制器改成"可单步驱动"的适配层
================================================================================

为什么需要这个文件
------------------
`missions/control_uav.py` 是无人机同学交付的成品，它的 `run_controller()` 是一个
`while True` 阻塞循环（自己 sleep、自己刷日志），只能"独占"整个进程跑。

但我们要的是**一条主循环里同时做三件事**：
    无人机飞 + 无人车开 + 小地图刷新

所以这里把 `run_controller()` 的循环体原样搬成一个 `step()`，
**控制逻辑一行没改**（起飞判定、航点推进、安全边界检查、探测合并全部照抄），
只是把 `while / sleep` 拿掉，交给调用方决定节奏。

    · control_uav.py 保持原样，随时还能单独 `python control_uav.py` 跑
    · 本文件只依赖它的公开函数与常量，不复制任何算法

用法
----
    import control_uav as cu
    from uav_runner import build_uav_vehicles, UavRunner

    vehicles = build_uav_vehicles(client, node_nos, logger)   # 用**共享的** client 注册
    planner  = cu.SafePlanner(cu.read_json(os.path.join(scene, "game.json")))
    starts   = dict((n, vehicles[n].position()) for n in cu.UAV_NAMES)
    routes   = cu.search_routes(planner, starts)

    runner = UavRunner(vehicles, planner, routes, logger)
    runner.start()
    while True:
        runner.step()            # 一帧（原来 while 里的一轮）
        print(runner.enemies())  # 已发现目标 [(x, y, z), ...]
    runner.stop()                # 悬停收尾
"""

from __future__ import print_function

import math
import time

import control_uav as cu


# ---------------------------------------------------------------------------
#  用**已有**的 client 注册三架无人机（不新建连接）
# ---------------------------------------------------------------------------
def build_uav_vehicles(client, node_nos, logger):
    """等价于 control_uav.connect_uavs()，但复用调用方的 SwarmAEClient。

    为什么要复用：整个任务只允许一个 client、一次注册，避免"显示端和控制端
    重复注册同名节点"这个尚未验证的兼容性问题。

    校验逻辑与 connect_uavs 完全一致：三个编号互不相同、注册码 200、
    真实 Actor 必须都是 vehicle.hrt.m10、actor id 不能重复。
    """
    if len(node_nos) != 3 or len(set(node_nos)) != 3 or any(n < 1 for n in node_nos):
        raise cu.SafetyError("Three distinct positive UAV node numbers are required")

    vehicles, actor_ids = {}, set()
    for name, number in zip(cu.UAV_NAMES, node_nos):
        result = cu.sdk_call(logger, client.register_node,
                             node_type="四旋翼", node_name=name,
                             node_no=number,
                             frame_timestamp=int(time.time() * 1000))
        if not isinstance(result, (tuple, list)) or len(result) < 3 or result[2] != 200:
            raise cu.SafetyError("Registration failed for %s/%d: %r"
                                 % (name, number, result))
        vehicle = cu.UavVehicle(name, result[1], logger)
        actor_id = vehicle.actor.id
        if actor_id in actor_ids:
            raise cu.SafetyError("Two UAV names resolved to the same actor")
        actor_ids.add(actor_id)
        vehicles[name] = vehicle
        logger.info("IDENTITY %s node_no=%d actor_id=%s type=%s",
                    name, number, actor_id, vehicle.actor.type_id)
    return vehicles


# ---------------------------------------------------------------------------
#  可单步驱动的控制器
# ---------------------------------------------------------------------------
class UavRunner(object):
    def __init__(self, vehicles, planner, routes, logger, use_default=False):
        self.vehicles = vehicles
        self.planner = planner
        self.routes = routes
        self.logger = logger
        self.use_default = bool(use_default)

        self.indices = dict((name, 0) for name in cu.UAV_NAMES)
        self.phases = dict((name, "takeoff") for name in cu.UAV_NAMES)
        self.stable = dict((name, 0) for name in cu.UAV_NAMES)
        self.last_goals = {}
        self.frame = 0
        self.next_status = time.monotonic()
        self.next_overrun = time.monotonic()

        self.started = False
        self.stopped = False
        self.error = None
        self.overruns = 0

    # ---------------- 对外 ----------------
    def start(self):
        if self.started:
            return
        with cu._lock:
            cu.discovered_enemies[:] = []
        cu.publish_detections(self.use_default)
        if self.use_default:
            self.logger.warning("SYNTHETIC defaults enabled until the first real discovery")
        self.started = True

    def enemies(self):
        """已发现的真实目标 [(x, y, z), ...]（米）。"""
        with cu._lock:
            return list(cu.discovered_enemies)

    def phases_snapshot(self):
        return dict(self.phases)

    def all_searching(self):
        return all(p in ("cruise", "hold") for p in self.phases.values())

    def all_takeoff(self):
        """三架是否都还卡在起飞阶段（用来自检飞控驱动是否生效）。"""
        return all(p == "takeoff" for p in self.phases.values())

    def stop(self):
        if self.stopped:
            return
        self.stopped = True
        try:
            cu.hover_uavs(self.vehicles, self.planner, self.logger)
        except Exception:
            self.logger.exception("hover cleanup failed")

    # ---------------- 一帧 ----------------
    def step(self):
        """执行原来 run_controller() 循环里的一轮。

        抛 cu.SafetyError 表示触发了安全边界/身份校验，调用方应停飞。
        """
        if not self.started:
            self.start()
        t_begin = time.monotonic()

        positions = dict((name, self.vehicles[name].position())
                         for name in cu.UAV_NAMES)
        commands, updates = {}, {}

        # ---- 先把三架的命令都算完，再统一下发（保持原实现的"整帧校验"）----
        for name in cu.UAV_NAMES:
            point, route = positions[name], self.routes[name]
            if not self.planner.point_safe(point):
                raise cu.SafetyError(name + " left the reserved safe flight envelope")

            if self.phases[name] == "takeoff":
                self.stable[name] = self.stable[name] + 1 if (
                    abs(point[2] - cu.ALTITUDE) <= cu.ALTITUDE_TOL
                    and cu.distance(point, route[0]) <= cu.WAYPOINT_TOL) else 0
                if self.stable[name] >= 5:
                    self.phases[name], self.indices[name] = "cruise", 1
                    self.logger.info("%s reached Z=250; starting search", name)
            elif abs(point[2] - cu.ALTITUDE) > 5.0:
                raise cu.SafetyError(name + " cruise altitude deviated by more than 5 m")

            index = self.indices[name]
            if self.phases[name] == "cruise":
                if index < len(route) and cu.distance(point, route[index]) <= cu.WAYPOINT_TOL:
                    index += 1
                    self.indices[name] = index
                if index >= len(route):
                    self.phases[name] = "hold"
                    self.logger.info("%s search complete; continuing detection in hover", name)

            goal = route[min(self.indices[name], len(route) - 1)]
            if not self.planner.segment_safe(point, goal):
                raise cu.SafetyError(name + " actual-position-to-waypoint segment is unsafe")

            speed = 0.0 if self.phases[name] == "hold" else cu.CRUISE_SPEED
            commands[name] = (goal[0], goal[1], cu.ALTITUDE, speed)
            if self.last_goals.get(name) != goal:
                updates[name] = goal

        if updates:
            cu.sync_goals(updates)
            self.last_goals.update(updates)
            for name, point in updates.items():
                self.logger.info("WAYPOINT %s %d/%d XY=%s Z=250", name,
                                 self.indices[name], len(self.routes[name]) - 1, point)

        for name in cu.UAV_NAMES:
            self.vehicles[name].target_point_control(*commands[name])

        # ---- 探测 ----
        self.frame += 1
        if self.frame % cu.DETECT_EVERY == 0:
            fresh = []
            for name in cu.UAV_NAMES:
                try:
                    point = self.vehicles[name].position()
                    result = cu.sdk_call(self.logger,
                                         self.vehicles[name].Vehicle_.detect_situation)
                    fresh.extend(cu.parse_detections(result, point))
                except Exception:
                    self.logger.exception("DETECTION failed for %s", name)
            added = cu.merge_detections(fresh)
            if added:
                cu.publish_detections(self.use_default)
                self.logger.info("DISCOVERED new=%s total=%d",
                                 added, len(self.enemies()))

        # ---- 状态日志（5 秒一次）----
        now = time.monotonic()
        if now >= self.next_status:
            self.logger.info("STATUS frame=%d positions=%s phases=%s enemies=%d",
                             self.frame, positions, self.phases,
                             len(self.enemies()))
            self.next_status = now + 5.0

        # ---- 超时（原实现打警告；这里只计数，不阻塞）----
        if now - t_begin > cu.DT and now >= self.next_overrun:
            self.overruns += 1
            self.logger.warning("Loop overran 50 ms by %.3f s; check SDK latency",
                                now - t_begin - cu.DT)
            self.next_overrun = now + 5.0
        return True


# ---------------------------------------------------------------------------
#  备用方案：运动学播放（当飞控动力学不可用时）
# ---------------------------------------------------------------------------

class _ZeroVec(object):
    """carla.Vector3D 拿不到时的兜底零向量。"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class KinematicUavRunner(object):
    """沿航线"搬"无人机，用 actor.set_transform()，不依赖飞控动力学。

    为什么要有这个
    --------------
    这台机器的 SDK 里 `launch_kinetic()` / `get_kinetic_data()` 都是空壳（返回 None），
    四旋翼的动力学通道起不来，`control_kinetic_simply_global()` 只返回 200 但没有任何运动。
    实测 `actor.set_transform()` 有效，所以退一步：每帧直接设置 actor 位姿。

    这不是"真实飞行"——不模拟动力学、不做避障、可能穿楼。
    它的价值是让整条链路（侦查 → 发现 → 派车 → 打击）能完整跑起来演示。

    平滑处理（否则会"抽搐"）
    ------------------------
    1. **转角圆弧过渡**：航线是可见性图的直线折线，到航点会瞬间掉头。
       这里对每个中间顶点用二次贝塞尔做圆角，半径默认 30 m。
    2. **按弧长等距重采样**（1.5 m 一个点）：保证任意位置的切向连续，
       也不会因为长短短不一而忽快忽慢。
    3. **偏航角限速**：每帧最多转 `YAW_RATE` 度/秒，并且做角差归一化
       （避免 ±180° 处来回翻）。
    4. **时间步长钳制**：按墙上时钟推进（帧率变化不影响速度），
       但单帧最多推进 0.15 s，卡顿时宁可慢一点也不要瞬移。
    """

    CLIMB_RATE = 12.0         # 爬升速度 m/s（别瞬移！瞬移会撞出伤害直接坠毁）
    MAX_STEP_M = 6.0          # 单帧最大位移 m，超过就钳住，杜绝瞬移
    CORNER_RADIUS = 30.0      # 转角圆弧半径 m（远小于 30 m 安全余量）
    SAMPLE_SPACING = 1.5      # 重采样间距 m
    YAW_RATE = 70.0           # 偏航角速度上限 度/秒
    MAX_DT = 0.15             # 单帧最多推进多少秒

    def __init__(self, vehicles, routes, logger, speed=None, dt=None,
                 use_default=False, altitude=None, clock=None,
                 virtual=False, known_targets=None, detect_radius=300.0):
        self.vehicles = vehicles
        self.routes = routes
        self.logger = logger
        self.speed = float(speed if speed is not None else cu.CRUISE_SPEED)
        self.dt = float(dt if dt is not None else cu.DT)
        self.use_default = bool(use_default)
        # 高度必须足够高，否则直线飞向敌人时会撞进楼里 —— 撞了会掉血，
        # 血量归零后 actor 失效，get_location() 抛异常，探测全部失效。
        # 实测 100 m 会被楼挡住，250 m 安全（与本课程推荐高度一致）。
        self.altitude = cu.ALTITUDE if altitude is None else float(altitude)
        self.alt = {}

        self.dist = dict((n, 0.0) for n in cu.UAV_NAMES)
        self.path = {}        # 平滑后的等距点
        self.cum = {}         # 对应累计弧长
        self.total = {}
        self.yaw = {}         # 平滑后的偏航角（度）
        for n in cu.UAV_NAMES:
            smooth = self._round_corners(routes[n], self.CORNER_RADIUS)
            pts, cum = self._resample(smooth, self.SAMPLE_SPACING)
            self.path[n] = pts
            self.cum[n] = cum
            self.total[n] = cum[-1] if cum else 0.0
            self.yaw[n] = 0.0
        # 路径就绪后再算初始朝向（_tangent 依赖 cum/path）
        for n in cu.UAV_NAMES:
            self.yaw[n] = self._tangent(n, 0.0)

        #: 当前正在飞向的航点（小地图用它画"目标星标"，与 UavRunner 同名同义）
        self.last_goals = dict((n, routes[n][0]) for n in cu.UAV_NAMES)

        self.frame = 0
        self.started = False
        self.stopped = False
        #: 可注入的时钟（测试时换成假时钟，好做确定性验证）
        self._clock = clock or time.monotonic
        self.next_status = self._clock()
        self.last_step = None
        self.last_detect = 0.0
        self.dead = set()          # 已失去 actor 的机（撞毁/血量归零）
        self.climbed = dict((n, False) for n in cu.UAV_NAMES)   # 是否已爬到目标高度
        self._vel_warned = set()
        #: True = 完全虚拟：不碰 actor 位姿，位置和探测全用数学计算。
        #  为什么需要：这批 M10 actor 的物理无法关闭（关物理会触发 UE4 断言崩溃），
        #  速度也清不掉（set_target_velocity 无效），于是永远自由落体（实测 366 m/s），
        #  任何"贴着物理写位姿"的做法都会抖、会撞、会坠。
        #  虚拟模式绕开物理：画面里飞机不动，但侦查逻辑与掉线完全正确。
        self.virtual = bool(virtual)
        self.known = [(float(p[0]), float(p[1])) for p in (known_targets or [])]
        self.detect_radius = float(detect_radius)
        self._z_warned = set()     # 已报过"高度被外力拉动"的机
        self._pos_fail = dict((n, 0) for n in cu.UAV_NAMES)
        self.logger.warning(
            "KINEMATIC MODE: 用 set_transform 播放航线，非真实飞行"
            "（转角圆弧 %.0f m / 重采样 %.1f m / 偏航限速 %.0f deg/s / "
            "高度 %s）",
            self.CORNER_RADIUS, self.SAMPLE_SPACING, self.YAW_RATE,
            ("固定 %.0f m" % self.altitude) if self.altitude is not None
            else "保持当前高度")

        try:
            from swarmae import SwarmAENode as SN
            self._carla = SN.carla
        except Exception as e:
            raise cu.SafetyError("拿不到 carla 模块，无法运动学播放: %r" % (e,))

    # ---------------- 几何处理 ----------------
    @staticmethod
    def _round_corners(route, radius):
        """把折线的每个中间顶点换成圆弧过渡。"""
        pts = [(float(p[0]), float(p[1])) for p in route]
        if len(pts) < 3 or radius <= 0:
            return pts
        out = [pts[0]]
        for i in range(1, len(pts) - 1):
            prev, vert, nxt = pts[i - 1], pts[i], pts[i + 1]
            d1 = math.hypot(vert[0] - prev[0], vert[1] - prev[1])
            d2 = math.hypot(nxt[0] - vert[0], nxt[1] - vert[1])
            if d1 < 1e-6 or d2 < 1e-6:
                out.append(vert)
                continue
            r = min(radius, d1 * 0.45, d2 * 0.45)
            a = (vert[0] + (prev[0] - vert[0]) * r / d1,
                 vert[1] + (prev[1] - vert[1]) * r / d1)
            b = (vert[0] + (nxt[0] - vert[0]) * r / d2,
                 vert[1] + (nxt[1] - vert[1]) * r / d2)
            out.append(a)
            steps = max(4, int(r))
            for k in range(1, steps):
                t = k / float(steps)
                x = (1 - t) ** 2 * a[0] + 2 * (1 - t) * t * vert[0] + t * t * b[0]
                y = (1 - t) ** 2 * a[1] + 2 * (1 - t) * t * vert[1] + t * t * b[1]
                out.append((x, y))
            out.append(b)
        out.append(pts[-1])
        return out

    @staticmethod
    def _resample(pts, spacing):
        """按弧长等距重采样；返回 (points, cumulative)。"""
        if len(pts) < 2:
            return list(pts), [0.0]
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        total = cum[-1]
        if total <= 0:
            return [pts[0]], [0.0]
        n = max(2, int(math.ceil(total / spacing)) + 1)
        out, out_cum = [], []
        j = 0
        for i in range(n):
            s = min(total, total * i / float(n - 1))
            while j + 1 < len(cum) - 1 and cum[j + 1] < s:
                j += 1
            seg = cum[j + 1] - cum[j]
            t = 0.0 if seg <= 1e-12 else (s - cum[j]) / seg
            out.append((pts[j][0] + (pts[j + 1][0] - pts[j][0]) * t,
                        pts[j][1] + (pts[j + 1][1] - pts[j][1]) * t))
            out_cum.append(s)
        return out, out_cum

    def _point_at(self, name, s):
        """弧长 s 处的 (x, y)，线性插值（点已经等距，足够平滑）。"""
        pts = self.path[name]
        cum = self.cum[name]
        if not pts:
            p = self.routes[name][0]
            return p[0], p[1]
        if s <= 0:
            return pts[0][0], pts[0][1]
        if s >= cum[-1]:
            return pts[-1][0], pts[-1][1]
        lo, hi = 0, len(cum) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        seg = cum[hi] - cum[lo]
        t = 0.0 if seg <= 1e-12 else (s - cum[lo]) / seg
        return (pts[lo][0] + (pts[hi][0] - pts[lo][0]) * t,
                pts[lo][1] + (pts[hi][1] - pts[lo][1]) * t)

    def _tangent(self, name, s):
        """用 s 前后各 3 m 的点估切向，避免逐点抖动。"""
        step = 3.0
        total = self.cum[name][-1] if self.cum[name] else 0.0
        a = self._point_at(name, max(0.0, s - step))
        b = self._point_at(name, min(total, s + step))
        if abs(b[0] - a[0]) < 1e-9 and abs(b[1] - a[1]) < 1e-9:
            return self.yaw.get(name, 0.0)
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

    def virtual_pose(self, name):
        """返回虚拟位姿 (x, y, z, yaw_deg)，供小地图/心跳使用。

        虚拟模式下真实 actor 会一直自由落体，它的坐标毫无意义；
        显示层必须用这套"数学位置"，否则小地图上的飞机会掉到楼里。
        """
        s = 0.0 if not self.climbed.get(name, False) else self.dist.get(name, 0.0)
        x, y = self._point_at(name, s)
        z = self.alt.get(name, self.altitude)
        return x, y, z, self.yaw.get(name, 0.0)

    def _waypoint_ahead(self, name, s):
        """弧长 s 之后要飞的那个原始航点（小地图星标用）。"""
        route = self.routes[name]
        best, best_d = route[-1], None
        acc = 0.0
        for a, b in zip(route, route[1:]):
            acc += cu.distance(a, b)
            if acc >= s:
                return b
        return best

    # ---------------- 对外 ----------------
    def _disable_gravity(self, actor, name):
        '''关掉该 actor 的重力（保留物理）。失败会明确写日志。'''
        fn = getattr(actor, "set_enable_gravity", None)
        if fn is None:
            self.logger.warning(
                "GRAVITY: %s 没有 set_enable_gravity，无法关重力（会持续下坠）", name)
            return False
        try:
            fn(False)
            self.logger.info("GRAVITY OFF %s", name)
            return True
        except Exception as e:
            self.logger.warning("set_enable_gravity(False) 失败 %s: %r", name, e)
            return False

    def _zero_velocity(self, actor, name):
        """把速度清成 0（重力才不会累积）。失败只提示一次。"""
        if name in self._vel_warned:
            return
        fn = getattr(actor, "set_target_velocity", None)
        if fn is None:
            self._vel_warned.add(name)
            self.logger.warning(
                "actor 没有 set_target_velocity，无法清速度；无人机会持续下坠")
            return
        try:
            v3 = getattr(self._carla, "Vector3D", None)
            fn(v3(0.0, 0.0, 0.0) if v3 is not None else _ZeroVec())
        except Exception as e:
            self._vel_warned.add(name)
            self.logger.warning("set_target_velocity 失败 %s: %r", name, e)

    def _settle_at_route_start(self):
        """把三架机复位到航线起点。

        ⚠️ 绝对不要在这里调 `set_simulate_physics(False)`！
        M10 这个 actor 底层派生自 ACarlaWheeledVehicle，关物理会去取
        `VehicleAnim`（动画实例），而四旋翼根本没有 —— 直接触发 UE4 断言：

            Assertion failed: VehicleAnim != nullptr
            [CarlaWheeledVehicle.cpp] [Line: 557]

        UE4 崩溃后 subject4 的 init_vehicles 会拿到 None，报
        `'NoneType' object has no attribute 'add_to_team'` 退出，
        仿真器彻底不可用。所以只做位姿复位，不碰物理开关。
        """
        for name in cu.UAV_NAMES:
            actor = getattr(self.vehicles[name], "actor", None)
            if self.virtual or actor is None:
                # 虚拟模式：只记录内部高度，绝不写 actor（物理会把它拽走）
                self.alt[name] = self.altitude
                self.climbed[name] = True
                self.dist[name] = 0.0
                continue
            # 【关键】先关掉这台 actor 的重力，再动位置。
            # `set_simulate_physics(False)` 会触发 UE4 断言崩溃（VehicleAnim），
            # 但 `set_enable_gravity(False)` 只关重力、保留物理，不会崩。
            # 重力一关，速度就不再累积，飞机不会被拽回地面。
            self._disable_gravity(actor, name)

            x, y = self._point_at(name, 0.0)
            yaw = self._tangent(name, 0.0)
            self.yaw[name] = yaw
            self.dist[name] = 0.0
            # 只记录"当前真实高度"作为爬升起点，绝不瞬移！
            # 实测：一帧内从 10 m 拉到 250 m 会直接撞出致命伤害（5 秒内两架 dead）。
            try:
                z = float(self.vehicles[name].position()[2])
            except Exception:
                z = self.altitude
            if abs(z - self.altitude) < 1.0:
                z = self.altitude
            self.alt[name] = z
            self.climbed[name] = abs(z - self.altitude) < 1.0
            if self.virtual:
                continue
            try:
                actor.set_transform(self._carla.Transform(
                    self._carla.Location(x, y, z),
                    self._carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)))
            except Exception:
                self.logger.exception("复位到航线起点失败: %s", name)

    def start(self):
        if self.started:
            return
        with cu._lock:
            cu.discovered_enemies[:] = []
        cu.publish_detections(self.use_default)
        self._settle_at_route_start()
        self.last_step = self._clock()
        self.started = True

    def enemies(self):
        with cu._lock:
            return list(cu.discovered_enemies)

    def phases_snapshot(self):
        out = {}
        for n in cu.UAV_NAMES:
            if n in self.dead:
                out[n] = "dead"
            elif not self.climbed.get(n, False):
                out[n] = "climb"
            else:
                out[n] = "hold" if self.dist[n] >= self.total[n] else "cruise"
        return out

    def all_takeoff(self):
        return False          # 运动学模式没有起飞阶段

    def all_searching(self):
        return all(n in self.dead or self.dist[n] >= self.total[n]
                   for n in cu.UAV_NAMES)

    def stop(self):
        self.stopped = True

    def step(self):
        if not self.started:
            self.start()
        self.frame += 1

        # 按墙上时钟推进；卡顿时宁可慢也不要瞬移
        now = self._clock()
        dt = (now - self.last_step) if self.last_step is not None else self.dt
        self.last_step = now
        dt = max(0.0, min(self.MAX_DT, dt))
        step_len = self.speed * dt

        for name in cu.UAV_NAMES:
            if name in self.dead:
                continue
            if self.total[name] <= 0:
                continue

            # ---- 先垂直爬升到目标高度，到位前不前进 ----
            if not self.climbed[name]:
                cur_z = self.alt.get(name, self.altitude)
                if cur_z < self.altitude:
                    cur_z = min(self.altitude, cur_z + self.CLIMB_RATE * dt)
                else:
                    cur_z = max(self.altitude, cur_z - self.CLIMB_RATE * dt)
                self.alt[name] = cur_z
                if abs(cur_z - self.altitude) < 0.5:
                    self.climbed[name] = True
                    self.logger.info("CLIMB DONE %s -> %.1f m", name, self.altitude)
                x, y = self._point_at(name, 0.0)
            else:
                # ---- 爬升到位后按航线推进（单帧位移有上限） ----
                self.dist[name] = min(self.total[name],
                                      self.dist[name] + min(step_len, self.MAX_STEP_M))
                x, y = self._point_at(name, self.dist[name])

            # 偏航角限速平滑（角差归一到 -180..180，避免 ±180 处翻转）
            target = self._tangent(name, self.dist[name])
            delta = (target - self.yaw[name] + 180.0) % 360.0 - 180.0
            limit = self.YAW_RATE * dt
            if limit > 0:
                delta = max(-limit, min(limit, delta))
            self.yaw[name] = (self.yaw[name] + delta + 180.0) % 360.0 - 180.0

            self.last_goals[name] = self._waypoint_ahead(name, self.dist[name])

            # 存活检查：血量归零/actor 失效后 get_location() 会抛异常。
            # 连续失败就判定该机已损毁，停止驱动并把相位标成 dead，
            # 否则心跳里只会显示 '?'，看不出发生了什么。
            try:
                real = self.vehicles[name].position()
                self._pos_fail[name] = 0
                if self.virtual:
                    continue
                # 诊断：我们每帧都设固定高度，若读回来差很多，
                # 说明还有别的东西在改这个 actor 的位姿（飞控驱动/物理）。
                drift = real[2] - self.alt.get(name, self.altitude)
                if abs(drift) > 5.0 and name not in self._z_warned:
                    self._z_warned.add(name)
                    self.logger.warning(
                        "ALT DRIFT %s: 设定 %.1f m，实测 %.1f m（差 %+.1f m）——"
                        " 有外力在改位姿，检查是否还有 driver.exe / main-release.exe 在跑",
                        name, self.alt.get(name, self.altitude), real[2], drift)
            except Exception:
                self._pos_fail[name] += 1
                if self._pos_fail[name] >= 20:
                    self.dead.add(name)
                    self.logger.error(
                        "UAV LOST %s：连续 %d 次读不到位姿，判定已损毁"
                        "（多半是撞楼掉血归零）", name, self._pos_fail[name])
                    continue

            vehicle = self.vehicles[name]
            actor = getattr(vehicle, "actor", None)
            if actor is None:
                continue
            try:
                actor.set_transform(self._carla.Transform(
                    self._carla.Location(x, y, self.alt.get(name, self.altitude)),
                    self._carla.Rotation(pitch=0.0, yaw=self.yaw[name], roll=0.0)))
                # 【关键】把速度清零。
                # set_transform 只改位置，不清速度；重力每帧给 actor 累加速度，
                # 等我们下一帧写位置时，物理已经按"累积速度"把它甩下去很远 ——
                # 表现就是"无人机不停地往初始位置（地面）掉"。
                # 清零后重力无法累积，飞机就稳在我们放的位置上。
                self._zero_velocity(actor, name)
            except Exception:
                self.logger.exception("set_transform failed for %s", name)

        # ---- 探测（每 10 帧一次，另按 0.5 s 兜底；低帧率时不漏检）----
        if (self.frame % cu.DETECT_EVERY == 0
                or (now - self.last_detect) >= 0.5):
            self.last_detect = now
            if self.virtual:
                # 虚拟探测：按数学位置算，语义与真机一致（水平距离 <= 半径）
                fresh = []
                for name in cu.UAV_NAMES:
                    if name in self.dead:
                        continue
                    px, py = self._point_at(
                        name, 0.0 if not self.climbed.get(name) else self.dist[name])
                    for tx, ty in self.known:
                        if math.hypot(tx - px, ty - py) <= self.detect_radius:
                            fresh.append((tx, ty, 0.0))
                added = cu.merge_detections(fresh)
                if added:
                    cu.publish_detections(self.use_default)
                    self.logger.info("DISCOVERED new=%s total=%d",
                                     added, len(self.enemies()))
                if now >= self.next_status:
                    self.logger.info("STATUS frame=%d virtual dist=%s enemies=%d",
                                     self.frame,
                                     dict((n, round(self.dist[n], 1))
                                          for n in cu.UAV_NAMES),
                                     len(self.enemies()))
                    self.next_status = now + 5.0
                return True
            fresh = []
            for name in cu.UAV_NAMES:
                try:
                    point = self.vehicles[name].position()
                    result = cu.sdk_call(self.logger,
                                         self.vehicles[name].Vehicle_.detect_situation)
                    fresh.extend(cu.parse_detections(result, point))
                except Exception:
                    self.logger.exception("DETECTION failed for %s", name)
            added = cu.merge_detections(fresh)
            if added:
                cu.publish_detections(self.use_default)
                self.logger.info("DISCOVERED new=%s total=%d",
                                 added, len(self.enemies()))

        if now >= self.next_status:
            self.logger.info("STATUS frame=%d progress=%s enemies=%d",
                             self.frame,
                             dict((n, round(self.dist[n], 1)) for n in cu.UAV_NAMES),
                             len(self.enemies()))
            self.next_status = now + 5.0
        return True