# -*- coding: utf-8 -*-
"""UAV-only search and detection. Python 3.6; importing never connects or flies."""
from __future__ import print_function

import argparse
import contextlib
import heapq
import json
import logging
import math
import os
import sys
import tempfile
import threading
import time


PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORKPLACE = os.path.join(PROJECT, "WorkPlace")
OUTPUT_DIR = os.path.join(PROJECT, "codex_outputs", "uav_module")
GOALS_PATH = os.path.join(WORKPLACE, "config", "goals.json")
DETECTED_PATH = os.path.join(WORKPLACE, "config", "detected.json")
STATE_PATH = os.path.join(OUTPUT_DIR, "enemy_state.json")
UAV_NAMES = ("UAV1", "UAV2", "UAV3")
DT = 0.05
ALTITUDE = 250.0
CRUISE_SPEED = 15.0
DETECT_EVERY = 10
DETECT_RADIUS = 300.0
CLUSTER_RADIUS = 15.0
STRIP_SPACING = 500.0
SAFETY_MARGIN = 30.0
ARENA_MARGIN = 10.0
TRACK_RESERVE = 10.0
CORNER_OFFSET = 5.0
WAYPOINT_TOL = 3.0
ALTITUDE_TOL = 2.0
DEFAULT_ENEMIES = ((1535.5, -889.3), (1727.0, -741.0), (1985.0, -626.0))

# Real discoveries only, represented by fixed (x, y, z) cluster representatives.
discovered_enemies = []
_lock = threading.RLock()
_running = False
_use_default = False


class SafetyError(RuntimeError):
    pass


def finite(value):
    if isinstance(value, bool):
        raise ValueError("Boolean is not a coordinate")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Non-finite coordinate")
    return value


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def read_json(path):
    with open(path, "r", encoding="utf-8-sig") as handle:
        return json.load(handle)


def atomic_json(path, data):
    """Replace a complete UTF-8 JSON file; leave the old file intact on failure."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".uav_", suffix=".json", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def sync_goals(updates, path=GOALS_PATH):
    """Read/merge/replace, touching only top-level UAV1/2/3 values.

    Refuse malformed existing JSON rather than discarding another member's goals.
    Atomic replacement prevents partial reads, but independent writers still need
    a shared single-writer convention to avoid concurrent read/modify/write loss.
    """
    if any(name not in UAV_NAMES for name in updates):
        raise SafetyError("Only UAV1/UAV2/UAV3 goals may be updated")
    if not updates:
        return
    data = read_json(path) if os.path.exists(path) else {}
    if not isinstance(data, dict):
        raise ValueError("goals.json must be an object; refusing to overwrite it")
    for name, point in updates.items():
        data[name] = [finite(point[0]), finite(point[1])]
    atomic_json(path, data)


def get_detected_enemies(use_default=False):
    """Return [(x, y), ...] in metres, never connect to the simulator.

    Same-process calls use live memory. Other processes read this module's last
    published enemy_state.json snapshot, so importing does not create a fleet.
    Synthetic defaults are never inserted into discovered_enemies.
    """
    with _lock:
        real = list(discovered_enemies)
        default_enabled = bool(use_default or _use_default)
        if not _running and not real:
            try:
                state = read_json(STATE_PATH)
                real = [(finite(p[0]), finite(p[1]), finite(p[2]))
                        for p in state.get("real_enemies", [])]
                default_enabled = default_enabled or bool(state.get("use_default", False))
            except (OSError, ValueError, TypeError, KeyError, IndexError, AttributeError):
                pass
        if real:
            return [(p[0], p[1]) for p in real]
        return list(DEFAULT_ENEMIES) if default_enabled else []


def publish_detections(use_default=False, detected_path=DETECTED_PATH,
                       state_path=STATE_PATH):
    with _lock:
        real = list(discovered_enemies)
        points = [(p[0], p[1]) for p in real]
        if not points and use_default:
            points = list(DEFAULT_ENEMIES)
        # Write the feed first: readers never see a state snapshot ahead of it.
        atomic_json(detected_path, {"points": [{"x": x, "y": y} for x, y in points],
                                     "extra": [], "__清空__": False})
        atomic_json(state_path, {"updated_at": time.time(),
                                 "use_default": bool(use_default),
                                 "synthetic": bool(use_default and not real),
                                 "real_enemies": real})


def merge_detections(points):
    """15 m XY clustering, fixed first-observed representatives (no drift)."""
    added = []
    with _lock:
        for point in points:
            if not any(distance(point, old) <= CLUSTER_RADIUS
                       for old in discovered_enemies):
                representative = tuple(finite(v) for v in point[:3])
                discovered_enemies.append(representative)
                added.append(representative)
    return added


def parse_detections(result, uav_position):
    """Strict SDK status/type/centimetre/XY-range filtering, no unit guessing."""
    if not isinstance(result, (tuple, list)) or len(result) < 3 or result[1] != 200:
        return []
    try:
        objects = json.loads(result[2]) if isinstance(result[2], (str, bytes)) else result[2]
    except (ValueError, TypeError):
        return []
    if not isinstance(objects, list):
        return []
    out = []
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        # In the supplied SDK strike means hostile/attackable, teammate is own side.
        # Unknown/Other/obstacles are deliberately not inferred to be enemies.
        if str(obj.get("Type", "")).strip().lower() not in ("strike", "enemy", "hostile"):
            continue
        try:
            if "HealthPoint" in obj and finite(obj["HealthPoint"]) <= 0:
                continue
            position = obj["Position"]
            point = tuple(finite(position[key]) / 100.0 for key in ("X", "Y", "Z"))
            if distance(point, uav_position) <= DETECT_RADIUS:
                out.append(point)
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return out


def expanded_bounds(poly, margin):
    return (min(p[0] for p in poly) - margin,
            min(p[1] for p in poly) - margin,
            max(p[0] for p in poly) + margin,
            max(p[1] for p in poly) + margin)


def inside_rectangle(point, rectangle):
    x0, y0, x1, y1 = rectangle
    return x0 <= point[0] <= x1 and y0 <= point[1] <= y1


def segment_hits_rectangle(start, end, rectangle):
    """Closed slab intersection: touching the protected rectangle also blocks."""
    lower, upper = 0.0, 1.0
    for axis, low, high in ((0, rectangle[0], rectangle[2]),
                            (1, rectangle[1], rectangle[3])):
        delta = end[axis] - start[axis]
        if abs(delta) < 1e-12:
            if start[axis] < low or start[axis] > high:
                return False
        else:
            first, last = (low - start[axis]) / delta, (high - start[axis]) / delta
            if first > last:
                first, last = last, first
            lower, upper = max(lower, first), min(upper, last)
            if lower > upper:
                return False
    return True


class SafePlanner(object):
    """Conservative rectangular envelopes + visibility graph/Dijkstra.

    Enlarging a polygon's AABB contains its true 30 m offset, including concave
    corners. This trades some usable airspace for a small, auditable stdlib-only
    planner. A rectangular arena is validated, not silently assumed.
    """
    def __init__(self, game):
        config = game["SUBJECT_ARENA"]
        arena = [(finite(p["x"]), finite(p["y"])) for p in config["arena"]]
        bounds = expanded_bounds(arena, 0.0)
        x0, y0, x1, y1 = bounds
        if (len(arena) != 4 or set(arena) != {(x0, y0), (x0, y1), (x1, y0), (x1, y1)}
                or x1 <= x0 or y1 <= y0):
            raise SafetyError("This planner requires the configured rectangular arena")
        # Prevent a self-crossed ordering from being accepted as a rectangle.
        if any(a[0] != b[0] and a[1] != b[1]
               for a, b in zip(arena, arena[1:] + arena[:1])):
            raise SafetyError("Invalid arena vertex order")
        self.arena = bounds
        self.flight_arena = (x0 + ARENA_MARGIN, y0 + ARENA_MARGIN,
                             x1 - ARENA_MARGIN, y1 - ARENA_MARGIN)
        self.polygons = [[(finite(p["x"]), finite(p["y"])) for p in poly]
                         for poly in config["no_fly_arena"]]
        if len(self.polygons) != 3 or any(len(poly) < 3 for poly in self.polygons):
            raise SafetyError("Expected exactly three valid no-fly polygons")
        self.protected = [expanded_bounds(poly, SAFETY_MARGIN) for poly in self.polygons]
        self.routing = [expanded_bounds(poly, SAFETY_MARGIN + TRACK_RESERVE)
                        for poly in self.polygons]
        self.corners = []
        for box in self.routing:
            a, b, c, d = box
            pad = CORNER_OFFSET
            for point in ((a-pad, b-pad), (a-pad, d+pad),
                          (c+pad, b-pad), (c+pad, d+pad)):
                if self.point_safe(point, planning=True):
                    self.corners.append(point)

    def point_safe(self, point, planning=False):
        boxes = self.routing if planning else self.protected
        return (inside_rectangle(point, self.flight_arena)
                and not any(inside_rectangle(point, box) for box in boxes))

    def segment_safe(self, start, end, planning=False):
        boxes = self.routing if planning else self.protected
        return (self.point_safe(start, planning) and self.point_safe(end, planning)
                and not any(segment_hits_rectangle(start, end, box) for box in boxes))

    def route(self, start, goal):
        start, goal = tuple(start[:2]), tuple(goal[:2])
        if not self.point_safe(start, True) or not self.point_safe(goal, True):
            raise SafetyError("Route endpoint lies outside arena or inside a protected envelope: %r -> %r"
                              % (start, goal))
        if self.segment_safe(start, goal, True):
            return [start] if distance(start, goal) < 1e-6 else [start, goal]
        nodes = [start, goal] + self.corners
        graph = [[] for _ in nodes]
        for i, first in enumerate(nodes):
            for j in range(i + 1, len(nodes)):
                second = nodes[j]
                if self.segment_safe(first, second, True):
                    weight = distance(first, second)
                    graph[i].append((j, weight))
                    graph[j].append((i, weight))
        best, previous, queue = {0: 0.0}, {}, [(0.0, 0)]
        while queue:
            cost, index = heapq.heappop(queue)
            if cost != best[index]:
                continue
            if index == 1:
                indices = [1]
                while indices[-1] != 0:
                    indices.append(previous[indices[-1]])
                return [nodes[i] for i in reversed(indices)]
            for neighbor, weight in graph[index]:
                candidate = cost + weight
                if candidate < best.get(neighbor, float("inf")):
                    best[neighbor], previous[neighbor] = candidate, index
                    heapq.heappush(queue, (candidate, neighbor))
        raise SafetyError("No safe route exists; no direct-flight fallback is allowed")


def strip_rows(low, high):
    rows, value = [low], low + STRIP_SPACING
    while value < high - 1e-6:
        rows.append(value)
        value += STRIP_SPACING
    if high > rows[-1] + 1e-6:
        rows.append(high)
    return rows


def search_routes(planner, starts):
    """Three adjacent east-side subareas, each swept back and forth in 500 m rows.

    Transit from each measured start to its first row is also safely routed and
    detected. The last row is clipped to y=-60, hence its gap is only 330 m.
    """
    routes = {}
    rows = strip_rows(-890.0, -60.0)
    for number, name in enumerate(UAV_NAMES):
        left = 1200.0 + number * (1000.0 / 3.0)
        right = 1200.0 + (number + 1) * (1000.0 / 3.0)
        sweep = []
        for index, y in enumerate(rows):
            ends = (left, right) if index % 2 == 0 else (right, left)
            sweep.extend((x, y) for x in ends)
        route = [tuple(starts[name][:2])]
        for goal in sweep:
            route.extend(planner.route(route[-1], goal)[1:])
        if not all(planner.segment_safe(a, b, True) for a, b in zip(route, route[1:])):
            raise SafetyError("Invalid generated route for " + name)
        routes[name] = route
    return routes


def choose_scene(explicit=None):
    if explicit:
        scene = os.path.abspath(explicit)
    else:
        choices = (os.path.join(PROJECT, "Materials", "practice", "practice1_Scene"),
                   os.path.join(PROJECT, "environment", "subject_config", "subject4"))
        scene = next((p for p in choices if os.path.isfile(os.path.join(p, "game.json"))), None)
        if scene is None:
            raise ValueError("Cannot find game.json; pass --scene SCENE_DIRECTORY")
    for filename in ("game.json", "vehicles.json"):
        if not os.path.isfile(os.path.join(scene, filename)):
            raise ValueError("Missing scene file: " + os.path.join(scene, filename))
    return scene


def scene_uavs(scene):
    """Infer 1-based configured vehicle order; live actor type is checked later."""
    entries, ordinal = [], 0
    for group in read_json(os.path.join(scene, "vehicles.json")):
        for vehicle in group["vehicles"]:
            ordinal += 1
            if group["type"] == "vehicle.hrt.m10":
                raw = vehicle["position"]
                point = raw.split(",") if isinstance(raw, str) else raw
                if len(point) != 3:
                    raise ValueError("UAV position must have x,y,z")
                entries.append((ordinal, tuple(finite(p) for p in point)))
    if len(entries) != 3:
        raise SafetyError("Expected exactly three configured HRT-M10 UAVs")
    return entries


class _SdkLog(object):
    def __init__(self, logger):
        self.logger = logger

    def write(self, value):
        if value.strip():
            self.logger.warning("SDK: %s", value.strip())

    def flush(self):
        pass


def sdk_call(logger, method, *args, **kwargs):
    # The bundled SDK prints some errors internally; keep those out of the console.
    stream = _SdkLog(logger)
    with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
        return method(*args, **kwargs)


class UavVehicle(object):
    """Minimal UAV-only adapter, same target_point_control shape as Fleet.vehicle.

    No Fleet construction or UGV control imports. The known bundled SDK's actor
    handle is required to validate the *actual* actor, not its registration label.
    """
    def __init__(self, name, node, logger):
        if name not in UAV_NAMES:
            raise SafetyError("Non-UAV platform rejected")
        self.name, self.Vehicle_, self.logger = name, node, logger
        self.actor = getattr(node, "_SwarmAENode__actor", None)
        self.validate()

    def validate(self):
        if self.actor is None or getattr(self.actor, "type_id", None) != "vehicle.hrt.m10":
            raise SafetyError("%s is not backed by an HRT-M10 actor; refusing all flight commands"
                              % self.name)
        if finite(self.actor.health_point) <= 0:
            raise SafetyError(self.name + " is damaged; reset/reconcile the scene first")

    def position(self):
        self.validate()
        values = sdk_call(self.logger, self.Vehicle_.get_location)
        if not isinstance(values, (tuple, list)) or len(values) < 3:
            raise SafetyError(self.name + " has no valid position")
        return tuple(finite(value) for value in values[:3])

    def target_point_control(self, x, y, z, v):
        self.validate()
        if abs(z - ALTITUDE) > 1e-9 or v not in (0.0, CRUISE_SPEED):
            raise SafetyError("Only Z=250 and speed=15/hover=0 are permitted")
        result = sdk_call(self.logger, self.Vehicle_.control_kinetic_simply_global,
                          finite(x), finite(y), ALTITUDE, finite(v), int(time.time() * 1000))
        # 【集成方修改】状态码位置在不同 SDK 方法/版本里不一致：
        #   apply_vehicle_control 返回 (frame_timestamp, code)   -> code 在第 2 位
        #   control_kinetic_simply 文档列的是 (code, frame_timestamp) -> code 在第 1 位
        # 原实现只认 result[0] == 200，若真机是 (ts, code) 就会把每次都判成失败、
        # 永远无法起飞。这里改成前两位里出现 200 即视为成功（时间戳不可能是 200ms）。
        if not isinstance(result, (tuple, list)) or 200 not in list(result)[:2]:
            raise SafetyError("%s rejected flight command: %r" % (self.name, result))


def connect_uavs(host, port, node_nos, logger):
    if len(node_nos) != 3 or len(set(node_nos)) != 3 or any(n < 1 for n in node_nos):
        raise SafetyError("Three distinct positive UAV node numbers are required")
    # Some supplied SDK modules parse sys.argv when imported. Do not expose our flags.
    original = sys.argv
    try:
        sys.argv = original[:1]
        with contextlib.redirect_stdout(_SdkLog(logger)), contextlib.redirect_stderr(_SdkLog(logger)):
            from swarmae.SwarmAEClient import SwarmAEClient
    finally:
        sys.argv = original
    client = sdk_call(logger, SwarmAEClient, ue_ip=host, ue_port=port)
    vehicles, actor_ids = {}, set()
    # Validate ALL three identities before the caller can issue any movement.
    for name, number in zip(UAV_NAMES, node_nos):
        result = sdk_call(logger, client.register_node, node_type="四旋翼", node_name=name,
                          node_no=number, frame_timestamp=int(time.time() * 1000))
        if not isinstance(result, (tuple, list)) or len(result) < 3 or result[2] != 200:
            raise SafetyError("Registration failed for %s/%d: %r" % (name, number, result))
        vehicle = UavVehicle(name, result[1], logger)
        actor_id = vehicle.actor.id
        if actor_id in actor_ids:
            raise SafetyError("Two UAV names resolved to the same actor")
        actor_ids.add(actor_id)
        vehicles[name] = vehicle
        logger.info("IDENTITY %s node_no=%d actor_id=%s type=%s",
                    name, number, actor_id, vehicle.actor.type_id)
    return vehicles


def hover_uavs(vehicles, planner, logger):
    """Best-effort UAV-only hold; never invoke Fleet.stop_all()."""
    for name in UAV_NAMES:
        if name not in vehicles:
            continue
        try:
            point = vehicles[name].position()
            if not planner.point_safe(point):
                logger.error("%s telemetry is already outside the safety envelope; emergency stationary hold", name)
            vehicles[name].target_point_control(point[0], point[1], ALTITUDE, 0.0)
        except Exception:
            logger.exception("Could not hold %s", name)


def run_controller(vehicles, planner, routes, logger, use_default=False):
    global _running, _use_default
    _running, _use_default = True, bool(use_default)
    indices = dict((name, 0) for name in UAV_NAMES)
    phases = dict((name, "takeoff") for name in UAV_NAMES)
    stable = dict((name, 0) for name in UAV_NAMES)
    last_goals, frame, next_tick = {}, 0, time.monotonic()
    next_status, next_overrun = next_tick, next_tick
    try:
        with _lock:
            discovered_enemies[:] = []
        publish_detections(use_default)
        if use_default:
            logger.warning("SYNTHETIC defaults enabled until the first real discovery")
        while True:
            positions = dict((name, vehicles[name].position()) for name in UAV_NAMES)
            commands, updates = {}, {}
            # Validate all three next commands before sending any of this frame.
            for name in UAV_NAMES:
                point, route = positions[name], routes[name]
                if not planner.point_safe(point):
                    raise SafetyError(name + " left the reserved safe flight envelope")
                if phases[name] == "takeoff":
                    stable[name] = stable[name] + 1 if (
                        abs(point[2] - ALTITUDE) <= ALTITUDE_TOL
                        and distance(point, route[0]) <= WAYPOINT_TOL) else 0
                    if stable[name] >= 5:
                        phases[name], indices[name] = "cruise", 1
                        logger.info("%s reached Z=250; starting search", name)
                elif abs(point[2] - ALTITUDE) > 5.0:
                    raise SafetyError(name + " cruise altitude deviated by more than 5 m")
                index = indices[name]
                if phases[name] == "cruise":
                    if index < len(route) and distance(point, route[index]) <= WAYPOINT_TOL:
                        index += 1
                        indices[name] = index
                    if index >= len(route):
                        phases[name] = "hold"
                        logger.info("%s search complete; continuing detection in hover", name)
                goal = route[min(indices[name], len(route)-1)]
                if not planner.segment_safe(point, goal):
                    raise SafetyError(name + " actual-position-to-waypoint segment is unsafe")
                speed = 0.0 if phases[name] == "hold" else CRUISE_SPEED
                commands[name] = (goal[0], goal[1], ALTITUDE, speed)
                if last_goals.get(name) != goal:
                    updates[name] = goal
            if updates:
                sync_goals(updates)
                last_goals.update(updates)
                for name, point in updates.items():
                    logger.info("WAYPOINT %s %d/%d XY=%s Z=250", name,
                                indices[name], len(routes[name])-1, point)
            for name in UAV_NAMES:
                vehicles[name].target_point_control(*commands[name])
            frame += 1
            if frame % DETECT_EVERY == 0:
                fresh = []
                for name in UAV_NAMES:
                    try:
                        point = vehicles[name].position()
                        result = sdk_call(logger, vehicles[name].Vehicle_.detect_situation)
                        fresh.extend(parse_detections(result, point))
                    except Exception:
                        logger.exception("DETECTION failed for %s", name)
                added = merge_detections(fresh)
                if added:
                    publish_detections(use_default)
                    logger.info("DISCOVERED new=%s total=%d", added, len(discovered_enemies))
            now = time.monotonic()
            if now >= next_status:
                logger.info("STATUS frame=%d positions=%s phases=%s enemies=%d",
                            frame, positions, phases, len(discovered_enemies))
                next_status = now + 5.0
            next_tick += DT
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                if now >= next_overrun:
                    logger.warning("Loop overran 50 ms by %.3f s; check SDK latency", -delay)
                    next_overrun = now + 5.0
                next_tick = time.monotonic()
    finally:
        hover_uavs(vehicles, planner, logger)
        _running = False


def make_logger():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.join(OUTPUT_DIR, time.strftime("run_uav_%H%M%S.log"))
    logger = logging.getLogger("control_uav")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    for old in list(logger.handlers):
        logger.removeHandler(old)
        old.close()
    handler = logging.FileHandler(path, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(handler)
    return logger, path


def main(argv=None):
    global _running, _use_default
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", help="Directory containing game.json and vehicles.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--node-nos", nargs=3, type=int, metavar=("UAV1", "UAV2", "UAV3"),
                        help="Override scene-order inference; actual actor types are still checked")
    parser.add_argument("--use-default", action="store_true",
                        help="Publish synthetic coordinates only until real enemies are detected")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--output-only", action="store_true",
                      help="With --use-default: test feeds without importing SDK or flying")
    mode.add_argument("--plan-only", action="store_true",
                      help="Offline route export; never update shared config or connect")
    args = parser.parse_args(argv)
    if args.output_only and not args.use_default:
        parser.error("--output-only requires --use-default")
    logger, log_path = make_logger()
    try:
        if args.output_only:
            with _lock:
                discovered_enemies[:] = []
                _use_default = True
            publish_detections(True)
            logger.warning("OUTPUT ONLY: published synthetic defaults; no simulator connection")
            return 0
        scene = choose_scene(args.scene)
        planner = SafePlanner(read_json(os.path.join(scene, "game.json")))
        entries = scene_uavs(scene)
        node_nos = args.node_nos or [entry[0] for entry in entries]
        logger.info("CONFIG scene=%s node_nos=%s Z=250 speed=15 DT=0.05", scene, node_nos)
        if args.plan_only:
            starts = dict((name, entry[1]) for name, entry in zip(UAV_NAMES, entries))
            routes = search_routes(planner, starts)
            atomic_json(os.path.join(OUTPUT_DIR, "search_plan.json"),
                        {"scene": scene, "node_nos": node_nos, "altitude_m": ALTITUDE,
                         "speed_mps": CRUISE_SPEED, "strip_spacing_m": STRIP_SPACING,
                         "safety_margin_m": SAFETY_MARGIN, "arena": planner.arena,
                         "no_fly_polygons": planner.polygons, "routes": routes})
            logger.info("OFFLINE plan exported; waypoint counts=%s",
                        dict((name, len(route)) for name, route in routes.items()))
            return 0
        vehicles = connect_uavs(args.host, args.port, node_nos, logger)
        starts = dict((name, vehicles[name].position()) for name in UAV_NAMES)
        routes = search_routes(planner, starts)
        # Planning and registration have not issued any movement; only this loop flies.
        run_controller(vehicles, planner, routes, logger, args.use_default)
        return 0
    except KeyboardInterrupt:
        logger.info("Interrupted; UAV-only hover cleanup completed")
        return 0
    except Exception:
        logger.exception("UAV module stopped")
        sys.stderr.write("UAV module stopped; see log: %s\n" % log_path)
        return 1
    finally:
        _running = False


if __name__ == "__main__":
    sys.exit(main())
