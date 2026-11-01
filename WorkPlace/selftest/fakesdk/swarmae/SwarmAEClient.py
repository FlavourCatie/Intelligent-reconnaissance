# -*- coding: utf-8 -*-
"""离线冒烟测试用假 SDK：模拟课程 swarmae 的最小可用子集。"""
import json
import math
import types

FIXED_DT = 0.05


class _Location(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Rotation(object):
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class _Transform(object):
    def __init__(self, location=None, rotation=None):
        self.location = location
        self.rotation = rotation


carla = types.SimpleNamespace(Location=_Location, Rotation=_Rotation,
                              Transform=_Transform)


class FakeActor(object):
    def __init__(self, node_type, actor_id, node=None):
        self.type_id = ("vehicle.hrt.m10" if "旋翼" in str(node_type)
                        else "vehicle.imv.rb")
        self.health_point = 100.0
        self.id = actor_id
        self.transform = None
        self.set_transform_count = 0
        self.node = node

    def set_enable_gravity(self, enabled):
        """假实现：记录重力开关。"""
        self.gravity_enabled = bool(enabled)

    def set_target_velocity(self, vec):
        """假实现：记录被清零的次数（真机上是清物理速度）。"""
        self.zero_vel_count = getattr(self, "zero_vel_count", 0) + 1

    def set_transform(self, tf):
        """和真机一致：改 actor 位姿后 get_location() 也跟着变。"""
        self.transform = tf
        self.set_transform_count += 1
        if self.node is None or tf is None:
            return
        try:
            self.node.x = tf.location.x
            self.node.y = tf.location.y
            self.node.z = tf.location.z
            if tf.rotation is not None:
                self.node.yaw = math.radians(tf.rotation.yaw)
        except Exception:
            pass


class FakeWeapon(object):
    def __init__(self):
        self._ammo = type("A", (), {"x": 95, "y": 15, "z": 2})()

    def get_weapon_ammo(self):
        return (0, self._ammo, 0)

    def set_weapon_status(self, *a, **k):
        return (0, 200)


_TARGET_CACHE = [None]


def _real_targets():
    """从 targets.json 读真实目标坐标，供假探测使用（只读一次）。"""
    if _TARGET_CACHE[0] is not None:
        return _TARGET_CACHE[0]
    import os
    root = os.environ.get("FAKE_PROJECT_ROOT", ".")
    path = os.path.join(root, "Materials", "practice", "practice1_Scene",
                        "targets.json")
    out = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        for group in raw:
            for pt in group.get("points", []):
                parts = [float(v) for v in str(pt["position"]).split(",")]
                out.append((parts[0], parts[1]))
    except Exception:
        pass
    _TARGET_CACHE[0] = out
    return out


class FakeNode(object):
    _next_id = [100]

    def __init__(self, node_type, name, no):
        self.node_type = str(node_type)
        self.name = name
        self.no = no
        self.x = -1829.6 + (no - 1) * 18.0
        self.y = -517.7 + (no - 1) * 4.0
        self.z = 100.0 if "旋翼" in self.node_type else 2.1
        self.yaw = 0.0
        self.v = 0.0
        self._weapon = FakeWeapon()
        FakeNode._next_id[0] += 1
        self._SwarmAENode__actor = FakeActor(self.node_type, FakeNode._next_id[0],
                                             node=self)

    def get_location(self):
        return (self.x, self.y, self.z, 0)

    def get_attitude(self):
        return (math.degrees(self.yaw), 0.0, 0.0, 0.0, 0)

    def get_velocity(self):
        vx = math.cos(self.yaw) * self.v
        vy = math.sin(self.yaw) * self.v
        return (vx, vy, 0.0, self.v, 0.0, 0.0, 0.0, 0.0, self.v,
                0.0, 0.0, 0.0, 0)

    def get_size(self):
        return {"x": 4.0, "y": 2.0, "z": 1.5}

    def get_color(self):
        return {"r": 1.0, "g": 0.3, "b": 0.3}

    def get_node_info(self):
        return (self.name, self.no, "蓝", self.node_type, "fake", None)

    def get_node_id(self):
        return self._SwarmAENode__actor.id

    def get_weapon(self):
        return (0, self._weapon, 0, 200)

    def get_sensor(self):
        raise RuntimeError("fake sdk: no camera")

    def detect_situation(self):
        """模拟"靠近真实目标就能发现它"：250 m 内最近的 targets.json 目标。"""
        best, bd = None, 250.0
        for tx, ty in _real_targets():
            d = math.hypot(tx - self.x, ty - self.y)
            if d < bd:
                best, bd = (tx, ty), d
        if best is None:
            return (0, 200, json.dumps([]))
        objs = [{
            "Type": "strike",
            "Type_Name": "dynamic.prop.barricade",
            "HealthPoint": 4200,
            "Position": {"X": best[0] * 100.0, "Y": best[1] * 100.0, "Z": 60.0},
            "Rotation": {"Yaw": 0},
        }]
        return (0, 200, json.dumps(objs))

    def apply_vehicle_control(self, throttle, steer, brake, hand_brake, gear):
        acc = throttle * 8.0 - brake * 12.0
        self.v = max(0.0, self.v + acc * FIXED_DT)
        self.yaw += steer * 0.6 * FIXED_DT * min(1.0, self.v / 6.0)
        self.x += math.cos(self.yaw) * self.v * FIXED_DT
        self.y += math.sin(self.yaw) * self.v * FIXED_DT
        return (0, 200)

    def control_kinetic_simply_global(self, x, y, z, v, ts):
        d = math.hypot(x - self.x, y - self.y)
        step = min(d, max(v, 0.0) * FIXED_DT)
        if d > 1e-6:
            self.x += (x - self.x) / d * step
            self.y += (y - self.y) / d * step
            self.yaw = math.atan2(y - self.y, x - self.x)
        dz = z - self.z
        self.z += max(-4.0 * FIXED_DT, min(4.0 * FIXED_DT, dz))
        self.v = v
        return (0, 200)


class FakeGame(object):
    def __init__(self, roadnet_path, offset):
        self._img = None
        self._roadnet_path = roadnet_path
        self._offset = offset
        self.stages = []

    def get_road_network(self):
        if self._img is None:
            from PIL import Image
            self._img = Image.open(self._roadnet_path)
        return (self._img, 1.0, self._offset, 0, 200)

    def stage_start(self, s):
        self.stages.append(("start", s))
        return (0, 200)

    def stage_complete(self, s):
        self.stages.append(("complete", s))
        return (0, 200)


class SwarmAEClient(object):
    _nodes = {}

    def __init__(self, ue_ip="127.0.0.1", ue_port=2000):
        import os
        import sys
        # 模拟真机 SDK 的坑：__init__ 里用 argparse 直接解析整个 sys.argv，
        # 只认 -i/-p/-s/-n/-t。设 FAKE_SDK_PARSE_ARGV=1 打开这个行为用于回归测试。
        if os.environ.get("FAKE_SDK_PARSE_ARGV") == "1":
            import argparse
            parser = argparse.ArgumentParser(description="fake sdk client")
            parser.add_argument("-i", "--ip", default="127.0.0.1")
            parser.add_argument("-p", "--port", type=int, default=2000)
            parser.add_argument("-s", "--subject", type=int, default=1)
            parser.add_argument("-n", "--number", type=int, default=10)
            parser.add_argument("-t", "--team", default="")
            parser.parse_args()          # 未知参数 -> SystemExit(2)
        self.ue_ip = ue_ip
        self.ue_port = ue_port
        root = os.environ.get("FAKE_PROJECT_ROOT", ".")
        self._game = FakeGame(
            os.path.join(root, "Materials", "practice", "easy_follow", "lib",
                         "roadnet.png"),
            (-2094.5277099609375, -2637.31982421875))

    def get_game(self):
        return (0, self._game, 200)

    def get_world(self):
        return (0, None, 200)

    def register_node(self, node_type=None, node_name=None, node_no=None,
                      frame_timestamp=None):
        key = node_name
        if key not in SwarmAEClient._nodes:
            SwarmAEClient._nodes[key] = FakeNode(node_type, node_name, node_no)
        return (0, SwarmAEClient._nodes[key], 200)
