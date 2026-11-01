# -*- coding: utf-8 -*-
"""
scenefile.py —— 场景目标（敌人 / 障碍物 / 我方起点）的读写
============================================================

`subject4.exe` 启动时会读同目录下的这几个 json 来生成 3D 场景：

    vehicles.json  我方 3 车 3 机的位置
    targets.json   敌方目标（可打击 / 可阻挡）
    elements.json  雷区 / 电磁区 / 禁飞区
    game.json      任务区、阶段线

所以"生成敌人"和"重置场景"本质上是**改这些文件 + 重启 subject4.exe**，
SDK 本身没有运行时生成 Actor 的接口。

本模块被三处共用，保证格式一致：
    · missions/live_track.py    把目标画到地图上
    · tools/spawn_targets.py    生成敌人并写回 targets.json
    · tools/reset_scene.py      还原备份
"""

from __future__ import print_function

import json
import os
import shutil

# ---------------------------------------------------------------------------
#  目标类型表：类型名 -> (中文名, 建议弹种, 威胁等级)
#  弹种说明（官方文档）：0=子弹 1=40炮 2=反坦克炮
#  注意：lib/swarm_api.py 里实际调用用的是 3/4/6，与文档对不上，以真机为准。
# ---------------------------------------------------------------------------
TYPE_INFO = {
    "dynamic.prop.barricade":       ("路障（旁挂士兵）", 1, "medium"),
    "dynamic.prop.hardbarricade":   ("加固路障",         1, "medium"),
    "dynamic.prop.buildingsoldier": ("建筑内人员",       1, "medium"),
    "dynamic.prop.soldier":         ("士兵",             0, "low"),
    "vehicle.ifv.stryker":          ("敌方装甲车",       2, "high"),
    "dynamic.prop.pillbox":         ("简易工事",         1, "medium"),
    "Building":                     ("有人员建筑",       1, "medium"),
}

THREAT_ORDER = {"high": 3, "medium": 2, "low": 1, None: 0}


def type_cn(name):
    return TYPE_INFO.get(name, (name, 1, None))[0]


def type_ammo(name):
    return TYPE_INFO.get(name, (name, 1, None))[1]


def type_threat(name):
    return TYPE_INFO.get(name, (name, 1, None))[2]


# ---------------------------------------------------------------------------
#  结构
# ---------------------------------------------------------------------------
class Target(object):
    """一个敌方目标/障碍物。坐标单位：米（与 targets.json 一致）。"""

    __slots__ = ("type", "x", "y", "z", "assoc")

    def __init__(self, type_name, x, y, z=0.0, assoc=""):
        self.type = type_name
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.assoc = assoc          # 所属主目标（barricade 旁的士兵会指回路障）

    @property
    def cn(self):
        return type_cn(self.type)

    @property
    def threat(self):
        return type_threat(self.type)

    @property
    def ammo(self):
        return type_ammo(self.type)

    def to_dict(self):
        return dict(type=self.type, x=self.x, y=self.y, z=self.z,
                    cn=self.cn, threat=self.threat, ammo=self.ammo)

    def __repr__(self):
        return "<Target %s (%.1f, %.1f)>" % (self.type, self.x, self.y)


def _parse_offset(s):
    """'1.2, 7, 1.32' -> (1.2, 7.0, 1.32)"""
    parts = [float(v) for v in str(s).split(",")]
    if len(parts) < 3:
        parts += [0.0] * (3 - len(parts))
    return parts[0], parts[1], parts[2]


# ---------------------------------------------------------------------------
#  读
# ---------------------------------------------------------------------------
def load_targets(path):
    """把 targets.json 展开成 Target 列表（含 barricade 旁挂的士兵）。"""
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    out = []
    for group in raw:
        names = group.get("types") or []
        if isinstance(names, str):
            names = [names]
        main_name = names[0] if names else "unknown"
        for pt in group.get("points", []):
            x, y, z = _parse_offset(pt["position"])
            out.append(Target(main_name, x, y, z))
            for fl in pt.get("flowers", []) or []:
                fx, fy, fz = _parse_offset(fl["position"])
                out.append(Target(fl.get("type", "dynamic.prop.soldier"),
                                  x + fx, y + fy, z + fz, assoc=main_name))
    return out


def load_vehicles(path):
    """我方起点：返回 [{'type':.., 'x':.., 'y':.., 'z':..}]"""
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    out = []
    for group in raw:
        for v in group.get("vehicles", []):
            x, y, z = _parse_offset(v["position"])
            out.append(dict(type=group.get("type"), x=x, y=y, z=z,
                            detectRadius=v.get("detectRadius")))
    return out


def load_game(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def arena_polygon(game):
    """任务区顶点，返回 [(x,y), ...]"""
    return [(p["x"], p["y"]) for p in game.get("SUBJECT_ARENA", {}).get("arena", [])]


def _poly(points):
    return [(float(p["x"]), float(p["y"])) for p in points]


def load_environment(game_path):
    """读 game.json，取出所有**区域类**环境要素，供地图绘制。

    返回：
        {
          "arena":  [(x,y), ...],                 # 任务区（一个多边形）
          "no_fly": [[(x,y), ...], ...],          # 禁飞区（多个多边形）
          "mine":   [(x,y), ...],                 # 雷区
          "emc":    [(x,y), ...],                 # 电磁干扰区
          "lines":  {"SUBJECT_1": {"start": (p1, p2), "end": (p1, p2)}, ...},
        }
    缺项时对应值为空列表 / 空字典，不会抛异常。
    """
    game = load_game(game_path)
    a = game.get("SUBJECT_ARENA", {}) or {}

    env = {
        "arena": _poly(a.get("arena", []) or []),
        "no_fly": [_poly(poly) for poly in (a.get("no_fly_arena", []) or [])],
        "mine": _poly(a.get("mine_arena", []) or []),
        "emc": _poly(a.get("emc_arena", []) or []),
        "lines": {},
    }

    for i in range(1, 10):
        key = "SUBJECT_%d" % i
        blk = game.get(key)
        if not isinstance(blk, dict):
            continue
        entry = {}
        for side in ("start_line", "end_line"):
            pts = blk.get(side) or []
            if len(pts) >= 2:
                entry[side.replace("_line", "")] = (_poly([pts[0]])[0],
                                                    _poly([pts[1]])[0])
        if entry:
            env["lines"][key] = entry
    return env


# ---------------------------------------------------------------------------
#  写
# ---------------------------------------------------------------------------
def build_targets_json(targets, merge_same_type=True):
    """把 Target 列表还原成 subject4.exe 认识的 targets.json 结构。

    同类型合并到一个 group 里，count 自动填。
    """
    groups = {}
    order = []
    for t in targets:
        if t.assoc:                      # 旁挂的士兵由主目标带出，不单独成组
            continue
        key = t.type
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(t)

    out = []
    for key in order:
        pts = []
        for t in groups[key]:
            pts.append({"position": "%.2f, %.2f, %.2f" % (t.x, t.y, t.z),
                        "rotation": "0,0,0"})
        out.append({"types": [key], "count": len(pts), "points": pts})
    return out


def save_targets(path, targets, backup=True):
    """写 targets.json。默认先备份原文件为 .orig（只备份一次，不覆盖旧备份）。"""
    if backup:
        backup_once(path)
    data = build_targets_json(targets)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    os.replace(tmp, path)
    return len(data)


def backup_once(path):
    """首次调用时把原文件存成 <name>.orig；已存在则不覆盖。"""
    bak = path + ".orig"
    if not os.path.exists(bak) and os.path.exists(path):
        shutil.copy2(path, bak)
    return bak


def restore_backup(path):
    """从 .orig 还原。返回 True 表示还原成功。"""
    bak = path + ".orig"
    if not os.path.exists(bak):
        return False
    shutil.copy2(bak, path)
    return True


def summary(targets):
    """按类型统计，给命令行打印用。"""
    from collections import Counter
    c = Counter(t.type for t in targets)
    return [(k, type_cn(k), n) for k, n in c.most_common()]


# ---------------------------------------------------------------------------
#  自检
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(here))
    scene = os.path.join(root, "Materials", "practice", "practice1_Scene")
    tp = os.path.join(scene, "targets.json")
    vp = os.path.join(scene, "vehicles.json")

    ts = load_targets(tp)
    print("targets.json -> %d 个目标" % len(ts))
    for k, cn, n in summary(ts):
        print("   %-30s %-16s %d" % (k, cn, n))
    print("\n我方起点：")
    for v in load_vehicles(vp):
        print("   %-22s (%.1f, %.1f, %.1f) r=%s"
              % (v["type"], v["x"], v["y"], v["z"], v["detectRadius"]))

    # 往返测试：读 -> 写 -> 再读，数量应一致（旁挂士兵不单独成组，所以只比主目标）
    mains = [t for t in ts if not t.assoc]
    rebuilt = build_targets_json(mains)
    print("\n往返重建分组数 = %d（主目标 %d 个）" % (len(rebuilt), len(mains)))
