# -*- coding: utf-8 -*-
"""P4/P5 自测：编队接入主循环 + 无人车开局位置优化。

分两层：
  A. 纯算法（不连仿真）：deploy / formation / convoy；
  B. 主循环状态机：用假 UgvAgent 跑 FormationRuntime（前沿待机 → 僚车跟随 → 分裂）。

跑法：
    python WorkPlace\\tactics\\selftest_p4.py
"""

from __future__ import print_function

import argparse
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(HERE)
PROJECT = os.path.dirname(WORKPLACE)
for _p in (HERE, WORKPLACE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                          # noqa: E402

from tactics.zones import Zone              # noqa: E402
from tactics import deploy                  # noqa: E402
from tactics.formation import Formation, project_on_path   # noqa: E402
from tactics.convoy import ConvoyController  # noqa: E402


ZONE = "-772,-1006,128,-306"


# ---------------------------------------------------------------------------
#  A. 开局位置优化
# ---------------------------------------------------------------------------
def check_deploy():
    binary, raw, scale, off = deploy.load_binary()
    zone = Zone.parse(ZONE, name="deploy_zone")
    starts = deploy.choose_starts(binary, zone, n=3, margin=150.0,
                                  min_sep=180.0, raw=raw, scale=scale, off=off)
    assert len(starts) == 3, "展开点数量不对: %d" % len(starts)
    for s in starts:
        x, y = s["pos"]
        assert zone.contains(x, y, margin=140.0), "展开点离边界太近: %s" % (s,)
        px, py = deploy.w2p(x, y, scale, off)
        assert binary[py, px], "展开点不在路网骨架上: %s" % (s,)
        assert s["deg"] == 2, "展开点不在直路上: %s" % (s,)
        assert s["width_m"] is None or s["width_m"] >= 5.5, "展开点路太窄: %s" % (s,)
        assert 0.0 <= s["yaw"] < 360.0, "朝向非法: %s" % (s,)
        # 朝向必须基本沿道路：把点沿朝向推 8 m，仍应踩在真实路面上
        hx = x + 8.0 * math.cos(math.radians(s["yaw"]))
        hy = y + 8.0 * math.sin(math.radians(s["yaw"]))
        w = deploy.road_width_at(raw, (hx, hy), math.radians(s["yaw"]),
                                 scale=scale, off=off)
        assert w is not None and w >= 4.0, "朝向没沿道路: %s width=%s" % (s, w)
    for i in range(3):
        for j in range(i + 1, 3):
            d = math.hypot(starts[i]["pos"][0] - starts[j]["pos"][0],
                           starts[i]["pos"][1] - starts[j]["pos"][1])
            assert d >= 179.0, "两车开局太近: %.1f m" % d
    cov = starts[0]["coverage_m"]
    assert cov <= 320.0, "开局覆盖半径太大: %.1f m" % cov
    # 确定性与导出/回读
    again = deploy.choose_starts(binary, zone, n=3, margin=150.0,
                                 min_sep=180.0, raw=raw, scale=scale, off=off)
    assert [s["pos"] for s in again] == [s["pos"] for s in starts], "选点不稳定"
    tmp = os.path.join(tempfile.mkdtemp(prefix="deploy_"), "starts.json")
    deploy.save_starts(starts, tmp)
    back = deploy.load_starts(tmp)
    assert len(back) == 3 and back[0]["pos"] == starts[0]["pos"], "导出/回读不一致"
    print("  [ok] deploy: 3 个展开点 %s，覆盖半径 %.0f m，最小间距 %.0f m"
          % ([(round(s["pos"][0]), round(s["pos"][1])) for s in starts],
             cov, starts[0]["min_sep_m"]))


def check_deploy_empty():
    """任务区里没有路时必须干净地返回空，而不是死循环/抛异常。"""
    binary, raw, scale, off = deploy.load_binary()
    z = Zone.parse("5000,5000,5100,5100", name="empty")
    starts = deploy.choose_starts(binary, z, n=3, raw=raw, scale=scale, off=off)
    assert starts == [], "空区域不该选出点: %s" % (starts,)
    print("  [ok] deploy: 空区域返回 []")


# ---------------------------------------------------------------------------
#  B. 编队几何
# ---------------------------------------------------------------------------
def _dist_to_path(path, p):
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return project_on_path(path, cum, p)[1]


def check_follow():
    f = Formation(kind="column", spacing=12.0, tau=0.0)
    path = [(0.0, 0.0), (100.0, 0.0)]
    pts = f.follow_targets(path, (80.0, 0.0), 2)
    assert abs(pts[0][0] - 68.0) < 1e-6 and abs(pts[1][0] - 56.0) < 1e-6, pts

    # 集结点：距终点 150 m 就停（路径只有 100 m → 集结点在起点）
    pts2 = f.follow_targets(path, (80.0, 0.0), 2, stop_before_goal=150.0)
    assert pts2 == [None, None], pts2

    # 长一点的路径：领航车还在 180 m，僚车最多到 300-150=150 m，且保持间距
    long_path = [(0.0, 0.0), (300.0, 0.0)]
    pts3 = f.follow_targets(long_path, (180.0, 0.0), 3, stop_before_goal=150.0)
    assert [round(p[0], 1) for p in pts3] == [150.0, 138.0, 126.0], pts3

    # 拐角：跟随点必须仍在路径上（不会像"按朝向偏移"那样偏进路外）
    corner = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]
    pts4 = f.follow_targets(corner, (100.0, 20.0), 2)
    for p in pts4:
        assert p is not None
        assert _dist_to_path(corner, p) < 1e-6, "拐角跟随点偏出路径: %s" % (p,)

    # 楔形左右交替；偏出路面时退回中心线
    fw = Formation(kind="wedge", spacing=10.0, lateral=4.0, tau=0.0)
    w = fw.follow_targets([(0.0, 0.0), (200.0, 0.0)], (100.0, 0.0), 3)
    assert (w[0][1] > 0) != (w[1][1] > 0), "楔形没有左右分开: %s" % (w,)
    fw2 = Formation(kind="wedge", spacing=10.0, lateral=4.0, tau=0.0)
    w2 = fw2.follow_targets([(0.0, 0.0), (200.0, 0.0)], (100.0, 0.0), 1,
                            project=lambda x, y: None)
    assert abs(w2[0][1]) < 1e-6, "楔形没退回中心线: %s" % (w2,)

    # 领航车还没启动：不给出目标，避免僚车往身后倒车
    assert f.follow_targets(path, (5.0, 0.0), 2) == [None, None]
    print("  [ok] formation: 沿路径跟随 / 集结点 / 拐角 / 楔形回退 / 起步保护")


def check_convoy():
    path = [(0.0, 0.0), (300.0, 0.0)]
    c = ConvoyController(kind="column", spacing=12.0, rally=200.0,
                         move_tol=12.0, min_interval=0.0)
    members = [{"name": "UGV2", "pos": (40.0, 0.0)},
               {"name": "UGV3", "pos": (80.0, 0.0)}]
    out = c.update(1.0, {"name": "UGV1", "pos": (200.0, 0.0),
                         "path": path, "kind": "staging"}, members)
    assert len(out) == 2, out
    # UGV3 离领航车更近 → 应排 1 号位（自己的跟随点更靠前）
    assert out["UGV3"][0] > out["UGV2"][0], out
    assert c.order == ["UGV3", "UGV2"], c.order

    # 同一位置再更新：滞回生效，不重复下发
    out2 = c.update(2.0, {"name": "UGV1", "pos": (200.0, 0.0),
                          "path": path, "kind": "staging"}, members)
    assert out2 == {}, out2

    # 领航车前进 30 m → 目标移动超过阈值，重新下发
    out3 = c.update(3.0, {"name": "UGV1", "pos": (230.0, 0.0),
                          "path": path, "kind": "staging"}, members)
    assert out3 and out3["UGV3"][0] > out["UGV3"][0], (out, out3)

    # 换领航车：历史清空，成员顺序重新排
    out4 = c.update(4.0, {"name": "UGV2", "pos": (100.0, 0.0),
                          "path": path, "kind": "strike"},
                    [{"name": "UGV1", "pos": (230.0, 0.0)},
                     {"name": "UGV3", "pos": (80.0, 0.0)}])
    assert "UGV2" not in out4, out4
    assert len(out4) == 2, out4
    print("  [ok] convoy: 排序 / 滞回 / 领航车切换")


def check_road_width():
    raw = np.zeros((40, 120), dtype=bool)
    raw[16:24, 10:110] = True          # 8 m 宽的东西向道路
    pos = (50.0, 20.0)                 # 世界坐标=像素坐标，offset=0, scale=1
    w_x = deploy.road_width_at(raw, pos, 0.0, scale=1.0, off=(0.0, 0.0))
    assert w_x == 8.0, "东西向路宽应为 8 m，实测 %s" % w_x
    w_y = deploy.road_width_at(raw, pos, math.pi / 2, scale=1.0, off=(0.0, 0.0),
                               limit_m=200.0)
    # 沿南北行驶时，横向是东西向 → 量到的是路的长度 100 m，而不是宽度
    assert w_y == 100.0, "南北向应量到路长 100 m，实测 %s" % w_y
    print("  [ok] deploy: 路宽测量")


# ---------------------------------------------------------------------------
#  C. 主循环状态机（假 UgvAgent，不连仿真）
# ---------------------------------------------------------------------------
class _Pose(object):
    def __init__(self, x, y, z=2.1, theta=0.0):
        self.x_, self.y_, self.z_, self.theta_ = x, y, z, theta


class _State(object):
    def __init__(self, pose):
        self.pose_ = pose


class DummyUgv(object):
    def __init__(self, x, y):
        self.pose = _Pose(x, y)
        self.stopped = False

    def get_state(self):
        return _State(self.pose)

    def get_transform(self):
        return self.pose

    def apply_control(self, *a, **k):
        pass


class DummyAgent(object):
    """只实现 FormationRuntime 用到的接口，路径直接连成直线。"""

    def __init__(self, ugv, name, target, binary=None, raw=None, scale=1.0,
                 off=(0.0, 0.0), role="strike", kind="strike"):
        self.ugv = ugv
        self.name = name
        self.target = (float(target[0]), float(target[1]))
        self.role = role
        self.kind = kind
        self.status = "drive"
        self.path = [(ugv.pose.x_, ugv.pose.y_), self.target]

    def start(self):
        self.status = "drive"

    def retarget(self, target, role=None, kind=None):
        if role is not None:
            self.role = role
        if kind is not None:
            self.kind = kind
        self.target = (float(target[0]), float(target[1]))
        self.path = [(self.ugv.pose.x_, self.ugv.pose.y_), self.target]
        self.status = "drive"
        return True


def check_runtime_integration():
    # 假 SDK 必须排在 sys.path 前面，否则 import control_ugv 会去找真 swarmae
    fake = os.path.join(WORKPLACE, "selftest", "fakesdk")
    if fake not in sys.path:
        sys.path.insert(0, fake)
    missions = os.path.join(WORKPLACE, "missions")
    if missions not in sys.path:
        sys.path.insert(0, missions)
    os.environ.setdefault("FAKE_PROJECT_ROOT", PROJECT)
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib
    matplotlib.use("Agg")

    import full_mission as fm

    binary, raw, scale, off = deploy.load_binary()
    zone = Zone.parse(ZONE, name="runtime_zone")
    from tactics.planner import Tactics
    tmp = tempfile.mkdtemp(prefix="runtime_plan_")
    tac = Tactics(zone, os.path.join(tmp, "strike_plan.json"))
    # 模拟无人机刚扫过区域西侧（东侧留下明确前沿，编队才知道该往哪待机）
    for x in range(-700, -300, 80):
        for y in range(-950, -350, 80):
            tac.mark_coverage(x, y, r=120)

    args = argparse.Namespace(formation="column", formation_spacing=12.0,
                              formation_lateral=4.0, formation_rally=200.0,
                              no_staging=False)
    ugvs = [DummyUgv(-500.0, -650.0), DummyUgv(-300.0, -700.0),
            DummyUgv(-150.0, -550.0)]
    road_points = fm.build_road_points(binary, scale, off, step=40)
    # 按网格抽样会只剩几十个点（骨架只有 1 px 宽），这里必须按路面像素抽样
    assert len(road_points) >= 1000, "路网候选点太少: %d" % len(road_points)
    rt = fm.FormationRuntime(args, tac, binary, raw, scale, off, road_points)

    orig_cls = fm.ugv.UgvAgent
    fm.ugv.UgvAgent = DummyAgent
    try:
        agents = []
        contacts = {0: {"pos": (-400.0, -600.0), "t": 10.0}}
        # 无人机没完全升空前：即使车载传感器已有 contact，也不许动车
        rt.update(now=9.0, el=9.0, agents=agents, ugvs=ugvs, enemies=[],
                  contacts=contacts, uav_ready=False)
        assert agents == [], "无人机未升空时无人车就动了: %s" % (
            [(a.name, a.role) for a in agents],)
        rt.update(now=10.0, el=10.0, agents=agents, ugvs=ugvs, enemies=[],
                  contacts=contacts, uav_ready=True)
        assert len(agents) == 1, "前沿待机应先由一辆领航车起步: %s" % (
            [(a.name, a.role, a.kind) for a in agents],)
        lead = agents[0]
        assert lead.role == "lead" and lead.kind == "staging", (
            lead.role, lead.kind)
        assert rt.staging_target is not None, "没选到前沿待机点"

        # 领航车前进 40 m 后，僚车才拿到队列位置（避免起步时往身后倒车）
        sx, sy = lead.ugv.pose.x_, lead.ugv.pose.y_
        tx, ty = lead.target
        L = math.hypot(tx - sx, ty - sy) or 1.0
        lead.ugv.pose.x_ = sx + (tx - sx) / L * min(40.0, L * 0.5)
        lead.ugv.pose.y_ = sy + (ty - sy) / L * min(40.0, L * 0.5)
        rt.last_update = 0.0
        rt.update(now=12.0, el=12.0, agents=agents, ugvs=ugvs, enemies=[],
                  contacts=contacts, uav_ready=True)
        assert len(agents) == 3, "前沿待机没有形成 3 台车的编队: %s" % (
            [(a.name, a.role, a.kind) for a in agents],)
        follow = [a for a in agents if a.role == "follow"]
        assert len(follow) == 2, [(a.name, a.role) for a in agents]

        # 没动 → 不重复下发/不新增 agent
        n_before = len(agents)
        rt.update(now=13.0, el=13.0, agents=agents, ugvs=ugvs, enemies=[],
                  contacts=contacts, uav_ready=True)
        assert len(agents) == n_before, "滞回失效，重复创建 agent"

        # 接敌：UGV1 从编队里分裂成打击车，其余僚车改为跟随它
        striker = [a for a in agents if a.name == "UGV1"][0]
        rt.on_split("UGV1")
        striker.retarget((-400.0, -600.0), role="strike", kind="strike")
        striker.path = [(ugvs[0].pose.x_, ugvs[0].pose.y_), (0.0, 0.0),
                        (400.0, 0.0), (-400.0, -600.0)]
        # 打击车走出 60 m 后，编队才有可跟随的区段
        sx, sy = striker.path[0]
        tx, ty = striker.path[1]
        L = math.hypot(tx - sx, ty - sy) or 1.0
        striker.ugv.pose.x_ = sx + (tx - sx) / L * 60.0
        striker.ugv.pose.y_ = sy + (ty - sy) / L * 60.0
        # 让另外两台的阶段时间推进，避开 0.5 s 节流
        rt.last_update = 0.0
        rt.update(now=20.0, el=20.0, agents=agents, ugvs=ugvs,
                  enemies=[(-400.0, -600.0)])
        followers = [a for a in agents if a.role == "follow"]
        assert len(followers) == 2, [(a.name, a.role) for a in agents]
        for a in followers:
            assert a.kind == "follow"
            assert a.target != striker.target, "僚车不该和打击车同点"
        assert rt.staging_done, "接敌后没有停止前沿待机"

        # 分裂：UGV2 转打击，清掉跟随历史；下一拍它不能再被拉回编队
        rt.on_split("UGV2")
        f2 = [a for a in agents if a.name == "UGV2"][0]
        f2.retarget((-350.0, -620.0), role="strike", kind="strike")
        rt.last_update = 0.0
        rt.update(now=30.0, el=30.0, agents=agents, ugvs=ugvs,
                  enemies=[(-400.0, -600.0), (-350.0, -620.0)])
        assert f2.role == "strike" and f2.kind == "strike", (
            f2.role, f2.kind)
        assert all(not (a.name == "UGV2" and a.role == "follow") for a in agents), \
            "分裂后的车又被编队拉回去了"
    finally:
        fm.ugv.UgvAgent = orig_cls
    print("  [ok] runtime: 前沿待机 → 编队跟随 → 接敌切换 → 分裂")


def check_agent_role_params():
    """僚车与打击车的到点容差/跟车间距必须不同，否则编队收不拢。"""
    fake = os.path.join(WORKPLACE, "selftest", "fakesdk")
    if fake not in sys.path:
        sys.path.insert(0, fake)
    missions = os.path.join(WORKPLACE, "missions")
    if missions not in sys.path:
        sys.path.insert(0, missions)
    os.environ.setdefault("FAKE_PROJECT_ROOT", PROJECT)
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib
    matplotlib.use("Agg")

    from control_ugv import UgvAgent
    binary, raw, scale, off = deploy.load_binary()
    v = DummyUgv(-518.0, -624.0)
    a = UgvAgent(v, "UGV1", (-450.0, -624.0), binary, raw, scale, off,
                 role="follow")
    assert a.front_gap == 8.0 and a.arrive_dist == 4.0, (
        a.front_gap, a.arrive_dist)
    assert a.retarget((-400.0, -624.0), role="strike", kind="strike")
    assert a.front_gap == 18.0 and a.arrive_dist == 1.0, (
        a.front_gap, a.arrive_dist)
    print("  [ok] UgvAgent: 僚车/打击车的跟车间距与到点容差")


def check_route_and_contacts():
    """合围路线校验 + 车载传感器补盲。"""
    fake = os.path.join(WORKPLACE, "selftest", "fakesdk")
    if fake not in sys.path:
        sys.path.insert(0, fake)
    missions = os.path.join(WORKPLACE, "missions")
    if missions not in sys.path:
        sys.path.insert(0, missions)
    os.environ.setdefault("FAKE_PROJECT_ROOT", PROJECT)
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib
    matplotlib.use("Agg")

    import full_mission as fm
    from tactics import slots as S
    from tactics.slots import RoadMask

    binary, raw, scale, off = deploy.load_binary()
    mask = RoadMask.from_png(os.path.join(WORKPLACE, "assets", "roadnet_raw.png"),
                            meta_path=os.path.join(WORKPLACE, "config",
                                                   "roadnet_meta.json"))
    enemy = (-390.5, -625.3)
    kind, branches = S.classify(mask, enemy)
    assert kind == "junction", kind
    ss = S.build_slots(mask, enemy, branches)
    slots = dict((s["sid"], s["pos"]) for s in ss)
    west = fm.road_path(binary, scale, off, (-500.0, -625.0), slots["W"])
    east = fm.road_path(binary, scale, off, (-500.0, -625.0), slots["E"])
    assert fm.route_violation(west, enemy, slots["W"]) is None
    assert fm.route_violation(east, enemy, slots["E"]) is not None, \
        "西侧的车不该被允许接东侧阵位（路线会穿过敌人）"
    east_ok = fm.road_path(binary, scale, off, (-300.0, -625.0), slots["E"])
    assert fm.route_violation(east_ok, enemy, slots["E"]) is None

    # 车载传感器：只收 Soldier/Stryker，且同一点 25 m 内合并
    class _Ob(object):
        Type = ["Soldier", "RaiderBuggy", "Stryker"]
        x_ue = [-400.0, -100.0, -401.0]
        y_ue = [-600.0, -100.0, -601.0]
        Health = [100.0, 100.0, 100.0]

    class _V(object):
        def detect_fun(self):
            return _Ob()

    class _A(object):
        ugv = _V()

    zone = Zone.parse(ZONE, name="contact_zone")
    store = {}
    fm.scan_ugv_contacts([_A()], [], 10.0, store, zone)
    assert len(store) == 1, store
    pos = list(store.values())[0]["pos"]
    assert math.hypot(pos[0] + 400.0, pos[1] + 600.0) < 2.0, pos
    fm.scan_ugv_contacts([_A()], [], 11.0, store, zone)
    assert len(store) == 1, "同一敌人被重复登记"
    fm.scan_ugv_contacts([], [], 50.0, store, zone)
    assert len(store) == 0, "30 s 未再看到应过期"
    print("  [ok] 合围路线校验 / 车载传感器补盲")


def main():
    print("[P4/P5] 自测开始")
    check_deploy()
    check_deploy_empty()
    check_follow()
    check_convoy()
    check_road_width()
    check_runtime_integration()
    check_agent_role_params()
    check_route_and_contacts()
    print("[P4/P5] 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
