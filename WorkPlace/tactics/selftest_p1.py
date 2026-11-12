# -*- coding: utf-8 -*-
"""P1 自测：覆盖前沿 / 编队 / 滚动重分配。

跑法：
    python WorkPlace\\tactics\\selftest_p1.py
"""

from __future__ import print_function

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(HERE)
for p in (HERE, WORKPLACE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tactics.zones import Zone                   # noqa: E402
from tactics.coverage import CoverageGrid        # noqa: E402
from tactics.formation import Formation          # noqa: E402
from tactics import assign                       # noqa: E402


def check_coverage():
    z = Zone.parse("-772,-1006,128,-306", name="zone")
    cov = CoverageGrid(z, cell=50.0, edge_clearance=100.0)
    # 扫西半边
    # 只覆盖西侧一条带（模拟无人机刚扫完西边），留出明确的前沿
    for x in range(-750, -350, 40):
        for y in range(-950, -350, 40):
            cov.mark(x, y, r=120)

    # 贴着边界的点必须被排除
    near_edge = (-770, -1000)
    assert cov.inner_ok(*near_edge) is False, "贴边点没被排除"

    # 交界带上的点应被选中；深覆盖区的点不应被选中
    cand = []
    for x in range(-700, 130, 50):
        for y in range(-1000, -300, 50):
            cand.append((float(x), float(y)))
    pts = cov.frontier_points(cand, max_pts=3, min_gap=100.0)
    assert pts, "前沿点一个都没选出来"
    for (x, y) in pts:
        assert cov.inner_ok(x, y)
        assert cov.on_frontier(x, y)
    # 已覆盖内部的点（比如 -600,-650）不该是前沿
    assert not cov.on_frontier(-600, -650, band=0), "覆盖内部的点被当成前沿"
    print("  [ok] coverage: %d 个前沿点 %s" % (len(pts), [(round(a), round(b)) for a, b in pts]))


def check_formation():
    f = Formation(kind=Formation.COLUMN, spacing=10.0, lateral=4.0, tau=0.0)
    t = f.targets((0.0, 0.0), 0.0, n_members=2, dt=0.2)
    assert abs(t[0][0] + 10.0) < 1e-6 and abs(t[0][1]) < 1e-6, t
    assert abs(t[1][0] + 20.0) < 1e-6, t

    # 楔形：左右交替
    f2 = Formation(kind=Formation.WEDGE, spacing=10.0, lateral=4.0, tau=0.0)
    w = f2.targets((0.0, 0.0), 0.0, n_members=3, dt=0.2)
    assert (w[0][1] > 0) != (w[1][1] > 0), "楔形没有左右分开: %s" % (w,)

    # 路线吸附失败时，楔形退回纵队
    f3 = Formation(kind=Formation.WEDGE, spacing=10.0, lateral=4.0, tau=0.0)
    r = f3.targets((0.0, 0.0), 0.0, n_members=1, dt=0.2,
                   project=lambda x, y: None)
    assert abs(r[0][1]) < 1e-6, "楔形没退回纵队: %s" % (r,)

    # 路宽自适应
    assert Formation.choose_kind(12.0) == Formation.WEDGE
    assert Formation.choose_kind(6.0) == Formation.COLUMN

    # 平滑过渡：队形参数突变时，偏移量应逐步收敛而不是一步跳过去
    f4 = Formation(kind=Formation.COLUMN, spacing=40.0, lateral=0.0, tau=0.6)
    a = f4.targets((0.0, 0.0), 0.0, n_members=1, dt=0.2)[0]
    assert abs(a[0] + 40.0) < 1e-6, a
    f4.spacing = 10.0
    b = f4.targets((0.0, 0.0), 0.0, n_members=1, dt=0.2)[0]
    assert -40.0 < b[0] < -10.0, "偏移没有平滑收敛: %s -> %s" % (a, b)
    print("  [ok] formation: 纵队/楔形/回退/路宽/平滑")


def check_assign():
    vehicles = [
        {"vid": "UGV1", "pos": (0.0, 0.0), "commit": "L0"},
        {"vid": "UGV2", "pos": (100.0, 0.0), "commit": "L0"},
        {"vid": "UGV3", "pos": (1000.0, 0.0), "commit": "L0"},
    ]
    targets = [
        {"tid": "E1", "kind": "junction", "junction_degree": 3, "wait": 10.0,
         "slots": [{"sid": "N", "pos": (0.0, 50.0)},
                   {"sid": "E", "pos": (50.0, 0.0)},
                   {"sid": "S", "pos": (0.0, -50.0)}]},
    ]
    eta = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1]) / 8.0

    plan = assign.solve(vehicles, targets, eta)
    assert plan is not None and plan["groups"], "没生成计划"
    g = plan["groups"][0]
    assert g["kind"] == "junction" and g["fire_rule"] == "all_arrived_minus_gap"
    assert len(g["slots"]) == 3, "三个阵位应该有车"

    # 最近的 UGV1 应该被派到离它最近的阵位（E 或 N/S 中最近）
    d = {s["vid"]: math.hypot(s["pos"][0], s["pos"][1]) for s in g["slots"]}
    assert d["UGV1"] <= 70.0, "最近的车的阵位不近: %s" % d

    # 同样的输入再解一次：没有改进 -> 返回 None（防抖）
    plan["_J"] = plan["_J"] * 0.5      # 假装上一次特别优
    again = assign.solve(vehicles, targets, eta, prev=plan, improvement=0.20)
    assert again is None, "没有改进却下发了新计划"

    # 明显更优时允许改派
    plan["_J"] = plan["_J"] * 3.0
    better = assign.solve(vehicles, targets, eta, prev=plan, improvement=0.20)
    assert better is not None, "明显更优却不改派"

    # 集结点与 GO 时刻
    assign.apply_holds(better, lambda tid, s: (s["pos"][0], s["pos"][1]))
    for s in better["groups"][0]["slots"]:
        assert s["hold"] is not None
    assert better["groups"][0].get("go_at") is not None
    print("  [ok] assign: 瓶颈指派/防抖/改派/集结点")


def main():
    print("[P1] 自测开始")
    check_coverage()
    check_formation()
    check_assign()
    print("[P1] 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
