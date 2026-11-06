# -*- coding: utf-8 -*-
"""P0 自测：区域参数化 + 计划文件原子写/热加载。

跑法：
    python WorkPlace\\tactics\\selftest_p0.py
全部通过返回 0，否则抛异常。
"""

from __future__ import print_function

import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(HERE)
for p in (HERE, WORKPLACE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tactics.zones import Zone, ZoneError          # noqa: E402
from tactics import plan_io                        # noqa: E402


def check_zone():
    z = Zone.parse("-772,-1006,128,-306", name="junction_core")
    assert z.contains(-500, -600)
    assert not z.contains(0, 0)
    z.assert_inside((-500, -600), "target")
    try:
        z.assert_inside((500, -600), "target")
    except ZoneError:
        pass
    else:
        raise AssertionError("越界点没有被拦下")

    # 缺参数必须报错，而不是悄悄用默认值
    for bad in (None, "", "1,2,3", "a,b,c,d"):
        try:
            Zone.parse(bad)
        except ZoneError:
            continue
        raise AssertionError("非法 --zone 没有被拒绝: %r" % (bad,))

    # 顺手校验一下候选区都在 arena 内
    arena = (-1900, -1108, 2277, 341)
    assert arena[0] <= z.x0 and z.x1 <= arena[2], "zone 横向越出 arena"
    assert arena[1] <= z.y0 and z.y1 <= arena[3], "zone 纵向越出 arena"
    print("  [ok] zone 解析 / 越界拦截 / arena 包含")


def check_plan_io():
    tmpdir = tempfile.mkdtemp(prefix="plan_io_")
    path = os.path.join(tmpdir, "strike_plan.json")
    store = plan_io.PlanStore(path)

    assert store.load()["groups"] == []
    plan = plan_io.make_plan(
        groups=[plan_io.make_group(
            "E3", "junction",
            slots=[plan_io.make_slot("UGV1", "E", (905, 700), (860, 700), eta=21.3)],
            junction_degree=4, gap_branch="N", t_star=24.0, go_at=21.0)],
        commitment={"UGV1": "L1"},
        t=100.0)
    store.write(plan)

    assert os.path.isfile(path), "计划文件没写出来"
    again = plan_io.PlanStore(path)
    data = again.load()
    assert data["groups"][0]["slots"][0]["vid"] == "UGV1"
    assert data["commitment"]["UGV1"] == "L1"
    assert data["plan_id"] == 1

    # 热加载：写第二次必须被 changed() 感知
    time.sleep(0.05)
    store.write(plan)
    assert again.changed(), "mtime 变化没被感知"
    assert again.load()["plan_id"] == 2

    # 原子性：文件内部不能出现半截 JSON
    with open(path, "r", encoding="utf-8") as fp:
        import json
        json.load(fp)
    print("  [ok] plan_io 原子写 / 编号递增 / mtime 热加载")


def main():
    print("[P0] 自测开始")
    check_zone()
    check_plan_io()
    print("[P0] 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
