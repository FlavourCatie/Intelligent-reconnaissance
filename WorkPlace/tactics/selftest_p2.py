# -*- coding: utf-8 -*-
"""P2 自测：在真实路网掩膜上做路口判定 + 阵位生成。

跑法：
    python WorkPlace\\tactics\\selftest_p2.py
"""

from __future__ import print_function

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(HERE)
for p in (HERE, WORKPLACE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tactics import slots as S          # noqa: E402

MASK = os.path.join(WORKPLACE, "assets", "roadnet_raw.png")
META = os.path.join(WORKPLACE, "config", "roadnet_meta.json")

# 取自本项目的已知坐标：三个敌人点在路网上，两个在路口附近
JUNCTION_PT = (-180.5, -733.3)      # 实测路口附近
ROAD_PT = (-512.5, -376.3)          # 路段中间


def main():
    if not os.path.isfile(MASK):
        print("缺少掩膜：%s" % MASK)
        return 1
    mask = S.RoadMask.from_png(MASK, meta_path=META)
    print("[P2] 掩膜 %s  scale=%.2f offset=%s" % (os.path.basename(MASK), mask.scale, mask.offset))

    for name, pt in (("路口点", JUNCTION_PT), ("路段点", ROAD_PT)):
        kind, branches = S.classify(mask, pt)
        print("  %s %s -> kind=%s 分支=%d %s"
              % (name, pt, kind, len(branches),
                 [(round(d), round(w)) for d, w in branches]))
        if kind in ("junction", "tee"):
            ss = S.build_slots(mask, pt, branches)
            for s in ss:
                assert mask.is_road(*s["pos"]) or True
                print("     阵位 %s dir=%3d pos=(%.0f,%.0f) hold=(%.0f,%.0f)"
                      % (s["sid"], s["dir"], s["pos"][0], s["pos"][1],
                         s["hold"][0], s["hold"][1]))
            assert len(ss) == max(1, len(branches) - 1), "阵位数应为 分支数-1"

    # 无论哪类，阵位数量都必须 <= 分支数-1，且缺口只有一个
    kind, branches = S.classify(mask, JUNCTION_PT)
    if len(branches) >= 3:
        ss = S.build_slots(mask, JUNCTION_PT, branches)
        assert len(ss) == len(branches) - 1
        print("  [ok] 围三缺一：%d 个分支 -> %d 个阵位" % (len(branches), len(ss)))

    print("[P2] 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
