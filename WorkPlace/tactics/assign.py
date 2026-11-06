# -*- coding: utf-8 -*-
"""滚动时域任务分配：3 车 × M 阵位，瓶颈指派 + 承诺分级 + 防抖。

为什么不用启发式：|V|=3 时组合只有几十到几百种，穷举即严格最优，
引入拍卖/强化学习只会带来不确定性。

用法：
    plan = solve(vehicles, targets, eta_fn, prev=prev_plan,
                 improvement=0.20, max_preempt=2)
    if plan: publish(plan)      # None 表示「不比现在好，保持不动」

数据结构：
    vehicles = [{"vid": "UGV1", "pos": (x, y), "commit": "L0|L1|L2"}, ...]
    targets  = [{"tid": "E3", "kind": "junction",
                 "slots": [{"sid": "E", "pos": (x, y)}, ...]}, ...]
    eta_fn   = lambda vehicle, slot_pos: 秒数（由调用方给，通常用路网最短路长度 / 速度）
"""

from __future__ import print_function

import itertools

# 代价权重（先手调，熵权法留作消融实验）
W_BOTTLENECK = 0.50     # 最晚到达时刻（合围由最后一辆决定）
W_COVERAGE = 0.30       # 阵位覆盖率
W_WAIT = 0.15           # 目标已等待时长（越老越优先）
W_SWITCH = 0.05         # 改派惩罚（防抖）

INF = float("inf")


def _coverage_penalty(members, target):
    """没被占的阵位越多，惩罚越大（0 = 全占满）。"""
    d = len(target.get("slots") or [])
    if d <= 0:
        return 0.0
    return max(0.0, (d - members) / float(d))


def _evaluate(assign, vehicles, targets, eta, wait):
    """assign: [(vi, ti, si), ...] -> (bottleneck, sum, J)"""
    if not assign:
        return (INF, INF, INF)
    times = []
    for (vi, ti, si) in assign:
        v = vehicles[vi]
        slot = targets[ti]["slots"][si]
        times.append(eta(v["pos"], slot["pos"]))
    bottleneck = max(times)
    total = sum(times)
    cov = 0.0
    for ti in set(t for (_v, t, _s) in assign):
        n = sum(1 for (_v, t, _s) in assign if t == ti)
        cov += _coverage_penalty(n, targets[ti])
    cov /= max(1, len(set(t for (_v, t, _s) in assign)))
    wait_pen = 0.0
    for ti in set(t for (_v, t, _s) in assign):
        wait_pen += wait.get(targets[ti]["tid"], 0.0)
    wait_pen /= max(1, len(set(t for (_v, t, _s) in assign)))
    J = (W_BOTTLENECK * bottleneck + W_COVERAGE * cov
         + W_WAIT * wait_pen)
    return (bottleneck, total, J)


def _prune_slots(targets, min_gap=45.0):
    """同一目标内，两两夹角 <45° 的阵位只保留一个（同一条路不能派两台车）。"""
    for t in targets:
        slots = t.get("slots") or []
        kept = []
        for s in slots:
            d = s.get("dir")
            if d is None:
                kept.append(s)
                continue
            if any(abs(((d - k.get("dir", 999.0)) + 180.0) % 360.0 - 180.0) < min_gap
                   for k in kept if k.get("dir") is not None):
                continue
            kept.append(s)
        t["slots"] = kept
    return targets

def solve(vehicles, targets, eta_fn, prev=None, improvement=0.20,
          max_preempt=2, timeout_targets=3):
    """返回新计划 dict，或 None（保持现状）。

    prev: 上一次的计划（同结构），用于改派惩罚与改进阈值判断
    """
    if not vehicles or not targets:
        return None
    targets = _prune_slots([dict(t, slots=list(t.get("slots") or [])) for t in targets])



    # 候选阵位：每个 target 的每个 slot 展开成一列
    cols = []
    for ti, t in enumerate(targets):
        for si, s in enumerate(t.get("slots") or []):
            cols.append((ti, si))
    if not cols:
        return None
    if len(cols) > 12:
        cols = cols[:12]          # 3 车场景不可能需要这么多，防爆

    wait = {}
    for t in targets:
        wait[t["tid"]] = float(t.get("wait", 0.0))

    # 已锁定（L2）的车：固定去原阵位
    fixed = {}
    locked_v = set()
    if prev:
        for g in prev.get("groups", []):
            for s in g.get("slots", []):
                if (prev.get("commitment") or {}).get(s["vid"]) == "L2":
                    for vi, v in enumerate(vehicles):
                        if v["vid"] == s["vid"]:
                            locked_v.add(vi)
                            fixed[vi] = s

    best = None       # (key, assign, bottleneck, total, J)
    n_v = len(vehicles)
    idx_all = list(range(len(cols)))
    # 阵位可能少于车（例如只剩一个普通目标）：只派 k 辆车，连「派哪 k 辆」一起枚举
    k = min(n_v, len(cols))
    for vs in itertools.combinations(range(n_v), k):
        for combo in itertools.permutations(idx_all, k):
            assign = [(vs[i], cols[combo[i]][0], cols[combo[i]][1]) for i in range(k)]
            # 锁定（L2）的车只允许留在原目标上
            bad = False
            for (vi, ti, _si) in assign:
                if vi in locked_v:
                    want = fixed.get(vi)
                    if want and want.get("tid") and targets[ti]["tid"] != want["tid"]:
                        bad = True
                        break
            if bad:
                continue
            # 【分支互斥】同一目标的两个阵位若方向夹角 <45°，视为同一条路，禁止同时占用
            _dirs = {}
            _clash = False
            for (_vi, _ti, _si) in assign:
                _d = targets[_ti]["slots"][_si].get("dir")
                if _d is None:
                    continue
                for _dd in _dirs.get(_ti, []):
                    if abs(((_d - _dd) + 180.0) % 360.0 - 180.0) < 45.0:
                        _clash = True
                        break
                if _clash:
                    break
                _dirs.setdefault(_ti, []).append(_d)
            if _clash:
                continue
            bottleneck, total, J = _evaluate(assign, vehicles, targets, eta_fn, wait)
            key = (round(bottleneck, 3), round(total, 3))
            if best is None or key < best[0]:
                best = (key, assign, bottleneck, total, J)

    if best is None:
        return None
    key, assign, bottleneck, total, J = best

    # 防抖：必须明显优于上一次
    if prev is not None:
        old_J = prev.get("_J")
        if old_J is not None and J >= old_J * (1.0 - improvement):
            return None
        seen = {}
        for g in prev.get("groups", []):
            for s in g.get("slots", []):
                seen[s["vid"]] = g["target_id"]
        changes = sum(1 for (vi, ti, _si) in assign
                      if seen.get(vehicles[vi]["vid"]) not in (None, targets[ti]["tid"]))
        if changes > max_preempt:
            return None

    # 组装计划
    groups = {}
    commitment = {}
    for (vi, ti, si) in assign:
        t = targets[ti]
        g = groups.setdefault(t["tid"], {
            "target_id": t["tid"], "kind": t.get("kind", "road"),
            "junction_degree": int(t.get("junction_degree", 0)),
            "gap_branch": t.get("gap_branch"),
            "slots": [], "fire_rule": ("all_arrived_minus_gap"
                                       if t.get("kind") == "junction" else "single"),
            "state": "S3",
        })
        g["slots"].append({
            "vid": vehicles[vi]["vid"], "branch": t["slots"][si].get("sid"),
            "pos": list(t["slots"][si]["pos"]), "hold": None,
            "eta": round(eta_fn(vehicles[vi]["pos"], t["slots"][si]["pos"]), 1),
        })
        commitment[vehicles[vi]["vid"]] = "L1"

    for g in groups.values():
        g["t_star"] = round(max(s["eta"] for s in g["slots"]), 1)

    return {"groups": list(groups.values()), "commitment": commitment,
            "_J": J, "bottleneck": bottleneck, "total": total}


def apply_holds(plan, hold_of, assault=3.0):
    """给每个阵位补集结点与 GO 时刻（hold_of: (target, slot) -> (x, y)）。"""
    for g in plan.get("groups", []):
        for s in g.get("slots", []):
            if s.get("hold") is None:
                s["hold"] = list(hold_of(g["target_id"], s))
        t_star = g.get("t_star")
        if t_star is not None:
            g["go_at"] = round(max(0.0, t_star - assault), 1)
    return plan
