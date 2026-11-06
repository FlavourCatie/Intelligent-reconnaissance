# -*- coding: utf-8 -*-
"""
sweep_plan.py —— 牛耕式（boustrophedon）覆盖扫描航线
================================================================================

为什么用扫描
------------
侦查开始时**敌人位置未知**，所以不能"直奔敌人"，而要对任务区做**覆盖扫描**，
保证区域里任意一点都进入过无人机探测半径。

核心参数
--------
* `R`      : 探测半径（本课程 = 300 m）
* 条带间距 : **2R**（= 600 m）。两条相邻扫描线的探测圆刚好相切，不留缝；
             再大就会出现漏检带。

为什么要按长边扫描
------------------
同样面积下，扫描线**平行于长边**时条带数最少 → 总航程最短、转弯次数最少。

三机如何均衡
------------
先算出每条扫描线的**实际长度**（绕开禁飞区后可能不等长），
再按"累计长度三等分"切分给三架机 —— 不是简单按条数均分，
否则遇到长短不一的条带会出现明显不均衡。

输出
----
三条航点序列（UAV1/2/3），可直接喂给运动学模型或飞控。

用法
----
    # 默认：整个任务区，R=300 m，3 架机
    python WorkPlace\\missions\\sweep_plan.py

    # 只扫某个矩形区域（x0,y0,x1,y1），并画图
    python WorkPlace\\missions\\sweep_plan.py --region -1900,-1110,2280,340 --plot

    # 保存成 json 供 full_mission 使用
    python WorkPlace\\missions\\sweep_plan.py --json WorkPlace\\config\\uav_sweep.json
"""

from __future__ import print_function

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(HERE)
PROJECT_DIR = os.path.dirname(WORKPLACE_DIR)
EASY_FOLLOW_DIR = os.path.join(PROJECT_DIR, "Materials", "practice", "easy_follow")

sys.path.insert(0, EASY_FOLLOW_DIR)
sys.path.insert(0, WORKPLACE_DIR)

import numpy as np
if not hasattr(np, "mat"):
    np.mat = np.asmatrix

DEFAULT_SCENE = os.path.join(PROJECT_DIR, "Materials", "practice", "practice1_Scene")
DETECT_R = 300.0          # 与 control_uav.DETECT_RADIUS 保持一致


def log(msg):
    print("[sweep] " + msg)
    sys.stdout.flush()


# ---------------------------------------------------------------------------
#  区域
# ---------------------------------------------------------------------------
def arena_bounds(arena):
    xs = [p[0] for p in arena]
    ys = [p[1] for p in arena]
    return min(xs), min(ys), max(xs), max(ys)


def load_region(scene_dir, margin=30.0):
    """从 game.json 取任务区包围盒，四周留 margin。"""
    from core import scenefile
    game = scenefile.load_game(os.path.join(scene_dir, "game.json"))
    arena = scenefile.arena_polygon(game)
    if not arena:
        raise SystemExit("game.json 里没有 arena")
    x0, y0, x1, y1 = arena_bounds(arena)
    return (x0 + margin, y0 + margin, x1 - margin, y1 - margin)


# ---------------------------------------------------------------------------
#  牛耕式条带
# ---------------------------------------------------------------------------
def make_strips(region, spacing, along_x=None):
    """在矩形区域内生成牛耕式扫描线（每条是两端点）。

    along_x=True 表示扫描线沿 X 方向（适用于东西向长的区域），
    否则沿 Y 方向。默认取长边方向。
    """
    x0, y0, x1, y1 = region
    w, h = x1 - x0, y1 - y0
    if along_x is None:
        along_x = w >= h

    strips = []
    if along_x:
        # 扫描线沿 x，在 y 上每 spacing 一条
        n = max(1, int(math.ceil(h / spacing)))
        # 让条带均匀铺满：间距取 h/n，且 <= spacing（保证不漏检）
        step = h / float(n)
        for i in range(n):
            y = y0 + step * (i + 0.5)
            if i % 2 == 0:
                strips.append([(x0, y), (x1, y)])
            else:
                strips.append([(x1, y), (x0, y)])     # 反向，形成牛耕
    else:
        n = max(1, int(math.ceil(w / spacing)))
        step = w / float(n)
        for i in range(n):
            x = x0 + step * (i + 0.5)
            if i % 2 == 0:
                strips.append([(x, y0), (x, y1)])
            else:
                strips.append([(x, y1), (x, y0)])
    return strips, step


def partition_rects(region, n):
    """把矩形区域按长边等分成 n 块子矩形（不重叠、无缝隙）。"""
    x0, y0, x1, y1 = region
    w, h = x1 - x0, y1 - y0
    rects = []
    if w >= h:
        step = w / float(n)
        for i in range(n):
            rects.append((x0 + step * i, y0, x0 + step * (i + 1), y1))
    else:
        step = h / float(n)
        for i in range(n):
            rects.append((x0, y0 + step * i, x1, y0 + step * (i + 1)))
    return rects


def subdivide_strips(strips, n_parts):
    """把每条扫描线再均分成 n_parts 段。

    扫描区域较小时（例如 900×900 m，2R=600 只能排 2 条线），
    条带数会少于无人机数，导致有机器没活干。这里把每条线再切细，
    保证三架机都有任务、且长度仍然均衡。
    """
    if n_parts <= 1:
        return strips
    out = []
    for (a, b) in strips:
        for k in range(n_parts):
            t0, t1 = k / float(n_parts), (k + 1) / float(n_parts)
            p0 = (a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0)
            p1 = (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)
            out.append([p0, p1])
    return out



def split_strip_safe(strip, planner, step=15.0):
    """Split a strip into safe segments, keeping only safe endpoints."""
    (ax, ay), (bx, by) = strip
    if planner is None:
        return [(tuple(strip[0]), tuple(strip[1]))]

    length = math.hypot(bx - ax, by - ay)
    n = max(1, int(math.ceil(length / step)))
    runs = []
    cur = None
    last = None

    def save_run():
        if cur is not None and last is not None:
            if math.hypot(last[0] - cur[0], last[1] - cur[1]) > 1.0:
                runs.append((cur, last))

    for i in range(n + 1):
        t = i / float(n)
        point = (ax + (bx - ax) * t, ay + (by - ay) * t)

        if planner.point_safe(point, True):
            if cur is None:
                cur = point
            elif not planner.segment_safe(last, point, True):
                save_run()
                cur = point
            last = point
        else:
            save_run()
            cur = None
            last = None

    save_run()
    return runs


def segment_len(strip):
    (ax, ay), (bx, by) = strip
    return math.hypot(bx - ax, by - ay)


# ---------------------------------------------------------------------------
#  按航程均衡分配给 n 架机
# ---------------------------------------------------------------------------
def balance_by_len(lengths, n_uav):
    """把有序的若干段按"累计长度等分"切成 n 组，返回下标分组。"""
    total = sum(lengths)
    m = len(lengths)
    if m == 0 or total <= 0:
        return [[] for _ in range(n_uav)]
    cum = [0.0]
    for L in lengths:
        cum.append(cum[-1] + L)

    cuts = [0]
    for k in range(1, n_uav):
        target = total * k / float(n_uav)
        best, best_d = None, None
        for i in range(cuts[-1] + 1, m):      # 只往后找，保证单调
            d = abs(cum[i] - target)
            if best_d is None or d < best_d:
                best, best_d = i, d
        if best is not None:
            cuts.append(best)
    cuts.append(m)

    groups = [list(range(cuts[k], cuts[k + 1])) for k in range(len(cuts) - 1)]
    while len(groups) < n_uav:
        groups.append([])
    # 有条带却有空组时，让最长的一组匀一条
    for gi in range(len(groups)):
        if not groups[gi]:
            donor = max(range(len(groups)), key=lambda k: len(groups[k]))
            if len(groups[donor]) > 1:
                groups[gi] = [groups[donor].pop()]
    return groups


def strip_routes(strip_run_lists, indices, start, keep_order=False):
    """把若干条扫描线（每条含若干安全子段）串成航点序列。

    线内子段保持原顺序（它们本来就在同一条直线上），
    线间按"离上一个落点最近"挑下一条 —— 这样才是真正的牛耕式。
    """
    pts = [tuple(start[:2])]
    cur = pts[0]
    if keep_order:
        for idx in indices:
            runs = strip_run_lists[idx]
            if not runs:
                continue
            a0, b0 = runs[0][0], runs[-1][1]
            forward = math.hypot(a0[0] - cur[0], a0[1] - cur[1]) <= \
                math.hypot(b0[0] - cur[0], b0[1] - cur[1])
            seq = runs if forward else [(b, a) for (a, b) in reversed(runs)]
            for a, b in seq:
                if math.hypot(a[0] - cur[0], a[1] - cur[1]) > 1.0:
                    pts.append((a[0], a[1]))
                pts.append((b[0], b[1]))
                cur = (b[0], b[1])
        return pts

    left = list(indices)
    while left:
        # 选离当前点最近的一条线（端点近就算近）
        def dist_to(idx):
            runs = strip_run_lists[idx]
            if not runs:
                return float("inf")
            a, b = runs[0][0], runs[-1][1]
            return min(math.hypot(a[0] - cur[0], a[1] - cur[1]),
                       math.hypot(b[0] - cur[0], b[1] - cur[1]))
        idx = min(left, key=dist_to)
        left.remove(idx)
        runs = strip_run_lists[idx]
        if not runs:
            continue
        # 方向：从近端进（保持牛耕来回）
        a0, b0 = runs[0][0], runs[-1][1]
        forward = math.hypot(a0[0] - cur[0], a0[1] - cur[1]) <= \
            math.hypot(b0[0] - cur[0], b0[1] - cur[1])
        seq = runs if forward else [(b, a) for (a, b) in reversed(runs)]
        for a, b in seq:
            if math.hypot(a[0] - cur[0], a[1] - cur[1]) > 1.0:
                pts.append((a[0], a[1]))
            pts.append((b[0], b[1]))
            cur = (b[0], b[1])
    return pts


def path_length(route):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(route, route[1:])
    )


def split_longest(strip_run_lists, n_want):
    """把最长的一条扫描线拆成两条，直到条数 >= n_want。

    被禁飞区筛掉几条线后，剩下的线可能比无人机还少，
    这时无论怎么切分都会有机没活干；把最长的线拆开即可。
    """
    guard = 0
    while len(strip_run_lists) < n_want and guard < 8:
        guard += 1
        best_i, best_len = None, -1.0
        for i, runs in enumerate(strip_run_lists):
            L = sum(segment_len(r) for r in runs)
            if L > best_len:
                best_i, best_len = i, L
        if best_i is None or not strip_run_lists[best_i]:
            break
        runs = strip_run_lists[best_i]
        mid = len(runs) // 2
        if mid == 0:
            # 只有一段：对半切开
            a, b = runs[0]
            mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
            first = [(a, (mx, my))]
            second = [((mx, my), b)]
        else:
            first, second = runs[:mid], runs[mid:]
        strip_run_lists[best_i] = first
        strip_run_lists.insert(best_i + 1, second)
    return strip_run_lists


def refine_runs(strip_run_lists, target):
    """把子段再对半切，直到总段数 >= target。

    段数太少时（例如被禁飞区筛掉一半），三架机怎么分都会有一架明显吃亏；
    加大粒度后按航程切分就能做到更均衡。
    """
    total = sum(len(r) for r in strip_run_lists)
    guard = 0
    while total < target and guard < 6:
        guard += 1
        for li, runs in enumerate(strip_run_lists):
            new_runs = []
            for (a, b) in runs:
                mx = (a[0] + b[0]) / 2.0
                my = (a[1] + b[1]) / 2.0
                if math.hypot(b[0] - a[0], b[1] - a[1]) < 2.0:
                    new_runs.append((a, b))
                else:
                    new_runs.append((a, (mx, my)))
                    new_runs.append(((mx, my), b))
            strip_run_lists[li] = new_runs
        total = sum(len(r) for r in strip_run_lists)
    return strip_run_lists


def choose_partition(strip_run_lists, starts, n_uav):
    """把有序的扫描线切成 n 组，使"最长那条航程"最小。

    直接用**真实航程**（含转场）评估，而不是只看条带长度 ——
    转场在扫描区域离出生点较远时占很大比重，只按条带长度分会很不均衡。
    扫描线数量一般 ≤ 10，枚举所有连续切分（C(m-1, n-1)）代价可忽略。
    """
    names = sorted(starts.keys())
    m = len(strip_run_lists)
    if m == 0:
        return [[] for _ in names]

    def evaluate(cuts):
        idx_groups = []
        prev = 0
        for c in list(cuts) + [m]:
            idx_groups.append(list(range(prev, c)))
            prev = c
        lens = []
        for name, idxs in zip(names, idx_groups):
            r = strip_routes(strip_run_lists, idxs, starts[name])
            lens.append(path_length(r))
        return max(lens), sum(lens), idx_groups

    # 方案一：按"最长优先"（LPT）分配到当前最短的那架 —— 不要求连续，
    # 对"某条扫描线本身就很长"的情况比连续切分更均衡。
    def lpt():
        load = [0.0] * n_uav
        buckets = [[] for _ in range(n_uav)]
        order = sorted(range(m),
                       key=lambda i: -sum(segment_len(r) for r in strip_run_lists[i]))
        for i in order:
            k = min(range(n_uav), key=lambda j: load[j])
            buckets[k].append(i)
            load[k] += sum(segment_len(r) for r in strip_run_lists[i])
        lens = []
        for name, idxs in zip(names, buckets):
            r = strip_routes(strip_run_lists, sorted(idxs), starts[name])
            lens.append(path_length(r))
        return max(lens), sum(lens), [sorted(b) for b in buckets]

    best = None
    if n_uav == 1:
        best = evaluate([])
    else:
        from itertools import combinations
        for cuts in combinations(range(1, m), min(n_uav - 1, m - 1)):
            if len(cuts) != n_uav - 1:
                continue
            res = evaluate(cuts)
            if best is None or res[0] < best[0] - 1e-6:
                best = res
        if best is None:
            return lpt()[2]
        try:
            alt = lpt()
            if alt[0] < best[0] - 1e-6:
                best = alt
        except Exception:
            pass
    return best[2]


def build_rects(region, n_uav, radius, starts, planner):
    """把区域切成 n 块矩形，每架机独立扫自己那一块。"""
    spacing = 1.4 * radius   # 圆形探测足迹：间距 √2·R 才能无缝覆盖（原来是 2R，条带之间留缝）
    rects = partition_rects(region, n_uav)
    routes, blocks = {}, []
    dead = []
    for name, rect in zip(sorted(starts.keys()), rects):
        strips, step = make_strips(rect, spacing)
        strip_run_lists = [split_strip_safe(s, planner) for s in strips]
        runs = [r for rs in strip_run_lists for r in rs]
        total = sum(segment_len(r) for r in runs)
        # 【补扫】只给"被禁飞区切断"的那几条条带各补两条（左右各偏 0.35·间距）。
        # 不做整块二次扫描：那样 UAV2 航程会翻倍（实测 4.1 km -> 7.5 km）。
        try:
            extra = []
            for s, rs in zip(strips, strip_run_lists):
                if len(rs) <= 1:          # 没被切断的条带不补
                    continue
                (x0, y0), (x1, y1) = s
                for d in (spacing * 0.35, -spacing * 0.35):
                    if abs(y1 - y0) < 1e-9:      # 水平条带：上下偏移
                        s2 = ((x0, y0 + d), (x1, y1 + d))
                    else:                        # 竖直条带：左右偏移
                        s2 = ((x0 + d, y0), (x1 + d, y1))
                    extra.extend(split_strip_safe(s2, planner))
            if extra:
                strip_run_lists = strip_run_lists + [extra]
                runs = [r for rs in strip_run_lists for r in rs]
                total = sum(segment_len(r) for r in runs)
        except Exception:
            pass
        routes[name] = strip_routes(strip_run_lists,
                                    list(range(len(strip_run_lists))),
                                    starts[name], keep_order=True)
        blocks.append((rect, strips, step))
        if total < 50.0:
            dead.append(name)
    return routes, blocks, dead


def build(region, n_uav=3, radius=DETECT_R, starts=None, planner=None,
          partition="rects"):
    """一步到位：区域 → 三条航点序列。

    分配单位是**整条扫描线**（线内的安全子段保持顺序），
    这样既保证牛耕结构，又不会为了"就近"把同一条线上的子段打乱。
    """
    if starts is None:
        cx = (region[0] + region[2]) / 2.0
        cy = (region[1] + region[3]) / 2.0
        starts = dict(("UAV%d" % (i + 1), (cx, cy, 250.0)) for i in range(n_uav))

    if partition == "rects":
        routes, blocks, dead = build_rects(region, n_uav, radius, starts, planner)
        if dead:
            log("分区扫描有 %d 块被禁飞区覆盖（%s），改为全局条带均衡分配"
                % (len(dead), ", ".join(dead)))
            partition = "strips"
        else:
            strips = [s for (_r, ss, _st) in blocks for s in ss]
            step = blocks[0][2] if blocks else 0.0
            return routes, strips, [b[1] for b in blocks], step

    spacing = 1.4 * radius   # 圆形探测足迹：间距 √2·R 才能无缝覆盖（原来是 2R，条带之间留缝）
    strips, step = make_strips(region, spacing)
    # 扫描线条数少于无人机数时，把每条线再均分，保证每架机都有任务
    if len(strips) < n_uav:
        strips = subdivide_strips(strips, n_uav)

    # 每条扫描线按禁飞区切成安全子段（不绕行，只跳过禁飞区本身）
    strip_run_lists = [split_strip_safe(s, planner) for s in strips]
    totals = [sum(segment_len(r) for r in runs) for runs in strip_run_lists]

    if starts is None:
        cx = (region[0] + region[2]) / 2.0
        cy = (region[1] + region[3]) / 2.0
        starts = dict(("UAV%d" % (i + 1), (cx, cy, 250.0)) for i in range(n_uav))

    # 整条线都落在禁飞区里的，直接丢掉：否则它会白占一个分组，
    # 导致某一架机被分到"0 m 航程"（实测过）。
    kept = [r for r in strip_run_lists if r]
    if kept:
        strip_run_lists = kept
    strip_run_lists = split_longest(strip_run_lists, n_uav)
    strip_run_lists = refine_runs(strip_run_lists, 3 * n_uav)
    totals = [sum(segment_len(r) for r in runs) for runs in strip_run_lists]
    groups_idx = choose_partition(strip_run_lists, starts, n_uav)
    routes = {}
    for name, idxs in zip(sorted(starts.keys()), groups_idx):
        routes[name] = strip_routes(strip_run_lists, idxs, starts[name])

    # 供打印用的"分组长度"
    groups = []
    for idxs in groups_idx:
        groups.append([r for i in idxs for r in strip_run_lists[i]])
    return routes, strips, groups, step


def main():
    ap = argparse.ArgumentParser(description="牛耕式覆盖扫描航线生成")
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--region", default=None,
                    help="扫描区域 x0,y0,x1,y1（默认取 game.json 任务区）")
    ap.add_argument("--radius", type=float, default=DETECT_R,
                    help="探测半径 m（默认300），条带间距 = 2R")
    ap.add_argument("--n", type=int, default=3, help="无人机数量（默认3）")
    ap.add_argument("--json", default=None, help="把三条航点序列存成 json")
    ap.add_argument("--plot", action="store_true", help="画一张扫描示意图")
    ap.add_argument("--partition", choices=("rects", "strips"), default="rects",
                    help="rects=切成 3 块矩形各扫一块（默认）；strips=全局条带按航程均衡")
    ap.add_argument("--no-safe", action="store_true",
                    help="不做禁飞区绕行（纯几何牛耕）")
    # --region 的值可能以负号开头（x0<0），单独修正后再交给 argparse
    argv, i = [], 0
    raw = sys.argv[1:]
    while i < len(raw):
        a = raw[i]
        if a in ("--region",) and i + 1 < len(raw):
            argv.append("%s=%s" % (a, raw[i + 1]))
            i += 2
            continue
        argv.append(a)
        i += 1
    args = ap.parse_args(argv)

    if args.region:
        region = tuple(float(v) for v in args.region.replace("，", ",").split(",")[:4])
        assert len(region) == 4, "--region 需要 x0,y0,x1,y1"
    else:
        region = load_region(args.scene)
    x0, y0, x1, y1 = region
    log("扫描区域 x[%.0f, %.0f]  y[%.0f, %.0f]   %.0f × %.0f m"
        % (x0, x1, y0, y1, x1 - x0, y1 - y0))

    planner = None
    if not args.no_safe:
        try:
            import control_uav as cu
            planner = cu.SafePlanner(
                cu.read_json(os.path.join(args.scene, "game.json")))
            log("已加载安全包络（禁飞区绕行开启）")
        except Exception as e:
            log("加载 SafePlanner 失败，退化为纯几何扫描: %r" % (e,))

    routes, strips, groups, step = build(region, args.n, args.radius,
                                        planner=planner,
                                        partition=args.partition)

    log("探测半径 R = %.0f m → 条带间距 2R = %.0f m，实际 %.1f m"
        % (args.radius, 2 * args.radius, step))
    log("共 %d 条扫描线，分给 %d 架机：" % (len(strips), args.n))
    for i, g in enumerate(groups):
        lens = sum(segment_len(s) for s in g)
        log("  UAV%d: %d 条，条带总长 %.0f m" % (i + 1, len(g), lens))

    log("")
    for name in sorted(routes):
        r = routes[name]
        log("%s 航点 %d 个，总航程 %.0f m" % (name, len(r), path_length(r)))
        for k, (x, y) in enumerate(r, 1):
            tag = "起点" if k == 1 else ("扫描终点" if k == len(r) else "")
            print("      %2d. (%9.1f, %9.1f)  %s" % (k, x, y, tag))
        print("")

    if args.json:
        data = {
            "__说明__": "牛耕式扫描航点（按 2R 间距、三机按航程均衡）",
            "detect_radius": args.radius,
            "spacing": 2 * args.radius,
            "region": list(region),
            "routes": dict((k, [[round(x, 1), round(y, 1)] for x, y in v])
                           for k, v in routes.items()),
        }
        d = os.path.dirname(os.path.abspath(args.json))
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        log("已写入 %s" % args.json)

    if args.plot:
        try:
            _plot(region, strips, groups, routes, args.radius, args.json)
        except Exception as e:
            log("画图失败: %r" % (e,))
    return 0


def _plot(region, strips, groups, routes, radius, out_hint=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 5.2))
    x0, y0, x1, y1 = region
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fill=False, ec="#888888", ls="--"))
    colors = ["#1f77b4", "#2ca02c", "#d62728", "#9467bd"]
    for gi, g in enumerate(groups):
        c = colors[gi % len(colors)]
        for (a, b) in g:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=c, lw=1.4, alpha=0.55)
        r = routes[sorted(routes)[gi]]
        ax.plot([p[0] for p in r], [p[1] for p in r], color=c, lw=2.0,
                label="UAV%d  %d pts  %.0f m" % (gi + 1, len(r), path_length(r)))
        ax.plot(r[0][0], r[0][1], "o", color=c, ms=6)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_title("Boustrophedon coverage  (spacing = 2R = %.0f m)" % (2 * radius))
    ax.legend(loc="best", fontsize=9)
    ax.grid(alpha=0.25)
    out = os.path.join(WORKPLACE_DIR, "figs", "sweep_plan.png")
    fig.savefig(out, dpi=95, bbox_inches="tight")
    plt.close(fig)
    log("示意图已保存 %s" % out)


if __name__ == "__main__":
    sys.exit(main())
