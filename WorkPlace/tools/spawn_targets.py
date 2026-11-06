# -*- coding: utf-8 -*-
"""
spawn_targets.py —— 生成敌方目标并写入 targets.json
=====================================================

背景：SDK **没有**运行时生成 Actor 的接口。`subject4.exe` 启动时会读
同目录的 `targets.json` 来摆敌人，所以"生成敌人" = **改 json + 重启场景**
（重启用 tools/reset_scene.py）。

本脚本只负责"改 json"，不会去动正在跑的仿真。

用法
----
    # 1) 先看看现在场景里有什么（不改任何文件）
    python WorkPlace\\tools\\spawn_targets.py --list

    # 2) 随机生成 8 个敌人，撒在任务区内、贴着道路
    python WorkPlace\\tools\\spawn_targets.py --count 8 --seed 42

    # 3) 指定类型与数量
    python WorkPlace\\tools\\spawn_targets.py \
        --count 6 --types dynamic.prop.barricade,vehicle.ifv.stryker

    # 4) 保留原有目标，在它基础上追加 3 个
    python WorkPlace\\tools\\spawn_targets.py --count 3 --append

    # 5) 手动指定坐标（分号分隔）
    python WorkPlace\\tools\\spawn_targets.py --at "1500,-800; 1900,-600"

    # 6) 只想看会生成什么，不落盘
    python WorkPlace\\tools\\spawn_targets.py --count 5 --dry-run

生成规则
--------
* 只在**任务区 arena** 内落点（从 game.json 读）；
* 只在**道路像素**上落点（用 roadnet.png 当掩膜），避免生成到楼里；
* 互相之间至少 `--min-sep` 米，避免叠在一起；
* 首次写入前自动把原文件备份成 `targets.json.orig`（reset 脚本靠它还原）。

落盘后要让场景生效：
    python WorkPlace\\tools\\reset_scene.py --keep-enemies      # 只重启 scene，保留新敌人
"""

from __future__ import print_function

import argparse
import json
import os
import math
import random
import sys

WORKPLACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(WORKPLACE)
sys.path.insert(0, WORKPLACE)
sys.path.insert(0, os.path.join(PROJECT, "Materials", "practice", "easy_follow"))

from core import scenefile
from core.scenefile import Target

import numpy as np
from PIL import Image

DEFAULT_SCENE = os.path.join(PROJECT, "Materials", "practice", "practice1_Scene")
DEFAULT_ROADNET = os.path.join(PROJECT, "Materials", "practice", "easy_follow",
                               "lib", "roadnet.png")

#: 世界坐标 -> 像素 的默认偏移（click.py 实测值；换地图请改这里）
DEFAULT_OFFSET = (-2094.5277099609375, -2637.31982421875)

SPAWNABLE = [
    "dynamic.prop.barricade",
    "dynamic.prop.buildingsoldier",
    "vehicle.ifv.stryker",
    "dynamic.prop.soldier",
    "dynamic.prop.hardbarricade",
]


# ---------------------------------------------------------------------------
def point_in_polygon(x, y, poly):
    """射线法判断点是否在多边形内。poly = [(x,y), ...]"""
    inside = False
    n = len(poly)
    if n < 3:
        return True
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def load_road_mask(roadnet_path):
    """路网图 -> 布尔掩膜（True = 可站立的道路像素）。"""
    img = Image.open(roadnet_path).convert("L")
    arr = np.array(img)
    return arr > 30, arr.shape


def near_any_polygon(x, y, polys, margin):
    """点是否落在任一多边形内（把多边形按 margin 近似放大：测圆周上的点）。"""
    if not polys:
        return False
    if margin <= 0:
        return any(point_in_polygon(x, y, p) for p in polys)
    for k in range(8):
        ang = k * math.pi / 4.0
        if any(point_in_polygon(x + margin * math.cos(ang),
                                y + margin * math.sin(ang), p) for p in polys):
            return True
    return any(point_in_polygon(x, y, p) for p in polys)


def sample_road_points(mask, offset, arena, count, min_sep, rng, max_try=20000,
                      region=None, avoid=None, avoid_margin=30.0, rect=None):
    """在 arena 内的道路像素上采样 count 个点，返回世界坐标 [(x, y)]。"""
    h, w = mask.shape
    # 先把 arena 的包围盒换算成像素范围，缩小搜索空间
    xs = [p[0] for p in arena]
    ys = [p[1] for p in arena]
    px0 = max(0, int(min(xs) - offset[0]))
    px1 = min(w - 1, int(max(xs) - offset[0]))
    py0 = max(0, int(min(ys) - offset[1]))
    py1 = min(h - 1, int(max(ys) - offset[1]))

    cand = mask[py0:py1, px0:px1]
    ys_idx, xs_idx = np.nonzero(cand)
    if len(xs_idx) == 0:
        raise SystemExit("任务区范围内没有道路像素，检查 roadnet.png 和 world_offset")

    order = rng.sample(range(len(xs_idx)), min(len(xs_idx), max_try))
    picked = []
    for k in order:
        px = int(xs_idx[k]) + px0
        py = int(ys_idx[k]) + py0
        wx = px + offset[0]
        wy = py + offset[1]
        if not point_in_polygon(wx, wy, arena):
            continue
        if near_any_polygon(wx, wy, avoid, avoid_margin):
            # 落在禁飞区里的敌人：无人机规划不出航线，会导致"有一架不出动"
            continue
        if rect is not None:
            # (x0, y0, x1, y1)：矩形区域内均匀撒点（半图整体铺开，而不是挤成一小团）
            if not (rect[0] <= wx <= rect[2] and rect[1] <= wy <= rect[3]):
                continue
        if region is not None:
            # (cx, cy, rmin, rmax)：只取环带内的点
            cx, cy, rmin, rmax = region
            d2 = (wx - cx) ** 2 + (wy - cy) ** 2
            if d2 > rmax * rmax or d2 < rmin * rmin:
                continue
        ok = True
        for (ax, ay) in picked:
            if (ax - wx) ** 2 + (ay - wy) ** 2 < min_sep ** 2:
                ok = False
                break
        if ok:
            picked.append((wx, wy))
        if len(picked) >= count:
            break
    return picked


def list_scene(scene_dir):
    tp = os.path.join(scene_dir, "targets.json")
    if not os.path.isfile(tp):
        print("找不到 %s" % tp)
        return
    ts = scenefile.load_targets(tp)
    print("当前 %s：%d 个目标" % (os.path.basename(scene_dir), len(ts)))
    for k, cn, n in scenefile.summary(ts):
        print("   %-32s %-14s %d" % (k, cn, n))
    bak = tp + ".orig"
    print("\n备份文件 %s: %s" % (os.path.basename(bak),
                                "存在" if os.path.exists(bak) else "不存在（首次写入时创建）"))


def _fix_leading_dash(argv, value_opts):
    """把 "--opt -1.5,2" 改写成 "--opt=-1.5,2"：负坐标否则会被 argparse 当成选项。"""
    out = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in value_opts and i + 1 < len(argv):
            out.append("%s=%s" % (a, argv[i + 1]))
            i += 2
            continue
        out.append(a)
        i += 1
    return out


def main():
    ap = argparse.ArgumentParser(
        description="生成敌方目标并写入 targets.json",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default=DEFAULT_SCENE, help="场景目录（含 targets.json）")
    ap.add_argument("--roadnet", default=DEFAULT_ROADNET, help="路网图 png")
    ap.add_argument("--offset", default="%s,%s" % DEFAULT_OFFSET,
                    help="world_offset x,y（默认 click.py 实测值）")
    ap.add_argument("--count", type=int, default=8, help="生成几个（默认8）")
    ap.add_argument("--types", default="dynamic.prop.barricade",
                    help="类型，逗号分隔；可用: %s" % ",".join(SPAWNABLE))
    ap.add_argument("--seed", type=int, default=None, help="随机种子（便于复现）")
    ap.add_argument("--min-sep", type=float, default=60.0, help="目标之间最小间距 m")
    ap.add_argument("--rect", default=None,
                    help="矩形撒点区 x0,y0,x1,y1（半图整体铺开）")
    ap.add_argument("--near", default=None,
                    help="只在某点附近生成 x,y（配合 --radius，用于缩短侦查时间）")
    ap.add_argument("--radius", type=float, default=600.0,
                    help="--near 的外半径 m（默认600）")
    ap.add_argument("--off-road", type=float, default=14.0,
                    help="把敌人从道路中心横向挪开多少米（默认14）。"
                         "放在路正中间时，无人车会把「目标」本身当成挡路障碍，"
                         "反复倒车最后卡死；挪到路边就留出了车道。")
    ap.add_argument("--avoid-nofly", action="store_true", default=True,
                    help="避开 game.json 里的禁飞区（默认开）")
    ap.add_argument("--no-avoid-nofly", dest="avoid_nofly", action="store_false",
                    help="不避开禁飞区")
    ap.add_argument("--radius-min", type=float, default=0.0,
                    help="--near 的内半径 m（默认0）；设成200就是「只撒在200~radius之间」的环带")
    ap.add_argument("--append", action="store_true", help="保留原有目标，追加新的")
    ap.add_argument("--at", default=None, help='手动指定坐标 "x1,y1; x2,y2"')
    ap.add_argument("--dry-run", action="store_true", help="只打印，不写文件")
    ap.add_argument("--list", action="store_true", help="只列出现有目标然后退出")
    args = ap.parse_args(_fix_leading_dash(
        sys.argv[1:], ("--offset", "--at", "--scene", "--roadnet", "--types",
                       "--near", "--radius-min", "--off-road", "--rect")))

    if args.list:
        list_scene(args.scene)
        return

    tp = os.path.join(args.scene, "targets.json")
    gp = os.path.join(args.scene, "game.json")
    if not os.path.isfile(tp):
        raise SystemExit("找不到 %s" % tp)
    if not os.path.isfile(gp):
        raise SystemExit("找不到 %s（需要任务区范围）" % gp)

    game = scenefile.load_game(gp)
    arena = scenefile.arena_polygon(game)
    if not arena:
        print("game.json 里没有 arena，退化为不限制范围")

    rng = random.Random(args.seed)
    types = [t.strip() for t in args.types.split(",") if t.strip()]
    for t in types:
        if t not in SPAWNABLE:
            print("[警告] %s 不在已知类型表里，仍然允许写入" % t)

    existing = scenefile.load_targets(tp)
    mains = [t for t in existing if not t.assoc]

    # ---- 决定新目标位置 ----
    new = []
    if args.at:
        for chunk in args.at.replace("；", ";").split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            x, y = [float(v) for v in chunk.split(",")[:2]]
            new.append(Target(types[len(new) % len(types)], x, y, 2.69))
    else:
        offset = tuple(float(v) for v in args.offset.split(","))
        mask, _ = load_road_mask(args.roadnet)
        region = None
        rect = None
        if args.rect:
            _v = [float(v) for v in args.rect.replace('，', ',').split(',')[:4]]
            rect = (_v[0], _v[1], _v[2], _v[3])
            print('[矩形生成] x[%.0f, %.0f] y[%.0f, %.0f]'
                  % (rect[0], rect[2], rect[1], rect[3]))
        if args.near:
            cx, cy = [float(v) for v in args.near.replace('，', ',').split(',')[:2]]
            region = (cx, cy, max(0.0, args.radius_min), args.radius)
            if region[2] > 0:
                print('[就近生成] 中心 (%.1f, %.1f)  环带 %.0f~%.0f m'
                      % (cx, cy, region[2], args.radius))
            else:
                print('[就近生成] 中心 (%.1f, %.1f)  半径 %.0f m'
                      % (cx, cy, args.radius))
        avoid = []
        if args.avoid_nofly:
            try:
                env = scenefile.load_environment(gp)
                avoid = env.get("no_fly") or []
                if avoid:
                    print("[避让] 已载入 %d 个禁飞区，撒点时会避开它们" % len(avoid))
            except Exception as e:
                print("[避让] 读取禁飞区失败（不避让）: %r" % (e,))
        pts = sample_road_points(mask, offset, arena, args.count, args.min_sep, rng,
                                 region=region, avoid=avoid, rect=rect)
        if len(pts) < args.count:
            print("[警告] 只找到 %d 个可行点（要求 %d），可以调小 --min-sep"
                  % (len(pts), args.count))
        # 把敌人从路面中心挪到路边：留出车道，无人车才不会把目标当障碍
        if args.off_road > 0 and pts:
            moved = []
            for (x, y) in pts:
                best = None
                for ang_deg in (0, 90, 180, 270, 45, 135, 225, 315):
                    ang = math.radians(ang_deg)
                    nx = x + args.off_road * math.cos(ang)
                    ny = y + args.off_road * math.sin(ang)
                    if not point_in_polygon(nx, ny, arena):
                        continue
                    if near_any_polygon(nx, ny, avoid, 0.0):
                        continue
                    best = (nx, ny)
                    break
                moved.append(best or (x, y))
            pts = moved
            print("[路侧] 敌人已从路中心横向挪开 %.0f m（留出车道）" % args.off_road)

        for i, (x, y) in enumerate(pts):
            z = 2.69 if types[i % len(types)] != "vehicle.ifv.stryker" else 2.40
            new.append(Target(types[i % len(types)], x, y, z))

    final = (mains + new) if args.append else new

    print("\n将写入 %d 个目标（%s）:" % (len(final), "追加" if args.append else "覆盖"))
    for k, cn, n in scenefile.summary(final):
        print("   %-32s %-14s %d" % (k, cn, n))
    print("\n坐标预览（前 10 个）:")
    for t in new[:10]:
        px = int(t.x - DEFAULT_OFFSET[0])
        py = int(t.y - DEFAULT_OFFSET[1])
        print("   %-30s (%8.1f, %8.1f)  pixel(%4d,%4d)" % (t.type, t.x, t.y, px, py))

    if args.dry_run:
        print("\n--dry-run：不写文件")
        return

    n_groups = scenefile.save_targets(tp, final)
    print("\n[+] 已写入 %s（%d 个类型分组）" % (tp, n_groups))
    print("    原文件已备份为 %s" % (tp + ".orig"))
    print("\n让新敌人生效：")
    print("    python WorkPlace\\tools\\reset_scene.py --keep-enemies")


if __name__ == "__main__":
    main()
