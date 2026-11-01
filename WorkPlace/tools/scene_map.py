# -*- coding: utf-8 -*-
"""
scene_map.py —— 把 practice1_Scene 的场景要素画到路网图上，并输出坐标清单
=========================================================================

用途：
    1. 验证"世界坐标 -> 像素坐标"转换是否正确（肉眼比对路网图）；
    2. 给需求经理 / 组内开会时一张能指着讲的地图；
    3. 打印每个敌方目标的 (世界坐标, 像素坐标, 距起点距离)，
       后面做侦察遍历 / 打击排序 / 合围选点都以这张表为基础。

输入：
    Materials/practice/practice1_Scene/game.json     区域、阶段线
    Materials/practice/practice1_Scene/targets.json  敌方目标
    Materials/practice/practice1_Scene/vehicles.json 我方起车点
    Materials/practice/practice1_Scene/elements.json 雷区/电磁区/禁飞区图标
    Materials/practice/easy_follow/lib/roadnet.png   路网图(4794x4794)

输出：
    WorkPlace/figs/scene_planning_overlay.png
    WorkPlace/figs/scene_targets.csv

运行：
    python WorkPlace/tools/scene_map.py
"""

import os
import sys
import json
import csv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from PIL import Image

import coord

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SCENE_DIR = os.path.join(PROJECT_ROOT, "Materials", "practice", "practice1_Scene")
ROAD_PNG = os.path.join(PROJECT_ROOT, "Materials", "practice", "easy_follow", "lib", "roadnet.png")
OUT_DIR = os.path.join(PROJECT_ROOT, "WorkPlace", "figs")

#: vehicles.json / targets.json 里的坐标单位是"米"（见 coord.py 顶部说明）
#: 若实测发现是厘米，把下面这个系数改成 0.01 即可
SCENE_UNIT_TO_M = 1.0


def load_json(name):
    with open(os.path.join(SCENE_DIR, name), "r", encoding="utf-8") as f:
        return json.load(f)


def parse_pos(s):
    """'-1829.6, -517.7, 2.1' -> (-1829.6, -517.7, 2.1) 单位: 米"""
    parts = [float(v) for v in s.split(",")]
    return tuple(v * SCENE_UNIT_TO_M for v in parts)


def collect_targets(targets_raw):
    """把 targets.json 展开成 [(类别, 名称, x, y, z)]，含 barricade 旁挂的 soldier。"""
    out = []
    for group in targets_raw:
        names = group.get("types", [])
        main_name = names[0] if names else "unknown"
        for pt in group.get("points", []):
            x, y, z = parse_pos(pt["position"])
            out.append((main_name, x, y, z))
            # barricade 附近的士兵（flowers 是相对主目标的偏移）
            for fl in pt.get("flowers", []):
                fx, fy, fz = parse_pos(fl["position"])
                out.append((fl.get("type", "soldier"), x + fx, y + fy, z + fz))
    return out


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    game = load_json("game.json")
    targets_raw = load_json("targets.json")
    vehicles = load_json("vehicles.json")

    arena = game["SUBJECT_ARENA"]
    obstacles = collect_targets(targets_raw)

    # ---- 我方起点（取 UGV 第一辆作为基准点）----
    ugv_group = next(g for g in vehicles if g["type"] == "vehicle.imv.rb")
    start_x, start_y, _ = parse_pos(ugv_group["vehicles"][0]["position"])

    # ---- 示例终点：与 plan.py 中的 (-1500, -217) 一致 ----
    END = (-1500.0, -217.0)

    # ---- 打印坐标清单 ----
    print("=" * 96)
    print(f"起点 (UGV1) : world=({start_x:.1f}, {start_y:.1f})  "
          f"pixel={coord.world_to_pixel(start_x, start_y)}")
    print(f"示例终点    : world=({END[0]:.1f}, {END[1]:.1f})  "
          f"pixel={coord.world_to_pixel(*END)}")
    print("=" * 96)
    print(f"{'#':>2} {'类型':<28} {'世界X':>9} {'世界Y':>9} {'像素X':>6} {'像素Y':>6} {'距起点(m)':>10}")
    print("-" * 96)

    rows = []
    for i, (name, x, y, z) in enumerate(obstacles):
        px, py = coord.world_to_pixel(x, y)
        d = coord.dist(start_x, start_y, x, y)
        rows.append((i, name, x, y, px, py, d))
        print(f"{i:>2} {name:<28} {x:>9.1f} {y:>9.1f} {px:>6} {py:>6} {d:>10.1f}")

    dists = [r[6] for r in rows]
    print("-" * 96)
    print(f"目标数={len(rows)}  最近={min(dists):.1f} m  最远={max(dists):.1f} m")
    print(f"路网图幅   : {coord.MAP_SIZE_PX} px, 覆盖 world "
          f"x[{coord.MAP_WORLD_BOUNDS[0]:.0f},{coord.MAP_WORLD_BOUNDS[1]:.0f}] "
          f"y[{coord.MAP_WORLD_BOUNDS[2]:.0f},{coord.MAP_WORLD_BOUNDS[3]:.0f}]")
    print(f"图幅对角线 : {coord.dist(coord.MAP_WORLD_BOUNDS[0], coord.MAP_WORLD_BOUNDS[2], coord.MAP_WORLD_BOUNDS[1], coord.MAP_WORLD_BOUNDS[3]):.0f} m")

    # ---- CSV 输出 ----
    csv_path = os.path.join(OUT_DIR, "scene_targets.csv")
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["index", "type", "world_x_m", "world_y_m", "pixel_x", "pixel_y", "dist_from_start_m"])
        w.writerows(rows)
    print(f"\n[+] 目标清单已写出: {csv_path}")

    # ---- 画图 ----
    img = Image.open(ROAD_PNG)
    fig, ax = plt.subplots(figsize=(13, 13), dpi=110)
    ax.imshow(img, origin="upper")

    # 任务区域
    a = arena["arena"]
    ax.add_patch(Polygon([coord.world_to_pixel(p["x"], p["y"]) for p in a],
                         closed=True, fill=False, ec="#1f77b4", lw=2.2,
                         label="task arena"))

    # 禁飞区
    for i, poly in enumerate(arena.get("no_fly_arena", [])):
        ax.add_patch(Polygon([coord.world_to_pixel(p["x"], p["y"]) for p in poly],
                             closed=True, facecolor="#d62728", alpha=0.22,
                             ec="#d62728", lw=1.2,
                             label="no-fly zone" if i == 0 else None))

    # 雷区 / 电磁区
    for key, color, label in [("mine_arena", "#8c564b", "mine arena"),
                              ("emc_arena", "#9467bd", "emc arena")]:
        poly = arena.get(key, [])
        if poly:
            ax.add_patch(Polygon([coord.world_to_pixel(p["x"], p["y"]) for p in poly],
                                 closed=True, facecolor=color, alpha=0.18,
                                 ec=color, lw=1.2, label=label))

    # 敌方目标
    color_by_type = {
        "dynamic.prop.buildingsoldier": "#ff7f0e",
        "vehicle.ifv.stryker": "#000000",
        "dynamic.prop.barricade": "#e377c2",
        "dynamic.prop.soldier": "#bcbd22",
    }
    seen = set()
    for i, (name, x, y, z) in enumerate(obstacles):
        px, py = coord.world_to_pixel(x, y)
        c = color_by_type.get(name, "#333333")
        ax.plot(px, py, "o", ms=7, mfc=c, mec="white", mew=0.8,
                label=name if name not in seen else None, zorder=5)
        seen.add(name)
        ax.annotate(str(i), (px, py), xytext=(5, -5), textcoords="offset points",
                    fontsize=7, color="#111111", zorder=6)

    # 起点 / 终点
    sx, sy = coord.world_to_pixel(start_x, start_y)
    ex, ey = coord.world_to_pixel(*END)
    ax.plot(sx, sy, "*", ms=22, mfc="#00ff00", mec="black", mew=1.2, label="own start", zorder=7)
    ax.plot(ex, ey, "*", ms=20, mfc="#00ccff", mec="black", mew=1.2, label="goal (-1500,-217)", zorder=7)

    # 探测/通信半径示意：UAV detectRadius=30000, UGV=10000（见 vehicles.json）
    # units 若为厘米 -> 300 m / 100 m
    for r, c, lbl in [(300.0, "#2ca02c", "UAV r=300m"), (100.0, "#ff0000", "UGV r=100m")]:
        ax.add_patch(Circle((sx, sy), r, fill=False, ec=c, ls="--", lw=1.0, label=lbl))

    ax.set_title("practice1_Scene overlay  (1 px = 1 m, world_offset applied)", fontsize=13)
    ax.set_xlabel("pixel x")
    ax.set_ylabel("pixel y")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax.set_xlim(0, img.size[0])
    ax.set_ylim(img.size[1], 0)

    out_png = os.path.join(OUT_DIR, "scene_planning_overlay.png")
    fig.tight_layout()
    fig.savefig(out_png)
    print(f"[+] 场景叠加图已写出: {out_png}")

    # 局部放大：起点区域（看清 3 车 3 机与最近的敌方目标）
    ax.set_xlim(max(0, sx - 450), sx + 450)
    ax.set_ylim(min(img.size[1], sy + 450), max(0, sy - 450))
    out_zoom = os.path.join(OUT_DIR, "scene_start_zoom.png")
    fig.savefig(out_zoom)
    print(f"[+] 起点局部图已写出: {out_zoom}")


if __name__ == "__main__":
    main()
