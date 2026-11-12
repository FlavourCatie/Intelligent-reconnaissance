# -*- coding: utf-8 -*-
"""无人车开局位置优化：在任务区路网上选 N 个「展开点」，而不是挤在同一个出生点。

为什么需要
----------
仿真里 UGV 出生点写死在场地西侧（x≈-1860），离半图任务区 x[-772,128] 有 1 km，
按 8 m/s 算开局要空跑 2 分钟；之前用 ``--ugv-start`` 手动平移，三台车又挤成一个点，
在区域边界互相顶死（交接文档问题 #7）。

本模块把「开局停哪儿」变成一个可解释的优化问题：

    候选点 = 任务区内、离边界 >= margin、在最大连通路网分量上的骨架点
    目标   = 极小化「任务区内任意路网点到最近展开点的距离」（k-center 覆盖）
    约束   = 两两间距 >= min_sep（不挤在一起）、尽量选直路（骨架 8 邻域 = 2）
    朝向   = 沿道路切线，并朝向任务区中心（起步不用掉头）

求解用「最远点优先」贪心（k-center 的 2 近似），再做两轮局部替换。
3 个点、几百个候选，离线一次几十毫秒到一秒，不会拖慢主循环。

用法
----
    # 离线看选点结果（不连仿真）
    python WorkPlace\\tactics\\deploy.py --zone "-772,-1006,128,-306"

    # 代码里用
    from tactics import deploy
    starts = deploy.choose_starts(binary, zone, n=3,
                                  raw=raw_bin, scale=scale, off=offset)
    # -> [{"vid": "UGV1", "pos": (x, y), "yaw": 90.0, "coverage_m": 343.7, ...}, ...]
"""

from __future__ import print_function

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(HERE)

DEFAULT_MARGIN = 150.0      # 展开点离任务区边界的最小距离 m
DEFAULT_MIN_SEP = 180.0     # 两两最小间距 m
DEFAULT_STEP = 10           # 候选点采样步长（骨架像素）
DEFAULT_MIN_WIDTH = 5.5     # 可接受的最小路宽 m（可用 --min-width 放宽）

_EIGHT = [(-1, -1), (-1, 0), (-1, 1),
          (0, -1), (0, 1),
          (1, -1), (1, 0), (1, 1)]


# ---------------------------------------------------------------------------
#  基础数据
# ---------------------------------------------------------------------------
def load_binary(assets_dir=None):
    """读 WorkPlace/assets 下的骨架图与真实路宽图。

    返回 (binary, raw, scale, offset)。与 control_ugv.load_binary 的区别是
    不依赖 SDK，可离线跑。
    """
    from PIL import Image
    assets_dir = assets_dir or os.path.join(WORKPLACE, "assets")
    skl = np.array(Image.open(os.path.join(assets_dir, "skeletonized_image.png")))
    binary = np.all(skl >= 200, axis=-1) if skl.ndim == 3 else (skl >= 200)
    raw_path = os.path.join(assets_dir, "roadnet_raw.png")
    raw = None
    if os.path.isfile(raw_path):
        raw = np.array(Image.open(raw_path).convert("L")) >= 128
    scale, off = 1.0, (0.0, 0.0)
    meta = os.path.join(WORKPLACE, "config", "roadnet_meta.json")
    if os.path.isfile(meta):
        try:
            with open(meta, "r", encoding="utf-8-sig") as fp:
                m = json.load(fp)
            scale = float(m.get("scale", scale))
            off = tuple(m.get("offset", off))
        except Exception:
            pass
    return binary, raw, scale, off


def w2p(x, y, scale, off):
    return (int(round((float(x) - off[0]) / scale)),
            int(round((float(y) - off[1]) / scale)))


def p2w(px, py, scale, off):
    return float(px) * scale + off[0], float(py) * scale + off[1]


def _bbox_scaled(binary, zone, margin, scale, off):
    px0 = int(math.ceil((zone.x0 + margin - off[0]) / scale))
    px1 = int(math.floor((zone.x1 - margin - off[0]) / scale))
    py0 = int(math.ceil((zone.y0 + margin - off[1]) / scale))
    py1 = int(math.floor((zone.y1 - margin - off[1]) / scale))
    h, w = binary.shape[:2]
    px0, px1 = max(0, px0), min(w, px1 + 1)
    py0, py1 = max(0, py0), min(h, py1 + 1)
    if px1 <= px0 or py1 <= py0:
        return None
    return px0, py0, px1, py1


# ---------------------------------------------------------------------------
#  候选点筛选
# ---------------------------------------------------------------------------
def _degrees(binary, xs, ys):
    """骨架点的 8 邻域连接数（不含自己）：2=直路，3=丁字，>=4=十字。"""
    h, w = binary.shape[:2]
    deg = np.zeros(len(xs), dtype=np.int16)
    for dx, dy in _EIGHT:
        xx = xs + dx
        yy = ys + dy
        ok = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
        vals = np.zeros(len(xs), dtype=np.int16)
        if np.any(ok):
            vals[ok] = binary[yy[ok], xx[ok]].astype(np.int16)
        deg += vals
    return deg


def _ray(raw, px, py, dx, dy, limit=80):
    """从 (px,py) 沿 (dx,dy) 数连续路面像素。"""
    h, w = raw.shape[:2]
    n = 0
    x, y = int(px), int(py)
    while n < limit:
        x += dx
        y += dy
        if x < 0 or y < 0 or x >= w or y >= h:
            break
        if not raw[y, x]:
            break
        n += 1
    return n


def _width_m(raw, px, py, scale):
    """8 方向最小对向路面宽度（m）。

    直路取垂向可通行宽度；斜路在 8 个方向里也能近似到垂向。
    """
    if raw is None:
        return None
    pairs = [((1, 0), (-1, 0)), ((0, 1), (0, -1)),
             ((1, 1), (-1, -1)), ((1, -1), (-1, 1))]
    best = None
    for (d1, d2) in pairs:
        w = _ray(raw, px, py, d1[0], d1[1]) + 1 + _ray(raw, px, py, d2[0], d2[1])
        if best is None or w < best:
            best = w
    return float(best) * float(scale)


def _largest_component(binary, box):
    """只保留包围盒内最大的连通分量（避免选到孤立路段）。"""
    px0, py0, px1, py1 = box
    sub = np.asarray(binary[py0:py1, px0:px1], dtype=bool)
    try:
        from scipy.ndimage import label
        lab, n = label(sub, structure=np.ones((3, 3), dtype=int))
        if n <= 1:
            return binary, sub
        counts = np.bincount(lab.ravel())
        counts[0] = 0
        keep = int(np.argmax(counts))
        mask = (lab == keep)
    except Exception:
        return binary, sub
    full = np.zeros_like(binary, dtype=bool)
    full[py0:py1, px0:px1] = mask
    return full, mask


def candidate_points(binary, zone, margin=DEFAULT_MARGIN, step=DEFAULT_STEP,
                     raw=None, scale=1.0, off=(0.0, 0.0),
                     min_width=DEFAULT_MIN_WIDTH, straight_only=True,
                     relax=True, max_candidates=1500):
    """返回候选展开点列表 [{"pos": (x,y), "px","py","deg","width"}...]。"""
    box = _bbox_scaled(binary, zone, margin, scale, off)
    if box is None:
        return []
    comp, _sub = _largest_component(binary, box)
    px0, py0, px1, py1 = box
    ys, xs = np.nonzero(comp[py0:py1, px0:px1])
    if len(xs) == 0:
        return []
    xs = xs + px0
    ys = ys + py0
    # 行主序等间隔取样：骨架线是 1 px 宽，按像素序号抽样比按格网抽样更稳
    xs = xs[::max(1, int(step))]
    ys = ys[::max(1, int(step))]
    deg = _degrees(binary, xs, ys)

    keep = np.ones(len(xs), dtype=bool)
    if straight_only:
        keep &= (deg == 2)
    out = []
    for i in range(len(xs)):
        if not keep[i]:
            continue
        px, py = int(xs[i]), int(ys[i])
        w = _width_m(raw, px, py, scale)
        if (min_width is not None and w is not None and w < min_width):
            continue
        wx, wy = p2w(px, py, scale, off)
        out.append({"pos": (wx, wy), "px": px, "py": py,
                    "deg": int(deg[i]), "width": w})
    if (not out) and relax:
        # 直路/宽路条件太苛刻就逐级放宽，最后退化为"任意路点"
        return candidate_points(binary, zone, margin=max(60.0, margin * 0.6),
                                step=step, raw=raw, scale=scale, off=off,
                                min_width=None, straight_only=False,
                                relax=False, max_candidates=max_candidates)
    if len(out) > max_candidates:
        # 大区域（例如整张图）候选太多，等间隔抽稀；最远点优先本来就不需要全量
        stride = int(math.ceil(len(out) / float(max_candidates)))
        out = out[::stride]
    return out


# ---------------------------------------------------------------------------
#  几何小工具
# ---------------------------------------------------------------------------
def _tangent(binary, px, py, radius=14):
    """骨架点局部主方向（单位向量）；点太少时返回 None。"""
    h, w = binary.shape[:2]
    x0, x1 = max(0, px - radius), min(w, px + radius + 1)
    y0, y1 = max(0, py - radius), min(h, py + radius + 1)
    ys, xs = np.nonzero(binary[y0:y1, x0:x1])
    if len(xs) < 6:
        return None
    xs = xs.astype(float) - (px - x0)
    ys = ys.astype(float) - (py - y0)
    xs -= xs.mean()
    ys -= ys.mean()
    cov = np.array([[np.mean(xs * xs), np.mean(xs * ys)],
                    [np.mean(xs * ys), np.mean(ys * ys)]])
    try:
        vals, vecs = np.linalg.eigh(cov)
    except Exception:
        return None
    vx, vy = float(vecs[0, -1]), float(vecs[1, -1])
    L = math.hypot(vx, vy)
    if L < 1e-9:
        return None
    return vx / L, vy / L


def heading_towards(binary, pos, towards, scale=1.0, off=(0.0, 0.0)):
    """展开点朝向：沿道路切线，符号取「朝向任务区中心」的一侧（度，0=+x）。"""
    px, py = w2p(pos[0], pos[1], scale, off)
    tan = _tangent(binary, px, py)
    tx, ty = (tan if tan is not None else (1.0, 0.0))
    vx, vy = float(towards[0]) - float(pos[0]), float(towards[1]) - float(pos[1])
    if tx * vx + ty * vy < 0.0:
        tx, ty = -tx, -ty
    if math.hypot(vx, vy) < 1e-6:
        tx, ty = 1.0, 0.0
    return math.degrees(math.atan2(ty, tx)) % 360.0


def road_width_at(raw, pos, heading, scale=1.0, off=(0.0, 0.0), limit_m=40.0):
    """测 (pos) 处、沿 heading 方向行驶时的可通行路宽（米）。

    沿 heading 的垂直方向向两侧数真实路面像素；遇到非路/边界即停。
    raw 为 None 时返回 None（调用方退回纵队）。
    """
    if raw is None:
        return None
    px, py = w2p(pos[0], pos[1], scale, off)
    h, w = raw.shape[:2]
    if px < 0 or py < 0 or px >= w or py >= h:
        return None
    dx, dy = math.cos(float(heading)), math.sin(float(heading))
    perp = (-dy, dx)
    limit = max(1, int(float(limit_m) / max(1e-6, float(scale))))

    def run(sx, sy):
        n = 0
        x, y = float(px), float(py)
        while n < limit:
            x += sx
            y += sy
            ix, iy = int(round(x)), int(round(y))
            if ix < 0 or iy < 0 or ix >= w or iy >= h or not raw[iy, ix]:
                break
            n += 1
        return n

    total = run(perp[0], perp[1]) + 1 + run(-perp[0], -perp[1])
    return total * float(scale)


def nearest_road_point(binary, x, y, scale=1.0, off=(0.0, 0.0), search=80):
    """离 (x,y) 最近的路网点（世界坐标）；找不到返回 None。"""
    from collections import deque
    px, py = w2p(x, y, scale, off)
    h, w = binary.shape[:2]
    if 0 <= px < w and 0 <= py < h and binary[py, px]:
        return p2w(px, py, scale, off)
    seen = set([(px, py)])
    q = deque([(px, py, 0)])
    while q:
        cx, cy, d = q.popleft()
        if d >= search:
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if (nx, ny) in seen or nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if binary[ny, nx]:
                return p2w(nx, ny, scale, off)
            seen.add((nx, ny))
            q.append((nx, ny, d + 1))
    return None


# ---------------------------------------------------------------------------
#  k-center 选点
# ---------------------------------------------------------------------------
def _coverage_stats(chosen_xy, sample_xy):
    """max / mean / p95：任务区路网样本点到最近展开点的距离。"""
    if len(chosen_xy) == 0 or len(sample_xy) == 0:
        return None
    c = np.asarray(chosen_xy, dtype=float)
    s = np.asarray(sample_xy, dtype=float)
    d = np.empty(len(s), dtype=float)
    # 分块，避免 3×4000 的中间矩阵其实很小，这里直接算也够快
    dists = np.hypot(s[:, None, 0] - c[None, :, 0],
                     s[:, None, 1] - c[None, :, 1])
    d = dists.min(axis=1)
    return {"max": float(d.max()), "mean": float(d.mean()),
            "p95": float(np.percentile(d, 95))}


def choose_starts(binary, zone, n=3, margin=DEFAULT_MARGIN,
                  min_sep=DEFAULT_MIN_SEP, step=DEFAULT_STEP,
                  raw=None, scale=1.0, off=(0.0, 0.0),
                  min_width=DEFAULT_MIN_WIDTH, refine=2):
    """选 n 个开局展开点。

    返回 [{"vid","pos","yaw","coverage_m","min_sep_m","width_m","deg"}...]；
    排序固定为 UGV1..n（按 x 再按 y，便于复现与日志对照）。
    """
    cands = candidate_points(binary, zone, margin=margin, step=step, raw=raw,
                             scale=scale, off=off, min_width=min_width)
    if len(cands) < n:
        # 任务区太小/路网太稀：放宽间距，仍不够就放宽边界；到下限还不行就放弃
        if margin <= 60.0 and min_sep <= 60.0:
            return []
        return choose_starts(binary, zone, n=n,
                             margin=max(60.0, margin * 0.6),
                             min_sep=max(60.0, min_sep * 0.5), step=step,
                             raw=raw, scale=scale, off=off,
                             min_width=None, refine=refine)
    pts = np.array([c["pos"] for c in cands], dtype=float)
    cx, cy = zone.center()

    # 任务区路网样本（用全部骨架像素，抽样 2 倍提速；大区域再截断）
    box = _bbox_scaled(binary, zone, margin, scale, off)
    comp, _ = _largest_component(binary, box)
    px0, py0, px1, py1 = box
    ys, xs = np.nonzero(comp[py0:py1, px0:px1])
    all_xy = np.stack([xs[::2] + px0, ys[::2] + py0], axis=1).astype(float)
    all_xy = np.stack([all_xy[:, 0] * scale + off[0],
                       all_xy[:, 1] * scale + off[1]], axis=1)
    if len(all_xy) > 8000:
        all_xy = all_xy[::int(math.ceil(len(all_xy) / 8000.0))]
    # 候选一多，局部替换的代价是 O(候选数×样本数)，按规模自动减轮次
    if len(pts) > 800:
        refine = min(int(refine), 1)
    if len(pts) > 1500:
        refine = 0

    # 1) 最远点优先（先选离区域中心最近的点，保证第一辆车在中心附近）
    d_center = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    chosen = [int(np.argmin(d_center))]
    while len(chosen) < n:
        dmin = np.min(np.stack([np.hypot(pts[:, 0] - pts[c, 0],
                                         pts[:, 1] - pts[c, 1])
                                for c in chosen], axis=1), axis=1)
        # 先满足 min_sep，再取最远；都不满足就只取最远
        ok = dmin >= min_sep
        j = int(np.argmax(np.where(ok, dmin, -1.0)))
        if not ok[j]:
            j = int(np.argmax(dmin))
        chosen.append(j)

    # 2) 局部替换：逐个位置试换候选，降低覆盖半径
    for _ in range(max(0, int(refine))):
        improved = False
        for slot in range(n):
            base = [c for i, c in enumerate(chosen) if i != slot]
            base_xy = [tuple(pts[c]) for c in base]
            base_stats = _coverage_stats(base_xy, all_xy)
            base_radius = base_stats["max"] if base_stats else 0.0
            best_j, best_key = chosen[slot], None
            for j in range(len(pts)):
                if j in base:
                    continue
                if any(math.hypot(pts[j, 0] - pts[c, 0], pts[j, 1] - pts[c, 1]) < min_sep
                       for c in base):
                    continue
                st = _coverage_stats(base_xy + [tuple(pts[j])], all_xy)
                if st is None:
                    continue
                key = (st["max"], st["mean"])
                if best_key is None or key < best_key:
                    best_j, best_key = j, key
            if best_key is not None and best_key[0] < base_radius - 1e-6:
                chosen[slot] = best_j
                improved = True
        if not improved:
            break

    # 3) 组装结果；按 x、y 排序，保证每次运行编号稳定
    chosen_xy = [tuple(pts[c]) for c in chosen]
    chosen_xy.sort(key=lambda p: (round(p[0], 1), round(p[1], 1)))
    stats = _coverage_stats(chosen_xy, all_xy) or {"max": 0.0, "mean": 0.0}
    min_sep_actual = min((math.hypot(a[0] - b[0], a[1] - b[1])
                          for i, a in enumerate(chosen_xy)
                          for b in chosen_xy[i + 1:]), default=0.0)
    out = []
    for i, p in enumerate(chosen_xy):
        # 找回该点对应的候选信息（取最近候选）
        k = int(np.argmin(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])))
        c = cands[k]
        out.append({
            "vid": "UGV%d" % (i + 1),
            "pos": (float(p[0]), float(p[1])),
            "yaw": heading_towards(binary, p, (cx, cy), scale=scale, off=off),
            "coverage_m": stats["max"],
            "mean_m": stats["mean"],
            "min_sep_m": min_sep_actual,
            "deg": c["deg"],
            "width_m": c["width"],
        })
    return out


# ---------------------------------------------------------------------------
#  终端展示 / 导出
# ---------------------------------------------------------------------------
def format_starts(starts):
    lines = []
    stats = starts[0] if starts else None
    if stats:
        lines.append("开局展开点：覆盖半径 %.0f m（区内最远路点到最近展开点），"
                     "平均 %.0f m，两两最小间距 %.0f m"
                     % (stats["coverage_m"], stats.get("mean_m", 0.0),
                        stats["min_sep_m"]))
    for s in starts:
        lines.append("  %s  位置 (%8.1f, %8.1f)  朝向 %5.1f°  路宽 %s m  骨架度数 %s"
                     % (s["vid"], s["pos"][0], s["pos"][1], s["yaw"],
                        ("%.1f" % s["width_m"]) if s.get("width_m") is not None else "-",
                        s.get("deg", "?")))
    return "\n".join(lines)


def save_starts(starts, path):
    data = {"schema": 1,
            "generated_by": "tactics/deploy.py",
            "starts": [{"vid": s["vid"],
                        "pos": [s["pos"][0], s["pos"][1]],
                        "yaw": s["yaw"],
                        "coverage_m": s["coverage_m"],
                        "min_sep_m": s["min_sep_m"],
                        "deg": s.get("deg"),
                        "width_m": s.get("width_m")} for s in starts]}
    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.isdir(d):
        os.makedirs(d)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=2)
    os.replace(tmp, path)
    return path


def load_starts(path):
    with open(path, "r", encoding="utf-8-sig") as fp:
        data = json.load(fp)
    out = []
    for s in data.get("starts", []):
        pos = s.get("pos") or [0.0, 0.0]
        out.append({"vid": s.get("vid", "UGV?"),
                    "pos": (float(pos[0]), float(pos[1])),
                    "yaw": float(s.get("yaw", 0.0)),
                    "coverage_m": float(s.get("coverage_m", 0.0)),
                    "min_sep_m": float(s.get("min_sep_m", 0.0)),
                    "deg": s.get("deg"),
                    "width_m": s.get("width_m")})
    return out


def main():
    ap = argparse.ArgumentParser(description="无人车开局位置优化（k-center 展开点）")
    ap.add_argument("--zone", required=True, help="任务区 x0,y0,x1,y1（必填，不给默认值）")
    ap.add_argument("--n", type=int, default=3, help="展开点数量（默认3）")
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN,
                    help="离任务区边界最小距离 m（默认150）")
    ap.add_argument("--min-sep", type=float, default=DEFAULT_MIN_SEP,
                    help="两两最小间距 m（默认180）")
    ap.add_argument("--step", type=int, default=DEFAULT_STEP,
                    help="候选采样步长 px（默认10）")
    ap.add_argument("--min-width", type=float, default=DEFAULT_MIN_WIDTH,
                    help="最小路宽 m（默认5.5；0 表示不限制）")
    ap.add_argument("--assets", default=None, help="资源目录（默认 WorkPlace/assets）")
    ap.add_argument("--json-out", default=None, help="把结果写成 JSON（可复现）")
    argv = sys.argv[1:]
    # --zone "-772,..." 会被 argparse 当成选项，这里先粘成 --zone=-772,...
    for opt in ("--zone", "--assets", "--json-out"):
        if opt in argv:
            i = argv.index(opt)
            if i + 1 < len(argv) and argv[i + 1].startswith("-"):
                argv[i:i + 2] = ["%s=%s" % (opt, argv[i + 1])]
    args = ap.parse_args(argv)

    for p in (HERE, WORKPLACE):
        if p not in sys.path:
            sys.path.insert(0, p)
    from tactics.zones import Zone
    zone = Zone.parse(args.zone, name="deploy_zone")
    binary, raw, scale, off = load_binary(args.assets)
    starts = choose_starts(binary, zone, n=args.n, margin=args.margin,
                           min_sep=args.min_sep, step=args.step, raw=raw,
                           scale=scale, off=off,
                           min_width=(args.min_width or None))
    print(format_starts(starts))
    if args.json_out:
        print("已写出：%s" % save_starts(starts, args.json_out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
