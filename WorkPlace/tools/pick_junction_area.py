# -*- coding: utf-8 -*-
"""在 practice1 的路网里挑一块「路口密集区」，输出可直接用的 --zone 参数。

做法：对骨架图做 8 邻域统计，邻居数 >=3 的像素视为路口；
再用滑动窗口统计窗口内路口数，取最密的一块。

用法：
    python WorkPlace\\tools\\pick_junction_area.py                 # 默认 900x700，打印 Top5
    python WorkPlace\\tools\\pick_junction_area.py --size 700,600 --top 8

注意：骨架图是「像素坐标」，需要 scale/offset 才能转世界坐标。
      若提供了 --offset/--scale（或 config/roadnet_meta.json 存在），会直接给出世界坐标 zone；
      否则只打印像素矩形，并提示补参数。
"""

from __future__ import print_function

import argparse
import json
import os
import sys

import numpy as np

THIS = os.path.dirname(os.path.abspath(__file__))
WORKPLACE = os.path.dirname(THIS)
PROJECT = os.path.dirname(WORKPLACE)
ASSETS = os.path.join(WORKPLACE, "assets")
META = os.path.join(WORKPLACE, "config", "roadnet_meta.json")


def load_meta():
    """可选：{"scale": 1.0, "offset": [ox, oy]}（world = px*scale + offset）。"""
    if not os.path.isfile(META):
        return None
    try:
        with open(META, "r", encoding="utf-8-sig") as fp:
            d = json.load(fp)
        return (float(d["scale"]), (float(d["offset"][0]), float(d["offset"][1])))
    except Exception:
        return None


def junction_map(png_path):
    from PIL import Image
    from skimage.morphology import skeletonize
    from scipy.ndimage import convolve

    img = np.array(Image.open(png_path))
    skel = np.all(img >= 200, axis=-1) if img.ndim == 3 else (img >= 200)
    sk = skeletonize(skel)
    k = np.ones((3, 3), dtype=int)
    k[1, 1] = 0
    nb = convolve(sk.astype(int), k, mode="constant")
    return sk & (nb >= 3)


def densest_windows(junc, win_px, top=5, step=None):
    """在路口像素图上滑窗，返回 [(count, x0, y0), ...] 从多到少。"""
    ys, xs = np.nonzero(junc)
    if len(xs) == 0:
        return []
    step = step or max(10, win_px // 12)
    out = []
    for x0 in range(int(xs.min()), int(xs.max()) - win_px + 1, step):
        m = (xs >= x0) & (xs < x0 + win_px)
        if m.sum() < 3:
            continue
        for y0 in range(int(ys[m].min()), int(ys[m].max()) - win_px + 1, step):
            n = int(((xs >= x0) & (xs < x0 + win_px)
                     & (ys >= y0) & (ys < y0 + win_px)).sum())
            out.append((n, x0, y0))
    out.sort(reverse=True)
    # 去重：相近的窗口只留一个
    picked = []
    for n, x0, y0 in out:
        if all(abs(x0 - px) > win_px // 3 or abs(y0 - py) > win_px // 3
               for _n, px, py in picked):
            picked.append((n, x0, y0))
        if len(picked) >= top:
            break
    return picked


def main():
    ap = argparse.ArgumentParser(description="挑一块路口密集区作为战术层任务区")
    ap.add_argument("--png", default=os.path.join(ASSETS, "skeletonized_image.png"))
    ap.add_argument("--size", default="900,700", help="窗口尺寸（米/像素），默认 900,700")
    ap.add_argument("--top", type=int, default=5)
    ap.add_argument("--scale", type=float, default=None, help="像素→米 的缩放（可选）")
    ap.add_argument("--offset", default=None, help="像素→世界 的偏移 ox,oy（可选）")
    args = ap.parse_args()

    if not os.path.isfile(args.png):
        raise SystemExit("找不到骨架图：%s" % args.png)

    meta = load_meta()
    scale = args.scale if args.scale is not None else (meta[0] if meta else None)
    offset = None
    if args.offset:
        offset = tuple(float(v) for v in args.offset.replace("，", ",").split(",")[:2])
    elif meta:
        offset = meta[1]

    junc = junction_map(args.png)
    total = int(junc.sum())
    print("骨架图 %s   路口像素 %d" % (os.path.basename(args.png), total))
    if total == 0:
        raise SystemExit("没找到路口，检查骨架图是否 ≥200 阈值")

    w_px = int(float(args.size.split(",")[0]))
    h_px = int(float(args.size.split(",")[1]))
    win = max(w_px, h_px)
    rows = densest_windows(junc, win, top=args.top)
    print("")
    print("候选区域（窗口 %dx%d 像素）：" % (w_px, h_px))
    for i, (n, x0, y0) in enumerate(rows, 1):
        if scale and offset:
            zx0 = x0 * scale + offset[0]
            zy0 = y0 * scale + offset[1]
            zx1 = (x0 + w_px) * scale + offset[0]
            zy1 = (y0 + h_px) * scale + offset[1]
            print("  #%d 路口 %2d 个   --zone \"%.0f,%.0f,%.0f,%.0f\"  (px %d,%d)"
                  % (i, n, zx0, zy0, zx1, zy1, x0, y0))
        else:
            print("  #%d 路口 %2d 个   像素矩形 x[%d,%d] y[%d,%d]"
                  % (i, n, x0, x0 + w_px, y0, y0 + h_px))
    if not (scale and offset):
        print("")
        print("提示：未提供像素→世界坐标的 scale/offset，只能给像素矩形。")
        print("      补 --scale 1.0 --offset ox,oy（或写入 config/roadnet_meta.json）即可得到 --zone。")
        print("      ox/oy 可取：world_x = px*scale + ox，用任意一个已知点反解。")


if __name__ == "__main__":
    main()
