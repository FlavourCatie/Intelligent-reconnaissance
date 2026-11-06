# -*- coding: utf-8 -*-
"""
live_track.py —— 战场态势显示（**纯显示层，不控制和规划任何东西**）
================================================================================

定位
----
本文件只做"看"：

    ✅ 读 6 台平台的实时位姿，画出位置 / 朝向 / 速度 / 实际走过的轨迹
    ✅ 画环境：任务区、禁飞区、雷区、电磁干扰区、（可选）阶段线
    ✅ 画敌人：初始为**空心圆**（未探测），被探测到后变**实心圆**
    ✅ 画目标位置：每台平台一个星标 + 到目标点的虚线
    ✅ 抓无人机俯视视角图片

    ❌ 不跑 A*、不规划路径、不发任何控制指令
    ❌ 不决定谁去哪

**移动由其他组员的脚本负责**。本脚本每帧只是"读一次状态、画一张图"，
所以可以和别人的控制程序同时跑（互不干扰地读状态即可）。

三条关键机制
------------
1. **敌人 空心 → 实心**
   启动时从 `targets.json` 读全部敌人（`spawn_targets.py` 生成的那份），
   画成空心红圈；运行中周期性调用 `detect_situation()`，把探测到的目标
   与已知列表按距离匹配（默认 20 m 内算同一个），匹配上就把它改成实心。
   探测不到但确实存在的目标，单独画成橙色叉号。

2. **目标位置的外部接口**
   其他组员把每台平台的目标写进 `WorkPlace/config/goals.json`，
   保存即生效（本脚本轮询文件修改时间，热重载）：

       {
           "UGV1": [1200, -400],
           "UAV1": {"x": 1400, "y": -500}
       }

   没写到的平台，**默认以离它最近的敌人位置作为目标**，星标画成空心
   （实心星 = 人工指定），并在 HUD 里标明来源。

3. **视角切换**
   `1`-`6` 聚焦某台、`0` 全城总览、`f` 循环、`+`/`-` 缩放、
   `s` 存图、`h` 隐藏帮助、`r` 重新按"最近敌人"推断目标。

用法
----
    # 0) 先自检（只读）
    python WorkPlace\\missions\\live_track.py --probe

    # 1) 看着就行（前提：CarlaUE4 + start.bat + subject4.exe 都已启动）
    python WorkPlace\\missions\\live_track.py

    # 2) 换目标文件 / 关掉探测 / 只看敌人
    python WorkPlace\\missions\\live_track.py --goals-json config\\goals.json
    python WorkPlace\\missions\\live_track.py --no-detect
    python WorkPlace\\missions\\live_track.py --detect-from uav

    # 3) 只看环境，不抓俯视图
    python WorkPlace\\missions\\live_track.py --save-every 0 --show-phase-lines

前置条件（顺序不能错）：
    CarlaUE4 → Materials\\simu_driver_windows\\start.bat（选 2 旋翼）
             → Materials\\practice\\practice1_Scene\\subject4.exe → 本脚本
    Python 必须是 3.6（SDK 的 egg 是 py3.6 专用）。
"""

from __future__ import print_function

import argparse
import json
import math
import os
import sys
import time
import traceback

# ---------------------------------------------------------------------------
# 路径：本文件在 WorkPlace/missions/ 下
# ---------------------------------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
PROJECT_DIR = os.path.dirname(WORKPLACE_DIR)
EASY_FOLLOW_DIR = os.path.join(PROJECT_DIR, "Materials", "practice", "easy_follow")
TOOLS_DIR = os.path.join(WORKPLACE_DIR, "tools")
SCENE_DIR = os.path.join(PROJECT_DIR, "Materials", "practice", "practice1_Scene")

for _p in (EASY_FOLLOW_DIR, TOOLS_DIR, WORKPLACE_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.lines import Line2D
from PIL import Image

# NumPy 2.x 兼容：lib/*.py 在类型注解里用了 np.mat（2.0 已移除）
if not hasattr(np, "mat"):
    np.mat = np.asmatrix
    log_note = "已为 NumPy 2.x 补上 np.mat -> np.asmatrix 兼容层"
else:
    log_note = None

from swarmae.SwarmAEClient import SwarmAEClient

import lib.swarm_api  # noqa: F401  （保证 easy_follow 的 lib 包可被 core.fleet 导入）
import coord

from core.fleet import Fleet
from core import scenefile


# ===========================================================================
#  小工具
# ===========================================================================
def log(msg):
    print("[live_track] " + msg)


def ensure_dir(p):
    if p and not os.path.isdir(p):
        os.makedirs(p)
    return p


# ---------------------------------------------------------------------------
# 中文字体：matplotlib 默认的 DejaVu Sans 没有汉字，图上会显示成方框（tofu）。
# ---------------------------------------------------------------------------
def _setup_cjk_font():
    try:
        from matplotlib import font_manager
    except Exception:
        return None
    candidates = ["Microsoft YaHei", "SimHei", "SimSun", "KaiTi",
                  "Noto Sans CJK SC", "Source Han Sans SC",
                  "PingFang SC", "WenQuanYi Micro Hei", "Arial Unicode MS"]
    try:
        available = {f.name for f in font_manager.fontManager.ttflist}
    except Exception:
        return None
    for c in candidates:
        if c in available:
            plt.rcParams["font.sans-serif"] = [c] + list(
                plt.rcParams.get("font.sans-serif", []))
            # HUD 用的是 monospace 家族，也得指过去，否则中文照样是方框
            plt.rcParams["font.monospace"] = [c] + list(
                plt.rcParams.get("font.monospace", []))
            plt.rcParams["axes.unicode_minus"] = False
            return c
    return None


CJK_FONT = _setup_cjk_font()


def T(zh, en):
    """有中文字体就用中文，没有就用英文，避免图上出现方框。"""
    return zh if CJK_FONT else en


# ---------------------------------------------------------------------------
# 路网图解析（SDK 的 get_road_network 会先请求竞赛服务器，失败才用本地内置图，
# 返回值可能不是 PIL 图，这里按类型识别，不让它炸）
# ---------------------------------------------------------------------------
def _looks_like_image(v):
    if hasattr(v, "size") and hasattr(v, "mode"):
        return True
    if isinstance(v, np.ndarray) and v.ndim >= 2:
        return True
    return False


def parse_road_network(ret):
    """从 get_road_network() 返回值里按类型认出 img / scale / world_offset。"""
    items = list(ret) if isinstance(ret, (tuple, list)) else [ret]
    img = scale = offset = None
    used = set()

    for i, v in enumerate(items):
        if _looks_like_image(v):
            img = v
            used.add(i)
            break

    for i, v in enumerate(items):
        if i in used:
            continue
        if (isinstance(v, (tuple, list)) and len(v) >= 2
                and all(isinstance(x, (int, float)) for x in v[:2])):
            offset = (float(v[0]), float(v[1]))
            used.add(i)
            break

    nums = [(i, v) for i, v in enumerate(items)
            if i not in used and isinstance(v, (int, float))]
    for i, v in nums:
        if abs(float(v) - 1.0) < 1e-9:
            scale = 1.0
            used.add(i)
            break
    if scale is None:
        for i, v in nums:
            if i in used:
                continue
            if 0 < abs(float(v)) < 100:
                scale = float(v)
                used.add(i)
                break
    return img, scale, offset, items


def describe_return(items):
    out = []
    for i, v in enumerate(items):
        extra = ""
        if hasattr(v, "size") and hasattr(v, "mode"):
            extra = " size=%s mode=%s" % (v.size, v.mode)
        elif isinstance(v, np.ndarray):
            extra = " shape=%s" % (v.shape,)
        out.append("  [%d] %-18s %r%s" % (i, type(v).__name__, v, extra))
    return "\n".join(out)


# ===========================================================================
#  场景（路网图 + 坐标换算）
# ===========================================================================
class Scene(object):
    def __init__(self, img, scale, world_offset):
        if img is None:
            raise ValueError("Scene 收到 img=None：路网图没取到（见诊断输出）")
        self.img = img
        self.arr = np.array(img)
        if self.arr.ndim == 2:
            self.arr = np.stack([self.arr] * 3, axis=-1)
        elif self.arr.ndim == 3 and self.arr.shape[2] == 4:
            self.arr = self.arr[:, :, :3]
        elif self.arr.ndim != 3:
            raise ValueError("路网图维度不对: type=%r shape=%r"
                             % (type(img).__name__, self.arr.shape))
        if self.arr.dtype != np.uint8:
            self.arr = np.clip(self.arr, 0, 255).astype(np.uint8)
        self.h, self.w = self.arr.shape[:2]
        #: 若有人还要用整图（例如 UavView 的合成俯视相机），就保留着别释放
        self.keep_full = False
        self.scale = float(scale)
        self.offset = (float(world_offset[0]), float(world_offset[1]))
        log("路网图 %dx%d  scale=%s  world_offset=(%.2f, %.2f)"
            % (self.w, self.h, self.scale, self.offset[0], self.offset[1]))

    def release_full_image(self):
        '''释放 4794×4794 的整图（约 69 MB）。

        LiveView 只用到降采样后的 disp；整图留着纯属浪费内存。
        释放后 to_px / to_world 等坐标换算完全不受影响。
        '''
        if getattr(self, "keep_full", False):
            return False
        if getattr(self, "arr", None) is not None:
            self.arr = None
            return True
        return False

    def to_px(self, x, y):
        return coord.world_to_pixel(x, y, self.scale, self.offset)

    def to_world(self, px, py):
        return coord.pixel_to_world(px, py, self.scale, self.offset)


# ===========================================================================
#  标记（敌人 / 障碍物）
# ===========================================================================
class Marker(object):
    """一个敌方目标或障碍物。"""

    __slots__ = ("index", "type", "x", "y", "z", "discovered", "extra", "kind")

    def __init__(self, type_name, x, y, z=0.0, index=-1, extra=False, kind="strike"):
        self.index = index
        self.type = type_name
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.discovered = False
        self.extra = bool(extra)
        self.kind = kind

    @property
    def cn(self):
        return scenefile.type_cn(self.type)

    def __repr__(self):
        return "<Marker %s (%.1f, %.1f)%s%s>" % (
            self.type, self.x, self.y,
            " 已发现" if self.discovered else "",
            " 额外" if self.extra else "")


def match_detections(detected, markers, tol=20.0):
    """把一次探测结果对齐到已知标记列表。

    返回 (found_index_set, extra_list)：
        found  —— 匹配上的已知标记下标（这些要变实心）
        extra  —— 已知列表里没有的探测结果（画成额外标记）
    """
    found = set()
    extras = []
    for d in detected:
        if d.get("kind") == "teammate":
            continue
        dx, dy = float(d["x"]), float(d["y"])
        best_i, best_d = -1, tol
        for i, m in enumerate(markers):
            dd = math.hypot(m.x - dx, m.y - dy)
            if dd < best_d:
                best_i, best_d = i, dd
        if best_i >= 0:
            found.add(best_i)
        else:
            dup = False
            for e in extras:
                if math.hypot(e["x"] - dx, e["y"] - dy) < tol:
                    dup = True
                    break
            if not dup:
                extras.append(d)
    return found, extras


# ===========================================================================
#  环境要素（任务区 / 禁飞区 / 雷区 / 电磁区 / 阶段线）
# ===========================================================================
class Environment(object):
    COLORS = {
        "arena": "#1f77b4",
        "no_fly": "#d62728",
        "mine": "#8c564b",
        "emc": "#9467bd",
    }

    def __init__(self, env_dict=None):
        env_dict = env_dict or {}
        self.arena = env_dict.get("arena") or []
        self.no_fly = env_dict.get("no_fly") or []
        self.mine = env_dict.get("mine") or []
        self.emc = env_dict.get("emc") or []
        self.lines = env_dict.get("lines") or {}

    def summary(self):
        return ("场景边界 %d 点 | 禁飞区 %d 个 | 雷区 %s | 电磁区 %s | 阶段线 %d 组"
                % (len(self.arena), len(self.no_fly),
                   "有" if self.mine else "无",
                   "有" if self.emc else "无",
                   len(self.lines)))

    def draw(self, scene, ax, with_legend=False, show_lines=False):
        C = Environment.COLORS

        def _poly(pts, **kw):
            px = [scene.to_px(x, y) for x, y in pts]
            ax.add_patch(Polygon(px, closed=True, **kw))
            return px

        if self.arena:
            # 场景自带的 arena 只是"地图边界"，不是我们这一轮的搜索范围；
            # 画成灰色虚线，把醒目的实线留给 LiveView.set_search_area()。
            _poly(self.arena, fill=False, ec="#7a7a7a", lw=1.2, ls=(0, (4, 3)),
                  zorder=3, label=T("场景边界", "arena") if with_legend else None)

        for i, pts in enumerate(self.no_fly):
            _poly(pts, facecolor=C["no_fly"], alpha=0.18, ec=C["no_fly"],
                  lw=1.2, zorder=4, hatch="//",
                  label=(T("禁飞区", "no-fly") if (with_legend and i == 0) else None))

        if self.mine:
            _poly(self.mine, facecolor=C["mine"], alpha=0.15, ec=C["mine"], lw=1.2,
                  zorder=4, label=T("雷区", "mine") if with_legend else None)

        if self.emc:
            _poly(self.emc, facecolor=C["emc"], alpha=0.15, ec=C["emc"], lw=1.2,
                  zorder=4, label=T("电磁区", "EMC") if with_legend else None)

        if show_lines:
            for key, blk in self.lines.items():
                for side, (p1, p2) in blk.items():
                    a = scene.to_px(*p1)
                    b = scene.to_px(*p2)
                    ax.plot([a[0], b[0]], [a[1], b[1]], "--", color="#bbbbbb",
                            lw=0.9, alpha=0.55, zorder=3)


# ===========================================================================
#  无人机俯视视角（真机相机优先，失败则用路网图合成）
# ===========================================================================
class UavView(object):
    HEADER_H = 50

    def __init__(self, scene, mode="auto", view_range=200.0, fov_deg=60.0):
        self.scene = scene
        scene.keep_full = True      # 合成相机要裁整图，别让 LiveView 释放掉
        self.mode = mode
        self.view_range = float(view_range)
        self.fov_deg = float(fov_deg)
        self.real_ok = None
        self.real_err = None
        self.saved = 0

    # ---------------- 真机相机 ----------------
    @staticmethod
    def _iter_sensors(ret):
        cands = []
        if ret is None:
            return cands
        if isinstance(ret, dict):
            cands = list(ret.values())
        elif isinstance(ret, (list, tuple)):
            stack = list(ret)
            while stack:
                it = stack.pop(0)
                if it is None:
                    continue
                if isinstance(it, (list, tuple, dict)):
                    stack.extend(list(it.values()) if isinstance(it, dict) else list(it))
                else:
                    cands.append(it)
        else:
            cands = [ret]
        return [c for c in cands if hasattr(c, "get_image_data")]

    @staticmethod
    def _to_pil(data):
        if data is None:
            return None
        if isinstance(data, Image.Image):
            return data
        if isinstance(data, np.ndarray):
            return Image.fromarray(data.astype(np.uint8))
        if isinstance(data, (tuple, list)):
            for it in data:
                img = UavView._to_pil(it)
                if img is not None:
                    return img
            return None
        if isinstance(data, (bytes, bytearray)):
            try:
                from io import BytesIO
                return Image.open(BytesIO(bytes(data)))
            except Exception:
                return None
        return None

    def grab_real(self, vehicle):
        if self.mode == "synth":
            return None
        if self.real_ok is False:
            return None
        node = vehicle.Vehicle_
        try:
            ret = node.get_sensor()
        except Exception as e:
            self.real_ok = False
            self.real_err = "get_sensor() 不可用: %r" % (e,)
            return None
        sensors = self._iter_sensors(ret)
        if not sensors:
            self.real_ok = False
            self.real_err = "get_sensor() 没返回带 get_image_data 的对象"
            return None
        for s in sensors:
            try:
                img = self._to_pil(s.get_image_data())
            except Exception as e:
                self.real_err = "get_image_data() 失败: %r" % (e,)
                continue
            if img is not None:
                self.real_ok = True
                return img.convert("RGB")
        self.real_ok = False
        return None

    # ---------------- 合成俯视图 ----------------
    def synth(self, name, pose, markers=None, stamp=None):
        scene = self.scene
        cx, cy = scene.to_px(pose.x, pose.y)
        half = int(self.view_range * scene.scale)
        W = half * 2

        left, top = cx - half, cy - half
        box = (max(0, left), max(0, top),
               min(scene.w, cx + half), min(scene.h, cy + half))
        if getattr(scene, "arr", None) is None:
            return None
        crop = Image.fromarray(scene.arr).crop(box).convert("RGB")
        view = Image.new("RGB", (W, W), (0, 0, 0))
        view.paste(crop, (max(0, -left), max(0, -top)))

        # 机头朝上：PIL 的 rotate(a) 把像素角 θ 变成 θ-a，
        # 要让 yaw 朝上（像素角 -90°）需 a = yaw+90；下面的标记用
        # (sin(ang-yaw), -cos(ang-yaw)) 恰好等于 R(-a)，两者一致。
        yaw_deg = math.degrees(pose.yaw)
        view = view.rotate(yaw_deg + 90.0, resample=Image.BICUBIC)

        arr = np.asarray(view).astype(np.int16)
        view = Image.fromarray(np.clip(arr + 14, 0, 255).astype(np.uint8))

        from PIL import ImageDraw
        H = self.HEADER_H
        canvas = Image.new("RGB", (W, W + H), (18, 18, 24))
        canvas.paste(view, (0, H))
        d = ImageDraw.Draw(canvas, "RGBA")

        mid = half
        y0 = H + half
        fov_r = math.radians(self.fov_deg / 2.0)
        r = W * 0.46
        for sign in (-1, 1):
            d.line([(mid, y0),
                    (mid + sign * r * math.sin(fov_r), y0 - r * math.cos(fov_r))],
                   fill=(0, 255, 120, 200), width=3)

        if markers:
            for m in markers:
                dx = (m.x - pose.x) * scene.scale
                dy = (m.y - pose.y) * scene.scale
                rr = math.hypot(dx, dy)
                if rr > self.view_range:
                    continue
                ang = math.atan2(dy, dx) - pose.yaw
                px = mid + rr * math.sin(ang)
                py = y0 - rr * math.cos(ang)
                if not (0 <= px < W and H <= py < H + W):
                    continue
                in_fov = abs(((ang + math.pi) % (2 * math.pi)) - math.pi) <= fov_r
                if in_fov and m.discovered:
                    fill = (255, 60, 60, 255)
                elif in_fov:
                    fill = (255, 200, 0, 255)
                else:
                    fill = (120, 120, 120, 180)
                d.ellipse([px - 5, py - 5, px + 5, py + 5],
                          fill=fill, outline=(0, 0, 0, 255))

        d.line([(mid - 18, y0), (mid + 18, y0)], fill=(255, 255, 255, 220), width=2)
        d.line([(mid, y0 - 18), (mid, y0 + 18)], fill=(255, 255, 255, 220), width=2)
        d.ellipse([mid - 6, y0 - 6, mid + 6, y0 + 6], outline=(0, 255, 255, 255), width=2)

        d.text((10, 7), "%s   z=%.0fm   yaw=%.0fdeg   v=%.1fm/s"
                        % (name, pose.z, yaw_deg, pose.v),
               fill=(255, 255, 255, 255))
        d.text((10, 28), "synth top-down view   range %.0fm   fov %.0fdeg%s"
                         % (self.view_range, self.fov_deg,
                            "" if stamp is None else "   t=%.1fs" % stamp),
               fill=(160, 220, 255, 255))
        return canvas

    def grab(self, name, pose, markers=None, stamp=None, vehicle=None):
        """优先真机相机，失败则合成。返回 (PIL.Image, "real"/"synth")。"""
        if self.mode != "synth" and vehicle is not None:
            img = self.grab_real(vehicle)
            if img is not None:
                return img, "real"
        return self.synth(name, pose, markers, stamp), "synth"

    def save(self, name, pose, outdir, markers=None, stamp=None, vehicle=None):
        img, src = self.grab(name, pose, markers, stamp, vehicle=vehicle)
        ensure_dir(outdir)
        path = os.path.join(outdir, "%s_t%07.1fs.png" % (name, stamp if stamp else 0.0))
        img.save(path)
        self.saved += 1
        return path, src


# ===========================================================================
#  目标簿：人工指定 + 默认"最近敌人"
# ===========================================================================
class GoalBook(object):
    """管理每台平台的目标位置。

    explicit —— 外部输入（goals.json / 命令行），优先级最高
    inferred —— 没输入时的默认：离该平台最近的敌人
    """

    def __init__(self, markers, explicit=None, json_path=None):
        self.markers = markers
        self.explicit = {}
        self.inferred = {}
        self.json_path = json_path
        self._mtime = None
        if explicit:
            self.set_explicit(explicit)
        if json_path:
            self.prime(json_path)

    # ---- 外部输入 ----
    def set_explicit(self, mapping):
        for k, v in mapping.items():
            if k.startswith("__"):
                continue
            if isinstance(v, dict):
                if v.get("x") is None or v.get("y") is None:
                    continue
                self.explicit[k] = (float(v["x"]), float(v["y"]))
            elif isinstance(v, (list, tuple)) and len(v) >= 2:
                self.explicit[k] = (float(v[0]), float(v[1]))
        log("外部指定的目标 %d 条: %s"
            % (len(self.explicit), ", ".join(sorted(self.explicit)) or "无"))

    def prime(self, path):
        try:
            self._mtime = os.path.getmtime(path) if path and os.path.isfile(path) else None
        except OSError:
            self._mtime = None

    def poll(self, path=None):
        """文件变了就重载，返回 True 表示有更新。"""
        path = path or self.json_path
        if not path or not os.path.isfile(path):
            return False
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return False
        if self._mtime is not None and abs(mtime - self._mtime) < 0.05:
            return False
        self._mtime = mtime
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            log("读取目标文件 %s 失败: %r" % (path, e))
            return False
        if not isinstance(data, dict):
            log("目标文件应是 JSON 对象 {平台名: [x, y]}")
            return False
        if data.get("__clear__") or data.get("__清空__"):
            self.explicit = {}
            log("已清空外部目标，全部回到默认（最近敌人）")
            return True
        self.set_explicit(data)
        return True

    # ---- 默认：最近敌人 ----
    def infer(self, snapshot):
        """按当前位置给每台平台挑最近的敌人。"""
        self.inferred = {}
        if not self.markers:
            return
        for name, p in snapshot.items():
            best, bd = None, float("inf")
            for m in self.markers:
                d = math.hypot(m.x - p.x, m.y - p.y)
                if d < bd:
                    best, bd = m, d
            if best is not None:
                self.inferred[name] = (best.x, best.y)
        log("默认目标（最近敌人）已推断 %d 台" % len(self.inferred))

    # ---- 对外 ----
    def goals(self):
        out = dict(self.inferred)
        out.update(self.explicit)
        return out

    def source(self, name):
        return "file" if name in self.explicit else "nearest"

    def sources(self):
        return {n: self.source(n) for n in self.goals()}

    def target_marker(self, name):
        """该平台目标是哪个敌人（用于 HUD 的 #编号）。"""
        g = self.goals().get(name)
        if g is None:
            return None
        for m in self.markers:
            if math.hypot(m.x - g[0], m.y - g[1]) < 1.0:
                return m
        return None


# ===========================================================================
#  外部探测结果（让负责无人机的同学换成自己的探测模型）
# ===========================================================================
class DetectionFeed(object):
    """从 JSON 读取"我判定发现了哪些目标"，用来替代/补充 SDK 的固定半径判定。

    SDK 自带的 detect_situation() 只按 **固定半径** 返回 Actor（半径来自
    vehicles.json 的 detectRadius）。如果无人机那边要做更细的模型
    （视场角、建筑遮挡、发现延迟、概率……），把结论写进这个文件即可，
    本脚本只负责上色，不关心你是怎么算出来的。

    文件格式（config/detected.json）：

        {
            "points":     [{"x": 1535.5, "y": -889.3}],   // 按坐标匹配已知目标
            "discovered": [8, 10],                        // 或直接给下标（0 开始）
            "extra":      [{"x": 900, "y": -700, "type": "dynamic.prop.soldier"}],
            "__清空__":   false                            // true = 清空所有"已发现"
        }

    · `points` / `discovered` 二选一即可，都会把目标点亮成实心
    · `extra` 是 targets.json 里没有的目标，画成橙色叉
    · 保存即生效（按文件修改时间热重载）
    """

    def __init__(self, path=None):
        self.path = path
        self._mtime = None
        self.points = []
        self.indices = []
        self.extra = []
        self.clear_requested = False

    def prime(self, path=None):
        path = path or self.path
        try:
            self._mtime = os.path.getmtime(path) if path and os.path.isfile(path) else None
        except OSError:
            self._mtime = None

    def poll(self, path=None):
        path = path or self.path
        if not path or not os.path.isfile(path):
            return False
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return False
        if self._mtime is not None and abs(mtime - self._mtime) < 0.05:
            return False
        self._mtime = mtime
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            log("读取探测结果 %s 失败: %r" % (path, e))
            return False
        if not isinstance(data, dict):
            log("探测结果应是 JSON 对象")
            return False

        self.clear_requested = bool(data.get("__清空__") or data.get("__clear__"))
        if self.clear_requested:
            self.points, self.indices, self.extra = [], [], []
            return True

        self.points = [p for p in (data.get("points") or [])
                       if isinstance(p, dict) and "x" in p and "y" in p]
        self.extra = [p for p in (data.get("extra") or [])
                      if isinstance(p, dict) and "x" in p and "y" in p]
        idx = []
        for i in (data.get("discovered") or []):
            try:
                idx.append(int(i))
            except Exception:
                pass
        self.indices = idx
        return True


# ===========================================================================
#  实时视图
# ===========================================================================
class LiveView(object):
    COLORS = {
        "ugv": ["#ff7f0e", "#d62728", "#8c564b"],
        "uav": ["#1f77b4", "#17becf", "#9467bd"],
    }
    HELP = "1-6 聚焦 | 0 总览 | f 循环 | +/- 缩放 | s 存图 | h 帮助 | r 重算默认目标"

    def __init__(self, scene, env, names, types, zoom=180.0, trail_max=1000,
                 focus=None, show_lines=False, title="live_track - display",
                 roi=None):
        self.scene = scene
        self.env = env
        self.names = list(names)
        self.types = dict(types)
        self.zoom = float(zoom)
        self.trail_max = int(trail_max)
        self._scan_seen = {}          # 每架机最近一次画过的扫描点
        self.scan_radius_px = 300.0   # 探测半径（像素=米），由 full_mission 注入
        self.focus = focus if focus in self.names else (self.names[0] if self.names else None)
        self.show_help = True
        self.show_lines = show_lines
        self.snap_count = 0
        self.snapshot_dir = None
        self.on_reinfer = None          # 回调：按 r 时触发重算默认目标
        self.goals = {}
        self.goal_sources = {}
        self.markers = []
        #: 固定观察窗口 (cx, cy, half) —— 世界坐标、单位米。
        #: 设置后右侧面板锁定这个范围，不再跟随 focus，也不会被键盘缩放改掉。
        #: 答辩演示时只关心任务所在的一小块区域，锁死比手动缩放清晰得多。
        self.roi = tuple(roi) if roi else None
        #: 本轮的搜索区域 (cx, cy, half)；由 set_search_area() 设置后画成醒目实线
        self.search_area = None
        self._sa_patch_all = None
        self._sa_patch_zoom = None

        self.trails = {n: {"x": [], "y": []} for n in self.names}

        max_disp = 1400
        # 用 ceil：宁可多降一档也不超过 max_disp，内存更可控
        f = max(1, int(math.ceil(max(scene.w, scene.h) / float(max_disp))))
        # 用 ascontiguousarray 拷贝一份降采样图，之后就可以把整图释放掉
        self.disp = np.ascontiguousarray(scene.arr[::f, ::f])
        try:
            if scene.release_full_image():
                log("已释放整幅路网图（降采样后约 %.1f MB）"
                    % (self.disp.nbytes / 1e6))
        except Exception:
            pass

        self.fig, (self.ax_all, self.ax_zoom) = plt.subplots(
            1, 2, figsize=(9.8, 7.0), gridspec_kw={"width_ratios": [1.25, 1.0]})
        for setter in (
            lambda: self.fig.canvas.manager.set_window_title(title),
            lambda: self.fig.canvas.set_window_title(title),
        ):
            try:
                setter()
                break
            except Exception:
                continue

        for ax, with_legend in ((self.ax_all, True), (self.ax_zoom, False)):
            ax.imshow(self.disp, extent=(0, scene.w, scene.h, 0),
                      interpolation="nearest", zorder=0)
            ax.set_xlabel("pixel x")
            ax.set_ylabel("pixel y")
            ax.grid(True, color="#666666", alpha=0.25, linewidth=0.5)
            if env is not None:
                env.draw(scene, ax, with_legend=with_legend,
                         show_lines=self.show_lines)

        # ---- 敌人三层 ----
        self.mk_hollow = self.ax_all.plot(
            [], [], "o", ms=10, mfc="none", mec="#ff5252", mew=1.8, linestyle="",
            zorder=7, label=T("敌人·未探测", "enemy (unknown)"))[0]
        self.mk_filled = self.ax_all.plot(
            [], [], "o", ms=9, mfc="#ff2d2d", mec="white", mew=0.9, linestyle="",
            zorder=8, label=T("敌人·已探测", "enemy (detected)"))[0]
        self.mk_extra = self.ax_all.plot(
            [], [], "X", ms=8, mfc="#ffb000", mec="black", mew=0.8, linestyle="",
            zorder=8, label=T("已消灭/未登记", "killed"))[0]
        self.mk_hollow_z = self.ax_zoom.plot(
            [], [], "o", ms=10, mfc="none", mec="#ff5252", mew=1.8,
            linestyle="", zorder=7)[0]
        self.mk_filled_z = self.ax_zoom.plot(
            [], [], "o", ms=9, mfc="#ff2d2d", mec="white", mew=0.9,
            linestyle="", zorder=8)[0]
        self.mk_extra_z = self.ax_zoom.plot(
            [], [], "X", ms=8, mfc="#ffb000", mec="black", mew=0.8,
            linestyle="", zorder=8)[0]

        # ---- 平台图元 ----
        self.trail_ln, self.marker_ln, self.arrow_ln = {}, {}, {}
        self.tag, self.ring_ln = {}, {}
        self.trail_ln_z, self.marker_ln_z, self.arrow_ln_z, self.ring_ln_z = {}, {}, {}, {}
        for n in self.names:
            c = self._color_of(n)
            mk = "s" if self.types.get(n) == "ugv" else "^"
            self.trail_ln[n], = self.ax_all.plot([], [], "-", color=c, lw=1.6,
                                                 alpha=0.8, zorder=6)
            self.marker_ln[n], = self.ax_all.plot([], [], mk, ms=11, mfc=c,
                                                  mec="white", mew=1.2, zorder=10)
            self.arrow_ln[n], = self.ax_all.plot([], [], "-", color=c, lw=2.0,
                                                 alpha=0.95, zorder=9)
            self.ring_ln[n], = self.ax_all.plot([], [], "o", ms=20, mfc="none",
                                                mec="#ffffff", mew=1.8,
                                                linestyle="", zorder=11)
            self.tag[n] = self.ax_all.text(0, 0, n, color=c, fontsize=9,
                                           fontweight="bold", zorder=12,
                                           ha="left", va="bottom")
            self.trail_ln_z[n], = self.ax_zoom.plot([], [], "-", color=c, lw=1.6,
                                                    alpha=0.8, zorder=6)
            self.marker_ln_z[n], = self.ax_zoom.plot([], [], mk, ms=13, mfc=c,
                                                     mec="white", mew=1.2, zorder=10)
            self.arrow_ln_z[n], = self.ax_zoom.plot([], [], "-", color=c, lw=2.2,
                                                    alpha=0.95, zorder=9)
            self.ring_ln_z[n], = self.ax_zoom.plot([], [], "o", ms=24, mfc="none",
                                                   mec="#ffffff", mew=1.8,
                                                   linestyle="", zorder=11)

        # ---- 目标星标 + 连线 ----
        self.goal_pts, self.goal_lines = {}, {}
        for n in self.names:
            self.goal_pts[n], = self.ax_all.plot(
                [], [], "*", ms=15, mfc=self._color_of(n), mec="white", mew=0.8,
                linestyle="", zorder=9)
            self.goal_lines[n], = self.ax_all.plot(
                [], [], ":", color=self._color_of(n), lw=1.2, alpha=0.7, zorder=5)
        self.ax_all.plot([], [], "*", ms=13, mfc="white", mec="black",
                         linestyle="", label=T("目标位置", "goal"))

        self.hud = self.ax_all.text(
            0.01, 0.01, "", transform=self.ax_all.transAxes,
            fontsize=9, family="monospace", va="bottom",
            bbox=dict(boxstyle="round", fc="black", ec="#555555", alpha=0.78),
            color="white", zorder=20)

        self.ax_all.legend(
            handles=[
                Line2D([], [], marker="s", ls="", color="#ff7f0e",
                       label=T("UGV 无人车", "UGV")),
                Line2D([], [], marker="^", ls="", color="#1f77b4",
                       label=T("UAV 无人机", "UAV")),
            ] + list(self.ax_all.get_legend_handles_labels()[0]),
            loc="upper right", fontsize=8, framealpha=0.9)
        self.ax_all.set_xlim(0, scene.w)
        self.ax_all.set_ylim(scene.h, 0)

        # 【界面改版】只保留局部地图：隐藏总览轴，局部图铺满窗口；
        # 图例与状态栏都改挂到局部图上，底部一行显示全局状态。
        try:
            self.ax_all.set_visible(False)
            # 地图占左上方，右侧留图例，下方留状态色块
            self.ax_zoom.set_position([0.235, 0.240, 0.400, 0.620])
            self.ax_all.get_legend().remove()
        except Exception:
            pass
        self.legend = self.ax_zoom.legend(
            handles=[
                Line2D([], [], marker="s", ls="", color="#ff7f0e",
                       label=T("UGV 无人车", "UGV")),
                Line2D([], [], marker="^", ls="", color="#1f77b4",
                       label=T("UAV 无人机", "UAV")),
            ] + list(self.ax_all.get_legend_handles_labels()[0]),
            loc="upper left", bbox_to_anchor=(-0.56, 1.00), fontsize=8.0,
            framealpha=0.92, borderaxespad=0.0)
        # 底部状态：每台车/机一个色块，另加一个总状态块（黑字 + 浅色底）
        self.status_texts = {}
        _styles = {"UAV": "#dcebff", "UGV": "#ffe6cc"}
        for _i, _nm in enumerate(self.names):
            _col = 0 if _i < 3 else 1
            _row = _i % 3
            _key = "UAV" if str(_nm).upper().startswith("UAV") else "UGV"
            self.status_texts[_nm] = self.fig.text(
                0.700, ((0.790 - _row * 0.035) if _i < 3 else (0.610 - _row * 0.035)), _nm,
                fontsize=8, family="monospace", color="black", va="center",
                bbox=dict(boxstyle="round,pad=0.35", fc=_styles[_key],
                          ec="#888888", lw=0.6))
        self.status_sum = self.fig.text(
            0.210, 0.090, "", fontsize=8.5, family="monospace", color="black",
            va="center", bbox=dict(boxstyle="round,pad=0.45", fc="#f2f2f2",
                                   ec="#888888", lw=0.6))

        # 不能用 tight_layout()：它会覆盖上面 set_position 的布局，把地图挤到角落、
        # 留出大片空白（之前就是这样）。手动固定边距。
        # 注意：这里不能再调 subplots_adjust / tight_layout ——
        # 它们会把上面 set_position 设定的位置覆盖回"网格第二格"（地图永远在右半边）。

        try:
            self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        except Exception:
            pass

    # ---------------- 颜色 / 键盘 ----------------
    def _color_of(self, name):
        group = "ugv" if self.types.get(name) == "ugv" else "uav"
        idx = 0
        for n in self.names:
            if self.types.get(n) != self.types.get(name):
                continue
            if n == name:
                break
            idx += 1
        return LiveView.COLORS[group][idx % 3]

    def _on_key(self, ev):
        k = (ev.key or "").lower()
        try:
            if k in "123456" and k:
                i = int(k) - 1
                if 0 <= i < len(self.names):
                    self.focus = self.names[i]
                    log("视角聚焦 -> %s" % self.focus)
            elif k in ("0", "o"):
                self.focus = None
                log("视角 -> 全城总览")
            elif k == "f":
                i = self.names.index(self.focus) if self.focus in self.names else -1
                self.focus = self.names[(i + 1) % len(self.names)]
                log("视角聚焦 -> %s" % self.focus)
            elif k in ("+", "="):
                self.zoom = max(30.0, self.zoom * 0.7)
                log("缩放 -> %.0f m" % self.zoom)
            elif k in ("-", "_"):
                self.zoom = min(3000.0, self.zoom / 0.7)
                log("缩放 -> %.0f m" % self.zoom)
            elif k == "h":
                self.show_help = not self.show_help
            elif k == "s":
                self.save_snapshot()
            elif k == "r":
                if self.on_reinfer:
                    self.on_reinfer()
        except Exception as e:
            log("按键处理异常: %r" % (e,))

    def save_snapshot(self):
        outdir = self.snapshot_dir or os.path.join(WORKPLACE_DIR, "figs")
        ensure_dir(outdir)
        self.snap_count += 1
        path = os.path.join(outdir, "live_snapshot_%s_%02d.png"
                            % (time.strftime("%H%M%S"), self.snap_count))
        self.fig.savefig(path, dpi=110)
        log("已存图: %s" % path)
        return path

    # ---------------- 敌人图层 ----------------
    def set_encircle(self, points, radius=45.0):
        """把需要合围打击的敌人用黄色圆圈标出来（半径按米，默认 45 m）。"""
        try:
            if getattr(self, "_enc_patches", None) is None:
                self._enc_patches = []
            for p in self._enc_patches:
                try:
                    p.remove()
                except Exception:
                    pass
            self._enc_patches = []
            import matplotlib.patches as mpatches
            for (x, y) in points or []:
                px, py = self.scene.to_px(x, y)
                for ax in (self.ax_zoom,):
                    c = mpatches.Circle((px, py), radius, fill=False,
                                        ec="#ffd400", lw=2.2, ls="-", zorder=9)
                    ax.add_patch(c)
                    self._enc_patches.append(c)
        except Exception:
            pass

    def set_markers(self, markers):
        self.markers = list(markers)
        hx, hy, fx, fy, ex, ey = [], [], [], [], [], []
        for m in self.markers:
            px, py = self.scene.to_px(m.x, m.y)
            if m.extra:
                ex.append(px)
                ey.append(py)
            elif m.discovered:
                fx.append(px)
                fy.append(py)
            else:
                hx.append(px)
                hy.append(py)
        for hp, pp, dx, dy in ((self.mk_hollow, self.mk_hollow_z, hx, hy),
                               (self.mk_filled, self.mk_filled_z, fx, fy),
                               (self.mk_extra, self.mk_extra_z, ex, ey)):
            hp.set_data(dx, dy)
            pp.set_data(dx, dy)

    def discovered_count(self):
        # 运行态给了真实数字就用真实数字（marker 可能不全，之前 HUD 会显示 0/4）
        if getattr(self, "discovered_override", None) is not None:
            return self.discovered_override
        return sum(1 for m in self.markers if m.discovered and not m.extra)

    def known_count(self):
        if getattr(self, "known_override", None) is not None:
            return self.known_override
        return sum(1 for m in self.markers if not m.extra)

    # ---------------- 目标 ----------------
    def set_search_area(self, cx, cy, half):
        """标出本轮无人机实际搜索的区域（青绿色实线框）。

        场景自带的 arena 是地图边界（灰色虚线），两者概念不同：
        arena 可能有几公里，而搜索区通常只比敌人分布大一点点。
        """
        self.search_area = (float(cx), float(cy), float(half))

    def set_goals(self, goals, sources=None):
        self.goals = dict(goals or {})
        self.goal_sources = dict(sources or {})

    # ---------------- 每帧更新 ----------------
    def _mark_cover(self, cx, cy):
        """把"探测半径覆盖到的格子"记下来，每格只画一次半透明方块。"""
        import matplotlib.patches as mpatches
        step = 200.0
        r = float(self.scan_radius_px)
        try:
            covered = self._covered
        except AttributeError:
            self._covered = covered = set()
        import math as _m
        c0x, c1x = int(_m.floor((cx - r) / step)), int(_m.floor((cx + r) / step))
        c0y, c1y = int(_m.floor((cy - r) / step)), int(_m.floor((cy + r) / step))
        for ix in range(c0x, c1x + 1):
            for iy in range(c0y, c1y + 1):
                if (ix, iy) in covered:
                    continue
                gx, gy = (ix + 0.5) * step, (iy + 0.5) * step
                if (gx - cx) ** 2 + (gy - cy) ** 2 > r * r:
                    continue
                covered.add((ix, iy))
                for _ax in (self.ax_all, self.ax_zoom):
                    _ax.add_patch(mpatches.Rectangle(
                        (ix * step, iy * step), step, step,
                        color="#66c2ff", alpha=0.12, lw=0, zorder=1))

    def update(self, snapshot, elapsed, extra=""):
        scene = self.scene
        moving = 0
        lines = ["t=%6.1f s   聚焦: %s   已发现 %d/%d"
                 % (elapsed, self.focus or T("总览", "overview"),
                    self.discovered_count(), self.known_count())]

        for name in self.names:
            p = snapshot.get(name)
            if p is None:
                continue
            cx, cy = scene.to_px(p.x, p.y)
            tr = self.trails[name]
            tr["x"].append(cx)
            tr["y"].append(cy)
            if len(tr["x"]) > self.trail_max:
                del tr["x"][0]
                del tr["y"][0]
            # 扫描覆盖：按 200 m 网格记账，每格只画一次（避免多个圆反复叠加变深）
            try:
                self._mark_cover(cx, cy)
            except Exception:
                pass
            if p.moving:
                moving += 1

            self.trail_ln[name].set_data(tr["x"], tr["y"])
            self.trail_ln_z[name].set_data(tr["x"], tr["y"])

            color = self._color_of(name) if p.moving else "#b0b0b0"
            # 轨迹跟着图标一起变灰：无人机飞完/损毁后整条航迹也变灰
            for tl in (self.trail_ln[name], self.trail_ln_z[name]):
                try:
                    tl.set_color(color if p.moving else "#6f6f6f")
                except Exception:
                    pass
            for mk_ln in (self.marker_ln[name], self.marker_ln_z[name]):
                mk_ln.set_data([cx], [cy])
                mk_ln.set_markerfacecolor(color)
                mk_ln.set_markeredgecolor("#ffffff" if p.moving else "#7a7a7a")

            L = 16.0 + 2.4 * min(abs(p.v), 20.0)
            exx = cx + L * math.cos(p.yaw)
            eyy = cy + L * math.sin(p.yaw)
            for arr in (self.arrow_ln[name], self.arrow_ln_z[name]):
                arr.set_data([cx, exx], [cy, eyy])
                arr.set_color(color)

            for ring in (self.ring_ln[name], self.ring_ln_z[name]):
                if name == self.focus:
                    ring.set_data([cx], [cy])
                    ring.set_markeredgecolor("#ffe066")
                else:
                    ring.set_data([], [])

            tag = name if not p.moving else "%s %.1f" % (name, abs(p.v))
            self.tag[name].set_text(tag)
            self.tag[name].set_color(color)
            self.tag[name].set_position((cx + 9, cy - 9))

            # 目标星标 + 虚线
            g = self.goals.get(name)
            if g is None:
                self.goal_pts[name].set_data([], [])
                self.goal_lines[name].set_data([], [])
                lines.append("%-5s %s v=%4.1f  %s"
                             % (name, ">>" if p.moving else "--", p.v,
                                T("无目标", "no goal")))
                continue
            gx, gy = scene.to_px(g[0], g[1])
            src = self.goal_sources.get(name, "nearest")
            # 人工指定 = 实心星；默认推断 = 空心星
            self.goal_pts[name].set_data([gx], [gy])
            self.goal_pts[name].set_markerfacecolor(
                self._color_of(name) if src == "file" else "none")
            self.goal_pts[name].set_markeredgecolor(
                "white" if src == "file" else self._color_of(name))
            self.goal_pts[name].set_markeredgewidth(0.8 if src == "file" else 1.8)
            self.goal_lines[name].set_data([cx, gx], [cy, gy])

            rem = math.hypot(g[0] - p.x, g[1] - p.y)
            lines.append("%-5s %s v=%4.1f  %s %5.0fm  [%s]"
                         % (name, ">>" if p.moving else "--", p.v,
                            T("剩余", "rem"), rem,
                            T("指定", "file") if src == "file" else T("默认", "nearest")))

        if extra:
            lines.append(extra)
        if self.show_help:
            lines.append(LiveView.HELP)
        lines.append(T("在动 %d / %d 台", "moving %d / %d") % (moving, len(self.names)))
        self.hud.set_text("\n".join(lines))
        try:
            _body = lines[1:1 + len(self.names)]
            for _i, _nm in enumerate(self.names):
                if _i < len(_body):
                    self.status_texts[_nm].set_text(_body[_i])
            _sum = lines[0]
            if 1 + len(self.names) < len(lines):
                _sum = _sum + "    " + lines[1 + len(self.names)]
            self.status_sum.set_text(_sum)
        except Exception:
            pass
        try:
            # 底部状态栏：一辆车/一架机一行，紧凑展示全局状态
            self.hud_zoom.set_text("\n".join(lines))
        except Exception:
            pass

        # ---- 搜索区域（青绿实线）----
        if self.search_area is not None:
            scx, scy, shalf = self.search_area
            p0 = scene.to_px(scx - shalf, scy - shalf)
            p1 = scene.to_px(scx + shalf, scy + shalf)
            x0, x1 = min(p0[0], p1[0]), max(p0[0], p1[0])
            y0, y1 = min(p0[1], p1[1]), max(p0[1], p1[1])
            for ax, attr in ((self.ax_all, "_sa_patch_all"),
                             (self.ax_zoom, "_sa_patch_zoom")):
                old = getattr(self, attr)
                if old is not None:
                    try:
                        old.remove()
                    except Exception:
                        pass
                from matplotlib.patches import Rectangle as _Rect
                r = _Rect((x0, y0), x1 - x0, y1 - y0, fill=False,
                          ec="#00e5b0", lw=2.2, ls="-", zorder=6)
                ax.add_patch(r)
                setattr(self, attr, r)
            try:
                self.ax_all.text(x0 + 6, y0 - 8,
                                 T("搜索区 ±%.0f m", "search ±%.0f m") % shalf,
                                 color="#00e5b0", fontsize=8, zorder=7)
            except Exception:
                pass

        self.ax_all.set_title(T("全城总览 · 环境与敌情", "Overview"))
        if self.roi is not None:
            rcx, rcy, rhalf = self.roi
            rx, ry = scene.to_px(rcx, rcy)
            rh = rhalf * getattr(scene, "scale", 1.0) or rhalf
            self.ax_zoom.set_xlim(rx - rh, rx + rh)
            self.ax_zoom.set_ylim(ry + rh, ry - rh)
            self.ax_zoom.set_title(T("任务区域  ±%.0f m", "ROI ±%.0f m") % rhalf)
        elif self.focus and self.focus in snapshot:
            p = snapshot[self.focus]
            lx, ly = scene.to_px(p.x, p.y)
            self.ax_zoom.set_xlim(lx - self.zoom, lx + self.zoom)
            self.ax_zoom.set_ylim(ly + self.zoom, ly - self.zoom)
            self.ax_zoom.set_title("%s  %s (±%.0f m)"
                                   % (T("聚焦", "Focus"), self.focus, self.zoom))
        else:
            self.ax_zoom.set_xlim(0, scene.w)
            self.ax_zoom.set_ylim(scene.h, 0)
            self.ax_zoom.set_title(T("总览（按 1-6 聚焦）", "Overview (1-6)"))

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def close(self):
        plt.close(self.fig)


# ===========================================================================
#  SDK 能力自检
# ===========================================================================
def probe(client, game):
    print("\n" + "=" * 76)
    print("SDK 能力自检 —— 只读，不会移动任何平台")
    print("=" * 76)

    ts = int(round(time.time() * 1000))
    _, ugv, code = client.register_node(node_type="四轮车", node_name="PROBE_UGV",
                                        node_no=1, frame_timestamp=ts)
    print("register_node 四轮车 -> code=%s" % (code,))
    if ugv is None:
        print("  注册失败，后面的检查无法进行")
        return

    print("\n[1] SwarmAENode 的公开方法：")
    print("   " + ", ".join(a for a in dir(ugv) if not a.startswith("_")))

    print("\n[2] 传感器接口：")
    for name in ("get_sensor", "set_sensor"):
        print("   %-14s %s" % (name, "存在" if hasattr(ugv, name) else "不存在"))
    if hasattr(ugv, "get_sensor"):
        try:
            ret = ugv.get_sensor()
            print("   get_sensor() 返回类型 = %r" % (type(ret),))
            if isinstance(ret, (tuple, list)):
                for i, it in enumerate(ret):
                    print("     [%d] %r%s" % (i, type(it),
                                              "  有 get_image_data" if hasattr(it, "get_image_data") else ""))
        except Exception as e:
            print("   get_sensor() 调用失败: %r" % (e,))

    print("\n[3] 探测接口：")
    try:
        ret = ugv.detect_situation()
        print("   detect_situation() 返回 %d 个值" % len(ret))
        if len(ret) >= 3:
            try:
                objs = json.loads(ret[2])
                print("   探测到 %d 个 Actor" % len(objs))
                for o in objs[:5]:
                    print("      Type=%-10r Type_Name=%-30r Pos=(%s, %s)"
                          % (o.get("Type"), o.get("Type_Name"),
                             o.get("Position", {}).get("X"),
                             o.get("Position", {}).get("Y")))
                print("   全部 Type_Name = %s"
                      % sorted(set(o.get("Type_Name") for o in objs)))
            except Exception as e:
                print("   JSON 解析失败: %r" % (e,))
    except Exception as e:
        print("   detect_situation() 失败: %r" % (e,))

    print("\n[4] 路网参数（逐项打印）：")
    try:
        ret = game.get_road_network()
        items = list(ret) if isinstance(ret, (tuple, list)) else [ret]
        print("   返回 %d 项：" % len(items))
        for line in describe_return(items).splitlines():
            print(line)
        img, scale, off, _ = parse_road_network(ret)
        print("\n   按类型识别：img=%s scale=%s offset=%s"
              % (("有 %s" % (img.size,)) if img is not None else "**没认出来**",
                 scale, off))
    except Exception as e:
        print("   get_road_network() 失败: %r" % (e,))

    print("\n完成。\n")


# ===========================================================================
#  参数
# ===========================================================================
def _fix_leading_dash(argv, value_opts):
    """把 "--opt -1,2" 改写成 "--opt=-1,2"，否则负坐标会被 argparse 当选项。"""
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


def parse_args():
    ap = argparse.ArgumentParser(
        description="战场态势显示（纯显示层：只读状态，不控制、不规划）",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)

    g = ap.add_argument_group("目标位置接口")
    g.add_argument("--goals-json", default=os.path.join(WORKPLACE_DIR, "config", "goals.json"),
                   help="轮询这个文件，改了就更新星标（默认 config/goals.json）")
    g.add_argument("--no-goals-json", action="store_true", help="不轮询目标文件")
    g.add_argument("--goals", default=None,
                   help='一次性指定，例 "UGV1:1200,-400;UAV1:1400,-500"')
    g.add_argument("--goals-file", default=None, help="从指定 JSON 读目标（一次性）")
    g.add_argument("--no-infer", action="store_true",
                   help="不按'最近敌人'推断默认目标（没输入就不画星标）")

    g = ap.add_argument_group("敌人探测")
    g.add_argument("--no-detect", action="store_true", help="不做探测（敌人保持空心）")
    g.add_argument("--detect-from", choices=("uav", "ugv", "all"), default="uav",
                   help="用哪些平台去探测（默认 uav）")
    g.add_argument("--detect-every", type=int, default=10,
                   help="每 N 帧探测一次（默认10，约0.5秒）")
    g.add_argument("--detect-tol", type=float, default=20.0,
                   help="探测结果与已知目标判定为同一个的距离阈值 m（默认20）")
    g.add_argument("--detected-json",
                   default=os.path.join(WORKPLACE_DIR, "config", "detected.json"),
                   help="外部探测结果文件：无人机那边把'发现了哪些目标'写这里"
                        "（默认 config/detected.json；改了就生效）")
    g.add_argument("--no-detected-json", action="store_true",
                   help="不读外部探测结果文件（只用 SDK 的固定半径判定）")

    g = ap.add_argument_group("显示")
    g.add_argument("--zoom", type=float, default=180.0, help="右图聚焦半径 m（默认180）")
    g.add_argument("--focus", default=None, help="启动聚焦哪台（UGV1..UAV3），0=总览")
    g.add_argument("--show-phase-lines", action="store_true", help="画阶段线（SUBJECT_1..6）")
    g.add_argument("--no-enemies", action="store_true", help="不读 targets.json（不画敌人）")
    g.add_argument("--no-env", action="store_true", help="不画环境区域")

    g = ap.add_argument_group("无人机俯视图")
    g.add_argument("--uav-view", choices=("auto", "real", "synth"), default="auto",
                   help="auto=先试真机相机失败则合成")
    g.add_argument("--view-range", type=float, default=200.0, help="俯视图半径 m")
    g.add_argument("--fov", type=float, default=60.0, help="相机视场角 deg")
    g.add_argument("--save-every", type=int, default=60, help="每 N 帧存一张（0=不存）")

    g = ap.add_argument_group("其它")
    g.add_argument("--duration", type=float, default=1e9, help="最长运行秒数")
    g.add_argument("--outdir", default=None, help="俯视图输出目录")
    g.add_argument("--probe", action="store_true", help="只做 SDK 能力自检然后退出")

    value_opts = ("--goals", "--goals-file", "--goals-json", "--focus", "--outdir")
    return ap.parse_args(_fix_leading_dash(sys.argv[1:], value_opts))


def parse_goals_arg(text):
    """'UGV1:1200,-400;UAV1:1400,-500' -> {name: [x, y]}"""
    out = {}
    for chunk in str(text).replace("；", ";").split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ":" not in chunk:
            raise ValueError("--goals 格式应为 名称:x,y，收到 %r" % chunk)
        name, xy = chunk.split(":", 1)
        parts = [p for p in xy.replace("，", ",").split(",") if p.strip()]
        if len(parts) < 2:
            raise ValueError("--goals 里 %s 坐标不完整: %r" % (name, xy))
        out[name.strip()] = [float(parts[0]), float(parts[1])]
    return out


def load_goals_file(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("%s 应是 JSON 对象 {平台名: [x, y]}" % path)
    return data


def load_known_markers(scene_dir=None):
    """从 targets.json 读敌人，做成 Marker 列表（初始全部未探测）。"""
    scene_dir = scene_dir or SCENE_DIR
    tp = os.path.join(scene_dir, "targets.json")
    if not os.path.isfile(tp):
        log("找不到 %s，跳过敌人标注" % tp)
        return []
    try:
        raw = scenefile.load_targets(tp)
    except Exception as e:
        log("读取 targets.json 失败: %r" % (e,))
        return []
    out = []
    for i, t in enumerate(raw):
        kind = "strike" if t.type in scenefile.TYPE_INFO else "obstacle"
        out.append(Marker(t.type, t.x, t.y, t.z, index=i, kind=kind))
    return out


def main():
    args = parse_args()
    if log_note:
        log(log_note)

    # ---------------- 连仿真器 ----------------
    log("连接 %s:%d ..." % (args.ip, args.port))
    client = SwarmAEClient(ue_ip=args.ip, ue_port=args.port)
    _, game, _ = client.get_game()

    if args.probe:
        probe(client, game)
        return

    # ---------------- 路网图（带识别 / 重试 / 本地兜底） ----------------
    ret = game.get_road_network()
    img, scale, world_offset, items = parse_road_network(ret)
    if img is None or world_offset is None:
        log("!! get_road_network() 没认出图/偏移，原始返回：")
        for line in describe_return(items).splitlines():
            log(line)
        try:
            game.stage_start("reconnaissance_start")
            time.sleep(0.5)
            ret = game.get_road_network()
            img, scale, world_offset, items = parse_road_network(ret)
            log("   重取：img=%s offset=%s" % (type(img).__name__, world_offset))
        except Exception as e:
            log("   stage_start 重试失败: %r" % (e,))
    if img is None:
        local = os.path.join(EASY_FOLLOW_DIR, "lib", "roadnet.png")
        if os.path.isfile(local):
            log("!! 改用本地副本: %s" % local)
            img = Image.open(local)
            scale = scale or 1.0
            world_offset = world_offset or coord.DEFAULT_WORLD_OFFSET
        else:
            raise SystemExit("拿不到路网图，本地副本也不存在: %s" % local)
    scene = Scene(img, scale, world_offset)

    # ---------------- 环境 ----------------
    env = None
    if not args.no_env:
        gp = os.path.join(SCENE_DIR, "game.json")
        if os.path.isfile(gp):
            try:
                env = Environment(scenefile.load_environment(gp))
                log("环境要素：" + env.summary())
            except Exception as e:
                log("读取环境要素失败: %r" % (e,))
        else:
            log("找不到 game.json，跳过环境绘制")

    # ---------------- 敌人 ----------------
    markers = [] if args.no_enemies else load_known_markers()
    if markers:
        log("targets.json 读到 %d 个目标，初始全部为空心（未探测）" % len(markers))

    # ---------------- 平台（只读） ----------------
    fleet = Fleet(client)
    names = fleet.names()
    types = {n: ("ugv" if n.startswith("UGV") else "uav") for n in names}
    snapshot = fleet.snapshot()

    # ---------------- 目标簿 ----------------
    explicit = {}
    if args.goals:
        explicit.update(parse_goals_arg(args.goals))
    if args.goals_file:
        explicit.update({k: v for k, v in load_goals_file(args.goals_file).items()
                         if not k.startswith("__")})
    goals_path = None if args.no_goals_json else args.goals_json
    book = GoalBook(markers, explicit=explicit, json_path=goals_path)
    if goals_path:
        d = os.path.dirname(goals_path)
        if d:
            ensure_dir(d)
        if not os.path.isfile(goals_path):
            try:
                with open(goals_path, "w", encoding="utf-8") as f:
                    json.dump({"__说明__": "改这里的目标位置，保存即生效；"
                                            "没写的平台默认取最近的敌人",
                               "UGV1": [1200, -400],
                               "UAV1": {"x": 1400, "y": -500}},
                              f, indent=4, ensure_ascii=False)
                log("已生成目标文件模板: %s" % goals_path)
            except Exception as e:
                log("生成 %s 失败: %r" % (goals_path, e))
        log("轮询目标文件（改了即生效）: %s" % goals_path)
    if not args.no_infer:
        book.infer(snapshot)

    # ---------------- 外部探测结果接口 ----------------
    detected_path = None if args.no_detected_json else args.detected_json
    feed = None
    if detected_path:
        d = os.path.dirname(detected_path)
        if d:
            ensure_dir(d)
        if not os.path.isfile(detected_path):
            try:
                with open(detected_path, "w", encoding="utf-8") as f:
                    json.dump({
                        "__说明__": "无人机那边把'判定发现了哪些目标'写这里，保存即生效。"
                                    "points 按坐标匹配，discovered 按下标(从0开始)，"
                                    "extra 是不在 targets.json 里的目标，__清空__=true 重置",
                        "points": [],
                        "discovered": [],
                        "extra": [],
                        "__清空__": False,
                    }, f, indent=4, ensure_ascii=False)
                log("已生成探测结果模板: %s" % detected_path)
            except Exception as e:
                log("生成 %s 失败: %r" % (detected_path, e))
        feed = DetectionFeed(detected_path)
        feed.prime()
        log("轮询外部探测结果（改了即生效）: %s" % detected_path)

    # ---------------- 视图 ----------------
    focus = None if args.focus == "0" else (args.focus or names[0])
    view = LiveView(scene, env, names, types, zoom=args.zoom,
                    focus=focus, show_lines=args.show_phase_lines)
    view.set_markers(markers)
    view.set_goals(book.goals(), book.sources())
    view.on_reinfer = lambda: (book.infer(fleet.snapshot()),
                               view.set_goals(book.goals(), book.sources()))
    log("视角：1-6 聚焦 / 0 总览 / f 循环 / +/- 缩放 / s 存图 / h 帮助 / r 重算默认目标")

    # ---------------- 俯视图输出 ----------------
    uav_view = UavView(scene, mode=args.uav_view,
                       view_range=args.view_range, fov_deg=args.fov)
    outdir = args.outdir or os.path.join(WORKPLACE_DIR, "figs", "uav_view")
    ensure_dir(outdir)

    detect_sources = {"uav": fleet.uav_names,
                      "ugv": fleet.ugv_names,
                      "all": names}[args.detect_from] if not args.no_detect else []

    t_start = time.time()
    prev = t_start
    frame = 0
    last_save = {}
    real_src = None
    discovered_log = set()
    ext_extra = []          # 外部探测接口给的额外目标（跨帧保留）

    try:
        while True:
            now = time.time()
            el = now - t_start
            if el > args.duration:
                break
            frame += 1

            snapshot = fleet.snapshot()

            # ---- 目标文件热重载 ----
            if book.poll(goals_path):
                if not args.no_infer:
                    book.infer(snapshot)
                view.set_goals(book.goals(), book.sources())

            # ---- 探测：把发现的敌人从空心变实心 ----
            sdk_extra = []
            refreshed = False

            # (1) SDK 的固定半径判定
            if detect_sources and frame % max(1, args.detect_every) == 1:
                detected = []
                for nm in detect_sources:
                    detected.extend(fleet.detect_targets(nm))
                if detected:
                    found, extras = match_detections(detected, markers,
                                                     tol=args.detect_tol)
                    for i in found:
                        markers[i].discovered = True
                        if i not in discovered_log:
                            discovered_log.add(i)
                            log("发现目标 #%d %s (%.1f, %.1f)"
                                % (i, markers[i].cn, markers[i].x, markers[i].y))
                    for e in extras:
                        sdk_extra.append(
                            Marker(e.get("type", "unknown"), e["x"], e["y"],
                                   e.get("z", 0.0), index=-1, extra=True,
                                   kind=e.get("kind", "obstacle")))
                refreshed = True

            # (2) 外部探测结果（无人机那边自己的模型）
            if feed is not None and feed.poll():
                if feed.clear_requested:
                    for m in markers:
                        m.discovered = False
                    ext_extra = []
                    discovered_log.clear()
                    log("外部探测接口请求清空：所有目标恢复空心")
                if feed.indices:
                    for i in feed.indices:
                        if 0 <= i < len(markers):
                            markers[i].discovered = True
                if feed.points:
                    found, _ = match_detections(feed.points, markers,
                                                tol=args.detect_tol)
                    for i in found:
                        markers[i].discovered = True
                for p in feed.extra:
                    ext_extra.append(
                        Marker(p.get("type", "unknown"), p["x"], p["y"],
                               p.get("z", 0.0), index=-1, extra=True,
                               kind=p.get("kind", "obstacle")))
                n_new = sum(1 for m in markers if m.discovered)
                log("外部探测结果已更新：已发现 %d / %d" % (n_new, len(markers)))
                refreshed = True

            if refreshed:
                view.set_markers(markers + sdk_extra + ext_extra)

            # ---- 画 ----
            extra = ""
            if args.no_detect:
                extra = T("探测已关闭（--no-detect）", "detect off")
            view.update(snapshot, el, extra)

            # ---- 无人机俯视图 ----
            if args.save_every > 0:
                for nm in fleet.uav_names:
                    if el - last_save.get(nm, -1e9) < args.save_every / 20.0:
                        continue
                    try:
                        p = snapshot[nm]
                        path, src = uav_view.save(nm, p, outdir,
                                                  markers + sdk_extra + ext_extra, el,
                                                  vehicle=fleet.vehicle(nm))
                        last_save[nm] = el
                        if real_src != src:
                            real_src = src
                            log("无人机俯视图来源: %s" % src)
                            if src == "synth" and uav_view.real_err:
                                log("  真机相机不可用: %s" % uav_view.real_err)
                    except Exception as e:
                        log("保存 %s 俯视图失败: %r" % (nm, e))

            sleep = 0.05 - (time.time() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        log("用户中断（Ctrl+C）")
    finally:
        log("无人机俯视图: %s（共 %d 张，来源=%s）"
            % (outdir, uav_view.saved, real_src))
        log("已探测 %d / %d 个目标" % (view.discovered_count(), view.known_count()))
        try:
            view.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
