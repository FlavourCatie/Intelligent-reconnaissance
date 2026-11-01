# -*- coding: utf-8 -*-
"""
draw_arch.py —— 生成四层架构图（对应课程要求的 传输/决策/对象/终端 分层）
================================================================================

输出：WorkPlace/figs/four_layer_arch.png
用途：中期答辩 PPT 直接插图；图上的模块名与仓库里的文件一一对应。

用法：
    python WorkPlace\\tools\\draw_arch.py
"""

from __future__ import print_function

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(HERE)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

DARK = "#1b6b39"
MID = "#1f7a3f"
LIGHT = "#2e8b4f"
TEXT = "#ffffff"

# ---- 纵向布局（axes 坐标 0~1，从下往上）----
Y_FLOW, H_FLOW = 0.012, 0.165        # 层间数据流
Y_BODY, H_BODY = 0.205, 0.415        # 三列正文
Y_HEAD, H_HEAD = 0.628, 0.070        # 三列标题
Y_TOP, H_TOP = 0.735, 0.225          # 传输层
Y_TITLE = 0.995                      # 大标题


def arrow(ax, x0, y0, x1, y1):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>",
                                 mutation_scale=13, lw=1.6, color=DARK,
                                 zorder=1))


def main():
    fig, ax = plt.subplots(figsize=(13.2, 8.6))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1.03)
    ax.axis("off")

    # ---------------- 大标题 ----------------
    ax.text(0.5, Y_TITLE, "系统四层架构（传输 / 决策 / 对象 / 终端）",
            ha="center", va="top", fontsize=17, fontweight="bold",
            color="#12341f")

    # ---------------- 传输层 ----------------
    ax.add_patch(Rectangle((0.02, Y_TOP), 0.96, H_TOP,
                           facecolor=DARK, edgecolor="none", zorder=2))
    ax.text(0.5, Y_TOP + H_TOP - 0.028, "传输层", ha="center", va="top",
            color=TEXT, fontsize=17, fontweight="bold", zorder=3)
    tp = [
        "① 统一接口协议：SDK 客户端封装、参数保护（core/sdk_boot.py）",
        "② 链路体检：进程存活、端口占用、启动顺序（core/preflight.py, tools/net_check.py）",
        "③ 数据契约：config/goals.json、config/detected.json（原子写 + mtime 热加载）",
        "④ 坐标与单位：世界坐标 / 栅格像素、米制统一（tools/coord.py）",
    ]
    ax.text(0.033, Y_TOP + H_TOP - 0.088, "\n".join(tp), ha="left", va="top",
            color=TEXT, fontsize=9.4, linespacing=1.75, zorder=3)

    # ---------------- 三列 ----------------
    cols = [
        (0.025, "决策层", [
            "全链路编排  full_mission.py",
            "",
            "覆盖扫描规划  sweep_plan.py",
            "  牛耕式条带，间距 2R",
            "  三机按真实航程均衡",
            "",
            "目标处理",
            "  路网吸附 → 8 m 去重",
            "  全排列最优 1:1 分配",
            "",
            "派车触发：目标数 / 侦查完成 / 超时",
            "调度：错峰发车、会车让行、超时收尾",
        ]),
        (0.355, "对象层", [
            "无人机  control_uav.py",
            "  安全包络 + 可见性图 Dijkstra",
            "  探测 300 m / 15 m/s / Z=250 m",
            "",
            "适配层  uav_runner.py",
            "  爬升→巡航→保持 状态机",
            "  运动学：圆角/重采样/限速",
            "",
            "无人车  control_ugv.py",
            "  骨架图 A* + 在线重规划",
            "  避障 / S 形超车 / 倒车退出",
        ]),
        (0.685, "终端层", [
            "战场态势  live_track.py",
            "  路网/任务区/禁飞区/雷区/电磁区",
            "  敌人：空心（未发现）→ 实心",
            "  目标星标 + 虚线连线",
            "  自动 ROI 锁定任务区域",
            "  无人机配色：在飞彩色，",
            "        侦查结束转灰",
            "  键盘：聚焦/缩放/存图",
            "",
            "演示入口  demo_midterm.py",
        ]),
    ]

    for cx, title, lines in cols:
        ax.add_patch(Rectangle((cx, Y_HEAD), 0.29, H_HEAD, facecolor=MID,
                               edgecolor="none", zorder=2))
        ax.text(cx + 0.145, Y_HEAD + H_HEAD / 2, title, ha="center", va="center",
                color=TEXT, fontsize=16, fontweight="bold", zorder=3)
        ax.add_patch(Rectangle((cx, Y_BODY), 0.29, H_BODY, facecolor=LIGHT,
                               edgecolor="none", zorder=2))
        ax.text(cx + 0.014, Y_BODY + H_BODY - 0.018, "\n".join(lines),
                ha="left", va="top", color=TEXT, fontsize=9.0,
                linespacing=1.62, zorder=3)
        # 与传输层双向箭头
        mid = cx + 0.145
        arrow(ax, mid - 0.022, Y_HEAD + H_HEAD + 0.004, mid - 0.022,
              Y_TOP - 0.004)
        arrow(ax, mid + 0.022, Y_TOP - 0.004, mid + 0.022,
              Y_HEAD + H_HEAD + 0.004)

    # ---------------- 层间数据流 ----------------
    ax.add_patch(Rectangle((0.025, Y_FLOW), 0.95, H_FLOW,
                           facecolor="#eef6f0", edgecolor=MID, lw=1.2, zorder=2))
    ax.text(0.04, Y_FLOW + H_FLOW - 0.022, "层间数据流", ha="left", va="top",
            color=DARK, fontsize=11.5, fontweight="bold", zorder=3)
    flow = [
        "① 决策层 → 对象层：扫描区域 / 目标点 / 航点序列（goals.json、uav_sweep.json）",
        "② 对象层 → 决策层：位姿与探测结果（detected.json，mtime 热加载）",
        "③ 对象层 → 终端层：实时位姿快照（同进程共享 Pose 对象）",
        "④ 终端层 → 决策层：人工干预（键盘重算目标 / 指定目标）",
    ]
    ax.text(0.04, Y_FLOW + H_FLOW - 0.062, "\n".join(flow), ha="left", va="top",
            color="#1c3b28", fontsize=9.0, linespacing=1.5, zorder=3)

    out = os.path.join(WORKPLACE_DIR, "figs", "four_layer_arch.png")
    d = os.path.dirname(out)
    if not os.path.isdir(d):
        os.makedirs(d)
    fig.savefig(out, dpi=110, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("已生成 %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
