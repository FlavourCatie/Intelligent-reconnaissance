# -*- coding: utf-8 -*-
"""
coord.py —— 世界坐标 <-> 路网像素坐标 统一转换工具
=====================================================

为什么需要这个文件：
    整个仿真里最容易出错、也最容易白跑一天的地方就是"坐标单位"。
    SDK 文档《附五》给出的唯一官方公式是：

        pixel_x = int(scale * (location_x - world_offset[0]))
        pixel_y = int(scale * (location_y - world_offset[1]))

    其中 scale / world_offset 必须从 AeGame.get_road_network() 实时取，
    不能写死。但 Materials 里的示例代码有两处把数值写死了（见下面的
    KNOWN_HARDCODED 说明），所以这里把转换集中到一处，方便统一修改。

关于单位（重要，团队内部必须统一口径）：
    * 路网图 roadnet.png 为 4794 x 4794，scale = 1。
      即 1 像素 = 1 米，图幅约 4.8 km x 4.8 km。
    * game.json / vehicles.json / targets.json 里的坐标是"米"，
      因为 arena 宽 2277.4 - (-1900.3) = 4177.7 m，正好铺满 4794 m 的图幅；
      如果按厘米理解，arena 只有 41 m 宽，与图幅完全不符。
    * 但 lib/swarm_api.py 的 detect_fun() 里写了 Position.X / 100，
      相当于它认为 detect_situation() 返回的是"厘米"。
    * get_location() 返回的坐标则被 plan.py 直接当"米"用（未除以 100）。

    => 结论：detect_situation() 与 get_location() 的单位很可能不一致，
       这是现有代码里潜藏的一个 bug 来源。
       请先运行 tools/unit_check.py 在真机上确认，再决定是否保留 /100。

KNOWN_HARDCODED（示例代码里写死的常量，改图后必须同步改）：
    1. lib/swarm_api.py  detect_fun():
           x1 = int(x1 + 2095)      # 2095 应为 -world_offset[0]
           y1 = int(y1 + 2637)      # 2637 应为 -world_offset[1]
    2. click.py:
           world_x = x + (-2094.5277099609375)
           world_y = y + (-2637.31982421875)
    3. plan.py / plan_control.py 的终点:
           end_x = int(-1500 - world_offset[0])   # 默认终点 (-1500, -217)

本文件用 click.py 实测得到的偏移量作为"离线默认值"，只用于
不连仿真器时的画图/算路；真正跑仿真时请调用 refresh_from_engine()。
"""

# ---------------------------------------------------------------------------
# 默认参数（离线画图用；来源：click.py 实测值，精度到 1e-9）
# ---------------------------------------------------------------------------
DEFAULT_WORLD_OFFSET = (-2094.5277099609375, -2637.31982421875)
DEFAULT_SCALE = 1.0

#: 路网图尺寸（Materials/practice/easy_follow/lib/roadnet.png），1 px = 1 m
MAP_SIZE_PX = (4794, 4794)

#: 图幅覆盖的世界坐标范围（米），由 offset + 尺寸推出
MAP_WORLD_BOUNDS = (
    DEFAULT_WORLD_OFFSET[0],
    DEFAULT_WORLD_OFFSET[0] + MAP_SIZE_PX[0] * DEFAULT_SCALE,
    DEFAULT_WORLD_OFFSET[1],
    DEFAULT_WORLD_OFFSET[1] + MAP_SIZE_PX[1] * DEFAULT_SCALE,
)


def world_to_pixel(x, y, scale=DEFAULT_SCALE, world_offset=DEFAULT_WORLD_OFFSET):
    """UE 世界坐标(米) -> 路网图像素坐标。与 SDK《附五》公式一致。"""
    px = int(round(scale * (x - world_offset[0])))
    py = int(round(scale * (y - world_offset[1])))
    return px, py


def pixel_to_world(px, py, scale=DEFAULT_SCALE, world_offset=DEFAULT_WORLD_OFFSET):
    """路网图像素坐标 -> UE 世界坐标(米)。world_to_pixel 的逆运算。"""
    x = px / scale + world_offset[0]
    y = py / scale + world_offset[1]
    return x, y


def in_map(px, py, size=MAP_SIZE_PX):
    """判断像素点是否落在路网图内（防止 A* 越界）。"""
    return 0 <= px < size[0] and 0 <= py < size[1]


def dist(x1, y1, x2, y2):
    """欧氏距离(米)。"""
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def refresh_from_engine(ae_game):
    """
    连上仿真器后调用，拿真实 scale / world_offset 覆盖默认值。

    用法：
        from swarmae.SwarmAEClient import SwarmAEClient
        import tools.coord as coord
        client = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
        _, game, _ = client.get_game()
        coord.refresh_from_engine(game)     # 打印并返回真实参数

    返回 (scale, world_offset)，同时把模块级默认值改成真实值，
    这样后续所有 world_to_pixel() 调用都自动是准的。
    """
    global DEFAULT_SCALE, DEFAULT_WORLD_OFFSET
    img, scale, world_offset, _, _ = ae_game.get_road_network()
    DEFAULT_SCALE = float(scale)
    DEFAULT_WORLD_OFFSET = (float(world_offset[0]), float(world_offset[1]))
    size = getattr(img, "size", None)
    print(f"[coord] road network img size = {size}")
    print(f"[coord] scale = {DEFAULT_SCALE}")
    print(f"[coord] world_offset = {DEFAULT_WORLD_OFFSET}")
    return DEFAULT_SCALE, DEFAULT_WORLD_OFFSET


if __name__ == "__main__":
    # 自检：往返转换 + 打印图幅范围
    print("MAP_SIZE_PX      =", MAP_SIZE_PX)
    print("world_offset     =", DEFAULT_WORLD_OFFSET)
    print("scale            =", DEFAULT_SCALE)
    print("MAP_WORLD_BOUNDS = x[%.1f, %.1f] y[%.1f, %.1f]" % MAP_WORLD_BOUNDS)

    for (wx, wy) in [(-1829.6, -517.7), (1267.9, -653.8), (-1500, -217)]:
        px, py = world_to_pixel(wx, wy)
        back = pixel_to_world(px, py)
        print(f"world({wx:>9.1f},{wy:>9.1f}) -> pixel({px:>5},{py:>5}) "
              f"-> world({back[0]:.1f},{back[1]:.1f})")

    # 与 detect_fun 里的写死值对照（应打印 True）
    px, py = world_to_pixel(0.0, 0.0)
    print("origin -> pixel", (px, py), "应为 (2095, 2637):", (px, py) == (2095, 2637))
