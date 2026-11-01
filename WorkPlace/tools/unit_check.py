# -*- coding: utf-8 -*-
"""
unit_check.py —— 上机前必须先跑的"事实核对"脚本
================================================

它要回答 4 个问题（现有示例代码里这 4 件事都不确定）：

    Q1  get_location() 返回的坐标单位是"米"还是"厘米"？
    Q2  detect_situation() 返回的 Position 单位是"米"还是"厘米"？
        （swarm_api.detect_fun() 里写了 /100，但 get_location() 的结果
          又被 plan.py 直接当米用 —— 两者必有一个是错的）
    Q3  Type / Type_Name 的真实取值到底是什么？
        （set_weapon() 里比对 'Barricade_BP'/'Stryker'/'Soldier'，
          data_struct.calculate_collision_radius() 里比对
          'BP_BarricadeHard'/'Barricade_BP'/'Stryker'，
          文档里写的是 Barricade/Stryker/Pillbox/Building/Soldier —— 三套写法）
    Q4  get_road_network() 的 scale / world_offset 实测值是多少？
        （现有代码把 2095 / 2637 写死了）

运行前置条件（顺序不能错）：
    1. CarlaUE4 - 快捷方式 启动仿真器
    2. Materials/simu_driver_windows/start.bat  启动飞控驱动（选 1 固定翼 或 2 旋翼）
    3. Materials/practice/practice1_Scene/subject4.exe  生成 3 车 3 机
    4. python WorkPlace/tools/unit_check.py

注意：本脚本必须在装了 swarmae 的 Python 3.6 环境里跑
（environment_forclass/environment/sdk/windows 下的 egg 只支持 py3.6）。
"""

import json
import math
import time

from swarmae.SwarmAEClient import SwarmAEClient

UE_IP = "127.0.0.1"
UE_PORT = 2000

#: click.py 里实测的历史偏移量，用来和新读到的值对比
OLD_OFFSET = (-2094.5277099609375, -2637.31982421875)


def hr(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def main():
    client = SwarmAEClient(ue_ip=UE_IP, ue_port=UE_PORT)
    _, game, _ = client.get_game()

    # ---------------- Q4: 路网参数 ----------------
    hr("Q4  get_road_network() 实测参数")
    img, scale, world_offset, _, code = game.get_road_network()
    print(f"  code         = {code}")
    print(f"  img.size     = {getattr(img, 'size', None)}   img.mode = {getattr(img, 'mode', None)}")
    print(f"  scale        = {scale}")
    print(f"  world_offset = {world_offset}")
    print(f"  历史写死值   = {OLD_OFFSET}")
    if world_offset is not None:
        dx = float(world_offset[0]) - OLD_OFFSET[0]
        dy = float(world_offset[1]) - OLD_OFFSET[1]
        print(f"  与历史值偏差 = ({dx:+.6f}, {dy:+.6f}) 米")
        print("  -> 偏差接近 0 说明 2095/2637 可以继续用；否则必须改共用的 coord.py")

    # ---------------- 注册节点 ----------------
    hr("注册 UGV1 / UAV1")
    ts = int(round(time.time() * 1000))
    _, ugv, code_u = client.register_node(node_type="四轮车", node_name="UGV1",
                                          node_no=1, frame_timestamp=ts)
    ts = int(round(time.time() * 1000))
    _, uav, code_v = client.register_node(node_type="四旋翼", node_name="UAV1",
                                          node_no=4, frame_timestamp=ts)
    print(f"  UGV register code = {code_u}")
    print(f"  UAV register code = {code_v}")
    if ugv is None:
        print("  !! 注册失败：检查仿真器 / start.bat / subject4.exe 是否都已启动")
        return

    # ---------------- Q1: get_location 单位 ----------------
    hr("Q1  get_location() 单位判断")
    lx, ly, lz, _ = ugv.get_location()
    print(f"  UGV1 get_location() = x={lx}  y={ly}  z={lz}")
    print("  判读方法：把这三个值和 practice1_Scene/vehicles.json 里")
    print("            第 1 辆车的 position = '-1829.6, -517.7, 2.1' 对比；")
    print("            若数值接近 -> 单位是米；若相差 100 倍 -> 单位是厘米。")

    # ---------------- Q2/Q3: detect 单位与类型名 ----------------
    hr("Q2/Q3  detect_situation() 原始返回")
    raw = ugv.detect_situation()
    print(f"  返回长度 = {len(raw)}（文档写的是 (t, code, res)，请留意顺序）")
    for i, item in enumerate(raw):
        print(f"  --- 第 {i} 项 ---")
        if isinstance(item, (str, bytes)):
            try:
                print("  " + json.dumps(json.loads(item), ensure_ascii=False)[:400])
            except Exception:
                print(f"  {item!r}"[:400])
        else:
            print(f"  {item!r}"[:400])

    # 如果返回的是 (t, code, res) 三元组，单独解析一次
    if len(raw) == 3 and isinstance(raw[2], (str, bytes)):
        _, dcode, dres = raw
        print(f"\n  按 (t, code, res) 解析: code={dcode}")
        objs = json.loads(dres)
        print(f"  探测到 {len(objs)} 个 Actor")
        names = set()
        for ob in objs:
            names.add(ob.get("Type_Name"))
            pos = ob.get("Position", {})
            print(f"    Type={ob.get('Type')!r:<14} Type_Name={ob.get('Type_Name')!r:<32} "
                  f"Position=({pos.get('X')}, {pos.get('Y')}, {pos.get('Z')})")
        print(f"\n  >> 真实的 Type_Name 集合 = {sorted(n for n in names if n)}")
        print("  >> 请把上面这行抄进 WorkPlace/config/target_map.py 做名字映射表，")
        print("     不要在代码里散落一堆字符串比较。")

        if objs:
            px = objs[0].get("Position", {}).get("X")
            if isinstance(px, (int, float)) and abs(px) > 0:
                print(f"\n  单位交叉验证：第 1 个 Actor 的 Position.X = {px}")
                print(f"    若与 get_location() 同量级 -> 单位一致（都是米）")
                print(f"    若是 get_location() 的 100 倍 -> detect 返回厘米，detect_fun 的 /100 才对")

    # ---------------- 高度 ----------------
    hr("额外：无人机起飞高度基准")
    if uav is not None:
        ux, uy, uz, _ = uav.get_location()
        print(f"  UAV1 get_location() z = {uz}")
        print("  说明：uav_control.py 里用 z=15 作为悬停目标、并判断 z<=14 为未就绪，")
        print("        请确认这个 15 的单位与上面一致，否则无人机会一直等不到 'All UAVs are ready'。")

    print("\n完成。把以上输出截图/复制给组内，作为后续所有坐标计算的口径依据。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断")
