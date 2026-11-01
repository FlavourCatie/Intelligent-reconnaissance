# -*- coding: utf-8 -*-
"""离线冒烟测试：用假 SDK 把 full_mission 的链路跑一遍（不连仿真器）。

验证目标：
  1. 四个模块能一起 import，接口签名对得上
  2. 阶段解锁 → 路网 → 无人机建航线 → UavRunner 逐帧推进
  3. 无人机探测 → 发现目标 → 匹配到 targets.json → 小地图标记为"已发现"
  4. 目标吸附到路网 + 三车分配 → UgvAgent 构造成功并 step 若干帧
  5. 退出清理（悬停 / 刹车）不抛异常
"""
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # 项目根
FAKE = os.path.join(HERE, "fakesdk")
MISSIONS = os.path.join(ROOT, "WorkPlace", "missions")

sys.path.insert(0, FAKE)
os.environ["FAKE_PROJECT_ROOT"] = ROOT

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.pause = lambda *a, **k: None                        # 去掉 non-interactive 警告

sys.path.insert(0, MISSIONS)
sys.path.insert(0, os.path.join(ROOT, "WorkPlace"))
sys.path.insert(0, os.path.join(ROOT, "Materials", "practice", "easy_follow"))

print("=" * 70)
print("1) 模块 import 与接口签名")
print("=" * 70)
import control_uav as cu
import control_ugv as ugv
import uav_runner
import full_mission as fm
import live_track as lt
print("   control_uav   ALTITUDE=%.0f  CRUISE=%.0f  DETECT_EVERY=%d"
      % (cu.ALTITUDE, cu.CRUISE_SPEED, cu.DETECT_EVERY))
print("   control_ugv   UgvAgent=%s  SPAWNS=%d 个"
      % (hasattr(ugv, "UgvAgent"), len(ugv.SPAWNS)))
print("   uav_runner    build_uav_vehicles=%s UavRunner=%s"
      % (hasattr(uav_runner, "build_uav_vehicles"),
         hasattr(uav_runner, "UavRunner")))
print("   资源图：")
for name in ("skeletonized_image.png", "roadnet_raw.png"):
    p = os.path.join(ROOT, "WorkPlace", "assets", name)
    print("     %-24s %s" % (name, "存在" if os.path.isfile(p) else "缺失!"))
    assert os.path.isfile(p), p

print()
print("=" * 70)
print("2) 场景准备（假 SDK）")
print("=" * 70)
from swarmae.SwarmAEClient import SwarmAEClient
client = SwarmAEClient()
_, game, _ = client.get_game()
fm.stage_unlock(game)
print("   阶段序列：", game.stages)

img, scale, off, items = lt.parse_road_network(game.get_road_network())
scene = lt.Scene(img, scale, off)
markers = lt.load_known_markers(os.path.join(ROOT, "Materials", "practice",
                                             "practice1_Scene"))
print("   已知目标 %d 个" % len(markers))

print()
print("=" * 70)
print("3) 无人机：注册 3 架 + 规划航线 + 推进 30 帧")
print("=" * 70)
logger, log_path = cu.make_logger()
scene_dir = os.path.join(ROOT, "Materials", "practice", "practice1_Scene")
entries = cu.scene_uavs(scene_dir)
node_nos = [e[0] for e in entries]
print("   vehicles.json 推出的无人机编号：", node_nos)
assert node_nos == [4, 5, 6], node_nos

uav_vehicles = uav_runner.build_uav_vehicles(client, node_nos, logger)

# 直接放到搜索区上方（跳过从 z=100 爬到 250 的 40 秒；巡航+探测才是要验的部分）
for i, n in enumerate(cu.UAV_NAMES):
    node = uav_vehicles[n].Vehicle_
    node.x, node.y, node.z = 1300.0 + i * 150.0, -700.0, cu.ALTITUDE
print("   已把三机放到搜索区上方：",
      [(n, round(uav_vehicles[n].position()[0], 0),
        round(uav_vehicles[n].position()[1], 0),
        round(uav_vehicles[n].position()[2], 1)) for n in cu.UAV_NAMES])

planner = cu.SafePlanner(cu.read_json(os.path.join(scene_dir, "game.json")))
starts = dict((n, uav_vehicles[n].position()) for n in cu.UAV_NAMES)
routes = cu.search_routes(planner, starts)
print("   航线点数：", dict((n, len(routes[n])) for n in cu.UAV_NAMES))
for n in cu.UAV_NAMES:
    assert len(routes[n]) >= 2, (n, routes[n])

runner = uav_runner.UavRunner(uav_vehicles, planner, routes, logger)
runner.start()
for i in range(600):
    runner.step()
print("   30 帧后 phases =", runner.phases_snapshot())
print("   已发现目标：", [(round(p[0], 1), round(p[1], 1)) for p in runner.enemies()])
enemies = runner.enemies()
assert enemies, "无人机没有发现任何目标（假 SDK 的探测应该能造出来）"

print()
print("=" * 70)
print("4) 把发现结果同步到小地图（空心 → 实心）")
print("=" * 70)
det = [{"x": p[0], "y": p[1]} for p in enemies]
found, extras = lt.match_detections(det, markers, tol=20.0)
print("   匹配已知目标下标 =", sorted(found), " 额外 =", len(extras))
for i in found:
    markers[i].discovered = True
print("   点亮后 discovered =", sum(1 for m in markers if m.discovered))
assert found, "发现的目标没能匹配上 targets.json（说明假数据坐标不在已知目标附近）"

print()
print("=" * 70)
print("5) 目标吸附路网 + 三车分配 + UgvAgent 构造")
print("=" * 70)
binary, raw_bin, ugv_scale, ugv_off = ugv.load_binary(game)
print("   骨架图像素 =", int(binary.sum()), " 真实路宽图像素 =", int(raw_bin.sum()))
assert binary.sum() > 0 and raw_bin.sum() > 0

pts = fm.snap_to_road(enemies, binary, ugv_scale, ugv_off)
print("   吸附到路网后：", [(round(p[0], 1), round(p[1], 1)) for p in pts])
assert pts, "目标吸附失败（附近找不到路网）"

ugv_vehicles = []
import lib.swarm_api as api
for i in range(3):
    ugv_vehicles.append(api.Vehicle(client, "UGV%d" % (i + 1), i + 1, "ugv"))
starts_xy = [(v.get_state().pose_.x_, v.get_state().pose_.y_) for v in ugv_vehicles]
chosen = fm.assign_targets(pts, starts_xy, 3)
print("   分配结果：", [(round(p[0], 1), round(p[1], 1)) for p in chosen])
assert len(chosen) == 3

agents = []
for i in range(3):
    a = ugv.UgvAgent(ugv_vehicles[i], "UGV%d" % (i + 1), chosen[i],
                     binary, raw_bin, ugv_scale, ugv_off)
    a.start()
    agents.append(a)
print("   3 个 agent 已启动，状态：", [a.status for a in agents])

for k in range(20):
    for a in agents:
        a.step()
print("   推进 20 帧后状态：", [(a.name, a.status) for a in agents])

print()
print("=" * 70)
print("6) 小地图（无窗口，只验证 update 不炸）")
print("=" * 70)
names = ["UGV1", "UGV2", "UGV3", "UAV1", "UAV2", "UAV3"]
types = dict([(n, "ugv") for n in names[:3]] + [(n, "uav") for n in names[3:]])
view = lt.LiveView(scene, None, names, types, zoom=300.0, focus="UAV1")
view.set_markers(markers)
view.set_goals({"UGV1": chosen[0], "UAV1": (routes["UAV1"][0][0], routes["UAV1"][0][1])},
               {"UGV1": "file", "UAV1": "nearest"})
snap = {}
for n, v in zip(names[:3], ugv_vehicles):
    snap[n] = fm.pose_of(n, "ugv", v.Vehicle_)
for n in names[3:]:
    snap[n] = fm.pose_of(n, "uav", uav_vehicles[n].Vehicle_)
view.update(snap, 1.0, "smoke test")
out = os.path.join(HERE, "full_mission_smoke.png")
view.fig.savefig(out, dpi=70)
print("   小地图截图：", out)
view.close()

runner.stop()
for v in ugv_vehicles:
    v.apply_control(0, 0, 1.0, False)

print()
print("=" * 70)
print("全部通过：链路各环节接口对得上")
print("=" * 70)
print("无人机日志：", log_path)
