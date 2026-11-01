# -*- coding: utf-8 -*-
"""回归测试：验证 SDK 解析 sys.argv 的坑已被 make_client 规避。

模拟真机行为：SwarmAEClient.__init__ 用 argparse 解析整个 sys.argv（只认 -i/-p/-s/-n/-t）。
如果我们的脚本带了自己的参数，直接构造就会 SystemExit —— 这正是
`--uav-mode kinematic` 报 "unrecognized arguments" 的原因。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HERE, "fakesdk"))
os.environ["FAKE_PROJECT_ROOT"] = ROOT
os.environ["FAKE_SDK_PARSE_ARGV"] = "1"          # 打开"真机行为"模拟

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.pause = lambda *a, **k: None

sys.path.insert(0, os.path.join(ROOT, "WorkPlace"))
sys.path.insert(0, os.path.join(ROOT, "WorkPlace", "missions"))
sys.path.insert(0, os.path.join(ROOT, "Materials", "practice", "easy_follow"))

from swarmae.SwarmAEClient import SwarmAEClient
from core.sdk_boot import make_client

print("=" * 70)
print("1) 直接构造（复现用户的报错）")
print("=" * 70)
sys.argv = ["full_mission.py", "--uav-mode", "kinematic"]
print("   sys.argv =", sys.argv)
try:
    c = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
    print("   !! 居然没报错（说明假 SDK 没模拟到位）")
    reproduced = False
except SystemExit as e:
    print("   如预期被 SDK 的 argparse 拒掉，退出码 =", e.code)
    reproduced = True
assert reproduced, "没能复现问题"

print()
print("=" * 70)
print("2) 通过 make_client 构造（应正常）")
print("=" * 70)
argv_before = list(sys.argv)
client = make_client("127.0.0.1", 2000)
print("   make_client 成功:", client)
print("   构造后 sys.argv 是否还原:", sys.argv == argv_before, sys.argv)
assert sys.argv == argv_before, "argv 没有还原"

print()
print("=" * 70)
print("3) 带参数跑完整的 full_mission.main()（运动学模式）")
print("=" * 70)
import full_mission as fm
fm.cu.CRUISE_SPEED = 150.0
sys.argv = ["full_mission.py", "--no-view", "--no-ugv", "--uav-mode", "kinematic",
            "--duration", "4", "--recon-timeout", "2"]
print("   sys.argv =", sys.argv[1:])
fm.main()
print("\n全部通过：带自定义参数时不再被 SDK 的 argparse 拦截。")
