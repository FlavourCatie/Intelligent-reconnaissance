# -*- coding: utf-8 -*-
"""跑真正的 full_mission.main()（假 SDK，无窗口）。"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HERE, "fakesdk"))
os.environ["FAKE_PROJECT_ROOT"] = ROOT

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.pause = lambda *a, **k: None

sys.path.insert(0, os.path.join(ROOT, "WorkPlace", "missions"))
sys.path.insert(0, os.path.join(ROOT, "WorkPlace"))
sys.path.insert(0, os.path.join(ROOT, "Materials", "practice", "easy_follow"))

mode = sys.argv[1] if len(sys.argv) > 1 else "uav-only"
import full_mission as fm

if mode == "uav-only":
    sys.argv = ["full_mission.py", "--no-view", "--no-ugv",
                "--duration", "6", "--recon-timeout", "3"]
elif mode == "with-ugv":
    sys.argv = ["full_mission.py", "--no-view", "--use-default-enemies",
                "--duration", "25", "--recon-timeout", "2", "--min-targets", "3"]
elif mode == "with-view":
    sys.argv = ["full_mission.py", "--use-default-enemies",
                "--duration", "12", "--recon-timeout", "2", "--min-targets", "2",
                "--view-every", "2"]
elif mode == "virtual":
    fm.cu.CRUISE_SPEED = 150.0
    sys.argv = ["full_mission.py", "--no-view", "--uav-mode", "virtual",
                "--duration", "40", "--recon-timeout", "30", "--min-targets", "3"]
elif mode == "kinematic":
    # 运动学模式：把巡航速度临时调大，让测试在几十秒内飞到搜索区
    fm.cu.CRUISE_SPEED = 150.0
    sys.argv = ["full_mission.py", "--no-view", "--uav-mode", "kinematic",
                "--duration", "60", "--recon-timeout", "40", "--min-targets", "3"]
elif mode == "sweep":
    fm.cu.CRUISE_SPEED = 150.0
    sys.argv = ["full_mission.py", "--uav-mode", "virtual", "--uav-route", "sweep", "--sweep-region", "auto",
                "--no-view", "--duration", "60", "--recon-timeout", "50"]
elif mode == "virtual-view":
    fm.cu.CRUISE_SPEED = 60.0
    sys.argv = ["full_mission.py", "--uav-mode", "virtual", "--view-roi", "auto", "--view-every", "4",
                "--duration", "40", "--recon-timeout", "30", "--min-targets", "3"]
elif mode == "kinematic-view":
    # 关键：带小地图，覆盖显示层（last_goals / set_goals 那条路径）
    fm.cu.CRUISE_SPEED = 50.0
    sys.argv = ["full_mission.py", "--uav-mode", "kinematic",
                "--duration", "70", "--recon-timeout", "60", "--min-targets", "3",
                "--view-every", "5"]
else:
    raise SystemExit("unknown mode")

print("### 模式:", mode, " argv:", sys.argv[1:])
fm.main()
print("### main() 正常返回")
