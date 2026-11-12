# 智能无人系统技术 —— 仿真框架说明（README）
# 项目名称-Intelligent-reconnaissance

> **这份文档是给组内成员的"入门地图"。**
>
> 这一份Readme主要是介绍了课程给出的代码环境已经完成的部分，简单指明我们需要做的内容
> 更细的逐文件代码解读、bug 清单、改造点定位，见 **[`docs/代码导读与开发指南.md`](docs/代码导读与开发指南.md)**。
> 可以把这份Readme当做介绍，指南作为开发的辅助内容

---
## 写在最前：组织架构与分工

**目前的分工按照表格进行**

| 姓名 | github账号 | 分工 |
|---|---|---|
| 张芮萌 | littleZ05 | "产品经理" 负责需求的调研，整体把关，参与代码的编写以及审核环节。
| 陈庚泉 | FlavourCatie | "技术负责人" 负责无人车、无人机两部分通信对接，以及寻路算法的改进工作，以及场景的生成算法，划分算法等。同时与葛伊诺、吴羽桐共同协商修改无人车、无人机内部组织代码。
| 葛伊诺 | geyinuo | "技术人员" 主要负责无人机部分内部代码的编写与bug修复，重点是三架无人机（或更多）如何写作完成侦查任务，以及如何将信息传递到无人车。
| 吴羽桐 | shimmer-ywt | "技术人员" 主要负责无人车部分内部代码的编写与bug修复，重点是三辆无人车（或更多）如何根据算法自动识别障碍物进行打击，根据无人机的信息进行布阵。
| 毛悦琮 | maoyuecong682 | "产品助理" 协助产品经理张芮萌完成需求端调查，以及结果的评审。

> 目前的分工是基于现有的调查进行，后续可能根据熟悉领域不同调整。

**整个代码编写以及提交的流程**

```
根据任务 Issue 在自己的分支 branch 完成编写
   │
   ▼
将自己的分支通过 push request 向 main分支提交
   │                                         
   ▼
代码审核员进行审核，决定并入main分支还是打回
```

> 五人均可编写代码，其中代码的审核部分由张芮萌、毛悦琮、陈庚泉进行，至少需要通过 1 人的审核代码才可以并入 main 分支（暂定） 

---

## 1. 课程资料包情况

**底层能力已经齐全，缺的是"任务编排层"。换句话说，功能算法部分不需要大改，需要考虑怎么安排功能任务调用算法。**

```
仿真器      CarlaUE4 + simu_driver_windows + practice1_Scene\subject4.exe
   │
   ▼
接口层      swarmae SDK —— SwarmAEClient / SwarmAENode / SwarmAEGame / SwarmAEWorld
   │                                          （官方提供，文档已转存到 WorkPlace/参考/）
   ▼
封装层      lib/swarm_api.py 里的 Vehicle 类   ← 齐全，可以直接用
   │          移动 · 开火 · 探测 · 取状态，各是一两个函数调用
   ▼
算法层      A* 规划 · Stanley/PID · 队形点计算 · 骨架吸附   ← 有，能跑，但是可以进一步改进
   │
   ▼
需求层      侦察 · 编队 · 合围打击 · 通信中继   ← 基本空白，我们要写的
              什么时候、派谁、去哪、打哪个
```

**不需要重造轮子，需要考虑如何按照合理顺序逻辑调用接口**
开火、移动、PID 控制、探测这些，**现在就已经是一两个函数调用的事**，不需要从零做。

---

## 2. 整体架构与目录归属

| 层 | 目录 | 状态 | 谁维护 |
|---|---|---|---|
| 仿真器 / 环境 | `environment_forclass/` | 可用，**不要改** | 课程提供 |
| 官方接口文档 | `environment_forclass/environment/说明/sdk接口说明-V1.4-20230817.docx` | — | 课程提供 |
| 教程（五讲） | `Materials/Autonomous_Vehicle_PnC_Tutorial-master/` | 只读参考 | 课程提供 |
| **封装 + 算法 + 跑通示例** | **`Materials/practice/easy_follow/`** | **能跑，重点改造区** | **我们** |
| 场景配置 | `Materials/practice/practice1_Scene/` | 可改 | 我们 |
| 我们自己的代码 | `WorkPlace/` | 待建 | 我们 |

> 约定：`Materials/` 和 `environment_forclass/` **只读不改**。我们要复用 `easy_follow/lib/*` 时用
> `sys.path.append(...)` 引用，不在 `WorkPlace` 里复制一份，避免"改了一处另一处没改"。

---

## 3. 已经能直接调用的能力

全部在 `Materials/practice/easy_follow/lib/swarm_api.py` 里，核心是一个 `Vehicle` 类，**用同一套接口同时管理无人机和无人车**，内部用 `_check_type()` 防止你调错（调错会抛 `PermissionError`）。

### 3.1 能力总表

| 能力 | 方法 | 入参 | 返回 | 谁能用 |
|---|---|---|---|---|
| 建节点 | `api.Vehicle(client, name, no, type)` | `type="ugv"/"uav"` | Vehicle 对象 | 通用 |
| 取位姿 | `get_transform()` | — | `Transform(x, y, theta[弧度], z)` | 通用 |
| 取状态 | `get_state()` | — | `State(pose, v, a, …)` | 通用 |
| 取 IMU | `get_imu_data()` | — | `ImuData(vx…r)` | 通用 |
| **车：行驶控制** | `apply_control(steer, throttle, brake, hand_brake)` | 各自 `[-1,1]` / `[0,1]` | 状态码 | **仅 ugv** |
| **机：飞行控制** | `target_point_control(x, y, z, v)` | 目标点(米) + 地速 | — | **仅 uav** |
| **探测** | `detect_fun()` | — | `DetStruct(Type, Health, x, y, z, position, yaw, x_ue, y_ue)` | 通用 |
| **开火** | `set_weapon(ob, fight_num)` | 探测结果 + 第几个目标 | — | 仅 ugv |

### 3.2 一个必须记住的坑：`apply_control` 的参数顺序

```python
# 封装层：steer 在第一位（注意！）
ugv.apply_control(steer, throttle, brake, hand_brake)

# 底层 SDK：throttle 在第一位（顺序相反）
node.apply_vehicle_control(throttle, steer, brake, hand_brake, gear)
```

**写反了不会报错，只会看到车"该转的时候加速"**。这是新人最常踩的一脚。

### 3.3 车和机的控制逻辑**本质不同**（设计时要心里有数）

| | 无人车 (ugv) | 无人机 (uav) |
|---|---|---|
| 控制方式 | **闭环**：给油门/转向/刹车 | **开环**：给目标点 + 地速，底层自动飞过去 |
| 你要写的算法 | Stanley 横向 + PID 纵向 | **航点序列 + 到达判定**，不需要低层控制器 |
| 状态量 | `v, a, yaw` 都可靠 | 主要用 `x, y, z`，姿态精度有限 |

所以：**不要试图给无人机写"速度闭环控制器"**——SDK 不给你这个接口。无人机的算法重点是"下一个航点选哪里"。

### 3.4 最小可运行示例

```python
import time
from swarmae.SwarmAEClient import SwarmAEClient

client = SwarmAEClient(ue_ip="127.0.0.1", ue_port=2000)
_, node, code = client.register_node(
    node_type="四轮车", node_name="UGV1", node_no=1,
    frame_timestamp=int(round(time.time() * 1000)))
print("注册结果:", code)          # 200 = 成功；50X/40X = 失败/超时

node.apply_vehicle_control(0.5, 0.0, 0.0, False, 0)   # 前进
time.sleep(3)
node.apply_vehicle_control(0.0, 0.0, 1.0, True, 0)    # 刹停
```

用封装层则更短：

```python
import lib.swarm_api as api
ugv = api.Vehicle(client, vehicle_name="UGV1", vehicle_no=1, vehicle_type="ugv")
ugv.apply_control(steer=0, throttle=0.5, brake=0, hand_brake=False)
```

### 3.5 算法层也已经有现成的（不必重写）

| 能力 | 位置 | 说明 |
|---|---|---|
| A\* 全局规划 | `lib/global_planner.py` `AStar(start_x,start_y,end_x,end_y,map_img)` | 输入骨架图，输出像素路径点 |
| 参考路径（样条/航向/曲率） | `lib/frenet_optimal_planner.py` `ReferencePath(x, y, resolution)` | 把稀疏路径点加密成可跟踪曲线 |
| 骨架化 + 吸附路网 | `lib/image_process.py` `skeleton_refinement()` / `skeleton_point()` | **把任意点吸附到最近道路**，合围/中继选点都要用 |
| 横向 Stanley | `lib/controller.py` `Stanley` | 有**两套**实现，别混用 |
| 纵向 PID | `lib/controller.py` `PID(Kp,Ki,Kd)` | — |
| **队形** | `lib/formation_calculate.py` `FormationToolkit` | **横向/纵向一字、正多边形（→V字）、沿路径偏移、车距保持**，功能最全但**尚未接入主流程** |
| 坐标换算 | `WorkPlace/tools/coord.py` | 世界↔像素，已自检通过 |

---

## 4. `practice` 里已经跑通的部分（**重点改造区**）

`Materials/practice/easy_follow/` 是**唯一"能完整跑下来"的示例集合**，也是我们要改的地方。共 4 个可执行脚本：

| 脚本 | 跑通到什么程度 | 可以直接抄什么 | 改造入口 |
|---|---|---|---|
| `plan.py`（161 行） | 取图 → 骨架 → 注册 3 车 3 机 → A\* → 样条 → **探测一次 + 连打 5 发** | 坐标换算、骨架吸附、A\*、探测+开火调用姿势 | 第 123 行终点写死、153-158 行开火循环 |
| `plan_control.py`（298 行） | **三车跟驰的完整闭环**（Stanley+PID+车距保持），有实时绘图 | 主循环骨架、控制器整定、绘图 | 第 224 行"三车同线"、第 270 行一个变量名 bug |
| `uav_control.py`（139 行） | 三机起飞悬停 + 相对位移（前 112 行是 `plan.py` 复制粘贴，真正逻辑只有 113-139 行） | 无人机"给点飞"的姿势、高度等待逻辑 | 整个 113-139 段，需要重写成航点飞行 |
| `click.py`（31 行） | 鼠标点图查像素/世界坐标 | 工具，直接拿来用 | 不用改，但里面偏移量写死 |

### 建议的上手验证顺序（每步都要"看到现象"才算通）

| 步 | 跑什么 | **你应该看到** | 不通时查 |
|---|---|---|---|
| 1 | `environment_forclass/environment/scripts_test/manual_control.py` | 方向键能让车前进后退 | Python 3.6 环境 / egg 是否装上 |
| 2 | `practice1_Scene/subject4.exe` + `plan.py` | 弹出 matplotlib 图，红色 A\* 路径叠在路网骨架上 | `swarmae` 导入、端口 2000、A\* 太慢（可降采样） |
| 3 | `plan_control.py` | 三台车沿路行驶，图上有橙色车身和轨迹 | 第 270 行的 `throttle`/`throttle_1` bug |
| 4 | `uav_control.py` | 三机升到同一高度，然后各自 ±25 m 位移 | `start.bat` 选的机型和 `z=15` 高度基准 |

### 跑通之后，改造的优先级

1. **修 `plan_control.py` 的 `throttle_1/brake_1`（第 270 行）** —— 不修会 NameError 崩。
2. **把"三车同线"换成"三条偏移线"** —— 用 `FormationToolkit.offset_path()`，这是"列队"的最小可用版本。
3. **改 `plan.py` 的终点** —— 现在的 `(-1500, -217)` 离起点只有 ~295 m，是教程的玩具终点，要换成敌方目标区。
4. **把 `stage_start/stage_complete` 从"连调 4 次"改成真实分期** —— 演示的节奏感就靠它。

---

## 5. 后续会用到的重要接口（**大部分尚未封装**）

这是本文的重点之一：**SDK 里能做的事，比现在代码用到的多得多。** 下面按任务分组，说明"以后做 XX 时你会用到它"。

> **验证状态说明**：下表除 `get_road_network()`、`get_weapon_ammo()`（已封装）和
> `stage_start/stage_complete`（`plan.py` 里连调 4 次走形式）之外，**其余在 `Materials/` 全部代码里一次都没有出现过**
> （已 grep 确认）。也就是说它们属于"**文档写了、但没人在真机上验证过**"。
> 用之前先在 `WorkPlace/tools/unit_check.py` 里加一段试调，别直接写进主流程。

### 5.1 侦察任务会用到的

| 接口 | 归属 | 作用 | 备注 |
|---|---|---|---|
| `get_game().get_task_info()` | Game | 拿任务区/禁飞区/阶段线的**结构化坐标**（arena、emc_arena、mine_arena、subject_1..6 的起止线） | 比读 `game.json` 更"官方" |
| `get_game().submit_plane_route(routes)` | Game | **阶段 1 一次性提交所有无人机的航线** | 格式 `[{"node_id":"1","route":[{"x":..,"y":..,"z":..}]}]`；多次提交会**覆盖** |
| `get_game().get_key_points()` | Game | 官方巡逻关键点 | 可当侦察航点的骨架 |
| `get_game().get_road_network()` | Game | 路网图 + `scale` + `world_offset` | 已在用，但数值被写死在别处 |
| `get_world().get_map()` | World | 高精地图 / 栅格地图 | 想做更精细的路径约束时用 |

> **这两点很关键**：`submit_plane_route` 和 `get_key_points` 是官方为"侦察阶段"设计的入口，而我们现在是逐点 `target_point_control` 硬飞——**用官方航线接口更贴合评测逻辑**，值得在演示里体现。

### 5.2 通信中继任务会用到的（**目前完全空白**）

| 接口 | 归属 | 作用 |
|---|---|---|
| `create_channel(node_ids, chann_type)` | `SwarmAEConnect` | 建立节点间通信通道，返回 `SwarmAEChannel`（点对点）或 `SwarmAEBroadcast`（广播） |
| `channel.send_message(node_id, msg_data, ts, topic)` | `SwarmAEChannel` | 发协同消息 |
| `channel.receive_message(node_id, ts, topic)` | `SwarmAEChannel` | 收协同消息 |
| `channel.get_channel_nodes(node_id)` | `SwarmAEChannel` | **查当前哪些节点可以和它通信** ← 判断"链路通不通"的直接依据 |
| `broadcast.send_message / Receive_message` | `SwarmAEBroadcast` | 广播收发 |
| `node.send_msg(msg)` / `node.receive_msg()` | `SwarmAENode` | 节点级简易收发（另一套，能力较弱） |
| `release_channel(channel)` | `SwarmAEConnect` | 回收通道 |

> **两处未确认**（要先验证再依赖）：
> ① `SwarmAEConnect` 对象**怎么拿到**——文档没写构造方式，需要实测（可能是 `client.get_connect()` 之类）；
> ② `SwarmAENode.send_msg` 和 `SwarmAEChannel.send_message` 是**两套并行**的通信机制，语义是否等价未知。
>
> 另外：文档只说 `detect_situation()` 返回"半径内的 Actor"，**没有视线/遮挡参数**。所以"被建筑遮挡"必须我们自己建模（用 `roadnet.png` 当建筑掩膜做连线采样）。

### 5.3 合围打击任务会用到的

| 接口 | 归属 | 作用 | 现状 |
|---|---|---|---|
| `weapon.get_weapon_ammo()` | Weapon | 查弹药余量/是否可用 | 已被 `set_weapon` 内部调用 |
| `weapon.set_weapon_status(...)` | Weapon | **实际开火**（按弹种+坐标+数量） | 已封装成 `set_weapon` |
| `weapon.get_weapon_fire()` | Weapon | 是否已开火（**做反馈闭环用**） | 🔜 未用 |
| `weapon.set_weapon_angle(barrel_angle, barrel_Altitude)` | Weapon | 手动调炮管角度 | 🔜 未用（`set_weapon` 会自动瞄） |
| `weapon.get_weapon_energy()` | Weapon | 剩余能量 | 🔜 未用 |
| `node.missile_fire(target_id)` | Node | 按目标 ID 发射导弹 | 🔜 未用，语义与 `set_weapon` 的关系待确认 |
| `node.get_damage()` | Node | 自己是否中弹/损伤 | 🔜 未用（做"被打"演示用） |
| `node.get_health()` | Node | 自己血量 | 🔜 未用 |

> **弹种**（文档）：`0`-子弹 95 发（伤害 20）、`1`-40 炮 15 发（伤害 1600）、`2`-反坦克炮 2 发（伤害 3000）。
> 但 `set_weapon` 里的实际调用第一参数是 `3/4/6`，与文档列的 `0/1/2` **对不上**——说明真实参数含义与文档表格不同。**以现有能跑的写法为准**，先别自己改数字。

### 5.4 环境约束 / 状态查询

| 接口 | 作用 | 什么时候用 |
|---|---|---|
| `world.get_zone(node_id)` | 节点周围区域（风场/林场/隔离带） | 想做"环境约束"加分项 |
| `world.get_interference(node_id)` | 周围干扰类型/位置/范围/强度 | **电磁干扰车**任务相关 |
| `world.get_light()` / `get_weather()` | 光照 / 云雨风雾 | 想做"夜间/恶劣天气"演示 |
| `node.get_sim_status()` | 仿真状态（正常/中断/完成待命） | **做全流程状态机时的"结束条件"** |
| `node.get_vehicle_status()` | 当前油门/转向/档位/刹车 | 调试控制器时对照 |
| `node.control_emc(bool)` / `set_emc_angle(angle)` | 电磁干扰开关 / 扇面角度 | 只有 `vehicle.auto.electron` 车支持 |

### 5.5 阶段控制（**演示的节奏全靠它**）

`get_game().stage_start(...)` / `stage_complete(...)`，共 7 组：

| 阶段 | 开始 / 结束 | 对应任务 |
|---|---|---|
| 侦察 | `reconnaissance_start` / `reconnaissance_end` | 侦察 |
| 无人机先遣侦察 | `vau_reconnaissance_start` / `vau_reconnaissance_end` | 侦察 |
| 局部路径规划 | `plan_start` / `plan_end` | 机动 |
| 扫雷 | `minesweeper_start` / `minesweeper_end` | 旁支 |
| 电磁干扰 | `emi_start` / `emi_end` | 旁支 |
| 侦打 | `hit_start` / `hit_end` | 打击 |
| 巡逻 | `patrol_start` / `patrol_end` | 旁支 |

---

## 6. 改之前必须知道的几个坑（摘要）

详细说明和行号见 [`docs/代码导读与开发指南.md`](docs/代码导读与开发指南.md)。

| # | 坑 | 影响 |
|---|---|---|
| 1 | `apply_control` 是 **steer 在前**，底层是 throttle 在前 | 写反了不报错，车会乱走 |
| 2 | `plan_control.py` 第 270 行 `throttle`/`brake` 漏了下标 | 首次触发 `accel_1<=0` 就 NameError |
| 3 | `plan_control.py` 三车用**同一条参考线** | 车会叠在一起，"列队"实际没实现 |
| 4 | `detect_fun()` 里 `Position.X / 100`，但 `get_location()` 结果按米用 | **单位很可能不一致**，用前先实测 |
| 5 | 坐标偏移 `+2095 / +2637` 写死在 `detect_fun` 和 `click.py` | 换地图就全错 |
| 6 | `skeleton_refinement()` 每次往**当前目录**写 ~68 MB 的 tga | 一天能塞满几个 G |
| 7 | `lib/udp.py` 依赖 `proto/` 包，但仓库里**没有** | 那套任务下发接口在 Windows 下走不通 |
| 8 | 无人机 `z=15` 悬停高度是写死的 | 侦察高度需求一变就得改 |
| 9 | `lib/init.py` 用 `gnome-terminal` | Linux 专用，Windows 下废 |
| 10 | 敌方 Stryker 距我方起点仅 **17 m** | 开局就贴脸，确认它会不会先开火 |

---

## 7. 目录与协作约定

```
WorkPlace/
├─ README.md                  ← 本文件：框架说明，先看这个
├─ 整合说明_完整链路.md         ★ 组员代码整合后的用法、改动、风险（先看这个）
├─ 组员开发说明_移动控制.md     给写移动逻辑的同学：任务边界 / 接口 / 注意事项
├─ 代码导读与开发指南.md        逐文件解读 + bug 清单 + 改造点定位
├─ 参考/
│   ├─ sdk接口说明_V1.4.txt       官方接口文档（转成文本，好检索）
│   └─ 课程任务说明_原文.txt      两个 PPT 的全文（4 个任务的要求原文）
├─ demo_midterm.py            ★ 答辩一键演示
├─ tools/
│   ├─ coord.py               坐标换算 + 自检
│   ├─ scene_map.py           场景叠加图 + 目标坐标表
│   ├─ spawn_targets.py       生成敌人（环带 / 避禁飞区 / 路侧偏移）
│   ├─ reset_scene.py         重置侦察（杀进程 / 还原 / 重启）
│   ├─ kill_all.py            一键清场 + 进程/端口体检
│   ├─ net_check.py           端口与链路检查
│   ├─ uav_diag.py            ★ 无人机链路一体诊断
│   ├─ draw_arch.py           生成四层架构图
│   ├─ manual_drive.py        手动驾驶测试（方向键开车 + 多车切换）
│   └─ unit_check.py          上机前核对：单位 / Type_Name / offset
├─ core/
│   ├─ fleet.py               接口层：位姿查询 / 分车目的地 / 规划控制
│   └─ scenefile.py           数据层：targets.json / game.json 读写
├─ missions/
│   ├─ full_mission.py        ★ 一条命令跑完整链路（侦查→打击+小地图），详见 §12
│   ├─ control_uav.py         无人机侦查（组员交付）
│   ├─ control_ugv.py         无人车打击（组员交付）
│   ├─ uav_runner.py          把 UAV 阻塞循环改成可单步驱动（适配层）
│   └─ live_track.py          战场态势显示（纯显示，不控制；详见 §9）
├─ assets/
│   ├─ skeletonized_image.png 1px 骨架路网（无人车 A* 用）
│   └─ roadnet_raw.png        真实填充路网（量路宽用）
├─ config/
│   ├─ goals.json             目标位置接口：其他组员改这里，图上星标立即更新
│   └─ detected.json          探测结果接口：无人机那边写"发现了哪些目标"
├─ selftest/                  离线冒烟测试（假 SDK，不需要仿真器）
└─ figs/                      成果图 + 俯视图 + 轨迹 csv
```

**四条约定：**

1. `Materials/` 和 `environment_forclass/` **只读不改**；复用 `lib/*` 用 `sys.path.append(...)`。
2. **所有坐标换算走 `tools/coord.py`**，业务代码里不要再出现 `+2095` 这类魔数。
3. **参数进 `config/`，不写死在算法里**（终点、R、队形、高度、弹药策略）。
4. 新增能力先写进 `WorkPlace/`，验证通过再考虑替换 `practice` 里的示例。

## 8. Git 仓库协作（GitHub）

仓库根目录已放好 `.gitignore` 和 `.gitattributes`。**结论：只提交约 1.3 MB / 41 个文件，不需要 Git LFS。**

### 8.1 进仓库 / 不进仓库

| | 内容 | 体积 | 为什么 |
|---|---|---|---|
| ✅ 进 | `WorkPlace/**` | 0.3 MB | 我们写的代码、文档、工具、成果图 |
| ✅ 进 | `Materials/practice/easy_follow/**`（除 `skeletonized_image.*`） | 1.0 MB | **要改的代码**，必须版本控制 |
| ✅ 进 | `Materials/practice/*_Scene/*.json` | 24 KB | 场景配置，我们会改（放敌人、调起点） |
| ✅ 进 | `.gitignore` / `.gitattributes` | 5 KB | — |
| ❌ 不进 | `environment_forclass/` | **10,021 MB** | 课程发的 UE4 环境 + SDK，含 5 个 >100 MB 的文件 |
| ❌ 不进 | `Materials/simu_driver_windows/` | 124 MB | 课程发的飞控驱动二进制 |
| ❌ 不进 | `Materials/Autonomous_Vehicle_PnC_Tutorial-master/` | 55 MB | 五讲教程，课程原样提供 |
| ❌ 不进 | `Materials/practice/*/subject4.exe` | 16 MB | 场景生成器，改 json 就够 |
| ❌ 不进 | `skeletonized_image.tga` | **66 MB** | **运行时生成物**，每次跑都重写 |
| ❌ 不进 | 根目录 PDF / PPTX / `.lnk` | 12 MB | 课程文档；`.lnk` 存的是本机绝对路径 |

> ⚠️ **`skeletonized_image.tga` 是最大的雷**：66 MB，由 `image_process.skeleton_refinement()` 在每次运行时写到你**当前工作目录**。
> 只要有人手抖提交过一次，它就永久留在 git 历史里（清历史得用 `git filter-repo`）。**先建好 `.gitignore`，再 `git add`。**

### 8.2 为什么不需要 Git LFS

体积问题已经用 `.gitignore` 解决了：仓库只有 1.3 MB，单文件最大 800 KB（`map.jpg`）。
GitHub 的仓库建议上限是 1 GB，1.3 MB 完全不在一个量级。

**什么时候才需要 LFS**（届时再加，不要提前加）：

* 想把**演示录屏**（mp4/gif，几十 MB）放进仓库 → 优先用 **GitHub Releases** 挂附件 + README 放链接，比 LFS 更省额度；
* 决定把 `subject4.exe` 或 SDK egg 也纳入版本管理；
* 存 PCD 点云 / rosbag 这类单文件几十 MB 的数据。

> GitHub 免费 LFS 额度是 **1 GB 存储 + 1 GB/月流量**，且成员必须先装 Git LFS 才能正常克隆——对课程项目是纯负担。
> 另外：**任何单文件超过 100 MB 会被 GitHub 直接拒绝 push**，而 `environment_forclass/` 里有 5 个这样的文件
> （最大 `CarlaUE4.pdb` 267 MB），所以那个目录必须排干净。

### 8.3 新成员首次拉取（**顺序很重要**）

课程材料不进仓库，所以克隆之后要手动补齐。**先克隆、再补料**，别反过来：

```powershell
git clone <仓库地址> Intelligent-Sensor        # 1. 先克隆，得到 WorkPlace + easy_follow
cd Intelligent-Sensor

# 2. 再从课程包里复制"不进仓库"的那几样进来（只复制这些）：
#    environment_forclass/
#    Materials/simu_driver_windows/
#    Materials/Autonomous_Vehicle_PnC_Tutorial-master/
#    Materials/practice/practice1_Scene/subject4.exe
#    Materials/practice/practice2_Scene/subject4.exe
```

补完后 `python WorkPlace\tools\scene_map.py` 应能直接跑——它只依赖 `roadnet.png` 和 scene json，这两样都在库里。

### 8.4 日常流程建议

```powershell
git switch -c feat/encircle        # 从 main 切分支，一人一分支
# ... 改代码 ...
git add -A
git commit -m "feat(encircle): 合围选点 + 三角形内点判定"
git push -u origin feat/encircle   # 然后去 GitHub 开 PR
```

* **`main` 永远保持"能跑"**：只合并走过 PR 的代码，避免演示前一天 main 起不来。
* commit 前缀统一：`feat:` 新功能 / `fix:` 修 bug / `docs:` 文档 / `chore:` 杂项。
* **不要提交本机绝对路径**：如遇 `utils.py`、`init.py` 里的固定路径，先改成相对路径或从 `config/` 读。

### 8.5 两个 Windows 上的小设置

```powershell
git config --global core.quotepath false   # 让 git status 正常显示中文文件名
git config --global core.autocrlf false    # 换行符交给 .gitattributes 管，别让 git 再插一手
```

---

## 9. 战场态势显示 `missions/live_track.py`（**纯显示层**）

> ⚠️ **定位已变更**：这个脚本现在**只负责看**——不跑 A\*、不规划路径、不发任何控制指令。
> **移动由其他组员的任务脚本负责**；规划/控制能力留在 `core/fleet.py` 里供他们调用。
>
> 附带好处：不再做骨架细化，**不会再往当前目录写那个 67 MB 的 `skeletonized_image.tga`**。

### 9.1 它显示什么

| 图层 | 内容 | 数据来源 | 样式 |
|---|---|---|---|
| **环境** | 任务区、**禁飞区**（3 个）、雷区、电磁区、（可选）阶段线 | `game.json` | 蓝框 / 红色斜纹 / 褐 / 紫 |
| **平台** | 6 台的位置、朝向、速度、**实际走过的轨迹** | 实时 `get_location()` / `get_attitude()` / `get_velocity()` | 车=方块、机=三角；在动亮色+箭头，停住变灰 |
| **敌人** | 全部按 `targets.json` 画出，**初始空心**；被探测到后变**实心** | `targets.json` + `detect_situation()` | 空心红圈 → 实心红点；未登记的画橙色 X |
| **目标** | 每台平台一个星标 + 到目标的虚线 | `config/goals.json`（人工）／最近的敌人（默认） | 实心星=人工指定，空心星=默认推断 |
| **无人机视角** | 俯视截图，定时落盘 | 真机相机或合成 | `figs/uav_view/` |

**不画 A\* 参考线**——图上的线只有两条：平台的**实际轨迹**，和平台到目标的**虚线**。

### 9.2 敌人「空心 → 实心」的判定

```
启动   从 targets.json 读全部敌人 → 全部 discovered=False → 空心红圈
运行   每 --detect-every 帧（默认10帧≈0.5s）调用一次 detect_situation()
       把探测到的坐标，和已知列表逐个比距离
         ├─ 距离 < --detect-tol（默认 20 m）→ 认为"就是它" → 该目标变实心
         └─ 都对不上                      → 说明是不在配置里的目标 → 画橙色 X
```

HUD 左上角实时显示 **已发现 N / 总数**，发现新目标时日志会打印一行：

```
[live_track] 发现目标 #8 路障（旁挂士兵）(1535.5, -889.3)
```

探测源默认只用**无人机**（`--detect-from uav`），可改成 `ugv` 或 `all`。

**探测半径不是脚本设的**，来自 `vehicles.json` 里每台平台的 `detectRadius`
（无人机 30000 / 无人车 10000，单位待实测确认）。想改模型的话有专门的接口：

```json
// WorkPlace/config/detected.json —— 无人机那边把"发现了哪些目标"写这里
{
    "points":     [{"x": 1535.5, "y": -889.3}],   // 按坐标匹配（推荐）
    "discovered": [8, 10],                        // 或按下标
    "extra":      [{"x": 900, "y": -700}],        // 不在 targets.json 里的
    "__清空__":   false                            // true = 全部恢复空心
}
```

保存即生效。这样负责无人机的同学可以自己实现视场角 / 遮挡 / 延迟 / 概率等模型，
本脚本只负责按结果上色。**用外部结果时建议同时加 `--no-detect`**，否则会和 SDK 的半径判定取并集。
详见 [组员开发说明_移动控制.md](docs/组员开发说明_移动控制.md) 第 5 节。

### 9.3 目标位置接口（给其他组员用）

**方式一：改文件（演示时最方便，保存即生效）**

```json
// WorkPlace/config/goals.json
{
    "UGV1": [1200, -400],
    "UAV1": {"x": 1400, "y": -500}
}
```

脚本每帧比对文件修改时间，改了就更新星标和虚线，**不需要重启**。

**方式二：命令行一次性指定**

```powershell
python WorkPlace\missions\live_track.py --goals "UGV1:1200,-400;UAV1:1400,-500"
python WorkPlace\missions\live_track.py --goals-file my_targets.json
```

**没写到的平台怎么办**：默认自动取**离它最近的敌人**作为目标（空心星标，HUD 里标 `[默认]`）。
按 `r` 可随时重算。不想要这个默认行为就加 `--no-infer`（那些平台就不画星标）。

**方式三：当模块用（推荐给写控制逻辑的同学）**

```python
from missions.live_track import LiveView, Scene, Environment
view = LiveView(scene, env, names, types)
view.set_markers(markers)
view.update(snapshot, elapsed)          # snapshot 是 {name: Pose}
```

> ⚠️ 提醒：如果控制脚本是**另一个进程**，两边都会 `register_node`；SDK 是否允许同名重复注册
> **尚未验证**。最稳妥的是让他们 `import` 本模块的 `LiveView`，单进程共用一份状态。

> 💡 建议：以后谁用 `core/fleet.py` 写控制逻辑，**也从这个 `goals.json` 读目标**，
> 这样图上画的星标和车实际去的地方就是同一个——演示时不会出现"星标在这、车往那跑"。

### 9.4 视角与按键

| 按键 | 作用 |
|---|---|
| `1`–`6` | 聚焦 UGV1/2/3、UAV1/2/3（右图放大，左图打黄光环） |
| `0` | 右图回到全城总览 |
| `f` | 循环切换聚焦对象 |
| `+` / `-` | 缩放（30 m ~ 3000 m） |
| `s` | 存图到 `figs/live_snapshot_*.png` |
| `h` | 隐藏/显示按键帮助 |
| `r` | 重算"最近敌人"默认目标 |

### 9.5 常用命令

```powershell
# 只自检，不动任何东西
python WorkPlace\missions\live_track.py --probe

# 正常看（前提：CarlaUE4 + start.bat + subject4.exe 都已启动）
python WorkPlace\missions\live_track.py

# 关掉探测（敌人一直空心）、关掉俯视图截图
python WorkPlace\missions\live_track.py --no-detect --save-every 0

# 显示阶段线（SUBJECT_1..6 的起止线）
python WorkPlace\missions\live_track.py --show-phase-lines

# 只用无人机探测、识别半径 30 m
python WorkPlace\missions\live_track.py --detect-from uav --detect-tol 30
```

### 9.6 已验证 / 未验证

用假 SDK（带简易运动学）做了端到端和断言测试：

| 项 | 结果 |
|---|---|
| 环境要素解析 | ✅ 任务区 4 点、禁飞区 3 个（4/8/8 点）、雷区、电磁区、6 组阶段线 |
| 敌人初始状态 | ✅ 22 个全部空心，`已发现 0/22` |
| 发现后变实心 | ✅ 点亮 2 个 → 空心 20 / 实心 2 |
| 探测匹配 + 去重 | ✅ 命中已知 #8；未登记的画额外标记；重复探测只算一个 |
| 目标星标落点 | ✅ 像素坐标与 `to_px()` 完全一致，误差 0 |
| 人工 vs 默认目标 | ✅ 人工=实心星、默认=空心星；`__示例__` 被正确忽略 |
| 目标虚线 | ✅ 从平台当前位置连到目标点 |
| 默认"最近敌人" | ✅ 未输入时全部走最近敌人（本场景最近的是出发线旁 17 m 的 Stryker） |
| **真机状态 / 相机 / 真实移动** | ❌ **未验证**，需上机 |

---

## 10. 场景工具：生成敌人 / 重置侦察

这两个脚本解决"场景里没敌人"和"想重跑一遍"的问题。
**注意：SDK 没有运行时生成 Actor 的接口**，所以两者的本质都是"改 json + 重启 `subject4.exe`"。

### 10.1 `tools/spawn_targets.py` —— 生成敌人

```powershell
python WorkPlace\tools\spawn_targets.py --list                 # 先看现有目标
python WorkPlace\tools\spawn_targets.py --count 8 --seed 42    # 随机撒 8 个
python WorkPlace\tools\spawn_targets.py --count 6 --types dynamic.prop.barricade,vehicle.ifv.stryker
python WorkPlace\tools\spawn_targets.py --count 3 --append     # 追加，不覆盖原有
python WorkPlace\tools\spawn_targets.py --at "1500,-800; 1900,-600"   # 手动坐标
python WorkPlace\tools\spawn_targets.py --count 5 --dry-run    # 只看不写
```

落点规则（避免生成到楼里）：只在**任务区 arena 内**、**道路像素上**、彼此至少 `--min-sep` 米。
首次写入会先把原文件备份成 `targets.json.orig`（`reset_scene.py` 靠它还原）。

### 10.2 `tools/reset_scene.py` —— 重置侦察流程

```powershell
python WorkPlace\tools\reset_scene.py --snapshot      # 【第一次用先跑这个】存基准
python WorkPlace\tools\reset_scene.py                 # 杀进程 → 还原 → 重启
python WorkPlace\tools\reset_scene.py --keep-enemies  # 保留新生成的敌人，只重启场景
python WorkPlace\tools\reset_scene.py --no-relaunch   # 只还原文件，不重启
python WorkPlace\tools\reset_scene.py --list          # 只看状态
```

三步：`taskkill subject4.exe` → 从 `*.orig` 还原 `targets/vehicles/elements/game.json` → 重新 `Popen` 场景进程。
只动场景进程，**不会**重启 CarlaUE4 和飞控驱动（要连 Carla 一起重启加 `--kill-carla`）。

### 10.3 三个脚本的配合

```powershell
# 典型演示前重置
python WorkPlace\tools\reset_scene.py --snapshot              # 只在第一次需要
python WorkPlace\tools\spawn_targets.py --count 8 --seed 42   # 换一批敌人
python WorkPlace\tools\reset_scene.py --keep-enemies          # 重启场景让新敌人生效
python WorkPlace\missions\live_track.py                       # 开跑（纯显示）
# 之后改 WorkPlace\config\goals.json 调整每台的目标星标
```

---

## 11. 手动驾驶测试 `tools/manual_drive.py`

联调前的"手感测试"：不跑任何算法，纯键盘开车，用来确认三件事——
① 车能正确响应油门/转向；② 多车切换不会串控；③ 换车后旧车能停住。

参考了 `environment_forclass/environment/scripts_test/manual_control.py`（方向键开车 + 数字键切车），
但有三处不同：

| | 参考脚本 `manual_control.py` | 本工具 |
|---|---|---|
| 按键来源 | **pygame**，要开一个 200×200 的假窗口 | **`msvcrt` 直接读控制台**，零依赖、无窗口 |
| 注册对象 | 3 车 | **只注册无人车**（默认 3 台），不碰无人机 |
| 切车行为 | 旧车保持上一帧油门继续跑 | **切走时旧车显式收油 + 刹车** |
| 控制接口 | `lib/vehicle.py` 的 `longitudinal_control` / `lateral_control`（两次调用） | `swarm_api.apply_control`（一次调用，不会漏改转向） |

### 按键

| 按键 | 作用 |
|---|---|
| `↑` / `↓` | 油门 +/－（松开后缓慢回落到 0，不是瞬间断油） |
| `←` / `→` | 左转 / 右转（松开后自动回正） |
| `空格` | 刹车到停（按住即刹，松开恢复） |
| `x` | 手刹开关 |
| `1` `2` `3` | 切换到第 N 台车 |
| `Tab` / `n` | 切到下一台 |
| `p` | 打印所有车的当前位置/速度/朝向 |
| `h` | 显示按键帮助 |
| `q` / `Esc` | 退出（所有车刹车 + 拉手刹） |

`W` `A` `S` `D` 也可以当方向键用。

### 用法

```powershell
# 前置：CarlaUE4 + start.bat + subject4.exe 都已启动
python WorkPlace\tools\manual_drive.py

# 直接开 2 号车
python WorkPlace\tools\manual_drive.py --start 2

# 离线自测（不需要仿真器，只验证按键解析和油门/转向积分）
python WorkPlace\tools\manual_drive.py --selftest
```

运行后每 0.2 秒刷新一行状态：

```
[2/3] UGV2  steer=+0.16  thr=+0.24  brake=0  hb=0  v= 3.21  pos=(-1811.6, -459.8)  yaw=  12.4°
```

### 已验证 / 未验证

| 项 | 结果 |
|---|---|
| 按键解析（含方向键两字节序列、半包续传） | ✅ 13/13 用例（`--selftest`） |
| 油门积分 + 松开回落 | ✅ 0.08→0.16→0.24，松开 40 帧归零 |
| 转向打到上限 + 自动回正 | ✅ ±0.8 封顶，松开归零 |
| 刹车优先 | ✅ 刹车时油门清零 |
| 手刹开关 | ✅ 按一次开、再按一次关 |
| 主循环（假 SDK 喂按键序列） | ✅ 3 车注册、切车时旧车收到"零油门+刹车"、退出时三车都收到刹车+手刹 |
| **真机手感 / 实际运动** | ❌ **未验证**，需上机 |

> ⚠️ 车号（`node_no`）必须和 `vehicles.json` 的顺序一致：前三台是 `vehicle.imv.rb`（无人车）。
> 如果你们的场景配置改过顺序，用 `--count` 和 `--start` 对一下。

---

## 12. 完整链路 `missions/full_mission.py`（整合组员代码）

组员交付的无人机侦查 + 无人车打击已经整合进来，**一条命令跑完整个流程**：

```powershell
# 启动顺序严格如下（依据 environment/kyxz.md 的 D4 说明；start.bat 要一直开着）
#   ① CarlaUE4
#   ② Materials\simu_driver_windows\start.bat  → 选 2（旋翼）
#   ③ Materials\practice\practice1_Scene\subject4.exe
python WorkPlace\missions\full_mission.py
```

> ⚠️ **飞控必须在 CarlaUE4 之后启动**。世界还不存在时启动飞控，它绑不上之后生成的飞机，
> 表现是：**无人车能跑，无人机一动不动，而且不报错**。
> 脚本启动时自动检查进程与启动顺序；也可单跑检查：
> `python WorkPlace\core\preflight.py` 或 `python WorkPlace\tools\uav_diag.py takeoff`。

无人机起飞侦查 → 发现敌人（小地图空心变实心）→ 攒够 N 个目标 → 三车各自 A\* 出发打击，
全程在小地图上实时显示。控制台每 5 秒打一行心跳：

```
[mission] t=   10s  机高=[150.2, 150.1, 149.8]/250  阶段=crui,crui,crui  已发现=1  车=待命
```

```powershell
python WorkPlace\missions\full_mission.py --no-ugv            # 只跑侦查
python WorkPlace\missions\full_mission.py --use-default-enemies --recon-timeout 60
python WorkPlace\missions\full_mission.py --takeoff-timeout 30   # 缩短起飞自检

# 飞控动力学不可用时的兜底（用 set_transform 沿航线播放，非真实飞行）
python WorkPlace\missions\full_mission.py --uav-mode kinematic
```

> 如果无人机始终不动：先跑 `python WorkPlace\tools\net_check.py` 看链路连接，
> 再跑 `python WorkPlace\tools\uav_diag.py kinetic`。实测这套课程包的
> `log_kinetic_data` / `launch_kinetic` 是空壳（返回 None），
> 此时用 `--uav-mode kinematic` 保证流程能演示。详见整合说明第十章。

**整合后的文件**

| 文件 | 来源 | 改动 |
|---|---|---|
| `missions/control_uav.py` | 葛伊诺 | 1 处：无人机指令返回码位置改成两种都接受 |
| `missions/control_ugv.py` | 吴羽桐 `step5_multi_ugv.py` | 1 处：路网图片路径 |
| `missions/uav_runner.py` | 我写的 | 把 UAV 的阻塞循环改成可单步驱动 |
| `missions/full_mission.py` | 我写的 | 编排 + 目标分配 + 小地图 |
| `assets/*.png` | 吴羽桐 | 骨架路网 / 真实路宽图 |

> ⚠️ `Partner/无人机/live_track.py` 与我的 `live_track.py` **MD5 完全一致**（就是我文件被拷走测过），
> 没有合并，直接忽略。

**为什么是单进程**：UAV、UGV、小地图共用一个 `SwarmAEClient`，**只注册一次节点**，
从而绕开组员文档里点名要求核实的"显示端与控制端重复注册同名节点"问题。

详细流程、目标分配算法、两处代码改动的原因、真机必查项、排错表，见
[整合说明_完整链路.md](docs/整合说明_完整链路.md)。

**全自动演示**（重置场景 + 随机就近敌人 + 复位载具 + 跑任务，一条命令）：

```powershell
python WorkPlace\demo_midterm.py
```

详见整合说明第十二章。

**离线自测**（不连仿真器，10 秒内验证接口是否对得上）：

```powershell
python WorkPlace\selftest\smoke_integration.py
python WorkPlace\selftest\run_main.py with-view
```

---

## 13. 运行前检查清单

- [ ] Python **3.6** 虚拟环境已建，`import swarmae` 成功（SDK 的 egg 是 `py3.6` 专用，3.13 装不上）
- [ ] `import cv2` 成功（`global_planner.py` 依赖，当前 anaconda 里没有）
- [ ] CarlaUE4 已启动，端口 2000 可达
- [ ] `simu_driver_windows\start.bat` 已选 `1 固定翼` 或 `2 旋翼`（会改 `config.yaml` 的 `Type:`）
- [ ] `practice1_Scene\subject4.exe` 已生成 3 车 3 机
- [ ] 当前工作目录可写（骨架化会写 68 MB 的 tga）
- [ ] 已跑过 `python WorkPlace\tools\unit_check.py`，并把输出存档
- [ ] 已跑过 `python WorkPlace\missions\live_track.py --probe`，确认相机接口是否存在
- [ ] 已跑过 `python WorkPlace\tools\reset_scene.py --snapshot`，存好场景基准
- [ ] `WorkPlace\config\goals.json` 里的目标星标符合本次演示
- [ ] **课程包路径不含空格**（飞控硬性要求）；带空格就 `python WorkPlace\tools\sim_link.py --create`，
      并从 `C:\kyxz\environment\simu_driver_windows\start.bat` 启动驱动
- [ ] 已跑过 `python WorkPlace\tools\env_guard.py`，FAIL 为 0

---

## 14. 排障专题（2026-11 补）

中期之后又挖出两个"静默失效"的大坑，都做成了工具 + 文档：

| 症状 | 先跑这个 | 结论 / 处理 |
|---|---|---|
| 无人机不动、驱动窗口只有横幅、`UAV#n: Connect Timeout` | `python WorkPlace\tools\sitl_probe.py` | 路径空格把 SITL 命令行劈开 → 无 MAVLink 心跳。处理：`python WorkPlace\tools\sim_link.py --create`，从 `C:\kyxz\...` 起 `start.bat`。详见 [docs/飞控调通_根因与修复.md](docs/飞控调通_根因与修复.md) |
| 环境"越用越怪"：昨天能跑今天报错、依赖版本对不上 | `python WorkPlace\tools\env_guard.py`（`--diff` 看漂移） | 按 [docs/环境防污染规范.md](docs/环境防污染规范.md) 还原；基线在 `env/requirements.txt` + `env/env_snapshot.json` |

三个工具都**只用标准库**、**只读不改环境**，随便哪个 Python 3 都能跑
（`env_guard.py` 建议在 `kyxz` 里跑，才能看到依赖与 SDK 的真实状态）。

配套模块：`core/env_paths.py` —— 统一解决"课程包到底在哪"（支持仓库内 / 工作区根 /
`C:\kyxz` 联接 / `SWARMAE_ENV_ROOT` 环境变量四种放法），新写工具请直接用它，别再写死相对路径。

