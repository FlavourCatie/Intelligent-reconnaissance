# -*- coding: utf-8 -*-
"""生成项目技术汇报 DOCX（2026-11-10 代码基线）。"""
import os
from docx import Document
from docx.shared import Cm, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.section import WD_SECTION
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "无人机-无人车协同侦察打击系统_技术汇报_2026-11-10.docx")
FONT = "Microsoft YaHei"
MONO = "Consolas"


# ---------------------------------------------------------------------------
#  基础格式
# ---------------------------------------------------------------------------
def set_cn(run, size=10.5, bold=False, color=None, font=FONT):
    run.font.name = font
    run.font.size = Pt(size)
    run.bold = bold
    if color:
        run.font.color.rgb = RGBColor(*color)
    rPr = run._element.get_or_add_rPr()
    rFonts = rPr.find(qn("w:rFonts"))
    if rFonts is None:
        rFonts = OxmlElement("w:rFonts")
        rPr.append(rFonts)
    rFonts.set(qn("w:eastAsia"), font)
    rFonts.set(qn("w:ascii"), font)
    rFonts.set(qn("w:hAnsi"), font)


def shade(cell, hex_color):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), hex_color)
    tcPr.append(shd)


def cell_margins(table, top=40, bottom=40, left=70, right=70):
    tblPr = table._tbl.tblPr
    mar = OxmlElement("w:tblCellMar")
    for k, v in (("top", top), ("bottom", bottom), ("left", left), ("right", right)):
        el = OxmlElement("w:" + k)
        el.set(qn("w:w"), str(v))
        el.set(qn("w:type"), "dxa")
        mar.append(el)
    tblPr.append(mar)


def para(text="", style=None, size=10.5, bold=False, color=None,
         align=WD_ALIGN_PARAGRAPH.JUSTIFY, space_after=4, indent=False):
    p = doc.add_paragraph(style=style)
    p.alignment = align
    p.paragraph_format.space_after = Pt(space_after)
    p.paragraph_format.line_spacing = 1.3
    if indent:
        p.paragraph_format.first_line_indent = Cm(0.74)
    r = p.add_run(text)
    set_cn(r, size=size, bold=bold, color=color)
    return p


def h1(text, page_break=False):
    if page_break:
        doc.add_page_break()
    p = doc.add_heading(level=1)
    r = p.add_run(text)
    set_cn(r, size=17, bold=True, color=(31, 56, 100))
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(8)


def h2(text):
    p = doc.add_heading(level=2)
    r = p.add_run(text)
    set_cn(r, size=13, bold=True, color=(45, 78, 125))
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after = Pt(4)


def h3(text):
    p = doc.add_heading(level=3)
    r = p.add_run(text)
    set_cn(r, size=11.5, bold=True, color=(60, 70, 85))
    p.paragraph_format.space_before = Pt(7)
    p.paragraph_format.space_after = Pt(3)


def bullets(items, size=10.5):
    for it in items:
        p = doc.add_paragraph(style="List Bullet")
        p.paragraph_format.space_after = Pt(2)
        p.paragraph_format.line_spacing = 1.25
        r = p.add_run(it)
        set_cn(r, size=size)


def code(text, size=8.5):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.5)
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(5)
    pPr = p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), "F2F4F7")
    pPr.append(shd)
    r = p.add_run(text)
    set_cn(r, size=size, font=MONO)
    return p


def table(headers, rows, widths=None, size=8.2, header_fill="D9E7F5",
          align_center=True):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = True
    cell_margins(t)
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = ""
        p = hdr[i].paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run(h)
        set_cn(r, size=size, bold=True, color=(25, 45, 70))
        shade(hdr[i], header_fill)
    for row in rows:
        cells = t.add_row().cells
        for i, val in enumerate(row):
            cells[i].text = ""
            p = cells[i].paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if align_center else WD_ALIGN_PARAGRAPH.LEFT
            p.paragraph_format.space_after = Pt(0)
            r = p.add_run(str(val))
            set_cn(r, size=size)
    for ri, row in enumerate(t.rows):
        trPr = row._tr.get_or_add_trPr()
        cant = OxmlElement("w:cantSplit")
        trPr.append(cant)
        if ri == 0:
            hdr = OxmlElement("w:tblHeader")
            hdr.set(qn("w:val"), "true")
            trPr.append(hdr)
    if widths:
        for row in t.rows:
            for i, w in enumerate(widths):
                row.cells[i].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)
    return t


def caption(text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(6)
    r = p.add_run(text)
    set_cn(r, size=8.5, color=(100, 100, 100))


def add_page_number(section):
    footer = section.footer
    p = footer.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run("第 ")
    set_cn(r, size=8.5, color=(120, 120, 120))
    fld = OxmlElement("w:fldSimple")
    fld.set(qn("w:instr"), "PAGE")
    p._p.append(fld)
    r2 = p.add_run(" 页")
    set_cn(r2, size=8.5, color=(120, 120, 120))


def header_text(section, text):
    hp = section.header.paragraphs[0]
    hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    r = hp.add_run(text)
    set_cn(r, size=8, color=(130, 130, 130))


# ---------------------------------------------------------------------------
doc = Document()
sec = doc.sections[0]
sec.page_height = Cm(29.7)
sec.page_width = Cm(21.0)
sec.top_margin = Cm(2.2)
sec.bottom_margin = Cm(2.0)
sec.left_margin = Cm(2.2)
sec.right_margin = Cm(2.0)

style = doc.styles["Normal"]
style.font.name = FONT
style.font.size = Pt(10.5)
style._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
style.paragraph_format.line_spacing = 1.3

add_page_number(sec)
header_text(sec, "无人机—无人车协同侦察打击系统 · 技术汇报")

# ===========================================================================
# 封面
# ===========================================================================
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.LEFT
p.paragraph_format.space_before = Pt(70)
r = p.add_run("项目技术汇报")
set_cn(r, size=16, bold=True, color=(70, 110, 165))

p = doc.add_paragraph()
p.paragraph_format.space_before = Pt(8)
r = p.add_run("无人机—无人车协同\n侦察与合围打击系统")
set_cn(r, size=28, bold=True, color=(24, 44, 80))

p = doc.add_paragraph()
p.paragraph_format.space_before = Pt(10)
r = p.add_run("算法 · 参数 · 代码实现 · 当前验证")
set_cn(r, size=15, color=(70, 80, 95))

p = doc.add_paragraph()
p.paragraph_format.space_before = Pt(38)
pPr = p._p.get_or_add_pPr()
shd = OxmlElement("w:shd")
shd.set(qn("w:fill"), "1F3864")
pPr.append(shd)
r = p.add_run("  ")
set_cn(r, size=2)

p = doc.add_paragraph()
p.paragraph_format.space_before = Pt(16)
r = p.add_run("代码基线：2026-11-10\n"
              "核心：WorkPlace/missions/full_mission.py（3195 行）\n"
              "Python 代码总量：约 16,645 行")
set_cn(r, size=10, color=(80, 80, 80))

p = doc.add_paragraph()
p.paragraph_format.space_before = Pt(150)
r = p.add_run("本报告直接依据当前工作区代码提取，\n"
              "不以早期 Markdown 文档中的历史参数为准。")
set_cn(r, size=9, color=(120, 120, 120))

# ===========================================================================
# 目录 / 阅读说明
# ===========================================================================
h1("目录与阅读说明", page_break=True)
items = [
    ("摘要", "系统定位、核心链路、当前结论"),
    ("1  系统组成与运行架构", "模块划分、进程、数据流"),
    ("2  场景、坐标与任务区域", "arena、半图、Zone、无传送"),
    ("3  任务主流程与状态机", "起飞—扫描—派车—合围—打击—收尾"),
    ("4  无人机感知与覆盖扫描", "牛耕、禁飞区、安全规划、探测聚类"),
    ("5  目标管理与敌情融合", "UAV/UGV 探测、marker、目标池"),
    ("6  战术层算法", "分类、阵位、分配、路线校验、合围"),
    ("7  无人车导航与控制", "A*、Stanley/PID、编队、避让、展开点"),
    ("8  打击、击杀与结束判定", "武器、目标锁定、释放、完成条件"),
    ("9  参数总表", "按模块汇总当前代码默认值"),
    ("10  当前验证与边界", "实测链路、已修问题、限制"),
    ("11  风险与改进建议", "按优先级给出工程建议"),
    ("12  代码索引与复现", "关键函数、配置、启动命令"),
]
for a, b in items:
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(3)
    r = p.add_run(a)
    set_cn(r, size=10.5, bold=True, color=(35, 60, 100))
    r2 = p.add_run("    " + b)
    set_cn(r2, size=9, color=(100, 100, 100))

para("说明：报告中的“当前值”均指本工作区 2026-11-10 的代码默认值或演示启动器显式值；"
     "演示启动器与 full_mission 默认值不同处会并列标注。", size=9, color=(110, 110, 110))

# ===========================================================================
# 摘要
# ===========================================================================
h1("摘要")
para("本系统面向“未知敌情的半图区域侦察—多车协同打击”任务：三架无人机对任务区做覆盖扫描，"
     "发现目标后由战术层判断目标是路段敌人还是需要多面合围的路口/丁字敌人；"
     "三台无人车在道路骨架上做 A* 路径规划，按编队/前沿待机/分裂/合围的状态机执行，"
     "最后由打击车在满足合围条件后开火，击杀后释放车辆并接下一批目标。", indent=True)

para("与“按已知敌人坐标直接冲过去”的演示不同，本实现把“敌人位置未知”作为前提："
     "无人机只做区域覆盖，不做目标预测；目标一旦被探测到才进入候选池；"
     "无人车在无人机完全升空并发现目标之前保持原位，避免出现“飞机还没起来、车已经出发”的时序错误。", indent=True)

para("当前代码的核心特征可以概括为六点：", bold=True)
bullets([
    "覆盖扫描：矩形分区/牛耕条带，条带间距 1.4R，禁飞区安全切段与断带补扫；",
    "目标融合：UAV 探测（R=300 m，15 m 聚类）+ 全车 UGV 近距 contact（25 m 合并、30 s 过期）；",
    "战术层：骨架路口分类 → 24 m 阵位 → 3 车×M 阵位穷举瓶颈分配 → 路线/多面校验；",
    "合围：最多 2 个不同方向阵位，车辆必须真实从侧翼道路接近；路径中把敌人 13 m 圆挖掉，禁止穿敌；",
    "控制：A* 8 邻域骨架路径 + Stanley 横向控制 + PID 纵向控制 + 领航-跟随编队；",
    "安全：20 m 活敌强制刹停、22 m 让行/12 s 正面让行、13 m 敌人避障、5 m 超车侧偏。",
])

para("截至本报告，最近一轮半图演示已观察到：第一个路口敌人由 UGV1 从 W 侧、UGV3 从 S 侧绕行合围，"
     "UGV1 在 24 m 距离开火并产生“已消灭”，两车释放后 UGV1 又被派往 E-758 路段目标。"
     "最后一处路口曾出现“三车同侧/战术层整拍回退”，根因已定位为 route_violation 对 UAV 三维目标点"
     "（x,y,z）的解包异常，以及 busy 目标被排除出候选池；对应修复已进入当前代码，仍需继续做完整回归。", indent=True)

# ===========================================================================
# 1 系统组成
# ===========================================================================
h1("1  系统组成与运行架构")
h2("1.1 代码模块")
para("当前 WorkPlace 下的 Python 代码约 16,645 行。核心模块与职责如下（行数为当前文件统计）：")
table(
    ["模块", "行数", "职责", "关键内容"],
    [
        ["missions/full_mission.py", "3195", "任务总控", "连接、区域、UAV/UGV 调度、战术决策、打击、结束判定"],
        ["missions/control_ugv.py", "793", "UGV 执行", "A*、Stanley/PID、到点、超车/倒车、避障"],
        ["missions/uav_runner.py", "753", "UAV 执行", "真实/运动学/虚拟三种飞行，航点推进与探测"],
        ["missions/control_uav.py", "838", "UAV 基础", "SDK 封装、安全规划、探测解析、起降"],
        ["missions/sweep_plan.py", "597", "覆盖规划", "牛耕条带、分区、禁飞区切段、航程均衡"],
        ["missions/live_track.py", "1649", "可视化", "小地图、环境、marker、轨迹、状态面板"],
        ["tactics/assign.py", "213", "任务分配", "瓶颈指派、覆盖/等待/改派代价、防抖"],
        ["tactics/slots.py", "262", "路口与阵位", "骨架分类、环采样、阵位/集结点"],
        ["tactics/deploy.py", "540", "开局展开", "k-center 选点、路宽、切向、覆盖评估"],
        ["tactics/formation.py", "237", "队形几何", "沿路径跟随、纵队/楔形、平滑"],
        ["tactics/convoy.py", "137", "编队控制", "领航/成员排序、滞回、集结点"],
        ["tactics/coverage.py", "85", "覆盖栅格", "50 m 网格、前沿点"],
        ["tactics/planner.py", "117", "战术总装", "目标→阵位→分配→计划文件"],
        ["tactics/zones.py", "94", "区域", "严格矩形 Zone，无默认值"],
        ["tactics/plan_io.py", "161", "计划读写", "原子写、mtime 热加载"],
        ["run_tactics_demo.ps1", "147", "演示启动器", "清场、起 UE/飞控/场景、跑任务"],
    ],
    widths=[4.2, 1.2, 3.0, 8.0], size=7.8, align_center=False)
caption("表 1-1  核心代码模块（2026-11-10 当前工作区）")

h2("1.2 运行进程与数据流")
para("一次演示由五个进程/角色组成：CarlaUE4 提供世界与 RPC 2000；main-release.exe + driver.exe 提供飞控驱动；"
     "subject4.exe 读取场景 JSON 生成 3 台 UGV、3 架 UAV 和敌人；full_mission.py 是任务总控；"
     "live_track.py 的小地图窗口由 full_mission 进程内启动。")
para("决策数据通过 JSON 解耦：strike_plan.json 保存“谁去哪个阵位”，goals.json 保存显示目标，"
     "detected.json 保存 UAV 探测点，enemy_state.json 保存探测快照。计划写入采用同目录临时文件 + os.replace，"
     "读侧按 mtime 热加载，因此决策层与执行层可以分进程，小地图也可以直接读取。")
doc.add_picture(os.path.join(os.path.dirname(os.path.abspath(__file__)), "report_arch.png"),
                width=Cm(16.2))
doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
caption("图 1-1  系统总体架构与数据流（依据当前代码绘制）")

h2("1.3 演示入口")
para("半图演示使用 run_tactics_demo.ps1 -HalfMap：脚本从 game.json 的 arena 自动取西半边，"
     "先清理路障、撒 7 个敌人（2 个在路口），再启动 UE4、飞控、场景和 full_mission。"
     "关键显式参数为 --area/--zone=半边、--uav-altitude=150、--sweep-radius=300、"
     "--duration=1500、--ugv-deadline=1200、--dispatch-gap=5、--shot-at=600。")
code("powershell -ExecutionPolicy Bypass -File WorkPlace\\run_tactics_demo.ps1 `\n"
     "  -HalfMap -NoTeleport -NoSpread -NoDeploy")

# ===========================================================================
# 2 场景/坐标
# ===========================================================================
h1("2  场景、坐标与任务区域")
h2("2.1 世界坐标与路网")
para("系统使用 UE4 世界坐标，单位为米；道路骨架图为 4794×4794 的 1 px 图像，"
     "缩放 scale=1.0，世界偏移约 (-2094.53, -2637.32)（config/roadnet_meta.json）。"
     "因此像素坐标与世界坐标可以近似一一对应，A* 的 1 像素代价≈1 m。")
para("完整 arena 为矩形 x∈[-1900.3, 2277.4]、y∈[-1108.1, 341.7]，"
     "包含 3 个禁飞区、雷区、电磁区，以及 6 组阶段线（game.json）。")
h2("2.2 半图区域与“显示 / 扫描 / 战术”三种边界")
para("演示取 arena 的西半边：x∈[-1900, 189]、y∈[-1108, 342]，尺寸约 2089×1450 m。"
     "full_mission 在 --area 上再向内收 35 m，得到实际扫描和小地图区域约 2019×1380 m，"
     "小地图标题显示为“任务区域 ±1010 m”。战术层 --zone 仍使用未内收的半图原边界。")
table(
    ["概念", "当前值 / 说明"],
    [
        ["世界坐标", "UE4 米制；scale=1.0；offset≈(-2094.53,-2637.32)"],
        ["arena", "x[-1900.3,2277.4]，y[-1108.1,341.7]"],
        ["半图 --zone", "x[-1900,189]，y[-1108,342]；2089×1450 m"],
        ["--area 扫描/显示", "--area 再内收 35 m；约 2019×1380 m，UI 显示 ±1010 m"],
        ["路网骨架", "4794×4794，1 px 宽；A* 8 邻域"],
        ["演示敌人", "targets.json 当前 7 个士兵，其中 2 个放置在路口"],
    ],
    widths=[4.5, 12.5], size=8.2, align_center=False)
caption("表 2-1  场景与区域参数")

h2("2.3 Zone 的严格性")
para("tactics/zones.py 的 Zone 不提供默认区域，也不会自动兜底：缺少 --zone 直接抛 ZoneError。"
     "这样撒点、巡逻、小地图、战术层都只能使用同一个显式矩形，避免“测试区域/半图区域混用”的事故。"
     "Zone.contains(x,y,margin) 是后续所有越界判断的统一入口。")

h2("2.4 无传送模式")
para("--no-teleport 有三层含义：不调用 teleport_ugvs() 把车复位到 SPAWNS；不自动执行 k-center 开局展开；"
     "关闭看门狗把区域外车辆 set_transform 拉回区域内的逻辑。配合 --no-spread 关闭 UAV 水平散开，"
     "整套演示从 vehicles.json 的原始位置起步。")
para("当前场景的原始位置在三车/三机都位于最左侧边界中间附近：UGV 约 (-1829.6,-517.7)、"
     "(-1859.6,-517.4)、(-1887.4,-517.4)；UAV 约 (-1875,-570)、(-1830,-495)、(-1875,-450)，高度 100 m。"
     "UGV 到骨架中心线的距离小于 1 m，因此不需要先“上马路”。")

# ===========================================================================
# 3 流程/状态机
# ===========================================================================
h1("3  任务主流程与状态机")
para("full_mission 的主循环节拍随无人机模式而变：kinematic/virtual 用 KINEMATIC_TICK=0.02 s（50 Hz），"
     "kinetic 用 control_uav.DT=0.05 s（20 Hz）；控制、探测、显示各自再节流。"
     "整个任务可以拆成十个阶段：")
table(
    ["阶段", "触发 / 判据", "主要动作", "关键输出"],
    [
        ["S0 启动", "进程与场景就绪", "连接 SDK、读 game.json/targets.json、注册 3 车 3 机", "区域、marker、路网"],
        ["S1 起飞", "任务开始", "UAV 爬升/散开；UGV 保持原位", "UAV 进入 crui/cruise/hold"],
        ["S2 扫描", "UAV 完全升空", "牛耕条带推进；按 300 m 半径探测", "discovered_enemies"],
        ["S3 发现/派车", "uav_ready 且 pool 非空", "UGV 侧翼 contact 合并、战术层求解", "strike_plan / goals"],
        ["S4 编队", "有打击目标", "领航-跟随；未接敌不前进", "跟随点/队形"],
        ["S5 分裂", "某车获得打击任务", "僚车 split 成打击车", "role=strike"],
        ["S6 合围", "路口/丁字目标", "不同分支阵位；到点 hold", "等待 2 面"],
        ["S7 打击", "合围条件满足", "目标锁定、set_weapon", "已消灭 / killed_pts"],
        ["S8 释放", "目标消灭或 8 s 无活敌", "finish、20 s 冷却、重新入池", "ugv_cooldown"],
        ["S9 收尾", "全歼/扫描完/deadline/duration", "停线程、降落、汇总", "exit_reason"],
    ],
    widths=[2.2, 4.5, 6.0, 4.0], size=7.8, align_center=False)
caption("表 3-1  任务阶段与判据")

h2("3.1 三个关键“门限”")
bullets([
    "升空/巡航门限：uav_recon_ready() 要求没有任何一架处于 take/climb，全部进入 crui/cruise/hold；"
    "虚拟模式按 cruise/hold 判断。只有满足门限才允许 UGV 派车。",
    "发现/接敌门限：FormationRuntime 在没有任何 UAV 目标且没有 UGV contact 时直接 return，"
    "不再执行“接敌前先往前沿开”；发现目标后才允许编队推进。",
    "合围门限：路口目标至少 2 个不同方向的实际车辆位置，且每辆车到自己的阵位小于 30 m；"
    "最多 2 个阵位；420 s 仍凑不齐才按实际就位数降级。",
])

h2("3.2 时间与节流")
table(
    ["项", "当前值", "说明"],
    [
        ["主循环", "0.02 s（kinematic/virtual，50 Hz）\n0.05 s（kinetic，20 Hz）", "KINEMATIC_TICK / cu.DT"],
        ["控制台心跳", "5 s", "--console-every"],
        ["小地图刷新", "1.2 s", "--view-interval"],
        ["UAV 控制周期", "0.2 s", "CMD_PERIOD"],
        ["UAV 探测", "每 10 帧或 ≥0.5 s", "DETECT_EVERY / last_detect"],
        ["UGV 探测缓存", "0.5 s", "detect_fun 阻塞，缓存降低频率"],
        ["UGV 开火循环", "2 s", "last_attack"],
        ["编队下发", "≥0.5 s", "FormationRuntime.last_update"],
        ["编队点滞回", "6 m / 1 s", "ConvoyController"],
    ],
    widths=[3.5, 4.5, 8.5], size=8.2, align_center=False)
caption("表 3-2  时间节流参数")

# ===========================================================================
# 4 无人机
# ===========================================================================
h1("4  无人机感知与覆盖扫描")
h2("4.1 牛耕式覆盖")
para("覆盖扫描的核心是 boustrophedon（牛耕）条带：在矩形区域内沿长边方向生成平行扫描线，"
     "相邻线方向交替，形成往返路径。代码里默认 partition=rects：先把区域按长边等分成 3 块，"
     "每架机独立扫自己那块；如果某块被禁飞区完全覆盖，则退回全局 strips 模式按航程均衡。")
para("条带间距取 1.4R（R=300 m，即 420 m），而不是 2R。原因是圆形探测足迹要无缝覆盖，"
     "相邻条带的间距不能超过 √2·R；代码注释中保留了“2R 会留缝”的历史说明，但当前实现以 1.4R 为准。"
     "半图 2019×1380 的每个分区高度约 460 m，实际生成 6 条扫描线（日志中的 step 约 336.5 m 是分区后均分的结果）。")
para("禁飞区处理分三层：① 每条扫描线按 15 m 步长采样，遇到不安全点就切段；"
     "② 对“被切断”的条带，在 ±0.35×间距处补两条平行线；③ 航段之间用 SafePlanner 的可见图最短路绕行，"
     "不做“直线硬闯”。")

h2("4.2 SafePlanner 安全规划")
para("SafePlanner 把 arena 和 3 个禁飞区都转成轴对齐包围盒：arena 内缩 10 m；"
     "禁飞区保护盒外扩 30 m；用于航路规划的 routing 盒再额外加 10 m（总 40 m）。"
     "点在盒内即不安全；线段与盒相交即不安全。绕行用可见图：起点、终点、每个禁飞区盒的 4 个角点，"
     "两两连线，若线段安全则建边，再用 Dijkstra 求最短路；找不到就抛 SafetyError，绝不回退为直线。")
para("这套做法的代价是用 AABB 包住凹多边形会多占一些空域，但换来可审计、无第三方几何库依赖、"
     "不会出现“绕过禁区时穿角”的问题。")

h2("4.3 三种飞行模式")
table(
    ["模式", "位置来源", "高度/速度", "适用场景", "主要风险"],
    [
        ["kinetic", "真实飞控/SITL", "演示 150 m；默认 250 m；15 m/s", "真实飞行验证", "起飞慢、受风/物理影响"],
        ["kinematic", "set_transform 播放航线", "爬升 12 m/s；单帧≤6 m；偏航≤70°/s", "稳定演示、可重复", "非真实动力学"],
        ["virtual", "纯数学位置/探测", "不碰 actor", "算法联调、最稳", "画面飞机不动"],
    ],
    widths=[2.0, 4.2, 4.3, 3.5, 3.0], size=7.8, align_center=False)
caption("表 4-1  UAV 飞行模式")

para("kinematic 模式先垂直爬升，到位前不前进；转角用二次贝塞尔圆弧（半径 30 m），"
     "按 1.5 m 等距重采样，偏航角每帧限速 70°/s，单帧最多推进 0.15 s，避免卡顿时瞬移。"
     "虚拟模式只更新数学位置和探测，不写 actor，因此完全不受重力影响。")

h2("4.4 探测与聚类")
para("探测由 detect_situation 返回 SDK 状态；parse_detections 只接受 Type 为 strike/enemy/hostile、"
     "HealthPoint>0、与 UAV 水平距离≤300 m 的目标，并把厘米坐标除以 100 转米。"
     "merge_detections 以 15 m 为聚类半径，保留首次出现位置作为代表点，避免同一目标因抖动反复生成新点。")
para("探测节流为每 10 帧一次，另有 0.5 s 兜底；探测结果写入 detected.json，同时更新内存 discovered_enemies。"
     "uav_recon_ready() 与探测是两件事：前者防止“飞机还在爬升就派车”，后者负责发现目标。")

h2("4.5 关键参数")
table(
    ["参数", "值", "文件/说明"],
    [
        ["探测半径 R", "300 m", "control_uav.DETECT_RADIUS；sweep --sweep-radius"],
        ["条带间距", "1.4R = 420 m", "sweep_plan.make_strips / build_rects"],
        ["安全切段步长", "15 m", "split_strip_safe"],
        ["补扫偏移", "±0.35×间距", "build_rects"],
        ["聚类半径", "15 m", "CLUSTER_RADIUS"],
        ["探测周期", "10 帧 / 0.5 s", "DETECT_EVERY"],
        ["arena 内缩", "10 m", "ARENA_MARGIN"],
        ["禁飞区保护", "30 m", "SAFETY_MARGIN"],
        ["航路预留", "40 m", "SAFETY_MARGIN+TRACK_RESERVE"],
        ["可见图角点", "5 m", "CORNER_OFFSET"],
        ["航点容差", "8 m", "WAYPOINT_TOL"],
        ["高度容差", "2 m", "ALTITUDE_TOL"],
        ["降落安全高度", "40 m", "SAFE_LAND_ALT"],
    ],
    widths=[4.0, 4.0, 8.5], size=8.0, align_center=False)
caption("表 4-2  UAV 覆盖与安全参数")

# ===========================================================================
# 5 目标管理
# ===========================================================================
h1("5  目标管理与敌情融合")
h2("5.1 三路目标来源")
para("系统同时维护三类敌情：")
bullets([
    "targets.json：场景静态目标，作为 marker 真值；用于显示、expected_enemies 总数、击杀点对齐；",
    "UAV discovered_enemies：飞行探测 + 15 m 聚类，进入 detected.json，是“已发现”的主要来源；",
    "UGV contacts：每 2 s 扫描所有已注册车辆（包括尚未接任务的），同类型 25 m 合并、30 s 过期，"
    "用于补无人机视野外的近距目标，并可直接点亮 marker。",
])
para("mark_contacts_on_markers 会把 contact 与 targets.json marker 做 80 m 最近邻匹配，"
     "点亮“已发现”，解决“车已经动了但 UI 还是空心”的问题；匹配容差比 20 m 的探测匹配宽，"
     "是因为 UGV 近距探测坐标抖动更大。")

h2("5.2 目标池与击杀点")
para("main 循环把 enemies 与 contacts 合并成 pool；当前代码特意保留“已被指派”的目标在池中，"
     "因为最后一个路口可能先只到一辆车，后续空闲车必须还能加入另一侧。击杀点 killed_pts 在池中按 40 m 排除。"
     "目标点去重按 25 m，避免同一个敌人被三台车重复领取。")
para("killed_pts 同时用于：目标池过滤、marker 置为已消灭、结束判定、UGV 路径避障过滤。"
     "当前击杀来源包括：Health<=0；以及 Soldier 开火当帧立即标记（因为士兵 5 连发基本必死，"
     "而尸体可能长期留在探测列表里，不能等“消失”才释放车辆）。")

h2("5.3 显示状态")
table(
    ["状态", "显示", "来源"],
    [
        ["未发现", "空心红圈", "targets.json 中 discovered=False"],
        ["已发现", "红色实心", "UAV/UGV 探测匹配到 marker"],
        ["已消灭", "黄色 X", "killed_pts 与 marker 对齐，marker.extra=True"],
        ["覆盖", "深色网格", "CoverageGrid，200 m 显示格、50 m 记账格"],
    ],
    widths=[3.0, 5.0, 8.5], size=8.2, align_center=False)
caption("表 5-1  小地图目标三态")

# ===========================================================================
# 6 战术层
# ===========================================================================
h1("6  战术层算法")
h2("6.1 路口/路段分类")
para("slots.classify 先用骨架图的 8 邻域连接数判定道路拓扑：2=直路、3=丁字、≥4=十字；"
     "如果骨架不可用，则退回环采样：以敌人为圆心、35 m 为半径，每 5° 采样一次，统计连续“有路”扇区；"
     "≥3 段为路口、2 段为丁字/直路（按两段夹角是否接近 180° 区分）、1 段为路段。"
     "环采样的最小扇区宽度为 25°，避免噪声小段被当成支路。")
table(
    ["分类", "判定", "阵位策略"],
    [
        ["road", "骨架 8 邻域=2 或环采样1段", "单车抵近，目标点=敌人；不要求多面"],
        ["tee", "骨架=3 或环采样2段且夹角非180°", "两分支都封，首尾夹击"],
        ["junction", "骨架≥4 或环采样≥3段", "每个非缺口分支一个阵位；最多取2面"],
    ],
    widths=[3.0, 6.5, 7.5], size=8.2, align_center=False)
caption("表 6-1  目标分类")

h2("6.2 阵位生成")
para("build_slots 对每个分支方向，在距敌人 standoff=24 m 处取阵位；若该点不在路面上，"
     "沿径向以 1.5/2.0/2.5 倍 standoff 试探吸附。集结点再向外退 hold_back=40 m，即 64 m 处，"
     "用于编队待机/到达前的缓冲。两个分支以内（丁字）不设缺口；三个以上分支时选“最接近 +y（北）”的分支作为缺口，"
     "其余分支生成阵位。两个夹角小于 30° 的分支会被合并，避免同一条路派两台车。")
para("当前合围最多同时占 2 个阵位（ENCIRCLE_MAX_SLOTS=2），因为第三面往往需要绕行 2 km，"
     "等待时间超过任务收益。")

h2("6.3 任务分配：穷举瓶颈指派")
para("assign.solve 面向 3 台车 × M 个阵位的小规模问题，直接穷举“选哪 k 台车 + 阵位排列”，"
     "不做拍卖/强化学习。综合代价为：")
code("J = 0.50 × bottleneck + 0.30 × coverage_penalty\n"
     "  + 0.15 × wait_penalty")
para("其中 bottleneck 是最晚到达时间（合围由最后一辆决定），coverage_penalty 是未占阵位比例，"
     "wait_penalty 是目标已等待时长。选择时先按 bottleneck、再按总路程做字典序比较；"
     "J 用于防抖/改进判断。W_SWITCH=0.05 在代码中保留为改派权重，但当前 _evaluate 尚未把它计入 J，"
     "实际防抖由“改进 20% + 最多改派 2 台”完成。相同目标内两个阵位夹角<45° 视为同一条路，禁止同时占用。")
para("防抖：默认要求新方案比上次好 20% 才改派；最多允许 2 台车改变目标；L2 承诺车辆固定原目标。"
     "当有新目标或原计划不可用时，full_mission 用 force=True 跳过防抖与承诺锁定，避免新敌人被旧计划挡住。")

h2("6.4 路线校验：不是“有路就派”")
para("分配层只看 ETA，可能把西侧车派到东侧阵位再让它穿过敌人。tactics_decide 在分配后做“车×阵位”重匹配，"
     "并用 route_violation 对每条 A* 路径做四项检查：")
table(
    ["检查", "阈值", "目的"],
    [
        ["绕行长度", "≤max(2.2×直线, 直线+250 m)", "防止为了到阵位绕 1 km"],
        ["穿敌", "路径到敌人<12 m（末6点除外）", "禁止从敌人身上穿过去"],
        ["贴其它活敌", "路径到其它敌人<15 m", "禁止穿过还没处理的路口"],
        ["接近方向", "最后~20 m 走向与“从分支外侧逼近”差≤70°", "保证真的从该侧翼来"],
    ],
    widths=[3.0, 7.5, 6.5], size=8.0, align_center=False)
caption("表 6-2  route_violation 判据")

para("如果直连路径穿敌，代码会在本车临时网格里把敌人 13 m 圆挖掉，重算绕行路径；"
     "绕行允许放宽到 4.5×直线/直线+1500 m。这样南侧车可以真的从南侧道路绕到 S 阵位，"
     "而不是“先穿过敌人、再拐到另一侧”。矩阵代价还会加起步大转角惩罚 450 m/π，"
     "避免 180° 原地掉头卡在路沿。")

h2("6.5 合围与开火规则")
para("合围目标在 _TACTICS_ENCIRCLE 中登记 (x,y,need_slots,t_first)，需要至少 2 个不同方向的实际车辆位置，"
     "且每辆车到自己的阵位小于 30 m。方向由车辆实际位置相对敌人的方位计算，而不是目标点方位，"
     "因此“车还在同侧、目标点在对面”不会被误算成一个分支。两方向夹角需大于 45°。")
para("最多等 420 s；超时按实际就位数降级（至少 1 面可打），但只有至少 2 个不同方向时才允许 2 面开火。"
     "开火只允许 kind=strike 的车；目标锁定为自己这一路的敌人（45 m 内），防止隔着 90 m 把另一侧敌人先打死。"
     "攻击距离上限 120 m。")

h2("6.6 目标池与“最后一处”问题")
para("当前修复后的 pool 保留已指派目标；killed_pts 只排除已击杀。这样最后一个路口先派一辆车后，"
     "后续空闲车仍然能看到该目标并补另一个阵位。此前把 busy 目标排除，导致最后一个路口只有一辆车能去；"
     "再叠加 route_violation 对 UAV 三维点 (x,y,z) 的解包异常，战术层整拍回退贪心，"
     "三辆车被派到同一个敌人中心点。当前代码已把 _live_points 统一转二维，并在 route_violation/route_hits_other 中"
     "按 p[0],p[1] 兼容三维，修复了该回退。")

# ===========================================================================
# 7 UGV
# ===========================================================================
h1("7  无人车导航与控制")
h2("7.1 路网 A*")
para("UGV 使用 1 px 宽骨架图做 8 邻域 A*：直边代价 1，斜边 1.414；起点/终点先吸附到最近可通行骨架点。"
     "路径为世界坐标折线，1 点≈1 m。障碍物（沙袋/工事）按 10 m 半径封格；合围敌人按 13 m 半径封格，"
     "只作用于该车的临时网格，不改全局路网。")
para("A* 的启发式是欧氏距离，保证可采纳；实际代码用 `heapq` + `closed` 集合，规模 4794² 但只扩展骨架像素，"
     "单次规划通常几十毫秒。")

h2("7.2 横向/纵向控制")
para("横向用 Stanley 控制器（k=1），预瞄距离 8 m；纵向用 PID（Kp=0.1, Ki=0, Kd=0）跟踪目标速度。"
     "直道目标速度 5 m/s，距拐角 20 m 内降为 2.5 m/s。转向角归一化到 ±30° 后映射为方向盘；"
     "加速度为正给油门，为负给刹车。")
table(
    ["参数", "值", "作用"],
    [
        ["TARGET_V", "5.0 m/s", "直道巡航"],
        ["SLOW_V", "2.5 m/s", "临近拐弯"],
        ["SLOW_RANGE", "20 m", "拐弯减速触发距离"],
        ["LOOKAHEAD", "8 m", "Stanley 预瞄"],
        ["ARRIVE_DIST", "1.0 m", "打击车到点容差"],
        ["follow arrive", "4.0 m", "僚车到跟随点容差"],
        ["FRONT_GAP", "18 m / 8 m", "打击车/僚车跟车距离"],
        ["MAX_RUNTIME", "600 s", "单段最长行驶"],
        ["MAX_REPLAN", "5", "最多重规划次数"],
        ["CLEARANCE", "10 m", "路障挡路判定"],
        ["DEV_WARN", "5 m", "偏离中心线告警"],
    ],
    widths=[4.0, 4.0, 8.5], size=8.0, align_center=False)
caption("表 7-1  UGV 控制参数")

h2("7.3 到点、倒车、超车与让行")
para("到达判定同时看“离路径终点”和“离目标点”，两者任一小于 arrive_dist 即到达，避免目标离路 14 m 时"
     "永远判不到。到达后：僚车 hold；路口阵位车 hold 等合围；路段打击车 finish。"
     "超时/撞停（速度<0.3 持续 8 s）会 finish stuck；倒车用于路口退出，倒 12 m、油门 -0.7，"
     "最多 150 s，然后重规划。")
para("超车只在“正前方有队友、距离<front_gap、且在车头前方”时触发；队友连续停死 3 s 后，"
     "在挡路车处沿垂直方向量两侧路宽，选较宽一侧做 5 m 侧偏的 S 形路径，越过挡路车后 8 m 回中心线。"
     "如果两侧都不够 2×1.5+1=4 m 的净宽，则不超车。")
para("调度层还有一条让行规则：两车距离<22 m 时，离自己目标更远的那台刹停 8 s；"
     "若两车“想走的方向”夹角接近 180°（正面相遇），刹停 12 s，让对方完成借道超车。"
     "另外任何 drive/reverse 车距活敌<20 m 立即刹停；自己的合围目标允许靠到 12 m 内（阵位在 24 m，路径已挖 13 m 洞）。")

h2("7.4 编队与前沿待机")
para("编队采用“沿领航车实际路径后退 spacing×i”的方式，而不是相对朝向偏移：领航车拐弯后，"
     "僚车目标仍在路中间。纵队间距 12 m；楔形横向偏移 4 m，左右交替；切换队形用 τ=0.6 s 一阶滤波。"
     "楔形需要路宽≥2×2.6+1.6=6.8 m，否则退回纵队。")
para("ConvoyController 还做两件事：成员按沿领航路径的弧长排序（离领航车近的在前）；"
     "目标移动<6 m 或距上次下发<1 s 不下发，避免每帧重规划。领航车 kind=strike 时，"
     "僚车最多跟到距目标 rally=200 m，即“跟到交火区外”。")
para("FormationRuntime 的状态：strike 领航优先；无 strike 时，若未 staging_done 且有前沿点，选一辆车去前沿；"
     "否则让未分配车辆跟随。只有在 uav_ready 且存在 UAV 目标或 UGV contact 时才允许推进。")

h2("7.5 开局展开：k-center")
para("deploy.choose_starts 在任务区路网上选 3 个展开点：候选点为离边界≥150 m、在最大连通分量上、"
     "路宽≥5.5 m 的直路骨架点（步长10）；目标是最小化“区内任意路网样本点到最近展开点的最大距离”，"
     "约束两两间距≥180 m。求解用最远点优先贪心（k-center 2 近似），再做最多 2 轮逐点局部替换。"
     "朝向沿道路切线并朝向区域中心。")
para("无传送演示不使用该模块（--no-ugv-deploy），车辆保持在场景原始出生点；"
     "需要自动展开时，full_mission 在 --tactics 且未 --no-teleport 时调用。")

# ===========================================================================
# 8 打击/结束
# ===========================================================================
h1("8  打击、击杀与结束判定")
h2("8.1 武器与伤害")
para("set_weapon 根据目标类型选择弹药：士兵用 5 发子弹（单发伤害约 20，总伤害 100）；"
     "Stryker 用反坦克弹（伤害 3000）；Barricade/Pillbox 与 Building 用 40 mm 炮（伤害 1600）。"
     "代码里士兵必定调用 6 号武器、5 连发；Stryker 只有在 Health>0 时才打。")
h2("8.2 打击循环")
para("开火循环每 2 s 一次，只处理 status 在 arrived/drive/reverse/hold 且 kind=strike 的车。"
     "每台车先 detect_fun，过滤 HOSTILE（Soldier/Stryker/Barricade/Pillbox/Building），"
     "再按目标锁定过滤：自己的合围敌人或目标点 45 m 内的目标；最后按距离选最近的一个开火。"
     "攻击范围 120 m。")
para("合围规则在开火前执行：如果目标被分类为 junction/tee，必须满足“至少 2 个不同方向的实际车辆位置、"
     "每车到阵位<30 m、方向夹角>45°、最多2面”；否则不进入 set_weapon，只打印等待合围。"
     "超时 420 s 后按实际就位数降级。")
h2("8.3 击杀与释放")
para("击杀判定有三条路径：① Health<=0；② Soldier 开火当帧立即标记（5 连发基本必死，尸体可能长期留在探测列表）；"
     "③ 目标从 last_seen 消失后按 engaged_pts 对齐 marker。标记后写入 killed_pts、marker.extra=True，"
     "并在下一拍释放车辆：目标点距阵位<45 m 立即释放；hold 车若目标已不在活敌列表，8 s 后释放。"
     "释放后进入 20 s 冷却，再重新参与派车。")
h2("8.4 结束条件")
table(
    ["条件", "判据", "说明"],
    [
        ["全歼", "killed_pts≥expected_enemies 且所有已发现目标 80 m 内都有击杀点，最近交火>5 s", "优先结束，不等扫描收尾"],
        ["扫描完", "UAV 全部 hold，UGV 无 drive，最近交火>10 s", "扫描结束且已发现目标清空"],
        ["UGV 超时", "从首次派车起 >ugv_deadline", "默认300 s，演示1200 s"],
        ["总时长", ">duration", "默认1800 s，演示1500 s"],
    ],
    widths=[2.5, 9.0, 5.5], size=8.0, align_center=False)
caption("表 8-1  任务结束条件")

# ===========================================================================
# 9 参数总表
# ===========================================================================
h1("9  参数总表（按当前代码）")
para("以下参数均来自当前代码；演示启动器覆盖 full_mission 默认值的地方用“演示/默认”并列。"
     "表中“当前值”是工程调参的第一手依据。")

h2("9.1 任务、时间与派车")
table(
    ["参数", "当前值", "位置 / 说明"],
    [
        ["--duration", "1800 s（演示1500）", "full_mission / run_tactics_demo"],
        ["--ugv-deadline", "300 s（演示1200）", "首次派车后计时"],
        ["--dispatch-gap", "4 s（演示5）", "错峰发车"],
        ["--recon-timeout", "300 s", "侦查超时"],
        ["--min-targets", "3（演示2）", "达到几个目标开始派车"],
        ["--ugv-after", "0 s", "发现后再等"],
        ["--takeoff-timeout", "240 s", "起飞告警"],
        ["--console-every", "5 s", "心跳"],
        ["--attack-range", "120 m", "开火距离"],
        ["--shot-at", "0（演示600）", "第 N 秒存小地图，不是射击参数"],
    ],
    widths=[4.0, 4.0, 8.5], size=7.9, align_center=False)
caption("表 9-1  任务与时间参数")

h2("9.2 无人机")
table(
    ["参数", "当前值", "说明"],
    [
        ["--uav-altitude", "250 m（演示150）", "三种模式统一"],
        ["CRUISE_SPEED", "15 m/s", "巡航"],
        ["DETECT_RADIUS", "300 m", "探测半径"],
        ["CLUSTER_RADIUS", "15 m", "探测聚类"],
        ["DETECT_EVERY", "10 帧", "另加0.5 s兜底"],
        ["SAFETY_MARGIN", "30 m", "禁飞区保护"],
        ["ARENA_MARGIN", "10 m", "arena内缩"],
        ["TRACK_RESERVE", "10 m", "航路额外预留"],
        ["CORNER_OFFSET", "5 m", "可见图角点"],
        ["WAYPOINT_TOL", "8 m", "航点到达"],
        ["ALTITUDE_TOL", "2 m", "高度容差"],
        ["KINEMATIC climb", "12 m/s", "爬升"],
        ["KINEMATIC max step", "6 m/帧", "防瞬移"],
        ["KINEMATIC corner", "30 m", "转角圆弧"],
        ["KINEMATIC resample", "1.5 m", "等距重采样"],
        ["KINEMATIC yaw", "70 °/s", "偏航限速"],
        ["KINEMATIC max dt", "0.15 s", "单帧时长"],
        ["SAFE_LAND_ALT", "40 m", "安全降落"],
    ],
    widths=[4.2, 4.0, 8.3], size=7.9, align_center=False)
caption("表 9-2  UAV 参数")

h2("9.3 战术层")
table(
    ["参数", "当前值", "说明"],
    [
        ["Zone 默认值", "无", "缺 --zone 直接报错"],
        ["--area 内收", "35 m", "扫描/显示"],
        ["CoverageGrid cell", "50 m", "覆盖记账"],
        ["frontier edge", "100 m", "离边界"],
        ["frontier min gap", "120 m", "前沿点间距"],
        ["路口环半径", "35 m", "ring_sectors"],
        ["环采样步长", "5°", "ring_sectors"],
        ["最小扇区", "25°", "去噪"],
        ["分支去重", "30°", "build_slots"],
        ["阵位 standoff", "24 m", "build_slots"],
        ["集结点 hold_back", "40 m", "build_slots"],
        ["阵位夹角去重", "45°", "assign._prune_slots"],
        ["W_BOTTLENECK", "0.50", "assign"],
        ["W_COVERAGE", "0.30", "assign"],
        ["W_WAIT", "0.15", "assign"],
        ["W_SWITCH", "0.05（预留，未计入J）", "assign"],
        ["改进阈值", "20%", "assign.solve"],
        ["最多改派", "2 台", "assign.solve"],
        ["合围最多面", "2", "ENCIRCLE_MAX_SLOTS"],
        ["合围超时", "420 s", "ENCIRCLE_TIMEOUT"],
        ["转向惩罚", "450 m/π", "_initial_turn_penalty"],
    ],
    widths=[4.0, 4.0, 8.5], size=7.9, align_center=False)
caption("表 9-3  战术层参数")

h2("9.4 路线与合围校验")
table(
    ["参数", "当前值", "说明"],
    [
        ["路径穿敌", "12 m", "route_violation"],
        ["路径贴其它敌", "15 m", "route_hits_other"],
        ["接近角容差", "±70°", "最后20 m"],
        ["直连绕行上限", "2.2× / +250 m", "route_violation"],
        ["挖洞绕行上限", "4.5× / +1500 m", "重算路径"],
        ["敌人挖洞半径", "13 m", "plan_from"],
        ["敌人刹停", "20 m（自己目标12）", "ENEMY_STOP_R"],
        ["目标锁定", "45 m", "开火候选过滤"],
        ["合围就位", "30 m", "到阵位"],
        ["释放半径", "45 m", "路口阵位"],
        ["释放宽限", "8 s", "hold 目标不在活敌列表"],
        ["冷却", "20 s", "重新派车"],
    ],
    widths=[4.0, 5.0, 7.5], size=7.9, align_center=False)
caption("表 9-4  路线/合围/释放参数")

h2("9.5 无人车")
table(
    ["参数", "当前值", "说明"],
    [
        ["TARGET_V", "5.0 m/s", "直道"],
        ["SLOW_V", "2.5 m/s", "拐弯"],
        ["SLOW_RANGE", "20 m", "减速距离"],
        ["LOOKAHEAD", "8 m", "Stanley"],
        ["Stanley k", "1", "control.Stanley"],
        ["PID Kp/Ki/Kd", "0.1/0/0", "纵向"],
        ["ARRIVE_DIST", "1.0 m", "打击车"],
        ["follow arrive", "4.0 m", "僚车"],
        ["FRONT_GAP", "18/8 m", "打击/僚车"],
        ["MAX_RUNTIME", "600 s", "单段"],
        ["MAX_REPLAN", "5", "重规划"],
        ["BLOCK_R", "10 px", "路障封格"],
        ["REVERSE_OVERSHOOT", "12 m", "倒车"],
        ["REVERSE_THROTTLE", "-0.7", "倒车油门"],
        ["CLEARANCE", "10 m", "路障挡路"],
        ["DEV_WARN", "5 m", "偏离告警"],
        ["STUCK time", "8 s", "速度<0.3"],
        ["OVERTAKE_TIMEOUT", "3 s", "前车停死"],
        ["OVERTAKE_OFFSET", "5 m", "侧偏"],
        ["OVERTAKE_LEAD", "12 m", "S形进入/退出"],
        ["OVERTAKE_HOLD", "8 m", "越过后保持"],
        ["OVERTAKE_MARGIN", "1 m", "安全余量"],
        ["CAR_HALF_W", "1.5 m", "车半宽"],
        ["YIELD_GAP", "22 m", "让行"],
        ["YIELD_HOLD", "8 s", "普通"],
        ["YIELD_HOLD_HEADON", "12 s", "正面"],
    ],
    widths=[4.2, 4.0, 8.3], size=7.8, align_center=False)
caption("表 9-5  UGV 参数")

h2("9.6 编队、展开点与界面")
table(
    ["参数", "当前值", "说明"],
    [
        ["formation", "column", "默认纵队"],
        ["formation-spacing", "12 m", "车间距"],
        ["formation-lateral", "4 m", "楔形横偏"],
        ["formation-rally", "200 m", "僚车止于目标前"],
        ["楔形触发路宽", "6.8 m", "2×2.6+1.6"],
        ["编队平滑 tau", "0.6 s", "一阶滤波"],
        ["convoy move_tol", "6 m", "滞回"],
        ["convoy min_interval", "1 s", "下发间隔"],
        ["deploy margin", "150 m", "离边界"],
        ["deploy min_sep", "180 m", "两两间距"],
        ["deploy step", "10 px", "候选采样"],
        ["deploy min_width", "5.5 m", "最小路宽"],
        ["view interval", "1.2 s", "小地图刷新"],
        ["view ROI", "±1010 m", "半图显示"],
        ["coverage display", "200 m", "显示网格"],
    ],
    widths=[4.2, 4.0, 8.3], size=7.9, align_center=False)
caption("表 9-6  编队/展开/界面参数")

h2("9.7 演示生成参数")
table(
    ["参数", "当前值", "说明"],
    [
        ["--count", "7", "HalfMap 演示"],
        ["--junction", "2", "2 个敌人放路口"],
        ["--off-road", "14 m", "敌人离路中心"],
        ["--avoid-margin", "330 m", "避开禁飞区"],
        ["--min-sep", "60 m", "敌人最小间距"],
        ["--types", "dynamic.prop.soldier", "演示敌人类型"],
        ["--rect", "半图 [-1900,-1108,189,342]", "撒点范围"],
    ],
    widths=[4.0, 5.0, 7.5], size=8.0, align_center=False)
caption("表 9-7  半图演示撒点参数")

# ===========================================================================
# 10 验证/边界
# ===========================================================================
h1("10  当前验证与实现边界")
h2("10.1 已跑通的链路")
bullets([
    "半图 2019×1380 m、7 个敌人（2 路口）撒点；三机从左侧边界升空，三车保持原位直到 UAV 完全升空并发现目标；",
    "首个路口 E-1488 由 UGV1@W、UGV3@S 两路合围；UGV3 走约 1094 点（约1.1 km）绕行到南侧；",
    "UGV1 在 24 m 距离开火，日志出现“已消灭”，UGV1/UGV3 释放，20 s 冷却后 UGV1 被派往 E-758；",
    "后续路段目标以编队/分裂方式逐批清理；小地图显示 7 个目标三态、覆盖和轨迹；",
    "无传送半图模式不调用 teleport_ugv/展开/看门狗拉回，UGV 从 -1829/-1859/-1887,-517 原始位置起步；",
    "selftest_p0～p4 全部通过；compileall 通过。",
])
h2("10.2 最近修复的三个关键问题")
table(
    ["现象", "根因", "当前处理"],
    [
        ["飞机未升空/UI无敌人，UGV已出发", "ready 用 tol=30（120 m 即算到位）；FormationRuntime 在 el≥3 就推进", "uav_recon_ready 要求无 take/climb；无目标/contact 不推进"],
        ["合围结束后三车锁定", "只等目标从探测列表消失才写 killed_pts；尸体长期可见", "Soldier 开火当帧标记；8 s hold 释放；20 s 冷却"],
        ["最后一个路口三车同侧/战术层回退", "route_violation 对 (x,y,z) 解包异常；busy 目标被排除", "统一二维；保留已指派目标在 pool；按 p[0],p[1] 兼容"],
    ],
    widths=[4.0, 7.0, 6.0], size=7.9, align_center=False)
caption("表 10-1  近期修复")

h2("10.3 仍需明确的边界")
bullets([
    "合围路线可能很长：半图南侧绕行约1.1 km，第三面可达2 km；420 s 超时是一种工程折中，不保证“总能三面”。",
    "UGV 近距探测存在 FOV/遮挡：24 m 阵位在部分角度可能看不到目标，代码用目标锁定/开火距离/当帧击杀标记兜底，"
    "但并非严格传感器模型；真实部署需做射线遮挡与多传感器融合。",
    "当前目标池/击杀依赖 targets.json marker 与瞬时探测；没有正式多目标跟踪器（ID 维持、航迹、置信度）。",
    "道路骨架是 1 px 宽，A* 不是完整车辆运动学规划；Stanley 跟踪 + 局部倒车/超车能跑通演示，"
    "但窄路会车、死胡同、动态障碍仍需更强局部规划。",
    "系统规模固定为 3 UAV/3 UGV；assign 的列数上限 12，目标很多时会截断；没有任务时间窗/优先级/弹药约束。",
    "没有通信延迟/丢包/断链模型；所有决策在单进程内完成，跨进程仅通过 JSON 文件。",
    "武器为 SDK 离散事件，不含弹道、命中概率、友军误伤模型；击杀判定有“当帧标记”的工程假设。",
])
h2("10.4 建议的下一步（按优先级）")
table(
    ["优先级", "建议", "理由"],
    [
        ["P0", "把“最后一处合围补位”做成显式重匹配：已指派目标持续参与 pool，空闲/跟随车随时可补另一面", "解决“先到一辆、后来车不补”的结构性问题"],
        ["P0", "增加目标跟踪器（ID/航迹/置信度），击杀以 Health+消失+多帧证据联合判定", "减少尸体/抖动导致的锁定与误杀"],
        ["P1", "把 2D 骨架 A* 扩展为带车辆转弯半径/车宽的 Hybrid A* 或 lattice", "窄路会车、倒车、合围阵位更可靠"],
        ["P1", "合围路线规划显式生成“分支外侧集结点→阵位”的两段路径，而不是只靠 A* 绕行", "保证从侧翼来，减少长绕行"],
        ["P2", "增加道路预约/会车规则（谁先过、谁让行、超车窗口）", "三车同路/正面相遇仍可能顶死"],
        ["P2", "把 assign 扩展为带时间窗、弹药、风险的多目标优化；保留穷举用于小规模", "任务规模变大后仍可解释"],
        ["P3", "增加通信中间件与失败重试；目前 JSON 文件解耦只适合单机演示", "真实系统需要容错"],
    ],
    widths=[2.0, 8.0, 7.0], size=7.8, align_center=False)
caption("表 10-2  改进路线")

# ===========================================================================
# 11 代码索引
# ===========================================================================
h1("11  代码索引与复现")
h2("11.1 关键函数")
table(
    ["能力", "函数 / 类", "文件（约行）"],
    [
        ["主循环", "main()", "full_mission.py:1594"],
        ["A* 路网", "road_path / control_ugv.astar", "full_mission.py:276 / control_ugv.py:92"],
        ["路线校验", "route_violation / route_hits_other", "full_mission.py:293/305"],
        ["合围匹配", "best_encircle_match", "full_mission.py:374"],
        ["战术决策", "tactics_decide", "full_mission.py:406"],
        ["编队状态机", "FormationRuntime", "full_mission.py:796"],
        ["UGV 执行", "UgvAgent", "control_ugv.py:228"],
        ["分配", "assign.solve", "tactics/assign.py:82"],
        ["路口/阵位", "classify / build_slots", "tactics/slots.py:184/207"],
        ["覆盖", "CoverageGrid", "tactics/coverage.py:18"],
        ["编队几何", "Formation.follow_targets", "tactics/formation.py:183"],
        ["编队控制", "ConvoyController.update", "tactics/convoy.py:78"],
        ["展开点", "choose_starts", "tactics/deploy.py:340"],
        ["扫描航线", "sweep_plan.build", "sweep_plan.py:463"],
        ["UAV 执行", "KinematicUavRunner.step", "uav_runner.py:615"],
        ["UAV 安全", "SafePlanner.route", "control_uav.py:320"],
    ],
    widths=[3.8, 7.0, 6.2], size=7.8, align_center=False)
caption("表 11-1  关键代码索引（行号随代码演进而略有偏移）")

h2("11.2 复现命令")
code("# 半图、无传送、无散开、无自动展开\n"
     "powershell -ExecutionPolicy Bypass -File WorkPlace\\run_tactics_demo.ps1 `\n"
     "  -HalfMap -NoTeleport -NoSpread -NoDeploy\n\n"
     "# 复用当前 targets.json（保持同一敌人分布）\n"
     "powershell -ExecutionPolicy Bypass -File WorkPlace\\run_tactics_demo.ps1 `\n"
     "  -HalfMap -NoSpawn -NoTeleport -NoSpread -NoDeploy\n\n"
     "# 自测\n"
     "python WorkPlace\\tactics\\selftest_p0.py\n"
     "python WorkPlace\\tactics\\selftest_p1.py\n"
     "python WorkPlace\\tactics\\selftest_p2.py\n"
     "python WorkPlace\\tactics\\selftest_p4.py")

h2("11.3 主要配置文件")
table(
    ["文件", "内容"],
    [
        ["config/strike_plan.json", "战术层计划：groups、slots、commitment、idle"],
        ["config/goals.json", "UGV/UAV 显示目标"],
        ["config/detected.json", "UAV 已发现点"],
        ["config/roadnet_meta.json", "路网 scale/offset"],
        ["config/uav_sweep.json", "扫描航线缓存"],
        ["scene/game.json", "arena、禁飞区、雷区、电磁区、阶段线"],
        ["scene/vehicles.json", "3 UGV + 3 UAV 初始位置/朝向"],
        ["scene/targets.json", "敌人类型、数量、位置；expected_enemies 来源"],
    ],
    widths=[5.0, 12.0], size=8.2, align_center=False)
caption("表 11-2  配置文件")

doc.add_paragraph()
p = doc.add_paragraph()
r = p.add_run("附：本报告中的公式、参数和判据均可在当前代码中直接落位；若参数与运行日志不一致，"
              "以启动器显式覆盖值为准，并在日志首部记录。")
set_cn(r, size=8.5, color=(110, 110, 110))

doc.core_properties.title = "无人机—无人车协同侦察打击系统技术汇报"
doc.core_properties.subject = "算法、参数与代码实现分析"
doc.core_properties.author = "项目组"
doc.core_properties.comments = "依据 2026-11-10 当前工作区代码生成"
doc.save(OUT)
print(OUT)
