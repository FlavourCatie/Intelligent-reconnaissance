# -*- coding: utf-8 -*-
"""
env_paths.py —— 课程环境目录的定位与"路径体检"
================================================================================

为什么单独抽一个模块
--------------------
课程包（CarlaUE4 / simu_driver / SDK）**不在仓库里**（10 GB，见 .gitignore），
每个人的放法又不一样：

    A 型（原教旨）：把仓库当成工作区根，课程包直接解到仓库根下
        <repo>/environment_forclass/environment/...
        <repo>/Materials/simu_driver_windows/...

    B 型（我们后来改成这样）：仓库单独 clone 一层，课程包放在工作区根
        <workspace>/environment_forclass/environment/...
        <workspace>/Intelligent-reconnaissance/  ← 仓库

    C 型（防空格用的软链接，见 tools/sim_link.py）
        C:\\kyxz\\environment\\...   指向上面任何一个真实目录

所以"课程包在哪"必须按顺序探测，不能写死相对路径。本模块只做两件事：
    1. `find_course_env()` / `find_simu_driver()` / `find_sitl_dir()`
       按固定优先级找目录，找不到返回 None（不抛异常）。
    2. `path_problems(path)` 检查路径里有没有空格 / 非 ASCII 字符。

为什么要查空格（这是本轮"飞控调不通"的根因）
--------------------------------------------
飞控驱动 `driver.exe` 是打包好的 Python 程序，它启动 ArduPilot SITL 时
把命令行拼成**一个字符串**（不是参数列表），其中 `--defaults <路径>` 没有加引号。
工作区路径一带空格，SITL 收到的参数就被劈开，`--serial0 udpclient:...` 直接丢失，
SITL 退回默认的 TCP 5760，飞控驱动永远收不到 MAVLink 心跳 →
`UAV#n: Connect Timeout!` → 飞机没有动力学 → `control_kinetic_simply()` 返回 200 但零位移。

复现与验证见 `tools/sitl_probe.py`，结论见
`docs/飞控调通_根因与修复.md`。
"""

from __future__ import print_function

import os

#: WorkPlace 目录（本文件的上一级）
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
#: 仓库根目录（WorkPlace 的上一级）
REPO_ROOT = os.path.dirname(WORKPLACE_DIR)
#: 工作区根：仓库再上一层（B 型放法时课程包在这里）
WORKSPACE_ROOT = os.path.dirname(REPO_ROOT)

#: 允许用环境变量直接指定，避免猜错
ENV_COURSE_ENV = "SWARMAE_ENV_ROOT"
ENV_SIMU_DRIVER = "SWARMAE_SIMU_DRIVER"

#: simu_driver 里 SITL 二进制所在子目录名
SITL_DIRS = {"quadrotor": "quadrotor_windows", "fixed_wing": "fixed_wing_windows"}


def _read_env(name):
    value = os.environ.get(name)
    if value and os.path.isdir(value):
        return os.path.abspath(value)
    return None


def _candidates_for_course_env():
    """按优先级返回候选的 `environment` 目录（里面有 simu_driver_windows）。"""
    roots = []
    explicit = _read_env(ENV_COURSE_ENV)
    if explicit:
        # 允许指到 environment / environment_forclass / 工作区根，统一往下降
        for rel in ("", "environment", os.path.join("environment_forclass", "environment")):
            if rel:
                roots.append(os.path.join(explicit, rel))
            else:
                roots.append(explicit)
    for base in (REPO_ROOT, WORKSPACE_ROOT):
        roots.append(os.path.join(base, "environment_forclass", "environment"))
        roots.append(os.path.join(base, "environment"))
    # 防空格软链接（tools/sim_link.py 建的）
    roots.append(os.path.join("C:\\", "kyxz", "environment"))
    return roots


def find_course_env():
    """返回课程 `environment` 目录（含 simu_driver_windows/CarlaUE4），找不到返回 None。"""
    for path in _candidates_for_course_env():
        if os.path.isdir(os.path.join(path, "simu_driver_windows")):
            return os.path.abspath(path)
    return None


def _candidates_for_simu_driver():
    explicit = _read_env(ENV_SIMU_DRIVER)
    if explicit:
        yield explicit
    env = find_course_env()
    if env:
        yield os.path.join(env, "simu_driver_windows")
    for base in (REPO_ROOT, WORKSPACE_ROOT):
        yield os.path.join(base, "Materials", "simu_driver_windows")
    yield os.path.join("C:\\", "kyxz", "environment", "simu_driver_windows")


def find_simu_driver():
    """返回 simu_driver_windows 目录（含 bin/driver/driver.exe），找不到返回 None。"""
    for path in _candidates_for_simu_driver():
        if os.path.isfile(os.path.join(path, "bin", "driver", "driver.exe")):
            return os.path.abspath(path)
    return None


def find_sitl_dir(kind="quadrotor"):
    """返回 SITL 目录（里面有 `model` 二进制），找不到返回 None。

    kind: 'quadrotor'（四旋翼）或 'fixed_wing'（固定翼），与 start.bat 的选项对应。
    """
    driver = find_simu_driver()
    if not driver:
        return None
    sub = SITL_DIRS.get(kind, SITL_DIRS["quadrotor"])
    path = os.path.join(driver, "bin", "driver", "bin", sub)
    if os.path.isfile(os.path.join(path, "model")):
        return os.path.abspath(path)
    return None


def find_driver_config():
    """返回 driver 的 config.yaml（里面有 Type: quadrotor / fixed_wing）。"""
    driver = find_simu_driver()
    if not driver:
        return None
    path = os.path.join(driver, "bin", "driver", "config.yaml")
    return path if os.path.isfile(path) else None


def read_driver_type():
    """读出 config.yaml 里的 Type 字段；读不到返回 None。"""
    path = find_driver_config()
    if not path:
        return None
    try:
        with open(path, "rb") as fp:
            for raw in fp:
                line = raw.decode("utf-8", "replace").strip()
                if line.startswith("Type:"):
                    return line.split(":", 1)[1].strip()
    except Exception:
        return None
    return None


#: tools/sim_link.py 建的"无空格入口"（有它就等于飞控的路径问题已解决）
SPACE_FREE_ALIASES = [
    os.path.join("C:\\", "kyxz", "environment"),
    os.path.join("D:\\", "kyxz", "environment"),
]


def find_space_free_entry():
    """返回一个"无空格且含飞控驱动"的入口目录；没有则 None。"""
    for path in SPACE_FREE_ALIASES:
        if os.path.isfile(os.path.join(path, "simu_driver_windows",
                                       "bin", "driver", "driver.exe")):
            return path
    return None


def path_problems(path):
    """检查路径本身对飞控 / 课程包是否"有毒"。

    返回 [(问题代号, 说明), ...]，代号：
        space     路径含空格 —— 飞控驱动的 SITL 命令行会被劈开（本轮根因）
        nonascii  路径含中文等非 ASCII 字符 —— 部分 C/C++ 组件按 ANSI 码页处理会乱码
    """
    if not path:
        return []
    problems = []
    if " " in path:
        problems.append(("space", "路径含空格：%s" % path))
    try:
        path.encode("ascii")
    except (UnicodeEncodeError, UnicodeDecodeError):
        problems.append(("nonascii", "路径含非 ASCII 字符：%s" % path))
    return problems


if __name__ == "__main__":
    print("REPO_ROOT      =", REPO_ROOT)
    print("WORKSPACE_ROOT =", WORKSPACE_ROOT)
    print("course env     =", find_course_env())
    print("simu_driver    =", find_simu_driver())
    print("SITL(quad)     =", find_sitl_dir("quadrotor"))
    print("driver Type    =", read_driver_type())
    for label, path in (("repo", REPO_ROOT), ("simu_driver", find_simu_driver())):
        for code, msg in path_problems(path):
            print("!! %s: [%s] %s" % (label, code, msg))
