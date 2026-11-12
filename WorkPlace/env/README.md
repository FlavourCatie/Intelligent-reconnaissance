# 从零配一遍课程环境（Windows / conda）

> 目的：换机器、或者环境被污染到修不回来时，15~30 分钟重建一套**和组内一致**的环境。
> 只写"必须做对"的步骤，坑位说明都放在后面的表里。
> 配套：`env/requirements.txt`（pip 层基线）、`../tools/env_guard.py`（体检）。

---

## 0. 前置

| 项 | 要求 |
|---|---|
| Python | **3.6.13**（64 位）。只有 3.6 能加载课程 SDK（`*.cp36-win_amd64.pyd`） |
| 课程包 | `environment_forclass`（10 GB）解压到**无空格**路径，例如 `C:\kyxz\environment_forclass` |
| 仓库 | 本项目，建议也放在无空格路径（如 `C:\work\Intelligent-reconnaissance`） |
| 网络 | 能用清华镜像；`repo.anaconda.com` / `pypi.org` 不通 |

> 路径一旦带空格，飞控必然超时（`driver.exe` 拼 SITL 命令行的老问题）。
> 已经放在带空格路径了？不用搬，建个联接即可：
> `python WorkPlace\tools\sim_link.py --create`

---

## 1. 建 conda 环境（走清华，别让 conda 去连官方源）

```powershell
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
conda config --set show_channel_urls yes
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

conda create -n kyxz python=3.6 -y --override-channels `
  -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
```

**为什么必须 `--override-channels`**：`.condarc` 里的 `defaults` 展开就是
`repo.anaconda.com`，不可达，会让求解卡满 15 分钟超时。
（若 `.condarc` 里有 `defaults` 或 `repo.anaconda.com` 条目，删掉。）

---

## 2. pip 层（精确版本，别升级）

```powershell
conda activate kyxz
python -m pip install -r WorkPlace\env\requirements.txt
```

装的是：`setuptools 44.1.0`、`numpy 1.19.5`、`Pillow 7.0.0`、`requests 2.22.0`、
`matplotlib 3.3.4`、`scipy 1.5.4`、`opencv-python 4.4.0.40`、`pygame 2.6.1`。

**为什么先装 setuptools 44.1.0**：新 setuptools 已经删掉 `easy_install` 命令，
而第 3 步必须用它（`.egg` 是 old-style）。

---

## 3. 课程 SDK（两个 egg，必须 easy_install）

```powershell
cd "C:\kyxz\environment\sdk\windows"      # 换成你的课程包路径

python -m pip install setuptools==44.1.0
easy_install -i https://pypi.tuna.tsinghua.edu.cn/simple swarmae_lib-0.1.1-py3.6-win-amd64.egg
easy_install -i https://pypi.tuna.tsinghua.edu.cn/simple swarmae-1.1-py3.6-win-amd64.egg
```

两条坑：

1. `easy_install` **不读 pip 的镜像配置**，必须显式 `-i`；
2. egg 的真实导入名不是 `carla` 而是 **`swarmae_lib`**（元数据里写的 `Name: carla` 是历史遗留）。

验证：

```powershell
python -c "import swarmae, swarmae_lib; from swarmae.SwarmAENode import SwarmAENode; print('SDK OK')"
```

---

## 4. 体检

```powershell
python WorkPlace\tools\env_guard.py
```

应当只有两类 WARN：仿真器/驱动没启动、`launch_kinetic` 是空壳（正常，V1.4 已删除）。
**不该有 FAIL。** 有 FAIL 就按它打印的"建议动作"处理。

确认干净后，存一份基线：

```powershell
python WorkPlace\tools\env_guard.py --snapshot WorkPlace\env\env_snapshot.json
```

---

## 5. 常见坑位速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `conda create` 卡 15 分钟 | 求解器去连 `repo.anaconda.com` | 用 `--override-channels -c <清华 main>` |
| `easy_install` 找不到命令 | setuptools ≥ 45 已移除 | 先装 `setuptools==44.1.0` |
| `import carla` 失败 | egg 顶层目录其实叫 `swarmae_lib` | `import swarmae_lib` |
| `ModuleNotFoundError: swarmae` | 用了别的 Python（如系统 3.14） | `conda activate kyxz` 再跑 |
| `ModuleNotFoundError: skimage` | `Materials/practice/easy_follow/lib/image_process.py` 依赖 scikit-image，**本机 kyxz 里没装**（组内不一致） | 统一后补进 `env/requirements.txt`（3.6 可用 `scikit-image==0.17.2`）；在此之前离线自测 `selftest/run_main.py` 跑不起来 |
| `SyntaxError` / `ImportError: DLL load failed` | Python 不是 3.6 / 不是 64 位 | 重建 3.6 环境 |
| 飞控 `UAV#n: Connect Timeout` | 路径带空格 | `python WorkPlace\tools\sim_link.py --create` |
| 端口 2000 两个监听者 | 开了两个 CarlaUE4 | `python WorkPlace\tools\kill_all.py` |
| 场景里没有车/机 | `subject4.exe` 没起或起早/起晚 | 顺序：CarlaUE4 → start.bat → subject4.exe |
| VSCode 解释器不对 | 默认解释器没指到 kyxz | 选 `kyxz`，或在 `.vscode/settings.json` 里指过去 |

---

## 6. 不要做的事

- 不要 `pip install -U` 任何已在基线里的包；
- 不要把与本课程无关的工具装进 `kyxz`（另建一个 `tools` 环境）；
- 不要改课程包里的 `bin/driver/config.yaml` 手工调机型（用 `start.bat` 选 1/2）；
- 不要把课程包、`.exe`、日志、截图提交进仓库（`.gitignore` 已覆盖，别 `-f`）；
- 不要在带空格路径下启动 `start.bat`（飞控会静默失效）。
