# -*- coding: utf-8 -*-
"""
sdk_boot.py —— 安全地创建 SwarmAEClient（避开 SDK 自己的 argparse）
================================================================================

踩过的坑
--------
课程 SDK 的 `SwarmAEClient.__init__` 内部会调用 `__get_args()`，
用 argparse 直接解析**整个 `sys.argv`**，它只认识这些参数：

    -i/--ip  -p/--port  -s/--subject  -n/--number  -t/--team

于是只要我们的脚本带了任何自己的参数，构造客户端时就会炸：

    usage: full_mission.py [-h] [-i IP] [-p PORT] [-s SUBJECT] [-n NUMBER] [-t TEAM]
    full_mission.py: error: unrecognized arguments: --uav-mode kinematic

（不带参数时正常，所以这个坑很容易被忽略——组员交付的
 `control_uav.connect_uavs()` 里也为此专门做了同样的规避。）

用法
----
    from core.sdk_boot import make_client
    client = make_client(host, port)      # 取代直接 SwarmAEClient(...)

`make_client` 在构造期间把 `sys.argv` 临时截成 `[argv0]`，构造完立刻还原，
所以调用方照常可以解析自己的参数。
"""

from __future__ import print_function

import contextlib
import sys


@contextlib.contextmanager
def quiet_argv():
    """在 with 块内把 sys.argv 缩成只剩程序名，退出时无条件还原。"""
    saved = sys.argv
    try:
        sys.argv = saved[:1]
        yield
    finally:
        sys.argv = saved


def make_client(ue_ip="127.0.0.1", ue_port=2000):
    """创建 SwarmAEClient，避开它内部对 sys.argv 的 argparse 解析。"""
    from swarmae.SwarmAEClient import SwarmAEClient
    with quiet_argv():
        return SwarmAEClient(ue_ip=ue_ip, ue_port=ue_port)


def register_node(client, **kwargs):
    """注册节点也走一遍保护（个别版本在 register 时也会碰 sys.argv）。"""
    with quiet_argv():
        return client.register_node(**kwargs)


if __name__ == "__main__":
    # 自检：只验证 argv 保护逻辑，不连仿真器
    original = ["prog", "--foo", "1", "--bar=2"]
    sys.argv = list(original)
    with quiet_argv():
        inside = list(sys.argv)
    print("with 内 argv =", inside, "（应为只有 ['prog']）")
    print("with 外 argv =", sys.argv, "（应与进入前一致）")
    assert inside == ["prog"]
    assert sys.argv == original
    print("argv 保护逻辑 OK")
