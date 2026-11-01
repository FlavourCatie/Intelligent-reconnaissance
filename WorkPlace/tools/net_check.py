# -*- coding: utf-8 -*-
"""
net_check.py —— 端口占用 / 飞控通道检查
================================================================================

回答两个问题：
    1. 2000（CARLA）到底被谁占用？有没有两个实例打架？
    2. 飞控驱动的三个端口（3301 消息 / 3302 指令 / 4444 主控）有没有在监听？
       —— 如果 3302 没在监听，说明命令根本没进飞控。

做法：纯 Win32 `GetExtendedTcpTable` + Toolhelp 进程快照，**不起子进程**，
所以不会被权限或管道限制影响。

用法
----
    python WorkPlace\\tools\\net_check.py
    python WorkPlace\\tools\\net_check.py --ports 2000,3301,3302,4444
    python WorkPlace\\tools\\net_check.py --all        # 列出所有监听端口

判读
----
    2000  有 1 个 CarlaUE4 在听          → 正常
    2000  有 2 个不同 PID 在听            → 两个仿真器实例，SDK 可能连错
    3301/3302 有 driver 在听              → 飞控在收指令
    3301/3302 没人听                      → 飞控没起来 / 起错目录
    4444  没人在听                        → main-release 没连上飞控
"""

from __future__ import print_function

import argparse
import os
import socket
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WORKPLACE_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, WORKPLACE_DIR)

TCP_TABLE_OWNER_PID_ALL = 5
UDP_TABLE_OWNER_PID = 1
AF_INET = 2
DEFAULT_PORTS = (2000, 3301, 3302, 4444)


# ---------------------------------------------------------------------------
def _listening_tcp():
    """返回 [(addr, port, pid, state, remote_addr, remote_port), ...]；失败返回 None。"""
    if os.name != "nt":
        return None
    try:
        import ctypes
        from ctypes import wintypes

        class MIB_TCPROW_OWNER_PID(ctypes.Structure):
            _fields_ = [("dwState", wintypes.DWORD),
                        ("dwLocalAddr", wintypes.DWORD),
                        ("dwLocalPort", wintypes.DWORD),
                        ("dwRemoteAddr", wintypes.DWORD),
                        ("dwRemotePort", wintypes.DWORD),
                        ("dwOwningPid", wintypes.DWORD)]

        iphlp = ctypes.WinDLL("iphlpapi", use_last_error=True)
        size = wintypes.DWORD(0)
        iphlp.GetExtendedTcpTable(None, ctypes.byref(size), False, AF_INET,
                                  TCP_TABLE_OWNER_PID_ALL, 0)
        if size.value == 0:
            return None
        buf = ctypes.create_string_buffer(size.value)
        ret = iphlp.GetExtendedTcpTable(buf, ctypes.byref(size), False, AF_INET,
                                        TCP_TABLE_OWNER_PID_ALL, 0)
        if ret != 0:
            return None
        count = wintypes.DWORD.from_buffer(buf).value
        rows = []
        offset = 4
        row_size = ctypes.sizeof(MIB_TCPROW_OWNER_PID)

        def htons_port(value):
            return ((value >> 8) & 0xFF) | ((value & 0xFF) << 8)

        def ip_str(value):
            b = value.to_bytes(4, "little")
            return ".".join(str(x) for x in b)

        for i in range(count):
            row = MIB_TCPROW_OWNER_PID.from_buffer(buf, offset + i * row_size)
            rows.append((ip_str(row.dwLocalAddr),
                         htons_port(row.dwLocalPort),
                         int(row.dwOwningPid),
                         int(row.dwState),
                         ip_str(row.dwRemoteAddr),
                         htons_port(row.dwRemotePort)))
        return rows
    except Exception:
        return None


def _listening_udp():
    """返回 [(端口, pid), ...]；失败返回 None。

    飞控的 MsgPort/CmdPort 也可能是 UDP，必须一起查，
    否则会误判成"端口没人监听"。
    """
    if os.name != "nt":
        return None
    try:
        import ctypes
        from ctypes import wintypes

        class MIB_UDPROW_OWNER_PID(ctypes.Structure):
            _fields_ = [("dwLocalAddr", wintypes.DWORD),
                        ("dwLocalPort", wintypes.DWORD),
                        ("dwOwningPid", wintypes.DWORD)]

        iphlp = ctypes.WinDLL("iphlpapi", use_last_error=True)
        size = wintypes.DWORD(0)
        iphlp.GetExtendedUdpTable(None, ctypes.byref(size), False, AF_INET,
                                  UDP_TABLE_OWNER_PID, 0)
        if size.value == 0:
            return None
        buf = ctypes.create_string_buffer(size.value)
        ret = iphlp.GetExtendedUdpTable(buf, ctypes.byref(size), False, AF_INET,
                                        UDP_TABLE_OWNER_PID, 0)
        if ret != 0:
            return None
        count = wintypes.DWORD.from_buffer(buf).value
        rows = []
        offset = 4
        row_size = ctypes.sizeof(MIB_UDPROW_OWNER_PID)
        for i in range(count):
            row = MIB_UDPROW_OWNER_PID.from_buffer(buf, offset + i * row_size)
            port = ((row.dwLocalPort >> 8) & 0xFF) | ((row.dwLocalPort & 0xFF) << 8)
            rows.append((port, int(row.dwOwningPid)))
        return rows
    except Exception:
        return None


def _pid_names():
    """pid -> 进程名（复用 core.preflight 的 Toolhelp 枚举）。"""
    try:
        from core import preflight
        table = preflight._process_table()
    except Exception:
        return {}
    if not table:
        return {}
    out = {}
    for name, pids in table.items():
        for pid in pids:
            out[pid] = name
    return out


def _probe_connect(port, timeout=0.25):
    """主动连一下端口，确认真的能建连（监听了也可能拒连）。"""
    try:
        s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        s.close()
        return True
    except Exception:
        return False


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="端口占用 / 飞控通道检查")
    ap.add_argument("--ports", default=",".join(str(p) for p in DEFAULT_PORTS),
                    help="要重点看的端口，逗号分隔")
    ap.add_argument("--all", action="store_true", help="列出所有监听端口")
    args = ap.parse_args()

    watch = set(int(p) for p in args.ports.split(",") if p.strip())
    rows = _listening_tcp()
    udp_rows = _listening_udp()
    if rows is None:
        print("读不到 TCP 表（非 Windows 或被限制）。请改用：")
        print('  Get-NetTCPConnection -State Listen | findstr ":2000"')
        return 1

    STATE = {1: "CLOSED", 2: "LISTEN", 3: "SYN_SENT", 4: "SYN_RCVD",
             5: "ESTABLISHED", 6: "FIN_WAIT1", 7: "FIN_WAIT2",
             8: "CLOSE_WAIT", 9: "CLOSING", 10: "LAST_ACK",
             11: "TIME_WAIT", 12: "DELETE_TCB"}

    listeners = {}
    for addr, port, pid, state, raddr, rport in rows:
        if state != 2:                      # 只统计 LISTEN
            continue
        listeners.setdefault(port, {}).setdefault("tcp", {}).setdefault(pid, 0)
        listeners[port]["tcp"][pid] += 1
    for port, pid in (udp_rows or []):
        listeners.setdefault(port, {}).setdefault("udp", {})
        listeners[port]["udp"][pid] = listeners[port]["udp"].get(pid, 0) + 1

    names = _pid_names()
    print("=" * 74)
    print("重点端口（TCP 监听 + UDP 绑定）")
    print("=" * 74)
    print("%-6s %-5s %-6s %-18s %s" % ("端口", "协议", "PID", "进程", "TCP可建连"))
    print("-" * 74)
    for port in sorted(watch):
        entry = listeners.get(port, {})
        if not entry:
            print("%-6d %-5s %-6s %-18s %s" % (port, "-", "-", "**无人监听**", "-"))
            continue
        for proto in ("tcp", "udp"):
            for pid in sorted(entry.get(proto, {})):
                name = names.get(pid, "?")
                ok = _probe_connect(port) if proto == "tcp" else "-"
                print("%-6d %-5s %-6d %-18s %s" % (port, proto.upper(), pid, name, ok))

    # ---- 主控链路的实际连接状态（关键：driver 有没有连上 main-release）----
    print()
    print("=" * 74)
    print("主控链路连接（4443/4444）—— driver 必须与 main-release 建立 ESTABLISHED")
    print("=" * 74)
    print("%-6s %-12s %-26s %-24s %s"
          % ("本地口", "状态", "本地", "对端", "进程"))
    print("-" * 74)
    found_conn = False
    for addr, port, pid, state, raddr, rport in rows:
        if port not in (4443, 4444) and rport not in (4443, 4444):
            continue
        if state not in (2, 5):
            continue
        found_conn = True
        print("%-6d %-12s %-26s %-24s %s"
              % (port, STATE.get(state, str(state)),
                 "%s:%d" % (addr, port),
                 "%s:%d" % (raddr, rport),
                 names.get(pid, "?")))
    if not found_conn:
        print("  （没有 4443/4444 的连接记录）")

    est = [r for r in rows if r[1] == 4444 and r[3] == 5]
    print()
    if not est:
        print("  !! 4444 上**没有 ESTABLISHED 连接**：driver 没连上 main-release。")
        print("     这就是飞机不动的原因 —— 飞控动力学从没收到过任何指令。")
        print("     处理：把 start.bat 的两个窗口都关掉，重新运行 start.bat；")
        print("           若还是一样，先单独启动 main-release.exe，等 3~5 秒再启动 driver.exe。")
    else:
        peers = sorted(set(names.get(r[2], str(r[2])) for r in est))
        print("  4444 上有 %d 条 ESTABLISHED 连接，来自: %s"
              % (len(est), ", ".join(peers)))

    print()
    # ---- 2000 端口 ----
    sim = listeners.get(2000, {}).get("tcp", {})
    if len(sim) == 0:
        print("  !! 2000 没有监听：CarlaUE4 没起来（或换了端口）")
    elif len(sim) > 1:
        print("  !! 2000 有 %d 个进程在监听：可能开了两个仿真器实例" % len(sim))
    else:
        pid = list(sim)[0]
        print("  2000 由 %s(pid=%d) 监听，正常" % (names.get(pid, "?"), pid))

    for port, label in ((3302, "飞控指令口"), (3301, "飞控消息口")):
        e = listeners.get(port, {})
        if not e:
            print("  !! %d（%s）无人监听" % (port, label))
        else:
            who = []
            for proto in ("tcp", "udp"):
                for pid in e.get(proto, {}):
                    who.append("%s(%s)" % (names.get(pid, "?"), proto.upper()))
            print("  %d（%s）由 %s 占用" % (port, label, ", ".join(who)))

    if args.all:
        print()
        print("=" * 74)
        print("全部监听端口")
        print("=" * 74)
        for port in sorted(listeners):
            who = []
            for proto in ("tcp", "udp"):
                for p in sorted(listeners[port].get(proto, {})):
                    who.append("%s/%s" % (names.get(p, "?"), proto.upper()))
            print("  %-6d %s" % (port, ", ".join(who)))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
