# -*- coding: utf-8 -*-
"""strike_plan.json 的读写：原子写 + mtime 热加载。

沿用现有 goals.json 的机制（先写临时文件再 os.replace），读侧用 mtime 判断是否变化，
这样决策层与执行层可以分进程、也能被小地图直接读取。

用法：
    store = PlanStore(r"WorkPlace\config\strike_plan.json")
    store.write(plan_dict)            # 原子写
    if store.changed():               # 只在文件变化时重新加载
        plan = store.load()
"""

from __future__ import print_function

import json
import os
import tempfile
import time

SCHEMA_VERSION = 1


def _default_plan():
    return {
        "schema": SCHEMA_VERSION,
        "plan_id": 0,
        "t_plan": 0.0,
        "line": {"leader": None, "members": [], "formation": "none", "anchor": None},
        "groups": [],
        "idle": [],
        "commitment": {},
    }


class PlanStore(object):
    """一个文件 + mtime 缓存。"""

    def __init__(self, path):
        self.path = os.path.abspath(path)
        self._mtime = None
        self._cache = None

    # ---------------- 写 ----------------
    def write(self, plan, bump_id=True):
        """原子写：同目录临时文件 → os.replace。返回写入的 plan。"""
        plan = dict(plan or {})
        plan.setdefault("schema", SCHEMA_VERSION)
        if bump_id:
            base = plan.get("plan_id")
            if not base and os.path.isfile(self.path):
                try:
                    with open(self.path, "r", encoding="utf-8") as _fp:
                        base = json.load(_fp).get("plan_id") or 0
                except Exception:
                    base = 0
            try:
                plan["plan_id"] = int(base or 0) + 1
            except Exception:
                plan["plan_id"] = 1
        plan["t_plan"] = float(plan.get("t_plan") or time.time())
        for key in ("line", "groups", "idle", "commitment"):
            plan.setdefault(key, _default_plan()[key])

        d = os.path.dirname(self.path)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        fd, tmp = tempfile.mkstemp(prefix=".strike_plan_", suffix=".json", dir=d or ".")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fp:
                json.dump(plan, fp, ensure_ascii=False, indent=2)
                fp.flush()
                os.fsync(fp.fileno())
            os.replace(tmp, self.path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        self._mtime = os.path.getmtime(self.path)
        self._cache = plan
        return plan

    # ---------------- 读 ----------------
    def _file_mtime(self):
        try:
            return os.path.getmtime(self.path)
        except OSError:
            return None

    def changed(self):
        """只判断、不改状态（否则 load() 会拿到旧缓存）。"""
        return self._file_mtime() != self._mtime

    def load(self, force=False):
        """返回计划 dict；文件不存在时返回空模板。"""
        m = self._file_mtime()
        if not force and self._cache is not None and m == self._mtime:
            return self._cache
        if m is None:
            self._cache = _default_plan()
            self._mtime = None
            return self._cache
        try:
            with open(self.path, "r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception:
            # 写到一半的坏文件：退回上一次缓存，不抛异常打断任务
            return self._cache if self._cache is not None else _default_plan()
        if not isinstance(data, dict):
            return _default_plan()
        for key, val in _default_plan().items():
            data.setdefault(key, val)
        self._mtime = m
        self._cache = data
        return data

    def clear(self):
        try:
            os.unlink(self.path)
        except OSError:
            pass
        self._cache = None
        self._mtime = None


# ---------------- 便捷构造 ----------------
def make_plan(groups=None, idle=None, line=None, commitment=None, t=None):
    plan = _default_plan()
    plan["groups"] = list(groups or [])
    plan["idle"] = list(idle or [])
    if line:
        plan["line"] = dict(line)
    if commitment:
        plan["commitment"] = dict(commitment)
    if t is not None:
        plan["t_plan"] = float(t)
    return plan


def make_group(target_id, kind, slots, junction_degree=0, gap_branch=None,
               t_star=None, go_at=None, state="S3"):
    return {
        "target_id": target_id,
        "kind": kind,
        "junction_degree": int(junction_degree),
        "gap_branch": gap_branch,
        "slots": list(slots),
        "t_star": t_star,
        "go_at": go_at,
        "fire_rule": "all_arrived_minus_gap" if kind == "junction" else "single",
        "state": state,
    }


def make_slot(vid, branch, pos, hold, eta=None):
    return {"vid": vid, "branch": branch,
            "pos": [float(pos[0]), float(pos[1])],
            "hold": [float(hold[0]), float(hold[1])],
            "eta": eta}
