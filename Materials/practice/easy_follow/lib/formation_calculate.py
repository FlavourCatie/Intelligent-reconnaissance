# lib/formation_calculate.py
# FormationToolkit：队形计算 + 参考路径构建/偏移 + 距离工具
# 坐标与符号：
# - 队形 (x, y)：x 向右为正，y 向前为正
# - offset_path 参数：
#   * lateral_offset：+左 / -右
#   * longitudinal_offset：+前 / -后（沿弧长 s 轴）
# 映射：lateral_offset = -x；longitudinal_offset = +y



import math
import numpy as np
from scipy.interpolate import CubicSpline
import lib.frenet_optimal_planner as fop
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt

class ReferencePath:
    """轻量级 ReferencePath：插值/等距重采样 + 保存 s/x/y/theta"""
    def __init__(self, interp_x_, interp_y_, interp_s_, interp_theta_, sample_resolution_):
        self.interp_x_ = interp_x_
        self.interp_y_ = interp_y_
        self.interp_s_ = interp_s_
        self.interp_theta_ = interp_theta_
        self.sample_resolution_ = float(sample_resolution_)

class FormationToolkit:
    def __init__(self):
        pass

    # ===================== 队形计算 =====================
    def calculate_formation_position(self, num_vehicles, radius, formation_type):
        """
        formation_type:
          1: 纵向一字（沿 y 方向展开）
          2: 横向一字（沿 x 方向展开）
          3: 正多边形，顶点朝纵向（0° 对准 y+）
          4: 正多边形，顶点朝横向（0° 对准 x+）
        返回：[(x, y), ...]，x 向右 +，y 向前 +
        """
        pos = []
        if num_vehicles <= 0:
            return pos
        if num_vehicles == 1:
            return [(0.0, 0.0)]

        if formation_type == 1:
            spacing = 2.0 * radius / max(1, num_vehicles - 1)
            for i in range(num_vehicles):
                x = 0.0
                y = -radius + i * spacing
                pos.append((x, y))
        elif formation_type == 2:
            spacing = 2.0 * radius / max(1, num_vehicles - 1)
            for i in range(num_vehicles):
                x = -radius + i * spacing
                y = 0.0
                pos.append((x, y))
        elif formation_type == 3:
            step = 2.0 * math.pi / num_vehicles
            for i in range(num_vehicles):
                ang = i * step
                x = radius * math.sin(ang)
                y = radius * math.cos(ang)
                pos.append((x, y))
        elif formation_type == 4:
            step = 2.0 * math.pi / num_vehicles
            for i in range(num_vehicles):
                ang = i * step + math.pi / num_vehicles
                x = radius * math.sin(ang)
                y = radius * math.cos(ang)
                pos.append((x, y))
        else:
            # 默认横向一字
            spacing = 2.0 * radius / max(1, num_vehicles - 1)
            for i in range(num_vehicles):
                x = -radius + i * spacing
                y = 0.0
                pos.append((x, y))
        return pos

    def formation_offsets_from_positions(self, positions):
        """
        (x, y) -> (lateral_offset, longitudinal_offset)
        规则：lateral_offset = -x；longitudinal_offset = +y
        """
        offsets = []
        for x, y in positions:
            offsets.append((-x, +y))
        return offsets

    # ===================== 参考路径构建（更鲁棒，含去重） =====================
    def make_reference_path(self, x, y, resolution=1.0):
        """
        根据输入稀疏点 (x, y) 构建等弧长采样的参考路径。
        自动剔除重复/极短平段，保证样条自变量严格递增。
        """
        x = np.asarray(x, dtype=float).ravel()
        y = np.asarray(y, dtype=float).ravel()
        if not (len(x) == len(y) and len(x) >= 2):
            raise ValueError("路径点至少 2 个，且 x/y 等长")

        # 先去掉相邻极短段（平段或重复点）
        eps = 1e-9
        keep = [0]
        for i in range(1, len(x)):
            if (abs(x[i] - x[keep[-1]]) > eps) or (abs(y[i] - y[keep[-1]]) > eps):
                keep.append(i)
        x = x[keep]
        y = y[keep]
        if len(x) < 2:
            raise ValueError("输入路径经去重后点数 < 2，无法构建样条。")

        # 严格递增的弧长
        s_raw = np.zeros_like(x)
        for i in range(1, len(x)):
            s_raw[i] = s_raw[i-1] + math.hypot(x[i] - x[i-1], y[i] - y[i-1])

        # 若仍有数值上非严格递增，再压缩
        keep2 = [0]
        for i in range(1, len(s_raw)):
            if s_raw[i] - s_raw[keep2[-1]] > eps:
                keep2.append(i)
        x = x[keep2]; y = y[keep2]; s_raw = s_raw[keep2]
        if len(x) < 2:
            raise ValueError("弧长仍非严格递增，无法构建样条。")

        # 样条（过点）
        sx = CubicSpline(s_raw, x, bc_type='natural')
        sy = CubicSpline(s_raw, y, bc_type='natural')

        total = float(s_raw[-1])
        npts = max(2, int(total / max(1e-3, resolution)) + 1)
        s_uni = np.linspace(0.0, total, npts)

        x_i = sx(s_uni)
        y_i = sy(s_uni)
        dx = sx(s_uni, 1)
        dy = sy(s_uni, 1)
        theta = np.arctan2(dy, dx)

        sample_resolution = float(total) / (npts - 1) if npts > 1 else float(total)
        return ReferencePath(
            x_i.reshape(-1, 1),
            y_i.reshape(-1, 1),
            s_uni.reshape(-1, 1),
            theta.reshape(-1, 1),
            sample_resolution
        )

   
    def offset_path(self, reference_path, lateral_offset=0.0, longitudinal_offset=0.0):
        """
        对参考路径进行横向（左右）和纵向（前后）偏移。

        参数:
        - reference_path: ReferencePath 对象，原始路径。
        - lateral_offset: 横向偏移量（左右），正值向左，负值向右。
        - longitudinal_offset: 纵向偏移量（前后），正值向前，负值向后。

        返回:
        - ReferencePath 对象，偏移后的路径。
        """
        # 获取原始路径点和法线方向
        interp_x = np.ravel(reference_path.interp_x_)  # 路径的 x 坐标
        interp_y = np.ravel(reference_path.interp_y_)  # 路径的 y 坐标
        interp_theta = np.ravel(reference_path.interp_theta_)  # 路径的航向角

        # 横向偏移：计算每个点在法线方向上的新位置
        offset_x = interp_x + lateral_offset * np.cos(interp_theta + np.pi / 2)
        offset_y = interp_y + lateral_offset * np.sin(interp_theta + np.pi / 2)

        # 纵向偏移：调整弧长并重新插值
        interp_s = reference_path.interp_s_ + longitudinal_offset
        spline_x = BSpline(np.ravel(reference_path.interp_s_), interp_x, 1)
        spline_y = BSpline(np.ravel(reference_path.interp_s_), interp_y, 1)

        # 插值点重新计算
        final_x = spline_x(interp_s)
        final_y = spline_y(interp_s)

        # 综合横向和纵向偏移的结果
        final_x = final_x + lateral_offset * np.cos(interp_theta + np.pi / 2)
        final_y = final_y + lateral_offset * np.sin(interp_theta + np.pi / 2)

        # 确保结果符合 ReferencePath 的输入格式
        final_x = np.mat(final_x).T
        final_y = np.mat(final_y).T

        return fop.ReferencePath(final_x, final_y, reference_path.sample_resolution_)

    # ===================== 最近点与带符号距离 =====================
    def closest_index_on_ref(self, ref, x, y):
        """L1 距离的最近插值点索引（与你原逻辑一致）"""
        dx = np.ravel(ref.interp_x_) - x
        dy = np.ravel(ref.interp_y_) - y
        dist = np.abs(dx) + np.abs(dy)
        return int(np.argmin(dist))

    def calculate_signed_path_distance(self, ref_path, index_1, index_2):
        """
        按你给的实现风格：累加相邻点的欧氏距离。
        注意：保持你那行返回符号（index_1 <= index_2 返回负值）。
        """
        start_index = min(index_1, index_2)
        end_index = max(index_1, index_2)
        distance = 0.0
        for i in range(start_index, end_index):
            dx = ref_path.interp_x_[i + 1] - ref_path.interp_x_[i]
            dy = ref_path.interp_y_[i + 1] - ref_path.interp_y_[i]
            distance += float(np.sqrt(dx**2 + dy**2))
        return -distance if index_1 <= index_2 else distance
    
    # 作图
    def plot_jidong_debug(self, base_x, base_y, ref_paths, states, hist_x, hist_y):
        plt.clf()
        plt.plot(base_x, base_y, linewidth=2, label='Base Path')

        cmap = plt.get_cmap('tab10')
        for i, ref_i in enumerate(ref_paths):
            rx, ry = [], []
            for k in range(len(ref_i.interp_x_)):
                try: rx.append(float(ref_i.interp_x_[k, 0]))
                except Exception: rx.append(float(ref_i.interp_x_[k]))
                try: ry.append(float(ref_i.interp_y_[k, 0]))
                except Exception: ry.append(float(ref_i.interp_y_[k]))
            plt.plot(rx, ry, linestyle='--', linewidth=1.2,
                     color=cmap(i % 10), label=f'UGV{i} ref')

        for i, st in enumerate(states):
            if i < len(hist_x) and len(hist_x[i]) > 1:
                plt.plot(hist_x[i], hist_y[i], linewidth=1.2,
                         color=cmap(i % 10), alpha=0.8)
            px, py = st.pose_.x_, st.pose_.y_
            plt.scatter([px], [py], s=35, color=cmap(i % 10),
                        edgecolors='k', zorder=3)
            plt.text(px, py, f" {i}", color=cmap(i % 10), fontsize=9)

        plt.axis('equal'); plt.grid(True, linestyle=':')
        plt.title('JiDong (Leader = UGV0)')
        plt.xlabel('X'); plt.ylabel('Y')
        plt.legend(loc='best'); plt.pause(0.001)
    
    # 队形监听
    def update_formation(self, task_msg, ugvs, ref_path,
                         last_ftype, last_fradius):
        formation_type   = getattr(task_msg, "formation_type", 1)
        formation_radius = getattr(task_msg, "formation_radius", 6.0)
        if (formation_type != last_ftype) or (formation_radius != last_fradius):
            print(f"[FORMATION] type={formation_type} radius={formation_radius:.2f}")
            last_ftype, last_fradius = formation_type, formation_radius

        positions = self.calculate_formation_position(len(ugvs), formation_radius, formation_type)
        formation_offsets = self.formation_offsets_from_positions(positions)

        ref_paths = [self.offset_path(ref_path, lat, lon) for lat, lon in formation_offsets]

        leader_lon = formation_offsets[0][1]
        desired_s_list = [-(lon_i - leader_lon) for _, lon_i in formation_offsets]

        return ref_paths, desired_s_list, last_ftype, last_fradius

    # 控制
    def control_step(self, ref_paths, desired_s_list, ref_path, s_arr_base,
                     ugvs, stanley, pid, dt, base_target_v, v_gain,
                     lookahead, step, print_every, hist_x, hist_y):

        states = []
        for i_car, ugv in enumerate(ugvs):
            st = ugv.get_state()
            states.append(st)
            hist_x[i_car].append(st.pose_.x_)
            hist_y[i_car].append(st.pose_.y_)

        idx_lead = self.closest_index_on_ref(ref_path,
                                             states[0].pose_.x_, states[0].pose_.y_)
        s_lead   = s_arr_base[idx_lead]

        ctrl_cmds = []
        for i_car, (ugv, state, ref_i) in enumerate(zip(ugvs, states, ref_paths)):
            front_x = state.pose_.x_ + lookahead * math.cos(state.pose_.theta_)
            front_y = state.pose_.y_ + lookahead * math.sin(state.pose_.theta_)
            min_idx = self.closest_index_on_ref(ref_i, front_x, front_y)

            target_x  = float(ref_i.interp_x_[min_idx])
            target_y  = float(ref_i.interp_y_[min_idx])
            target_th = float(ref_i.interp_theta_[min_idx])

            if i_car == 0:
                target_v_i = base_target_v
                dist_err = 0.0
            else:
                idx_i = self.closest_index_on_ref(ref_path, state.pose_.x_, state.pose_.y_)
                s_i   = s_arr_base[idx_i]
                s_lead_to_i = s_i - s_lead
                desire_s = desired_s_list[i_car]
                dist_err = s_lead_to_i - desire_s
                target_v_i = base_target_v - v_gain * dist_err
                target_v_i = max(0.5, min(12.0, target_v_i))

            delta, lat_err = stanley.StanleyControl(
                target_x, target_y, target_th,
                state.pose_.x_, state.pose_.y_, state.pose_.theta_,
                state.v_, target_v_i
            )
            accel = pid.update_control(target_v_i, state.v_, dt)

            steer = float(delta / 1.8)
            throttle, brake = (min(accel * 0.8, 1.0), 0.0) if accel > 0.0 else (0.0, min(-accel * 0.5, 1.0))
            ctrl_cmds.append((steer, throttle, brake))

            if step % print_every == 0:
                if i_car == 0:
                    print(f"[LEAD] s={s_lead:.2f} v={state.v_:.2f} v*={target_v_i:.2f} lat_err={lat_err:.2f}")
                else:
                    print(f"[UGV{i_car:02d}] dist_err={dist_err:+.2f} "
                          f"v={state.v_:.2f} v*={target_v_i:.2f}")

        for ugv, (steer, throttle, brake) in zip(ugvs, ctrl_cmds):
            ugv.apply_control(steer, throttle, brake, False)

        return states
