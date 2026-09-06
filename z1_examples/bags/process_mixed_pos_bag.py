#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
处理 ROS1 mixed_pos 实验 bag，并输出以下内容：
1. 总览图：aruco flag / K_aruco_pos_CTRL / Touch 输入
2. 各位姿源（target/current/end_effector）的时间序列图
3. 灵敏度拟合图（x/y/z 三轴）
4. 轨迹对比图（XY / XZ）
5. summary.csv：各段斜率、截距、R²、均值 K 等指标

使用前提：
- ROS1 环境（Noetic/Melodic 等）
- 可 import rosbag
- bag 中至少包含：
  /Touch_Pose_increment                  std_msgs/Float64MultiArray
  /aruco_single/aruco_detected_flag     std_msgs/Bool
  /ttttttttarget_pose                   geometry_msgs/PoseStamped
可选：
  /debug/current_pose                   geometry_msgs/PoseStamped
  /end_effector_pose                    geometry_msgs/PoseStamped
  /debug/k_aruco_pos_ctrl               std_msgs/Float64
  /Geomagic/joy                         sensor_msgs/Joy

示例：
source /opt/ros/noetic/setup.bash
python3 process_mixed_pos_bag.py \
    --bag /home/v233/z1_ws/src/z1_ros/z1_examples/bags/mixed_pos_exp/mixed_pos_trial_03.bag \
    --outdir /home/v233/z1_ws/src/z1_ros/z1_examples/bags/mixed_pos_figs_03_resized \
    --k-touch 0.001

如果自动选段不合适，可手动指定窗口：
python3 process_mixed_pos_bag.py \
    --bag mixed_pos_trial_01.bag \
    --outdir mixed_pos_figs_manual \
    --k-touch 0.001 \
    --off 12.0 22.0 \
    --on  35.0 45.0
"""

import os
import argparse
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 版式设定：小四约 12 pt；小三约 15 pt
ANNOTATION_FONT_SIZE = 20
LEGEND_FONT_SIZE = 20
SENSITIVITY_LEGEND_FONT_SIZE = 16
TITLE_FONT_SIZE = 24
LINE_COLORS = {
    # 降低饱和度后的三轴配色：红 / 绿 / 蓝
    "x": "#B55D5D",
    "y": "#5F8F68",
    "z": "#5E83B3",
}


def setup_matplotlib_font(font_name=None):
    """设置 matplotlib 中文字体；可手动指定，或自动从常见中文字体中选择。"""
    candidates = []
    if font_name:
        candidates.append(font_name)
    candidates.extend([
        'Noto Sans CJK SC',
        'Noto Serif CJK SC',
        'WenQuanYi Zen Hei',
        'WenQuanYi Micro Hei',
        'Source Han Sans SC',
        'Source Han Serif SC',
        'AR PL UMing CN',
        'AR PL UKai CN',
        'SimHei',
        'Microsoft YaHei',
        'Arial Unicode MS',
        'DejaVu Sans',
    ])

    available = {f.name for f in font_manager.fontManager.ttflist}
    chosen = None
    for name in candidates:
        if name in available:
            chosen = name
            break

    if chosen is not None:
        plt.rcParams['font.sans-serif'] = [chosen, 'DejaVu Sans']
        print(f'[INFO] Matplotlib 中文字体: {chosen}')
    else:
        print('[WARN] 未检测到可用中文字体，中文可能显示为方框。')
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans']

    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['font.size'] = ANNOTATION_FONT_SIZE
    plt.rcParams['axes.titlesize'] = TITLE_FONT_SIZE
    plt.rcParams['axes.titleweight'] = 'bold'
    plt.rcParams['axes.labelsize'] = ANNOTATION_FONT_SIZE
    plt.rcParams['xtick.labelsize'] = ANNOTATION_FONT_SIZE
    plt.rcParams['ytick.labelsize'] = ANNOTATION_FONT_SIZE
    plt.rcParams['legend.fontsize'] = LEGEND_FONT_SIZE

try:
    import rosbag
except ImportError as e:
    raise SystemExit(
        "无法导入 rosbag。请先 source 你的 ROS1 环境，例如：\n"
        "source /opt/ros/noetic/setup.bash\n"
        "然后再运行本脚本。"
    ) from e


TOPICS = {
    "touch_inc": "/Touch_Pose_increment",
    "flag": "/aruco_single/aruco_detected_flag",
    "target_pose": "/ttttttttarget_pose",
    "current_pose": "/debug/current_pose",
    "ee_pose": "/end_effector_pose",
    "k_ctrl": "/debug/k_aruco_pos_ctrl",
    "joy": "/Geomagic/joy",
}

POSE_SOURCES = {
    "target": TOPICS["target_pose"],
    "current": TOPICS["current_pose"],
    "ee": TOPICS["ee_pose"],
}


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def as_df(rows, columns=None):
    if len(rows) == 0:
        return pd.DataFrame(columns=columns if columns is not None else [])
    df = pd.DataFrame(rows)
    if "t" in df.columns:
        df = df.sort_values("t").reset_index(drop=True)
    return df


def read_bag(bag_path: str):
    """读取 bag 并转成 DataFrame。"""
    bag = rosbag.Bag(bag_path, "r")
    t0 = bag.get_start_time()
    t_end = bag.get_end_time()

    store = defaultdict(list)
    wanted_topics = list(TOPICS.values())

    for topic, msg, t in bag.read_messages(topics=wanted_topics):
        ts = t.to_sec() - t0

        if topic == TOPICS["touch_inc"]:
            data = list(msg.data)
            if len(data) >= 3:
                store["touch_inc"].append({
                    "t": ts,
                    "delta_x": float(data[0]),
                    "delta_y": float(data[1]),
                    "delta_z": float(data[2]),
                })

        elif topic == TOPICS["flag"]:
            store["flag"].append({"t": ts, "flag": int(bool(msg.data))})

        elif topic == TOPICS["k_ctrl"]:
            store["k_ctrl"].append({"t": ts, "k": float(msg.data)})

        elif topic in (TOPICS["target_pose"], TOPICS["current_pose"], TOPICS["ee_pose"]):
            key = [k for k, v in TOPICS.items() if v == topic][0]
            store[key].append({
                "t": ts,
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "z": float(msg.pose.position.z),
                "qx": float(msg.pose.orientation.x),
                "qy": float(msg.pose.orientation.y),
                "qz": float(msg.pose.orientation.z),
                "qw": float(msg.pose.orientation.w),
            })

        elif topic == TOPICS["joy"]:
            buttons = list(msg.buttons)
            grey = int(buttons[0]) if len(buttons) > 0 else 0
            white = int(buttons[1]) if len(buttons) > 1 else 0
            store["joy"].append({"t": ts, "grey": grey, "white": white})

    bag.close()

    data = {
        "touch_inc": as_df(store["touch_inc"], ["t", "delta_x", "delta_y", "delta_z"]),
        "flag": as_df(store["flag"], ["t", "flag"]),
        "k_ctrl": as_df(store["k_ctrl"], ["t", "k"]),
        "target_pose": as_df(store["target_pose"], ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]),
        "current_pose": as_df(store["current_pose"], ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]),
        "ee_pose": as_df(store["ee_pose"], ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]),
        "joy": as_df(store["joy"], ["t", "grey", "white"]),
        "t_start": 0.0,
        "t_end": float(t_end - t0),
    }
    return data


def step_hold_sample(event_t, event_v, query_t, default=None):
    """零阶保持采样：使用 query_t 时刻之前最近一次事件值。"""
    event_t = np.asarray(event_t, dtype=float)
    event_v = np.asarray(event_v)
    query_t = np.asarray(query_t, dtype=float)

    if event_t.size == 0:
        if default is None:
            return np.full_like(query_t, np.nan, dtype=float)
        return np.full(query_t.shape, default, dtype=float)

    idx = np.searchsorted(event_t, query_t, side="right") - 1
    if default is None:
        idx = np.clip(idx, 0, len(event_t) - 1)
        return event_v[idx]

    out = np.full(query_t.shape, default, dtype=float)
    valid = idx >= 0
    idx_valid = np.clip(idx[valid], 0, len(event_t) - 1)
    out[valid] = event_v[idx_valid]
    return out


def linear_interp_sample(event_t, event_v, query_t, default_left=None, default_right=None):
    event_t = np.asarray(event_t, dtype=float)
    event_v = np.asarray(event_v, dtype=float)
    query_t = np.asarray(query_t, dtype=float)

    if event_t.size == 0:
        return np.full_like(query_t, np.nan, dtype=float)

    left = event_v[0] if default_left is None else default_left
    right = event_v[-1] if default_right is None else default_right
    return np.interp(query_t, event_t, event_v, left=left, right=right)


def detect_flag_segments(flag_df: pd.DataFrame, t_end: float, min_duration: float = 0.5):
    """根据 flag 变化检测连续区段。"""
    segments = []
    if flag_df.empty:
        return segments

    flag_df = flag_df.sort_values("t").reset_index(drop=True)
    times = flag_df["t"].to_numpy(dtype=float)
    values = flag_df["flag"].to_numpy(dtype=int)

    seg_start = times[0]
    seg_val = values[0]

    for i in range(1, len(flag_df)):
        if values[i] != seg_val:
            seg_end = times[i]
            duration = seg_end - seg_start
            if duration >= min_duration:
                segments.append({
                    "flag": int(seg_val),
                    "start": float(seg_start),
                    "end": float(seg_end),
                    "duration": float(duration),
                })
            seg_start = times[i]
            seg_val = values[i]

    final_end = float(t_end)
    duration = final_end - seg_start
    if duration >= min_duration:
        segments.append({
            "flag": int(seg_val),
            "start": float(seg_start),
            "end": float(final_end),
            "duration": float(duration),
        })
    return segments


def choose_segments(segments, manual_off=None, manual_on=None):
    """优先使用手动窗口；否则自动选最长 OFF 和最长 ON。"""
    if manual_off is not None and manual_on is not None:
        off_seg = {"flag": 0, "start": float(manual_off[0]), "end": float(manual_off[1]), "duration": float(manual_off[1] - manual_off[0])}
        on_seg = {"flag": 1, "start": float(manual_on[0]), "end": float(manual_on[1]), "duration": float(manual_on[1] - manual_on[0])}
        return off_seg, on_seg

    off_candidates = [s for s in segments if s["flag"] == 0]
    on_candidates = [s for s in segments if s["flag"] == 1]

    off_seg = max(off_candidates, key=lambda s: s["duration"]) if off_candidates else None
    on_seg = max(on_candidates, key=lambda s: s["duration"]) if on_candidates else None
    return off_seg, on_seg


def slice_df_time(df: pd.DataFrame, start: float, end: float):
    if df.empty:
        return df.copy()
    return df[(df["t"] >= start) & (df["t"] <= end)].copy().reset_index(drop=True)


def fit_line(x, y):
    """一元线性拟合 y = slope*x + intercept。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]

    if len(x) < 3 or np.allclose(np.std(x), 0.0):
        return {
            "slope": np.nan,
            "intercept": np.nan,
            "r2": np.nan,
            "n": int(len(x)),
        }

    slope, intercept = np.polyfit(x, y, 1)
    y_hat = slope * x + intercept
    ss_res = np.sum((y - y_hat) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = np.nan if np.isclose(ss_tot, 0.0) else 1.0 - ss_res / ss_tot
    return {
        "slope": float(slope),
        "intercept": float(intercept),
        "r2": float(r2),
        "n": int(len(x)),
    }


def build_pose_analysis_df(pose_df: pd.DataFrame,
                           touch_df: pd.DataFrame,
                           flag_df: pd.DataFrame,
                           k_df: pd.DataFrame,
                           k_touch_pos_fixed: float):
    """
    在 pose 时间基准上，对齐 Touch 输入 / flag / K。
    按你的 mixed_pos 映射关系构造三轴输入：
      u_x = -delta_z * K_touch_pos_FIXED
      u_y = -delta_x * K_touch_pos_FIXED
      u_z =  delta_y * K_touch_pos_FIXED
    """
    if pose_df.empty:
        return pd.DataFrame()

    out = pose_df.copy().reset_index(drop=True)
    tq = out["t"].to_numpy(dtype=float)

    if touch_df.empty:
        out["u_x"] = np.nan
        out["u_y"] = np.nan
        out["u_z"] = np.nan
    else:
        dx = step_hold_sample(touch_df["t"].to_numpy(), touch_df["delta_x"].to_numpy(), tq, default=np.nan)
        dy = step_hold_sample(touch_df["t"].to_numpy(), touch_df["delta_y"].to_numpy(), tq, default=np.nan)
        dz = step_hold_sample(touch_df["t"].to_numpy(), touch_df["delta_z"].to_numpy(), tq, default=np.nan)
        out["u_x"] = -dz * k_touch_pos_fixed
        out["u_y"] = -dx * k_touch_pos_fixed
        out["u_z"] =  dy * k_touch_pos_fixed
        out["delta_x"] = dx
        out["delta_y"] = dy
        out["delta_z"] = dz

    out["flag"] = step_hold_sample(
        flag_df["t"].to_numpy() if not flag_df.empty else np.array([]),
        flag_df["flag"].to_numpy() if not flag_df.empty else np.array([]),
        tq,
        default=0,
    )

    if not k_df.empty:
        out["k_ctrl"] = step_hold_sample(k_df["t"].to_numpy(), k_df["k"].to_numpy(), tq, default=np.nan)
    else:
        out["k_ctrl"] = np.nan

    # 相对位姿（轨迹图用）
    out["x_rel"] = out["x"] - out["x"].iloc[0]
    out["y_rel"] = out["y"] - out["y"].iloc[0]
    out["z_rel"] = out["z"] - out["z"].iloc[0]

    out["u_norm"] = np.sqrt(out["u_x"] ** 2 + out["u_y"] ** 2 + out["u_z"] ** 2)
    out["p_rel_norm"] = np.sqrt(out["x_rel"] ** 2 + out["y_rel"] ** 2 + out["z_rel"] ** 2)
    return out


def add_flag_background(ax, segments, alpha=0.12):
    for seg in segments:
        color = "tab:red" if seg["flag"] == 1 else "tab:green"
        ax.axvspan(seg["start"], seg["end"], alpha=alpha, color=color)


def plot_overview(data, segments, outdir):
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    # 1) flag
    ax = axes[0]
    if not data["flag"].empty:
        ax.step(data["flag"]["t"], data["flag"]["flag"], where="post", color="black")
    ax.set_ylabel("flag")
    ax.set_title("mixed_pos 实验总览", fontweight="bold", fontsize=TITLE_FONT_SIZE)
    add_flag_background(ax, segments)
    ax.grid(True, alpha=0.3)

    # 2) K
    ax = axes[1]
    if not data["k_ctrl"].empty:
        ax.plot(data["k_ctrl"]["t"], data["k_ctrl"]["k"], label="K_aruco_pos_CTRL", color="tab:purple")
        ax.legend(loc="best", fontsize=LEGEND_FONT_SIZE)
    ax.set_ylabel("K")
    add_flag_background(ax, segments)
    ax.grid(True, alpha=0.3)

    # 3) Touch 增量 x/y/z
    ax = axes[2]
    if not data["touch_inc"].empty:
        ax.plot(data["touch_inc"]["t"], data["touch_inc"]["delta_x"], label="delta_x", color=LINE_COLORS["x"])
        ax.plot(data["touch_inc"]["t"], data["touch_inc"]["delta_y"], label="delta_y", color=LINE_COLORS["y"])
        ax.plot(data["touch_inc"]["t"], data["touch_inc"]["delta_z"], label="delta_z", color=LINE_COLORS["z"])
        ax.legend(loc="best", ncol=3, fontsize=LEGEND_FONT_SIZE)
    ax.set_ylabel("Touch inc")
    add_flag_background(ax, segments)
    ax.grid(True, alpha=0.3)

    # 4) white button
    ax = axes[3]
    if not data["joy"].empty:
        ax.step(data["joy"]["t"], data["joy"]["white"], where="post", label="white button", color="black")
        ax.legend(loc="best", fontsize=LEGEND_FONT_SIZE)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("white")
    add_flag_background(ax, segments)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "00_overview.png"), dpi=200)
    plt.close(fig)




def plot_touch_arm_timeseries(data, current_analysis_df: pd.DataFrame, segments, outdir):
    """
    新图：
    1) Touch inc
    2) flag
    3) current.x
    4) current.y
    5) current.z
    """
    if current_analysis_df.empty:
        return

    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)

    # 1) Touch 增量
    ax = axes[0]
    if not data["touch_inc"].empty:
        ax.plot(data["touch_inc"]["t"], data["touch_inc"]["delta_x"], label="delta_x", color=LINE_COLORS["x"])
        ax.plot(data["touch_inc"]["t"], data["touch_inc"]["delta_y"], label="delta_y", color=LINE_COLORS["y"])
        ax.plot(data["touch_inc"]["t"], data["touch_inc"]["delta_z"], label="delta_z", color=LINE_COLORS["z"])
        ax.legend(loc="best", ncol=3, fontsize=LEGEND_FONT_SIZE)
    ax.set_ylabel("Touch inc")
    add_flag_background(ax, segments)
    ax.grid(True, alpha=0.3)

    # 2) flag
    ax = axes[1]
    if not data["flag"].empty:
        ax.step(data["flag"]["t"], data["flag"]["flag"], where="post", color="black")
    ax.set_ylabel("flag")
    add_flag_background(ax, segments)
    ax.grid(True, alpha=0.3)

    # 3~5) current x/y/z
    axis_names = ["x", "y", "z"]
    for i, axis_name in enumerate(axis_names, start=2):
        ax = axes[i]
        ax.plot(current_analysis_df["t"], current_analysis_df[axis_name], color=LINE_COLORS[axis_name], label=f"current.{axis_name}")
        ax.legend(loc="lower right", fontsize=SENSITIVITY_LEGEND_FONT_SIZE)
        ax.set_ylabel(axis_name)
        add_flag_background(ax, segments)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Touch-机械臂末端时序图", y=0.995, fontsize=TITLE_FONT_SIZE, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "00_touch_arm_timeseries.png"), dpi=220)
    plt.close(fig)


def plot_time_series(analysis_df: pd.DataFrame, source_name: str, outdir: str, segments):
    if analysis_df.empty:
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    coord_names = ["x", "y", "z"]

    for i, c in enumerate(coord_names):
        ax = axes[i]
        color = LINE_COLORS.get(c, None)
        ax.plot(analysis_df["t"], analysis_df[c], label=f"{source_name}.{c}", color=color)
        ax.legend(loc="best", fontsize=LEGEND_FONT_SIZE)
        ax.set_ylabel(c)
        add_flag_background(ax, segments)
        ax.grid(True, alpha=0.3)

    ax = axes[3]
    ax.plot(analysis_df["t"], analysis_df["u_norm"], label="|u| (mapped Touch input)", color="tab:orange")
    if "k_ctrl" in analysis_df.columns and np.isfinite(analysis_df["k_ctrl"]).any():
        ax2 = ax.twinx()
        ax2.plot(analysis_df["t"], analysis_df["k_ctrl"], linestyle="--", label="K_ctrl", color="tab:purple")
        ax2.set_ylabel("K_ctrl")
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc="best", fontsize=LEGEND_FONT_SIZE)
    else:
        ax.legend(loc="best", fontsize=LEGEND_FONT_SIZE)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|u|")
    add_flag_background(ax, segments)
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"{source_name} 时间序列", y=0.995, fontsize=TITLE_FONT_SIZE, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"01_timeseries_{source_name}.png"), dpi=200)
    plt.close(fig)


def analyze_segment(analysis_df: pd.DataFrame, segment: dict, label: str, source_name: str):
    seg_df = slice_df_time(analysis_df, segment["start"], segment["end"])
    results = []
    if seg_df.empty:
        return seg_df, results

    pairs = [
        ("x", "u_x"),
        ("y", "u_y"),
        ("z", "u_z"),
    ]
    mean_k = np.nanmean(seg_df["k_ctrl"].to_numpy()) if "k_ctrl" in seg_df.columns else np.nan

    for pose_axis, input_axis in pairs:
        fit = fit_line(seg_df[input_axis], seg_df[pose_axis])
        results.append({
            "source": source_name,
            "segment": label,
            "flag": segment["flag"],
            "start": segment["start"],
            "end": segment["end"],
            "duration": segment["duration"],
            "axis": pose_axis,
            "input_axis": input_axis,
            "slope": fit["slope"],
            "intercept": fit["intercept"],
            "r2": fit["r2"],
            "n": fit["n"],
            "mean_k_ctrl": mean_k,
        })
    return seg_df, results


def plot_sensitivity_compare(seg_off_df, seg_on_df, source_name, outdir):
    if seg_off_df.empty or seg_on_df.empty:
        return

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    pairs = [
        ("x", "u_x"),
        ("y", "u_y"),
        ("z", "u_z"),
    ]

    for ax, (pose_axis, input_axis) in zip(axes, pairs):
        # OFF
        x0 = seg_off_df[input_axis].to_numpy(dtype=float)
        y0 = seg_off_df[pose_axis].to_numpy(dtype=float)
        ax.scatter(x0, y0, s=8, alpha=0.18, color="#7F8C8D", label="OFF(flag=0)")
        fit0 = fit_line(x0, y0)
        if np.isfinite(fit0["slope"]):
            xx = np.linspace(np.nanmin(x0), np.nanmax(x0), 100)
            yy = fit0["slope"] * xx + fit0["intercept"]
            ax.plot(xx, yy, linewidth=2.6, color="#4D4D4D",
                    label=f"OFF fit: slope={fit0['slope']:.4f}, R²={fit0['r2']:.3f}")

        # ON
        x1 = seg_on_df[input_axis].to_numpy(dtype=float)
        y1 = seg_on_df[pose_axis].to_numpy(dtype=float)
        ax.scatter(x1, y1, s=8, alpha=0.18, color="#D8A25E", label="ON(flag=1)")
        fit1 = fit_line(x1, y1)
        if np.isfinite(fit1["slope"]):
            xx = np.linspace(np.nanmin(x1), np.nanmax(x1), 100)
            yy = fit1["slope"] * xx + fit1["intercept"]
            ax.plot(xx, yy, linewidth=2.6, color="#C9791A",
                    label=f"ON fit: slope={fit1['slope']:.4f}, R²={fit1['r2']:.3f}")

        ax.set_xlabel(f"{input_axis}  (Touch 映射输入)")
        ax.set_ylabel(f"{pose_axis}  ({source_name} 位置)")
        ax.set_title(f"{pose_axis.upper()} 轴灵敏度对比")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="lower right", fontsize=SENSITIVITY_LEGEND_FONT_SIZE)

    fig.suptitle(f"{source_name}：调制前后灵敏度对比（线性拟合）", y=1.02, fontsize=TITLE_FONT_SIZE, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"02_sensitivity_{source_name}.png"), dpi=220, bbox_inches="tight")
    plt.close(fig)


def plot_trajectory_compare(seg_off_df, seg_on_df, source_name, outdir):
    if seg_off_df.empty or seg_on_df.empty:
        return

    # 各自减去本段首点，比较轨迹尺度
    off = seg_off_df.copy()
    on = seg_on_df.copy()
    for df in (off, on):
        df["x_rel_local"] = df["x"] - df["x"].iloc[0]
        df["y_rel_local"] = df["y"] - df["y"].iloc[0]
        df["z_rel_local"] = df["z"] - df["z"].iloc[0]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    axes[0].plot(off["x_rel_local"], off["y_rel_local"], label="OFF(flag=0)")
    axes[0].plot(on["x_rel_local"], on["y_rel_local"], label="ON(flag=1)")
    axes[0].set_xlabel("Δx [m]")
    axes[0].set_ylabel("Δy [m]")
    axes[0].set_title(f"{source_name} 轨迹对比：XY")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=LEGEND_FONT_SIZE)

    axes[1].plot(off["x_rel_local"], off["z_rel_local"], label="OFF(flag=0)")
    axes[1].plot(on["x_rel_local"], on["z_rel_local"], label="ON(flag=1)")
    axes[1].set_xlabel("Δx [m]")
    axes[1].set_ylabel("Δz [m]")
    axes[1].set_title(f"{source_name} 轨迹对比：XZ")
    axes[1].axis("equal")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=LEGEND_FONT_SIZE)

    fig.suptitle(f"{source_name} 轨迹对比", y=1.02, fontsize=TITLE_FONT_SIZE, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"03_trajectory_{source_name}.png"), dpi=220)
    plt.close(fig)


def save_segment_table(segments, outdir):
    if not segments:
        return
    df = pd.DataFrame(segments)
    df["state"] = df["flag"].map({0: "OFF", 1: "ON"})
    df = df[["state", "flag", "start", "end", "duration"]]
    df.to_csv(os.path.join(outdir, "detected_flag_segments.csv"), index=False, encoding="utf-8-sig")


def print_segments(segments):
    if not segments:
        print("[WARN] 没检测到有效 flag 区段。")
        return
    print("\n检测到的 flag 连续区段：")
    for i, seg in enumerate(segments):
        state = "ON(flag=1)" if seg["flag"] == 1 else "OFF(flag=0)"
        print(f"  [{i:02d}] {state}: start={seg['start']:.3f}s, end={seg['end']:.3f}s, duration={seg['duration']:.3f}s")


def main():
    parser = argparse.ArgumentParser(description="处理 mixed_pos 模式 rosbag 并画图")
    parser.add_argument("--bag", required=True, help="ROS1 .bag 文件路径")
    parser.add_argument("--outdir", default="mixed_pos_figs", help="输出目录")
    parser.add_argument("--k-touch", type=float, default=0.001,
                        help="K_touch_pos_FIXED，默认 0.001")
    parser.add_argument("--off", nargs=2, type=float, default=None,
                        metavar=("START", "END"), help="手动指定 OFF(flag=0) 窗口")
    parser.add_argument("--on", nargs=2, type=float, default=None,
                        metavar=("START", "END"), help="手动指定 ON(flag=1) 窗口")
    parser.add_argument("--min-seg", type=float, default=0.5,
                        help="自动检测 flag 区段时的最短时长阈值，默认 0.5 s")
    parser.add_argument("--font", default=None,
                        help='手动指定 Matplotlib 字体名，例如 "Noto Sans CJK SC"')
    args = parser.parse_args()

    setup_matplotlib_font(args.font)
    ensure_dir(args.outdir)
    data = read_bag(args.bag)

    # 自动检测区段
    segments = detect_flag_segments(data["flag"], data["t_end"], min_duration=args.min_seg)
    print_segments(segments)
    save_segment_table(segments, args.outdir)

    off_seg, on_seg = choose_segments(segments, manual_off=args.off, manual_on=args.on)
    if off_seg is None or on_seg is None:
        print("\n[WARN] 自动选择不到 OFF 和 ON 两类窗口。")
        print("请检查 bag 是否确实包含 flag=0 和 flag=1 两个阶段，或使用 --off / --on 手动指定时间窗口。")

    plot_overview(data, segments, args.outdir)

    summary_rows = []
    current_analysis_df = pd.DataFrame()

    for source_name, pose_topic in POSE_SOURCES.items():
        key = [k for k, v in TOPICS.items() if v == pose_topic][0]
        pose_df = data[key]
        if pose_df.empty:
            print(f"[INFO] 跳过 {source_name}：bag 中没有 {pose_topic}")
            continue

        analysis_df = build_pose_analysis_df(
            pose_df=pose_df,
            touch_df=data["touch_inc"],
            flag_df=data["flag"],
            k_df=data["k_ctrl"],
            k_touch_pos_fixed=args.k_touch,
        )

        analysis_df.to_csv(
            os.path.join(args.outdir, f"analysis_{source_name}.csv"),
            index=False,
            encoding="utf-8-sig",
        )

        if source_name == "current":
            current_analysis_df = analysis_df.copy()

        plot_time_series(analysis_df, source_name, args.outdir, segments)

        if off_seg is not None and on_seg is not None:
            seg_off_df, rows_off = analyze_segment(analysis_df, off_seg, "OFF", source_name)
            seg_on_df, rows_on = analyze_segment(analysis_df, on_seg, "ON", source_name)
            summary_rows.extend(rows_off)
            summary_rows.extend(rows_on)
            plot_sensitivity_compare(seg_off_df, seg_on_df, source_name, args.outdir)
            plot_trajectory_compare(seg_off_df, seg_on_df, source_name, args.outdir)

    if not current_analysis_df.empty:
        plot_touch_arm_timeseries(data, current_analysis_df, segments, args.outdir)

    if summary_rows:
        summary_df = pd.DataFrame(summary_rows)
        summary_df.to_csv(os.path.join(args.outdir, "summary_fit_results.csv"),
                          index=False, encoding="utf-8-sig")
        print("\n[OK] 已输出 summary_fit_results.csv")
        print(summary_df[["source", "segment", "axis", "slope", "intercept", "r2", "mean_k_ctrl"]])
    else:
        print("\n[WARN] 没有生成拟合结果。")

    print(f"\n[OK] 全部输出已保存到: {os.path.abspath(args.outdir)}")


if __name__ == "__main__":
    main()