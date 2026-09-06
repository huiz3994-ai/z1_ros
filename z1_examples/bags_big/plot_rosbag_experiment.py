#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# python3 plot_rosbag_experiment.py /.../xxx.bag --outdir /home/v233/z1_ws/src/z1_ros/z1_examples/bags_big/exp_out_03

import os
import math
import argparse
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    import rosbag
except Exception as e:
    raise RuntimeError(
        "无法导入 rosbag。请先 source ROS1 环境，例如: source /opt/ros/noetic/setup.bash"
    ) from e


TOPICS = {
    'target_pose': '/ttttttttarget_pose',
    'current_pose': '/debug/current_pose',
    'actual_pose': '/end_effector_pose',
    'aruco_raw': '/aruco_single/pose',
    'aruco_flag': '/aruco_single/aruco_detected_flag',
    'aruco_processed': '/debug/processed_aruco_pose',
    'realsense_raw': '/detected_plane_pose_link00',
    'realsense_processed': '/debug/processed_realsense_pose',
    'touch_increment': '/Touch_Pose_increment',
    'touch_joy': '/Geomagic/joy',
    'k_aruco_pos_ctrl': '/debug/k_aruco_pos_ctrl',
    'ctrl_mode': '/debug/ctrl_mode',
    'ik_solved': '/debug/ik_solved',
    'ik_solve_time': '/debug/ik_solve_time',
    'goal_joint_positions': '/goal_joint_positions',
    'joint_cmd': '/joint_group_position_controller/command',
    'joint_states': '/joint_states',
    'hf_goal': '/debug/high_freq/goal_positions',
    'hf_current': '/debug/high_freq/current_values',
    'hf_actual': '/debug/high_freq/actual_joint_positions',
    'hf_all_reached': '/debug/high_freq/all_reached',
}

CTRL_MODE_NAME = {
    1: 'only_touch',
    2: 'aruco_for_pos',
    3: 'aruco_for_ori',
    4: 'realsense_for_ori',
    5: 'mixed_pos',
}


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)



def quat_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw



def rad2deg(arr):
    return np.array(arr) * 180.0 / np.pi



def interp_series(times_src, values_src, times_dst):
    if len(times_src) == 0 or len(values_src) == 0 or len(times_dst) == 0:
        return None
    times_src = np.asarray(times_src, dtype=float)
    values_src = np.asarray(values_src, dtype=float)
    times_dst = np.asarray(times_dst, dtype=float)
    if values_src.ndim == 1:
        return np.interp(times_dst, times_src, values_src)
    out = np.zeros((len(times_dst), values_src.shape[1]), dtype=float)
    for i in range(values_src.shape[1]):
        out[:, i] = np.interp(times_dst, times_src, values_src[:, i])
    return out



def add_pose(store, t_rel, msg):
    p = msg.pose.position
    q = msg.pose.orientation
    rpy = quat_to_rpy(q.x, q.y, q.z, q.w)
    store['t'].append(t_rel)
    store['pos'].append([p.x, p.y, p.z])
    store['quat'].append([q.x, q.y, q.z, q.w])
    store['rpy'].append(list(rpy))



def add_float_array(store, t_rel, msg):
    store['t'].append(t_rel)
    store['data'].append(list(msg.data))



def add_float(store, t_rel, msg):
    store['t'].append(t_rel)
    store['data'].append(float(msg.data))



def add_bool(store, t_rel, msg):
    store['t'].append(t_rel)
    store['data'].append(1.0 if bool(msg.data) else 0.0)



def add_int(store, t_rel, msg):
    store['t'].append(t_rel)
    store['data'].append(int(msg.data))



def add_joy(store, t_rel, msg):
    store['t'].append(t_rel)
    store['buttons'].append(list(msg.buttons))
    store['axes'].append(list(msg.axes))



def add_joint_states(store, t_rel, msg):
    store['t'].append(t_rel)
    store['name'].append(list(msg.name))
    store['position'].append(list(msg.position))
    if hasattr(msg, 'velocity'):
        store['velocity'].append(list(msg.velocity))
    else:
        store['velocity'].append([])



def new_pose_store():
    return {'t': [], 'pos': [], 'quat': [], 'rpy': []}



def new_array_store():
    return {'t': [], 'data': []}



def new_scalar_store():
    return {'t': [], 'data': []}



def new_joy_store():
    return {'t': [], 'buttons': [], 'axes': []}



def new_joint_store():
    return {'t': [], 'name': [], 'position': [], 'velocity': []}



def get_pose_component(store, key):
    if store is None:
        return None, None
    t = np.asarray(store.get('t', []), dtype=float)
    vals = np.asarray(store.get(key, []), dtype=float)
    if t.size == 0 or vals.size == 0:
        return None, None
    if vals.ndim == 1:
        if vals.size % 3 != 0:
            return None, None
        vals = vals.reshape(-1, 3)
    if vals.ndim != 2 or vals.shape[1] < 3:
        return None, None
    n = min(len(t), len(vals))
    if n == 0:
        return None, None
    return t[:n], vals[:n, :3]



def get_array_component(store):
    if store is None:
        return None, None
    t = np.asarray(store.get('t', []), dtype=float)
    vals = np.asarray(store.get('data', []), dtype=float)
    if t.size == 0 or vals.size == 0:
        return None, None
    if vals.ndim == 1:
        vals = vals.reshape(-1, 1)
    if vals.ndim != 2:
        return None, None
    n = min(len(t), len(vals))
    if n == 0:
        return None, None
    return t[:n], vals[:n]



def plot_xyz(path, title, series_dict, ylabel='value'):
    labels = ['x', 'y', 'z']
    for dim in range(3):
        plt.figure(figsize=(11, 4.5))
        plotted = False
        for name, data in series_dict.items():
            t, pos = get_pose_component(data, 'pos')
            if t is None:
                continue
            plt.plot(t, pos[:, dim], label=name)
            plotted = True
        if not plotted:
            plt.close()
            continue
        plt.xlabel('time [s]')
        plt.ylabel(ylabel)
        plt.title(f'{title} - {labels[dim]}')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(f'{path}_{labels[dim]}.png', dpi=160)
        plt.close()



def plot_rpy_deg(path, title, series_dict):
    labels = ['roll', 'pitch', 'yaw']
    for dim in range(3):
        plt.figure(figsize=(11, 4.5))
        plotted = False
        for name, data in series_dict.items():
            t, rpy = get_pose_component(data, 'rpy')
            if t is None:
                continue
            plt.plot(t, rad2deg(rpy[:, dim]), label=name)
            plotted = True
        if not plotted:
            plt.close()
            continue
        plt.xlabel('time [s]')
        plt.ylabel('deg')
        plt.title(f'{title} - {labels[dim]}')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(f'{path}_{labels[dim]}.png', dpi=160)
        plt.close()



def plot_scalar(path, title, t, y, ylabel='value', step=False, yticks=None, yticklabels=None):
    if len(t) == 0:
        return
    plt.figure(figsize=(11, 4.2))
    if step:
        plt.step(t, y, where='post')
    else:
        plt.plot(t, y)
    plt.xlabel('time [s]')
    plt.ylabel(ylabel)
    plt.title(title)
    if yticks is not None:
        plt.yticks(yticks, yticklabels if yticklabels is not None else yticks)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()



def plot_touch_increment(path, store):
    t, data = get_array_component(store)
    if t is None:
        return
    labels = ['dx', 'dy', 'dz']
    for i in range(min(3, data.shape[1])):
        plt.figure(figsize=(11, 4.2))
        plt.plot(t, data[:, i])
        plt.xlabel('time [s]')
        plt.ylabel('increment')
        plt.title(f'Touch increment - {labels[i]}')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f'{path}_{labels[i]}.png', dpi=160)
        plt.close()



def plot_touch_buttons(path, joy_store):
    if len(joy_store['t']) == 0:
        return
    t = np.asarray(joy_store['t'])
    buttons = np.asarray(joy_store['buttons'], dtype=float)
    if buttons.ndim != 2 or buttons.shape[1] == 0:
        return
    plt.figure(figsize=(11, 4.2))
    max_btn = min(buttons.shape[1], 4)
    for i in range(max_btn):
        plt.step(t, buttons[:, i], where='post', label=f'button[{i}]')
    plt.xlabel('time [s]')
    plt.ylabel('state')
    plt.title('Touch buttons')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()



def plot_joint_comparison(path_prefix, title, series_dict):
    prepared = {}
    n_joints = None
    for name, store in series_dict.items():
        t, arr = get_array_component(store)
        if t is None:
            continue
        prepared[name] = (t, arr)
        if n_joints is None or arr.shape[1] < n_joints:
            n_joints = arr.shape[1] if n_joints is None else min(n_joints, arr.shape[1])
    if not prepared or n_joints is None or n_joints <= 0:
        return
    for j in range(min(6, n_joints)):
        plt.figure(figsize=(11, 4.5))
        for name, (t, arr) in prepared.items():
            plt.plot(t, arr[:, j], label=name)
        plt.xlabel('time [s]')
        plt.ylabel('rad')
        plt.title(f'{title} - J{j+1}')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(f'{path_prefix}_J{j+1}.png', dpi=160)
        plt.close()



def plot_pos_error(path_prefix, ref_pose, actual_pose):
    t_ref, pos_ref = get_pose_component(ref_pose, 'pos')
    t_act, pos_act = get_pose_component(actual_pose, 'pos')
    if t_ref is None or t_act is None:
        return
    pos_ref_i = interp_series(t_ref, pos_ref, t_act)
    if pos_ref_i is None:
        return
    err = pos_act - pos_ref_i
    labels = ['x', 'y', 'z']
    for i in range(3):
        plt.figure(figsize=(11, 4.2))
        plt.plot(t_act, err[:, i])
        plt.xlabel('time [s]')
        plt.ylabel('m')
        plt.title(f'Position tracking error (actual - reference) - {labels[i]}')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f'{path_prefix}_{labels[i]}.png', dpi=160)
        plt.close()



def plot_overview_position(path, target_pose, current_pose, actual_pose):
    prepared = []
    for name, data in [('target', target_pose), ('current', current_pose), ('actual', actual_pose)]:
        t, pos = get_pose_component(data, 'pos')
        if t is not None:
            prepared.append((name, t, pos))
    if not prepared:
        return
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    labels = ['x', 'y', 'z']
    for dim in range(3):
        ax = axes[dim]
        plotted = False
        for name, t, pos in prepared:
            ax.plot(t, pos[:, dim], label=name)
            plotted = True
        ax.set_ylabel(labels[dim] + ' [m]')
        ax.grid(True, alpha=0.3)
        if plotted:
            ax.legend(loc='best')
    axes[-1].set_xlabel('time [s]')
    fig.suptitle('Overview position XYZ')
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)



def summarize(summary_path, meta, stores):
    with open(summary_path, 'w', encoding='utf-8') as f:
        f.write('ROS bag plotting summary\n')
        f.write('=' * 60 + '\n')
        f.write(f"bag: {meta['bag_path']}\n")
        f.write(f"output_dir: {meta['outdir']}\n")
        f.write(f"time_window: start={meta['start']} end={meta['end']}\n")
        f.write('\nAvailable topic samples:\n')
        for k, v in stores.items():
            if 't' in v:
                f.write(f'- {k}: {len(v["t"])}\n')
        if len(stores['ctrl_mode']['data']) > 0:
            vals = sorted(set(int(x) for x in stores['ctrl_mode']['data']))
            names = [CTRL_MODE_NAME.get(v, str(v)) for v in vals]
            f.write('\nctrl_mode values seen: ' + ', '.join([f'{v}({n})' for v, n in zip(vals, names)]) + '\n')
        if len(stores['joint_states']['name']) > 0:
            first_names = stores['joint_states']['name'][0]
            f.write('\njoint_states names (first sample):\n')
            f.write(str(first_names) + '\n')



def main():
    parser = argparse.ArgumentParser(description='读取 ROS1 rosbag 并自动绘制机械臂实验关键图。')
    parser.add_argument('bag', help='输入 .bag 文件路径')
    parser.add_argument('--outdir', default=None, help='输出目录，默认: <bag_stem>_figures')
    parser.add_argument('--start', type=float, default=None, help='起始时间，单位秒，相对于 bag 起点')
    parser.add_argument('--end', type=float, default=None, help='结束时间，单位秒，相对于 bag 起点')
    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag)
    if not os.path.isfile(bag_path):
        raise FileNotFoundError(f'未找到 bag 文件: {bag_path}')

    if args.outdir is None:
        stem = os.path.splitext(os.path.basename(bag_path))[0]
        outdir = os.path.abspath(stem + '_figures')
    else:
        outdir = os.path.abspath(args.outdir)
    ensure_dir(outdir)

    stores = {
        'target_pose': new_pose_store(),
        'current_pose': new_pose_store(),
        'actual_pose': new_pose_store(),
        'aruco_raw': new_pose_store(),
        'aruco_processed': new_pose_store(),
        'realsense_raw': new_pose_store(),
        'realsense_processed': new_pose_store(),
        'touch_increment': new_array_store(),
        'touch_joy': new_joy_store(),
        'aruco_flag': new_scalar_store(),
        'k_aruco_pos_ctrl': new_scalar_store(),
        'ctrl_mode': new_scalar_store(),
        'ik_solved': new_scalar_store(),
        'ik_solve_time': new_scalar_store(),
        'goal_joint_positions': new_array_store(),
        'joint_cmd': new_array_store(),
        'joint_states': new_joint_store(),
        'hf_goal': new_array_store(),
        'hf_current': new_array_store(),
        'hf_actual': new_array_store(),
        'hf_all_reached': new_scalar_store(),
    }

    bag = rosbag.Bag(bag_path, 'r')
    try:
        bag_start = bag.get_start_time()
        start = args.start if args.start is not None else 0.0
        end = args.end if args.end is not None else None
        wanted_topics = list(TOPICS.values())

        for topic, msg, t in bag.read_messages(topics=wanted_topics):
            t_rel = t.to_sec() - bag_start
            if t_rel < start:
                continue
            if end is not None and t_rel > end:
                continue

            if topic == TOPICS['target_pose']:
                add_pose(stores['target_pose'], t_rel, msg)
            elif topic == TOPICS['current_pose']:
                add_pose(stores['current_pose'], t_rel, msg)
            elif topic == TOPICS['actual_pose']:
                add_pose(stores['actual_pose'], t_rel, msg)
            elif topic == TOPICS['aruco_raw']:
                add_pose(stores['aruco_raw'], t_rel, msg)
            elif topic == TOPICS['aruco_processed']:
                add_pose(stores['aruco_processed'], t_rel, msg)
            elif topic == TOPICS['realsense_raw']:
                add_pose(stores['realsense_raw'], t_rel, msg)
            elif topic == TOPICS['realsense_processed']:
                add_pose(stores['realsense_processed'], t_rel, msg)
            elif topic == TOPICS['touch_increment']:
                add_float_array(stores['touch_increment'], t_rel, msg)
            elif topic == TOPICS['touch_joy']:
                add_joy(stores['touch_joy'], t_rel, msg)
            elif topic == TOPICS['aruco_flag']:
                add_bool(stores['aruco_flag'], t_rel, msg)
            elif topic == TOPICS['k_aruco_pos_ctrl']:
                add_float(stores['k_aruco_pos_ctrl'], t_rel, msg)
            elif topic == TOPICS['ctrl_mode']:
                add_int(stores['ctrl_mode'], t_rel, msg)
            elif topic == TOPICS['ik_solved']:
                add_bool(stores['ik_solved'], t_rel, msg)
            elif topic == TOPICS['ik_solve_time']:
                add_float(stores['ik_solve_time'], t_rel, msg)
            elif topic == TOPICS['goal_joint_positions']:
                add_float_array(stores['goal_joint_positions'], t_rel, msg)
            elif topic == TOPICS['joint_cmd']:
                add_float_array(stores['joint_cmd'], t_rel, msg)
            elif topic == TOPICS['joint_states']:
                add_joint_states(stores['joint_states'], t_rel, msg)
            elif topic == TOPICS['hf_goal']:
                add_float_array(stores['hf_goal'], t_rel, msg)
            elif topic == TOPICS['hf_current']:
                add_float_array(stores['hf_current'], t_rel, msg)
            elif topic == TOPICS['hf_actual']:
                add_float_array(stores['hf_actual'], t_rel, msg)
            elif topic == TOPICS['hf_all_reached']:
                add_bool(stores['hf_all_reached'], t_rel, msg)
    finally:
        bag.close()

    # 末端位姿主图
    plot_xyz(os.path.join(outdir, 'Target-Current-Actual_pos'), 'Target vs Current vs Actual position', {
        'target': stores['target_pose'],
        'current': stores['current_pose'],
        'actual': stores['actual_pose'],
    }, ylabel='m')
    plot_rpy_deg(os.path.join(outdir, 'Target-Current-Actual_rpy'), 'Target vs Current vs Actual orientation', {
        'target': stores['target_pose'],
        'current': stores['current_pose'],
        'actual': stores['actual_pose'],
    })
    plot_overview_position(os.path.join(outdir, 'overview_position_xyz.png'), stores['target_pose'], stores['current_pose'], stores['actual_pose'])

    # 视觉输入对比图
    plot_xyz(os.path.join(outdir, 'Aruco-Processed-Actual_pos'), 'Aruco raw vs processed vs actual position', {
        'aruco_raw': stores['aruco_raw'],
        'aruco_processed': stores['aruco_processed'],
        'actual': stores['actual_pose'],
    }, ylabel='m')
    plot_rpy_deg(os.path.join(outdir, 'Aruco-Processed-Actual_rpy'), 'Aruco raw vs processed vs actual orientation', {
        'aruco_raw': stores['aruco_raw'],
        'aruco_processed': stores['aruco_processed'],
        'actual': stores['actual_pose'],
    })

    plot_xyz(os.path.join(outdir, 'Realsense-Processed-Actual_pos'), 'Realsense raw vs processed vs actual position', {
        'realsense_raw': stores['realsense_raw'],
        'realsense_processed': stores['realsense_processed'],
        'actual': stores['actual_pose'],
    }, ylabel='m')
    plot_rpy_deg(os.path.join(outdir, 'Realsense-Processed-Actual_rpy'), 'Realsense raw vs processed vs actual orientation', {
        'realsense_raw': stores['realsense_raw'],
        'realsense_processed': stores['realsense_processed'],
        'actual': stores['actual_pose'],
    })

    # 输入与状态
    plot_touch_increment(os.path.join(outdir, 'touch_increment'), stores['touch_increment'])
    plot_touch_buttons(os.path.join(outdir, 'touch_buttons.png'), stores['touch_joy'])

    plot_scalar(
        os.path.join(outdir, 'k_aruco_pos_ctrl.png'),
        'K_aruco_pos_CTRL',
        stores['k_aruco_pos_ctrl']['t'],
        stores['k_aruco_pos_ctrl']['data'],
        ylabel='value'
    )
    plot_scalar(
        os.path.join(outdir, 'aruco_detected_flag.png'),
        'aruco_detected_flag',
        stores['aruco_flag']['t'],
        stores['aruco_flag']['data'],
        ylabel='flag',
        step=True,
        yticks=[0, 1]
    )

    if len(stores['ctrl_mode']['t']) > 0:
        vals = sorted(set(int(x) for x in stores['ctrl_mode']['data']))
        labels = [CTRL_MODE_NAME.get(v, str(v)) for v in vals]
        plot_scalar(
            os.path.join(outdir, 'ctrl_mode.png'),
            'ctrl_mode',
            stores['ctrl_mode']['t'],
            stores['ctrl_mode']['data'],
            ylabel='mode',
            step=True,
            yticks=vals,
            yticklabels=labels,
        )

    plot_scalar(
        os.path.join(outdir, 'ik_solved.png'),
        'ik_solved',
        stores['ik_solved']['t'],
        stores['ik_solved']['data'],
        ylabel='success',
        step=True,
        yticks=[0, 1]
    )
    plot_scalar(
        os.path.join(outdir, 'ik_solve_time.png'),
        'ik_solve_time',
        stores['ik_solve_time']['t'],
        stores['ik_solve_time']['data'],
        ylabel='s'
    )

    # 关节图：优先用 high_freq debug，缺失时退回 joint_states/goal/cmd
    joint_actual_store = None
    if len(stores['hf_actual']['t']) > 0:
        joint_actual_store = stores['hf_actual']
    elif len(stores['joint_states']['t']) > 0:
        joint_actual_store = {
            't': stores['joint_states']['t'],
            'data': [p[:6] for p in stores['joint_states']['position'] if len(p) >= 6]
        }
        # 与时间长度对齐
        if len(joint_actual_store['data']) != len(joint_actual_store['t']):
            min_len = min(len(joint_actual_store['data']), len(joint_actual_store['t']))
            joint_actual_store['t'] = joint_actual_store['t'][:min_len]
            joint_actual_store['data'] = joint_actual_store['data'][:min_len]

    joint_goal_store = stores['hf_goal'] if len(stores['hf_goal']['t']) > 0 else stores['goal_joint_positions']
    joint_cmd_store = stores['hf_current'] if len(stores['hf_current']['t']) > 0 else stores['joint_cmd']

    plot_joint_comparison(
        os.path.join(outdir, 'Joint_Comparison'),
        'Joint comparison',
        {
            'goal': joint_goal_store if len(joint_goal_store['t']) > 0 else None,
            'command': joint_cmd_store if len(joint_cmd_store['t']) > 0 else None,
            'actual': joint_actual_store,
        }
    )

    if len(stores['hf_all_reached']['t']) > 0:
        plot_scalar(
            os.path.join(outdir, 'high_freq_all_reached.png'),
            'high_freq all_reached',
            stores['hf_all_reached']['t'],
            stores['hf_all_reached']['data'],
            ylabel='state',
            step=True,
            yticks=[0, 1]
        )

    plot_pos_error(os.path.join(outdir, 'tracking_error_pos'), stores['current_pose'], stores['actual_pose'])

    summarize(
        os.path.join(outdir, 'plot_summary.txt'),
        {
            'bag_path': bag_path,
            'outdir': outdir,
            'start': args.start,
            'end': args.end,
        },
        stores,
    )

    print('完成。输出目录:', outdir)


if __name__ == '__main__':
    main()