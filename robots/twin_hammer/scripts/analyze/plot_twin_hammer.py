#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R

def read_mocap_data(bag, topic, start_time=None, end_time=None, t0=None):
    xs, ys, zs, rolls, pitchs, yaws, times = [], [], [], [], [], [], []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        t_sec = t.to_sec()
        if start_time and t_sec < start_time:
            continue
        if end_time and t_sec > end_time:
            continue
        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)
        zs.append(msg.pose.position.z)
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rot = R.from_quat(q)
        e = rot.as_euler('xyz')
        rolls.append(e[0])
        pitchs.append(e[1])
        yaws.append(e[2])
        if t0 is not None:
            t_sec = t_sec - t0
        times.append(t_sec)
    return np.array(xs), np.array(ys), np.array(zs), np.array(rolls), np.array(pitchs), np.array(yaws), np.array(times)

def read_wrench_data(bag, topic, start_time=None, end_time=None, t0=None):
    xs, ys, zs, rolls, pitchs, yaws, times = [], [], [], [], [], [], []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        t_sec = t.to_sec()
        if start_time and t_sec < start_time:
            continue
        if end_time and t_sec > end_time:
            continue
        if topic == '/cfs/data':
            xs.append(msg.wrench.force.z)
            ys.append(msg.wrench.force.x)
            zs.append(msg.wrench.force.y)
            rolls.append(msg.wrench.torque.z)
            pitchs.append(msg.wrench.torque.x)
            yaws.append(msg.wrench.torque.y)
        else:
            xs.append(msg.wrench.force.x)
            ys.append(msg.wrench.force.y)
            zs.append(msg.wrench.force.z)
            rolls.append(msg.wrench.torque.x)
            pitchs.append(msg.wrench.torque.y)
            yaws.append(msg.wrench.torque.z)
        if t0 is not None:
            t_sec = t_sec - t0
        times.append(t_sec)
    return np.array(xs), np.array(ys), np.array(zs), np.array(rolls), np.array(pitchs), np.array(yaws), np.array(times)

def compute_rmse(x1, x2):
    return np.sqrt(np.mean((x1 - x2) ** 2))

def dynamic_ylim(data1, data2, margin_ratio=0.05):
    """データ範囲に基づいてY軸を自動調整"""
    dmin = min(np.min(data1), np.min(data2))
    dmax = max(np.max(data1), np.max(data2))
    margin = (dmax - dmin) * margin_ratio
    return [dmin - margin, dmax + margin]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot two PoseStamped topics and compute RMSE.")
    parser.add_argument("bagfile", help="Path to rosbag file")
    # parser.add_argument("--topic_a", required=True, help="Topic name for A")
    # parser.add_argument("--topic_b", required=True, help="Topic name for B")
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds")
    args = parser.parse_args()

    bag = rosbag.Bag(args.bagfile)
    t0 = bag.get_start_time()
    # Read data
    x_mocap, y_mocap, z_mocap, roll_mocap, picth_mocap, yaw_mocap, t_mocap = read_mocap_data(bag, '/twin_hammer/mocap/pose', args.start, args.end, t0)
    x_wrench_fb, y_wrench_fb, z_wrench_fb, roll_wrench_fb, pitch_wrench_fb, yaw_wrench_fb, t_wrench_fb = read_wrench_data(bag, '/twin_hammer/haptics_wrench', args.start, args.end, t0)
    x_wrench_cfs, y_wrench_cfs, z_wrench_cfs, roll_wrench_cfs, pitch_wrench_cfs, yaw_wrench_cfs, t_wrench_cfs = read_wrench_data(bag, '/cfs/data', args.start, args.end, t0)

    bag.close()

    # プロット
    plt.figure(figsize=(12, 12))
    plt.plot(x_mocap, y_mocap, 'r-', label='twin_hammer_mocap')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Device Trajectory')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.plot(t_wrench_fb, x_wrench_fb, 'r-', label='feedback_wrench')
    plt.plot(t_wrench_cfs, x_wrench_cfs, 'b-', label='row_wrench')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Wrench Compare X')
    plt.legend()
    plt.grid(True)
    y_lim = dynamic_ylim(x_wrench_fb, x_wrench_cfs)
    plt.ylim(y_lim[0],y_lim[1])
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.plot(t_wrench_fb, y_wrench_fb, 'r-', label='feedback_wrench')
    plt.plot(t_wrench_cfs, y_wrench_cfs, 'b-', label='row_wrench')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Wrench Compare Y')
    plt.legend()
    plt.grid(True)
    y_lim = dynamic_ylim(y_wrench_fb, y_wrench_cfs)
    plt.ylim(y_lim[0],y_lim[1])
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.plot(t_wrench_fb, z_wrench_fb, 'r-', label='feedback_wrench')
    plt.plot(t_wrench_cfs, z_wrench_cfs, 'b-', label='row_wrench')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title('Wrench Compare Z')
    plt.legend()
    plt.grid(True)
    y_lim = dynamic_ylim(z_wrench_fb, z_wrench_cfs)
    plt.ylim(y_lim[0],y_lim[1])
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.plot(t_wrench_fb, roll_wrench_fb, 'r-', label='feedback_wrench')
    plt.plot(t_wrench_cfs, roll_wrench_cfs, 'b-', label='row_wrench')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('Wrench Compare ROLL')
    plt.legend()
    plt.grid(True)
    y_lim = dynamic_ylim(roll_wrench_fb, roll_wrench_cfs)
    plt.ylim(y_lim[0],y_lim[1])
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.plot(t_wrench_fb, pitch_wrench_fb, 'r-', label='feedback_wrench')
    plt.plot(t_wrench_cfs, pitch_wrench_cfs, 'b-', label='row_wrench')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('Wrench Compare PITCH')
    plt.legend()
    plt.grid(True)
    y_lim = dynamic_ylim(pitch_wrench_fb, pitch_wrench_cfs)
    plt.ylim(y_lim[0],y_lim[1])
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.plot(t_wrench_fb, yaw_wrench_fb, 'r-', label='feedback_wrench')
    plt.plot(t_wrench_cfs, yaw_wrench_cfs, 'b-', label='row_wrench')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('Wrench Compare YAW')
    plt.legend()
    plt.grid(True)
    y_lim = dynamic_ylim(yaw_wrench_fb, yaw_wrench_cfs)
    plt.ylim(y_lim[0],y_lim[1])
    plt.tight_layout()
    plt.show()
