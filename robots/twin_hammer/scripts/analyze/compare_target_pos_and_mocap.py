#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R


def read_uav_nav_data(bag, topic, start_time=None, end_time=None, t0=None):
    xs, ys, zs, rolls, pitchs, yaws, times = [], [], [], [], [], [], []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        t_sec = t.to_sec()
        if start_time and t_sec < start_time:
            continue
        if end_time and t_sec > end_time:
            continue
        xs.append(msg.target_pos_x)
        ys.append(msg.target_pos_y)
        zs.append(msg.target_pos_z)
        rolls.append(msg.target_roll)
        pitchs.append(msg.target_pitch)
        yaws.append(msg.target_yaw)
        if t0 is not None:
            t_sec = t_sec - t0
        times.append(t_sec)
    return np.array(xs), np.array(ys), np.array(zs), np.array(rolls), np.array(pitchs), np.array(yaws), np.array(times)

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

def compute_rmse(x1, x2):
    return np.sqrt(np.mean((x1 - x2) ** 2))

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
    x_nav, y_nav, z_nav, roll_nav, pitch_nav, yaw_nav, t_nav = read_uav_nav_data(bag, '/gimbalrotor/uav/nav', args.start, args.end, t0)
    x_mocap, y_mocap, z_mocap, roll_mocap, pitch_mocap, yaw_mocap, t_mocap = read_mocap_data(bag, '/gimbalrotor/mocap/pose', args.start, args.end, t0)

    bag.close()

    # 補間 (uavとmocapのタイムスタンプを合わせる)
    if len(t_nav) == 0 or len(t_mocap) == 0:
        print("指定された時間範囲でデータがありません。")
        exit()

    # 共通の時間基準を作る（uavを基準にmocapを補間）
    t_common = np.linspace(max(t_nav[0], t_mocap[0]), min(t_nav[-1], t_mocap[-1]), min(len(t_nav), len(t_mocap)))
    x_nav_interp = np.interp(t_common, t_nav, x_nav)
    y_nav_interp = np.interp(t_common, t_nav, y_nav)
    z_nav_interp = np.interp(t_common, t_nav, z_nav)
    roll_nav_interp = np.interp(t_common, t_nav, roll_nav)
    pitch_nav_interp = np.interp(t_common, t_nav, pitch_nav)
    yaw_nav_interp = np.interp(t_common, t_nav, yaw_nav)
    x_mocap_interp = np.interp(t_common, t_mocap, x_mocap)
    y_mocap_interp = np.interp(t_common, t_mocap, y_mocap)
    z_mocap_interp = np.interp(t_common, t_mocap, z_mocap)
    roll_mocap_interp = np.interp(t_common, t_mocap, roll_mocap)
    pitch_mocap_interp = np.interp(t_common, t_mocap, pitch_mocap)
    yaw_mocap_interp = np.interp(t_common, t_mocap, yaw_mocap)

    # RMSE計算
    rmse_x = compute_rmse(x_nav_interp, x_mocap_interp)
    rmse_y = compute_rmse(y_nav_interp, y_mocap_interp)
    rmse_z = compute_rmse(z_nav_interp, z_mocap_interp)
    rmse_roll = compute_rmse(roll_nav_interp, roll_mocap_interp)
    rmse_pitch = compute_rmse(pitch_nav_interp, pitch_mocap_interp)
    rmse_yaw = compute_rmse(yaw_nav_interp, yaw_mocap_interp)
    # rmse_total = np.sqrt(np.mean((x_nav_interp - x_mocap_interp)**2 + (y_nav_interp - y_mocap_interp)**2 + (z_nav_interp - z_mocap_interp)**2))

    print(f"RMSE X: {rmse_x:.4f}")
    print(f"RMSE Y: {rmse_y:.4f}")
    print(f"RMSE Z: {rmse_z:.4f}")
    print(f"RMSE ROLL: {rmse_roll:.4f}")
    print(f"RMSE PITCH: {rmse_pitch:.4f}")
    print(f"RMSE YAW: {rmse_yaw:.4f}")
    # print(f"RMSE Total (3D): {rmse_total:.4f}")

    # プロット
    plt.figure(figsize=(8, 8))
    plt.plot(x_nav, y_nav, 'r-', label='target')
    plt.plot(x_mocap, y_mocap, 'b-', label='measured')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(8, 8))
    plt.plot(x_nav, z_nav, 'r-', label='target')
    plt.plot(x_mocap, z_mocap, 'b-', label='measured')
    plt.xlabel('X [m]')
    plt.ylabel('Z [m]')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(8, 8))
    plt.plot(y_nav, z_nav, 'r-', label='target')
    plt.plot(y_mocap, z_mocap, 'b-', label='measured')
    plt.xlabel('Y [m]')
    plt.ylabel('Z [m]')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()
