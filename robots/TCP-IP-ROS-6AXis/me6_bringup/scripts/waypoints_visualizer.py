#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
import numpy as np

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray

# YAMLは任意（無ければCSV/NPYだけでOK）
try:
    import yaml
except Exception:
    yaml = None


def try_delete(name: str):
    try:
        rospy.wait_for_service("/gazebo/delete_model", timeout=0.5)
        delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete(name)
    except Exception:
        pass


def load_points(points_file: str):
    """
    return: list of (x,y,z)
    supported:
      - .csv : each row "x,y,z" (header ok)
      - .npy : shape (N,3)
      - .yaml/.yml : e.g.
            points:
              - [x,y,z]
              - {x:..., y:..., z:...}
    """
    ext = os.path.splitext(points_file)[1].lower()

    if ext == ".npy":
        arr = np.load(points_file)
        arr = np.array(arr, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 3:
            raise ValueError(f"NPY must be shape (N,3). got {arr.shape}")
        return [(float(x), float(y), float(z)) for x, y, z in arr]

    if ext == ".csv":
        # headerありでもOKにするため genfromtxt を使う
        arr = np.genfromtxt(points_file, delimiter=",", dtype=float, comments="#")
        if arr.ndim == 1:
            # 1行だけの場合 (3,) になる
            if arr.size != 3:
                raise ValueError(f"CSV row must have 3 values. got {arr}")
            arr = arr.reshape(1, 3)
        # header混在でnanが出たら落とす
        arr = arr[~np.isnan(arr).any(axis=1)]
        if arr.shape[1] != 3:
            raise ValueError(f"CSV must have 3 columns. got {arr.shape}")
        return [(float(x), float(y), float(z)) for x, y, z in arr]

    if ext in [".yaml", ".yml"]:
        if yaml is None:
            raise RuntimeError("pyyaml is not installed, cannot read yaml/yml.")
        with open(points_file, "r") as f:
            data = yaml.safe_load(f)

        pts = data.get("points", data)  # pointsキーが無ければ直下を使う
        out = []
        for item in pts:
            if isinstance(item, (list, tuple)) and len(item) == 3:
                out.append((float(item[0]), float(item[1]), float(item[2])))
            elif isinstance(item, dict):
                out.append((float(item["x"]), float(item["y"]), float(item["z"])))
            else:
                raise ValueError(f"Unsupported yaml point format: {item}")
        return out

    raise ValueError(f"Unsupported file extension: {ext} (use .csv/.npy/.yaml)")


def main():
    rospy.init_node("spawn_waypoints_from_file_gazebo")

    pub_rate = float(rospy.get_param("~pub_rate", 10.0))  # Hz
    namespace = rospy.get_param("~model_name_prefix", "wp_")

    # これを指定する：座標ファイル
    # 例: rosrun pkg script.py _points_file:=/path/to/points.csv
    points_file = rospy.get_param("~points_file", "")

    if not points_file:
        rospy.logfatal("~points_file is empty. Provide a .csv/.npy/.yaml file path.")
        return

    # package:// を解決したいとき用（例: package://me6_bringup/data/points.csv）
    if points_file.startswith("package://"):
        # package://<pkg>/<relpath>
        rest = points_file[len("package://"):]
        pkg = rest.split("/")[0]
        rel = rest[len(pkg) + 1:]
        points_file = os.path.join(rospkg.RosPack().get_path(pkg), rel)

    if not os.path.exists(points_file):
        rospy.logfatal("points_file not found: %s", points_file)
        return

    # 読み込み
    try:
        points = load_points(points_file)
    except Exception as e:
        rospy.logfatal("Failed to load points_file: %s", str(e))
        return

    if len(points) == 0:
        rospy.logfatal("No points loaded from: %s", points_file)
        return

    n_points = int(rospy.get_param("~n_points", len(points)))
    if n_points <= 0:
        rospy.logfatal("~n_points must be > 0")
        return

    # pointsが多い場合は先頭n個を使う（必要なら random shuffle も可）
    points = points[:n_points]

    # --- Publishers ---
    way_pub = rospy.Publisher("way_points", PoseArray, queue_size=1)
    x_pub = rospy.Publisher("way_points_x", Float64MultiArray, queue_size=1)
    y_pub = rospy.Publisher("way_points_y", Float64MultiArray, queue_size=1)
    z_pub = rospy.Publisher("way_points_z", Float64MultiArray, queue_size=1)

    # marker model.sdf path
    pkg = "me6_bringup"
    rel = "models/marker_sphere/model.sdf"
    model_path = os.path.join(rospkg.RosPack().get_path(pkg), rel)

    if not os.path.exists(model_path):
        rospy.logfatal("model.sdf not found: %s", model_path)
        return

    with open(model_path, "r") as f:
        sdf_xml = f.read()

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    rospy.loginfo("Loaded %d waypoints from file: %s", len(points), points_file)
    rospy.loginfo("Spawning Gazebo markers ...")

    # orientation (identity)
    q = quaternion_from_euler(0, 0, 0)

    # PoseArray message（生成は1回）
    pa = PoseArray()
    pa.header.frame_id = "world"

    # 生成 → spawn
    for i, (x, y, z) in enumerate(points):
        name = f"{namespace}{i+1}"
        try_delete(name)

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

        pa.poses.append(pose)

        resp = spawn(name, sdf_xml, "", pose, "world")
        rospy.loginfo("Spawn %s: %s  (x=%.3f y=%.3f z=%.3f)",
                      name, resp.status_message, pose.position.x, pose.position.y, pose.position.z)

    rospy.loginfo("Start publishing waypoints continuously: /way_points, /way_points_x, /way_points_y, /way_points_z (%.1f Hz)", pub_rate)

    rate = rospy.Rate(pub_rate)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        pa.header.stamp = now
        way_pub.publish(pa)

        xs = [p.position.x for p in pa.poses]
        ys = [p.position.y for p in pa.poses]
        zs = [p.position.z for p in pa.poses]

        x_pub.publish(Float64MultiArray(data=xs))
        y_pub.publish(Float64MultiArray(data=ys))
        z_pub.publish(Float64MultiArray(data=zs))

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
