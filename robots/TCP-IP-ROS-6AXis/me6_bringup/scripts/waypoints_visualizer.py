#!/usr/bin/env python
import os
import random
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler

WORKSPACE = {
    "x": (-0.40, 0.40),
    "y": (-0.40, 0.40),
    "z": ( 0.05, 0.75),
}

def try_delete(name):
    try:
        rospy.wait_for_service("/gazebo/delete_model", timeout=0.5)
        delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete(name)
    except Exception:
        pass

def main():
    rospy.init_node("spawn_random_waypoints_gazebo")

    # --- Publisher: way_points (PoseArray) ---
    way_pub = rospy.Publisher(
        "way_points",          # topic name
        PoseArray,
        queue_size=1,
        latch=True             # keep last message for late subscribers
    )

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

    rospy.loginfo("Generating 5 random waypoints (Gazebo models) + publishing to /way_points ...")

    # orientation (identity)
    q = quaternion_from_euler(0, 0, 0)

    # PoseArray message
    pa = PoseArray()
    pa.header.frame_id = "world"      # この座標系で生成している
    pa.header.stamp = rospy.Time.now()

    # 5点生成 → publish用に保存 → spawn
    for i in range(5):
        name = f"wp_{i+1}"
        try_delete(name)

        pose = Pose()
        pose.position.x = random.uniform(*WORKSPACE["x"])
        pose.position.y = random.uniform(*WORKSPACE["y"])
        pose.position.z = random.uniform(*WORKSPACE["z"])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

        # まずPoseArrayに追加
        pa.poses.append(pose)

        # Gazeboにスポーン
        resp = spawn(name, sdf_xml, "", pose, "world")

        rospy.loginfo("Spawn %s: %s  (x=%.3f y=%.3f z=%.3f)",
                      name, resp.status_message,
                      pose.position.x, pose.position.y, pose.position.z)

    # --- publish once (latched) ---
    way_pub.publish(pa)
    rospy.loginfo("Published %d waypoints to /way_points (frame_id=%s).", len(pa.poses), pa.header.frame_id)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
