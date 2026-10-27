#!/usr/bin/env python3
"""Dry-run HOVER/mapping/enable/pause test for contact.launch hardware:=false."""
import time
import argparse
import atexit
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, UInt8, Int16MultiArray
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool, Empty as EmptyService
from dracomancer.msg import MappingState
from aerial_robot_msgs.msg import FlightNav
from gazebo_msgs.srv import GetLinkState


def wait_for(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.02)
    raise RuntimeError("timeout: " + description)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--push", action="store_true", help="also approach the wall through the joystick pipeline")
    args = parser.parse_args()
    rospy.init_node("contact_teleop_smoke")
    if rospy.get_param("/contact_haptics/hardware", True):
        raise RuntimeError("this test only accepts contact.launch hardware:=false")
    state = {"flight": None, "mapping": None, "enabled": False, "joints": {},
             "force": 0, "peak_force": 0, "max_torque": 0}
    rospy.Subscriber("/dragon/flight_state", UInt8, lambda m: state.update(flight=m.data))
    rospy.Subscriber("/dracomancer/mapping_state", MappingState, lambda m: state.update(mapping=m))
    rospy.Subscriber("/dracomancer/contact/enabled", Bool, lambda m: state.update(enabled=m.data))
    rospy.Subscriber("/dragon/joint_states", JointState,
                     lambda m: state["joints"].update(zip(m.name, m.position)))
    def contact_cb(msg):
        force = float(np.linalg.norm([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]))
        state.update(force=force, peak_force=max(state["peak_force"], force))

    rospy.Subscriber("/dragon/contact/wrench", WrenchStamped, contact_cb)
    rospy.Subscriber("/dracomancer/contact/torque", JointState,
                     lambda m: state.update(max_torque=max(state["max_torque"], max(map(abs, m.effort), default=0))))
    device = rospy.Publisher("/dracomancer/joint_cmd", JointState, queue_size=1, latch=True)
    mode = rospy.Publisher("/dracomancer/teleop_mode", Bool, queue_size=1, latch=True)
    start = rospy.Publisher("/dragon/teleop_command/start", Empty, queue_size=1)
    takeoff = rospy.Publisher("/dragon/teleop_command/takeoff", Empty, queue_size=1)
    land = rospy.Publisher("/dragon/teleop_command/land", Empty, queue_size=1)
    joystick = rospy.Publisher("/joystick/raw", Int16MultiArray, queue_size=1)
    navigation = rospy.Publisher("/dragon/uav/nav", FlightNav, queue_size=1)

    def cleanup():
        # Also clean up on an assertion failure: never leave a dry-run aircraft
        # pushing the wall while the test process exits.
        try:
            rospy.ServiceProxy("/gazebo/unpause_physics", EmptyService)()
            rospy.ServiceProxy("/contact_haptics/enable", SetBool)(False)
            mode.publish(Bool(False))
            land.publish(Empty())
            time.sleep(0.1)
        except rospy.ServiceException:
            pass

    atexit.register(cleanup)
    joints = JointState()
    joints.name = ["shoulder_abduction_adduction_joint", "shoulder_flexion_extension_joint",
                   "upper_arm_external_internal_rotation_joint", "elbow_flexion_extension_joint",
                   "wrist_supination_joint", "wrist_flexion_extension_joint", "wrist_abduction_adduction_joint"]
    joints.position = [-np.pi / 2, 0, 0, np.pi / 2, 0, 0, np.pi / 2]
    wait_for(lambda: start.get_num_connections() > 0 and takeoff.get_num_connections() > 0 and
             device.get_num_connections() > 0 and mode.get_num_connections() > 0, 10, "publishers")
    device.publish(joints)
    wait_for(lambda: state["flight"] is not None, 10, "initial flight state")
    if state["flight"] == 0:
        start.publish(Empty())
    if state["flight"] in (0, 1):
        wait_for(lambda: state["flight"] == 2, 20, "ARM_ON")
    if state["flight"] == 2:
        takeoff.publish(Empty())
    wait_for(lambda: state["flight"] == 5, 45, "HOVER")
    print("DRAGON reached HOVER", flush=True)
    mode.publish(Bool(True))
    wait_for(lambda: state["mapping"] is not None, 10, "mapping Jacobian")
    assert len(state["mapping"].jacobian) == 42
    rospy.wait_for_service("/contact_haptics/enable", timeout=10)
    enable = rospy.ServiceProxy("/contact_haptics/enable", SetBool)
    time.sleep(0.1)
    response = enable(True)
    assert response.success, response.message
    wait_for(lambda: state["enabled"], 2, "dry-run haptics enabled")
    if args.push:
        # Make a small, feasible arm movement, then turn the whole aircraft
        # through the existing navigation API. A 180-degree wrist jump crosses
        # singular shapes and can put the tail of link1 against the wall.
        joints.position[-1] -= 0.1
        device.publish(joints)
        wait_for(lambda: abs(state["joints"].get("joint1_yaw", 0) - joints.position[-1]) < 0.05,
                 20, "arm shape tracks existing mapping")
        get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

        def head_error():
            q = get_link("dragon::root", "world").link_state.pose.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return np.arctan2(np.sin(np.pi - yaw), np.cos(np.pi - yaw))

        deadline = time.monotonic() + 25
        nav = FlightNav()
        nav.target, nav.control_frame = FlightNav.COG, FlightNav.WORLD_FRAME
        nav.yaw_nav_mode = FlightNav.VEL_MODE
        while abs(head_error()) > 0.03 and time.monotonic() < deadline:
            nav.target_omega_z = float(np.clip(head_error(), -0.3, 0.3))
            navigation.publish(nav)
            time.sleep(0.05)
        nav.target_omega_z = 0
        navigation.publish(nav)
        assert abs(head_error()) < 0.08, "head did not turn toward the wall"
        if not state["enabled"]:
            response = enable(True)
            assert response.success, response.message
        wait_for(lambda: joystick.get_num_connections() > 0, 5, "joystick input")
        state["peak_force"] = 0
        deadline = time.monotonic() + 40
        print("Head aligned; approaching wall through joystick", flush=True)
        try:
            while time.monotonic() < deadline and (state["peak_force"] < 0.3 or state["max_torque"] < 1e-4):
                assert state["flight"] == 5, "DRAGON left HOVER during wall approach"
                joystick.publish(Int16MultiArray(data=[10000, 192]))
                time.sleep(0.025)
        finally:
            for _ in range(8):
                joystick.publish(Int16MultiArray(data=[-41, 192]))
                time.sleep(0.025)
        assert state["peak_force"] > 0.3, "head contact was not reached; root={}".format(
            get_link("dragon::root", "world").link_state.pose)
        wait_for(lambda: state["max_torque"] > 1e-4, 2, "nonzero contact haptic torque")
        assert state["max_torque"] <= 0.2 + 1e-9
        print("Wall contact {:.3f} N, peak requested haptic {:.3f} Nm".format(
            state["peak_force"], state["max_torque"]), flush=True)
    rospy.ServiceProxy("/gazebo/pause_physics", EmptyService)()
    wait_for(lambda: not state["enabled"], 0.4, "paused simulation watchdog")
    rospy.ServiceProxy("/gazebo/unpause_physics", EmptyService)()
    time.sleep(0.3)
    assert not state["enabled"], "haptics resumed without an explicit enable"
    response = enable(True)
    assert response.success, response.message
    wait_for(lambda: state["enabled"], 2, "explicit re-enable")
    assert enable(False).success
    mode.publish(Bool(False))
    land.publish(Empty())
    print("HOVER, 6x7 mapping, dry-run enable, pause-stop and explicit re-enable: OK", flush=True)
