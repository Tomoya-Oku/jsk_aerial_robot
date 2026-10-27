#!/usr/bin/env python3
"""Contact truth -> CoG-relative virtual work -> opt-in XM430 current output."""
import os
import sys
import threading
import time
import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, UInt8
from std_srvs.srv import SetBool, SetBoolResponse
from spinal.msg import ServoControlCmd
from spinal.srv import HapticConfig, HapticConfigRequest
from dracomancer.msg import MappingState

# Works both from source and through catkin's executable relay/install tree.
sys.path.insert(0, os.path.join(rospkg.RosPack().get_path("dracomancer"), "scripts", "control"))
from contact_model import ContactModel
from contact_math import reflect_wrench


class ContactHaptics:
    def __init__(self):
        self.settings = rospy.get_param("/dracomancer/contact/haptics")
        p = self.settings
        self.hardware = bool(rospy.get_param("~hardware", True))
        self.ids = list(p["servo_indices"])
        self.limits = np.asarray(p["torque_limits"], dtype=float)
        self.coefficients = np.asarray(p["torque_per_amp"], dtype=float)
        self.signs = np.asarray(p["current_signs"], dtype=float)
        self.timeout = float(p["input_timeout"])
        self.gain = float(p["gain"])
        if (len(self.ids) != 7 or len(set(self.ids)) != 7 or
                any(not isinstance(i, int) or i < 0 or i > 254 for i in self.ids) or
                self.limits.shape != (7,) or self.coefficients.shape != (7,) or self.signs.shape != (7,) or
                not all(np.all(np.isfinite(v)) for v in (self.limits, self.coefficients, self.signs)) or
                np.any(self.limits <= 0) or np.any(self.coefficients <= 0) or
                np.any(np.abs(self.signs) != 1) or self.timeout <= 0 or
                not np.isfinite(self.timeout) or not np.isfinite(self.gain) or self.gain < 0):
            raise ValueError("invalid contact haptic configuration")
        # Floor the absolute current bound: rounding may exceed the approved
        # nominal 0.2 Nm limit after conversion to 2.69 mA register units.
        self.current_limits = np.floor(self.limits / self.coefficients / 0.00269).astype(int)
        if np.any(self.current_limits < 1) or np.any(self.current_limits > 1193):
            raise ValueError("current limit outside XM430 range")
        self.model = None
        self.lock = threading.RLock()
        self.transition = threading.Lock()
        self.inputs, self.received, self.advanced = {}, {}, {}
        self.joints, self.joint_times = {}, {}
        self.mode, self.flight = bool(rospy.get_param("~teleop_mode", False)), None
        self.active = self.arming = False
        self.robot_names, self.device_names = [], []
        self.current_pub = rospy.Publisher("/dracomancer/servo/target_current", ServoControlCmd, queue_size=1)
        self.debug_pub = rospy.Publisher("/dracomancer/contact/torque", JointState, queue_size=1)
        self.enabled_pub = rospy.Publisher("/dracomancer/contact/enabled", Bool, queue_size=1, latch=True)
        self.config = rospy.ServiceProxy("/dracomancer/haptic_config", HapticConfig)
        rospy.Subscriber("/dragon/contact/wrench", WrenchStamped, lambda m: self.store("wrench", m, m.header.stamp), queue_size=1)
        rospy.Subscriber("/dracomancer/mapping_state", MappingState, lambda m: self.store("mapping", m, m.header.stamp), queue_size=1)
        rospy.Subscriber("/clock", Clock, lambda m: self.store("clock", m, m.clock), queue_size=1)
        rospy.Subscriber("/dragon/joint_states", JointState, self.joint_cb, queue_size=1)
        rospy.Subscriber("/dragon/flight_state", UInt8, self.flight_cb, queue_size=1)
        rospy.Subscriber("/dracomancer/teleop_mode", Bool, self.mode_cb, queue_size=1)
        rospy.Service("~enable", SetBool, self.enable)
        rospy.on_shutdown(self.shutdown)

    def store(self, key, msg, stamp):
        now = time.monotonic()
        with self.lock:
            old = self.inputs.get(key)
            previous = old[1] if old else None
            if previous is None or stamp > previous:
                self.advanced[key] = now
            elif stamp < previous:
                self.trip("simulation/input time moved backwards")
                self.advanced.pop(key, None)
            self.inputs[key] = (msg, stamp)
            self.received[key] = now

    def joint_cb(self, msg):
        now = time.monotonic()
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                self.joints[name] = float(value)
                self.joint_times[name] = (now, msg.header.stamp)

    def flight_cb(self, msg):
        with self.lock:
            self.flight, self.flight_time = int(msg.data), time.monotonic()

    def mode_cb(self, msg):
        with self.lock:
            self.mode = bool(msg.data)

    def ready(self):
        now = time.monotonic()
        if not self.mode or self.flight != 5 or now - getattr(self, "flight_time", 0) > self.timeout:
            return False
        if self.model is None:
            return False
        for key in ("clock", "mapping", "wrench"):
            if (key not in self.inputs or now - self.received[key] > self.timeout or
                    now - self.advanced.get(key, 0) > self.timeout):
                return False
        sim = self.inputs["clock"][1].to_sec()
        for key in ("mapping", "wrench"):
            stamp = self.inputs[key][1].to_sec()
            if stamp <= 0 or not -0.02 <= sim - stamp <= self.timeout:
                return False
        for name in self.model.required_joints:
            receipt, stamp = self.joint_times.get(name, (0, rospy.Time(0)))
            if now - receipt > self.timeout or not -0.02 <= sim - stamp.to_sec() <= self.timeout:
                return False
        return True

    def firmware(self, command):
        return self.config(command=command, indices=self.ids,
                           timeout_ms=int(self.settings["firmware_timeout_ms"]),
                           current_limits=self.current_limits.tolist())

    def enable(self, req):
        with self.transition:
            if not req.data:
                with self.lock:
                    self.trip("operator disabled", force=True)
                if self.hardware:
                    try:
                        self.firmware(HapticConfigRequest.DISARM)
                    except rospy.ServiceException:
                        pass  # The 200 ms MCU watchdog still runs without ROS.
                return SetBoolResponse(True, "disabled; explicit enable required to restart")
            with self.lock:
                if self.active:
                    return SetBoolResponse(True, "already enabled")
                if not self.ready():
                    return SetBoolResponse(False, "need fresh clock/contact/mapping/joints, HOVER and teleop_mode=true")
                self.arming = True
            try:
                if self.hardware:
                    response = self.firmware(HapticConfigRequest.CONFIGURE)
                    if not response.success:
                        raise ValueError("firmware rejected configuration")
                    self.wait_firmware("ready")
                    with self.lock:
                        if not self.arming or not self.ready():
                            raise ValueError("inputs expired during configuration")
                    if not self.firmware(HapticConfigRequest.ARM).success:
                        raise ValueError("firmware rejected ARM")
                    self.wait_firmware("armed")
                with self.lock:
                    if not self.arming or not self.ready():
                        raise ValueError("inputs expired during ARM")
                    self.active, self.arming = True, False
                return SetBoolResponse(True, "enabled" if self.hardware else "dry-run enabled; no current output")
            except (rospy.ServiceException, ValueError) as error:
                with self.lock:
                    self.trip(str(error), force=True)
                return SetBoolResponse(False, str(error))

    def wait_firmware(self, field):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if not self.arming or not self.ready():
                    raise ValueError("inputs expired during firmware handshake")
            state = self.firmware(HapticConfigRequest.STATUS)
            if state.success and getattr(state, field) and not state.fault:
                return
            time.sleep(0.02)
        raise ValueError("firmware readback timeout: " + field)

    def publish_current(self, units):
        if self.hardware:
            msg = ServoControlCmd()
            msg.stamp = rospy.Time.now()
            msg.index, msg.angles = self.ids, [int(v) for v in units]
            self.current_pub.publish(msg)

    def trip(self, reason, force=False):
        if self.active or self.arming or force:
            self.active = self.arming = False
            self.publish_current([0] * 7)
            # Stop the current heartbeat too. Spinal then latches torque OFF;
            # subsequent healthy inputs cannot re-enable either MCU or this node.
            rospy.logwarn("contact haptics stopped: %s", reason)

    def torque(self):
        mapping = self.inputs["mapping"][0]
        if len(mapping.robot_joints) != 6 or len(mapping.device_joints) != 7:
            raise ValueError("expected 6 robot and 7 device joints")
        if len(set(mapping.robot_joints)) != 6 or len(set(mapping.device_joints)) != 7:
            raise ValueError("duplicate mapping names")
        expected = ["shoulder_abduction_adduction_joint", "shoulder_flexion_extension_joint",
                    "upper_arm_external_internal_rotation_joint", "elbow_flexion_extension_joint",
                    "wrist_supination_joint", "wrist_flexion_extension_joint", "wrist_abduction_adduction_joint"]
        if list(mapping.device_joints) != expected:
            raise ValueError("unexpected device joint order")
        self.robot_names, self.device_names = list(mapping.robot_joints), expected
        matrix = np.asarray(mapping.jacobian).reshape(6, 7)
        contact = self.inputs["wrench"][0]
        if contact.header.frame_id != "dragon/link1":
            raise ValueError("contact wrench must be about the link1 origin, in dragon/link1")
        f, t = contact.wrench.force, contact.wrench.torque
        wrench = self.model.wrench_in_fc(self.joints, np.array([f.x, f.y, f.z, t.x, t.y, t.z]))
        jacobian = self.model.jacobian(self.joints, self.robot_names)
        _, limited = reflect_wrench(matrix, jacobian, wrench, self.gain, self.limits)
        return limited

    def tick(self):
        with self.lock:
            torque = np.zeros(7)
            if self.active or self.arming:
                if not self.ready():
                    self.trip("input/physics timeout or teleop/hover ended")
                else:
                    try:
                        if self.active:
                            torque = self.torque()
                        units = np.rint(self.signs * torque / self.coefficients / 0.00269)
                        self.publish_current(np.clip(units, -self.current_limits, self.current_limits))
                    except (ValueError, KeyError, np.linalg.LinAlgError) as error:
                        self.trip(str(error))
            debug = JointState()
            debug.header.stamp = rospy.Time.now()
            debug.name, debug.effort = self.device_names, torque.tolist()
            self.debug_pub.publish(debug)
            self.enabled_pub.publish(Bool(self.active))

    def shutdown(self):
        with self.lock:
            self.trip("shutdown", force=True)

    def run(self):
        # Wall-clock loop is essential: rospy.Rate stops with paused /clock.
        while not rospy.is_shutdown():
            if self.model is None and rospy.has_param("/dragon/robot_description"):
                self.model = ContactModel(rospy.get_param("/dragon/robot_description"))
            self.tick()
            time.sleep(0.025)


if __name__ == "__main__":
    rospy.init_node("contact_haptics")
    ContactHaptics().run()
