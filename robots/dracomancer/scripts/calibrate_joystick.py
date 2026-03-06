#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import yaml
import rospy

from std_msgs.msg import Int16MultiArray, Float32MultiArray


class JoystickCalibPublisher:
    def __init__(self):
        rospy.init_node("joystick_calib_publisher")

        self.in_topic = rospy.get_param("~in_topic", "/joystick/raw")
        self.out_topic = rospy.get_param("~out_topic", "/joystick/calib")
        self.axis_indices = rospy.get_param("~axis_indices", [0, 1])
        self.sample_rate = rospy.get_param("~sample_rate", 100)
        self.center_duration = rospy.get_param("~center_duration", 2.0)
        self.range_duration = rospy.get_param("~range_duration", 5.0)
        self.deadzone = rospy.get_param("~deadzone", 0.05)
        self.invert_axes = rospy.get_param("~invert_axes", [])   # 例: [1]
        self.save_yaml = rospy.get_param("~save_yaml", True)
        self.yaml_path = rospy.get_param("~yaml_path", "joystick_calibration.yaml")

        self.latest_raw = None
        self.center = None
        self.min_vals = None
        self.max_vals = None

        self.sub = rospy.Subscriber(self.in_topic, Int16MultiArray, self.cb_raw, queue_size=10)
        self.pub = rospy.Publisher(self.out_topic, Float32MultiArray, queue_size=10)

    def cb_raw(self, msg):
        self.latest_raw = list(msg.data)

    def wait_for_first_message(self, timeout_sec=5.0):
        rospy.loginfo("Waiting for joystick message on %s ...", self.in_topic)
        start = rospy.Time.now()
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            if self.latest_raw is not None:
                rospy.loginfo("Joystick message received.")
                return True
            if (rospy.Time.now() - start).to_sec() > timeout_sec:
                return False
            rate.sleep()
        return False

    def extract_axes(self, data):
        try:
            return [float(data[i]) for i in self.axis_indices]
        except Exception as e:
            rospy.logerr_throttle(1.0, "Failed to extract axes: %s", str(e))
            return None

    def sample_average(self, duration_sec):
        rate = rospy.Rate(self.sample_rate)
        n = max(1, int(duration_sec * self.sample_rate))

        sums = [0.0 for _ in self.axis_indices]
        count = 0

        for _ in range(n):
            if rospy.is_shutdown():
                break
            if self.latest_raw is not None:
                axes = self.extract_axes(self.latest_raw)
                if axes is not None:
                    for i, v in enumerate(axes):
                        sums[i] += v
                    count += 1
            rate.sleep()

        if count == 0:
            raise RuntimeError("No valid samples for center calibration.")

        return [s / count for s in sums]

    def sample_minmax(self, duration_sec):
        rate = rospy.Rate(self.sample_rate)
        n = max(1, int(duration_sec * self.sample_rate))

        mins = [float("inf") for _ in self.axis_indices]
        maxs = [float("-inf") for _ in self.axis_indices]
        count = 0

        for _ in range(n):
            if rospy.is_shutdown():
                break
            if self.latest_raw is not None:
                axes = self.extract_axes(self.latest_raw)
                if axes is not None:
                    for i, v in enumerate(axes):
                        mins[i] = min(mins[i], v)
                        maxs[i] = max(maxs[i], v)
                    count += 1
            rate.sleep()

        if count == 0:
            raise RuntimeError("No valid samples for min/max calibration.")

        return mins, maxs

    @staticmethod
    def normalize(raw, center, min_v, max_v):
        if raw >= center:
            denom = max(max_v - center, 1e-6)
            return (raw - center) / denom
        else:
            denom = max(center - min_v, 1e-6)
            return (raw - center) / denom

    @staticmethod
    def apply_deadzone(x, dz):
        if abs(x) < dz:
            return 0.0
        return x

    def save_calibration_yaml(self):
        if not self.save_yaml:
            return

        data = {
            "topic": self.in_topic,
            "axis_indices": self.axis_indices,
            "center": [float(x) for x in self.center],
            "min": [float(x) for x in self.min_vals],
            "max": [float(x) for x in self.max_vals],
            "deadzone": float(self.deadzone),
            "invert_axes": list(self.invert_axes),
        }

        out_dir = os.path.dirname(self.yaml_path)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)

        with open(self.yaml_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

        rospy.loginfo("Saved calibration yaml: %s", self.yaml_path)

    def run_calibration(self):
        if not self.wait_for_first_message():
            raise RuntimeError("No joystick messages received.")

        print("")
        print("=== Joystick Calibration ===")
        print("1) スティックに触らず中立位置にしてください。")
        input("   準備ができたら Enter を押してください: ")

        self.center = self.sample_average(self.center_duration)
        print("center =", self.center)

        print("")
        print("2) 次は可動範囲を測ります。")
        print("   Enter を押したら {} 秒間、各軸を端まで大きく動かしてください。".format(self.range_duration))
        input("   準備ができたら Enter を押してください: ")

        self.min_vals, self.max_vals = self.sample_minmax(self.range_duration)
        print("min =", self.min_vals)
        print("max =", self.max_vals)

        self.save_calibration_yaml()

        rospy.loginfo("Calibration completed.")
        rospy.loginfo("center=%s", self.center)
        rospy.loginfo("min=%s", self.min_vals)
        rospy.loginfo("max=%s", self.max_vals)

    def publish_loop(self):
        if self.center is None or self.min_vals is None or self.max_vals is None:
            raise RuntimeError("Calibration parameters are not initialized.")

        rospy.loginfo("Publishing calibrated joystick to %s", self.out_topic)

        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            if self.latest_raw is None:
                rate.sleep()
                continue

            axes = self.extract_axes(self.latest_raw)
            if axes is None:
                rate.sleep()
                continue

            calib = []
            for i, raw in enumerate(axes):
                val = self.normalize(raw, self.center[i], self.min_vals[i], self.max_vals[i])

                if i in self.invert_axes:
                    val = -val

                val = self.apply_deadzone(val, self.deadzone)
                val = max(-1.0, min(1.0, val))
                calib.append(float(val))

            msg = Float32MultiArray()
            msg.data = calib
            self.pub.publish(msg)

            sys.stdout.write("\rraw={} calib={}".format([int(x) for x in axes],
                                                        [round(x, 3) for x in calib]))
            sys.stdout.flush()

            rate.sleep()

    def run(self):
        self.run_calibration()
        print("")
        print("Calibration finished. Publishing started. Ctrl+C で終了します.")
        self.publish_loop()


if __name__ == "__main__":
    try:
        node = JoystickCalibPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(str(e))