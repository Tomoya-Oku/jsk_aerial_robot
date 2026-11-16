#!/usr/bin/env python3
"""Observe `shape_demo.launch motion:=true`; never publish robot commands."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/control"))
from shape_mapping import DRAGON_JOINTS, UrdfModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.)
    parser.add_argument("--output", default="/tmp/shape_probe.json")
    args = parser.parse_args()
    rospy.init_node("shape_probe", anonymous=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    commands, states, human, diagnostics, senders = [], [], [], [], set()

    def command(msg):
        commands.append((msg.header.stamp.to_sec(), [msg.position[msg.name.index(n)] for n in DRAGON_JOINTS]))
        senders.add(msg._connection_header.get("callerid"))

    rospy.Subscriber("/dragon/joints_ctrl", JointState, command)
    rospy.Subscriber("/dragon/joint_states", JointState, states.append)
    rospy.Subscriber("/dracomancer/joint_states", JointState, human.append)
    rospy.Subscriber("/dracomancer/shape/diagnostics", String,
                     lambda msg: diagnostics.append(json.loads(msg.data)))
    start = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-start < args.duration:
        time.sleep(.1)  # fail in wall time even if Gazebo stops publishing /clock
    if len(commands) < 2 or len(states) < 2 or not diagnostics:
        raise AssertionError("missing simulation feedback / commands / diagnostics")
    q = np.array([q for _, q in commands])
    delta = np.diff(q, axis=0)
    dt = np.diff([stamp for stamp, _ in commands])
    if np.any(dt < 0) or np.any(np.abs(delta[dt == 0]) > 1e-12):
        raise AssertionError("clock reversal or changed command at an identical stamp")
    max_speed = float(np.max(np.abs(delta[dt > 0])/dt[dt > 0, None]))
    actual = np.array([[msg.position[msg.name.index(n)] for n in DRAGON_JOINTS] for msg in states])

    def distribution(values):
        return dict(mean=float(np.mean(values)), p95=float(np.percentile(values, 95)), maximum=float(max(values)))

    tf_errors = {}
    configs = [("human", "/dracomancer/robot_description", human,
                "base_link", [n["link"] for n in rospy.get_param("/dracomancer_control_joint_angle/shape/human_points")], ""),
               ("dragon", "/dragon/robot_description", states, "link1", ["link4"], "dragon/")]
    for label, parameter, messages, base, tips, prefix in configs:
        model, errors = UrdfModel(rospy.get_param(parameter)), []
        for msg in messages[-35:-5:5]:
            for tip in tips:
                tr = buffer.lookup_transform(prefix+base, prefix+tip, msg.header.stamp, rospy.Duration(.3))
                # Gazebo feedback can overshoot a nominal bound under contact.
                # FK comparison must describe that measured state, not reject it.
                p = model.pose(base, tip, dict(zip(msg.name, msg.position)), enforce_limits=False)[:3, 3]
                t = tr.transform.translation
                errors.append(float(np.linalg.norm(p-[t.x, t.y, t.z])))
        tf_errors[label] = dict(samples=len(errors), maximum_m=max(errors))
    summary = dict(samples=len(commands), senders=sorted(senders), max_command_speed=max_speed,
                   command_displacement=np.ptp(q, axis=0).tolist(), actual_displacement=np.ptp(actual, axis=0).tolist(),
                   solve_ms=distribution([d["seconds"]*1000 for d in diagnostics if d.get("updated")]),
                   control_ms=distribution([d["control_seconds"]*1000 for d in diagnostics if "control_seconds" in d]),
                   timeout_updates=sum(d.get("updated", False) and d.get("status") == "timeout" for d in diagnostics),
                   updates=sum(d.get("updated", False) for d in diagnostics), tf_errors=tf_errors)
    Path(args.output).write_text(json.dumps(dict(summary=summary, diagnostics=diagnostics, commands=commands), indent=2))
    print(json.dumps(summary, indent=2))
    assert senders == {"/dracomancer_control_joint_angle"}, "competing command publisher"
    assert max_speed <= .1 + 1e-8, "command angular velocity exceeded"
    assert np.all(np.isfinite(q)) and np.all(np.abs(q) <= np.pi/2+1e-8)
    assert np.max(np.ptp(q, axis=0)) > 1e-3, "no command response"
    assert np.max(np.ptp(actual, axis=0)) > 1e-3, "no Gazebo response"
    assert all(error["maximum_m"] < 1e-8 for error in tf_errors.values()), "URDF/TF disagreement"


if __name__ == "__main__":
    main()
