#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Contact-wrench to human-joint torque mapping for Dracomancer Mk-II."""

import numpy as np


class ContactFeedback:
    """Apply tau_h = M.T * J_r.T * wrench.

    ``robot_jacobian`` maps robot joint velocity to the wrench-frame twist and
    ``mapping_jacobian`` maps human joint velocity to robot joint velocity.
    Keeping both matrices as inputs lets the fixed-link and global-shape
    mappings share this haptic implementation.
    """

    def __init__(self, robot_joint_count, human_joint_count, gain=1.0):
        self.robot_joint_count = int(robot_joint_count)
        self.human_joint_count = int(human_joint_count)
        self.gain = float(gain)

    def compute(self, wrench, robot_jacobian, mapping_jacobian):
        wrench = np.asarray(wrench, dtype=float)
        robot_jacobian = np.asarray(robot_jacobian, dtype=float)
        mapping_jacobian = np.asarray(mapping_jacobian, dtype=float)

        expected_robot = (6, self.robot_joint_count)
        expected_mapping = (self.robot_joint_count, self.human_joint_count)
        if wrench.shape != (6,):
            raise ValueError("contact wrench must have 6 elements")
        if robot_jacobian.shape != expected_robot:
            raise ValueError("robot Jacobian must have shape %s" % (expected_robot,))
        if mapping_jacobian.shape != expected_mapping:
            raise ValueError("mapping Jacobian must have shape %s" % (expected_mapping,))
        if not (np.all(np.isfinite(wrench)) and
                np.all(np.isfinite(robot_jacobian)) and
                np.all(np.isfinite(mapping_jacobian))):
            raise ValueError("contact feedback input contains non-finite values")

        robot_torque = robot_jacobian.T.dot(wrench)
        return self.gain * mapping_jacobian.T.dot(robot_torque)
