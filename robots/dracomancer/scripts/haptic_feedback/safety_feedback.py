#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Directional safety feedback model for Dracomancer Mk-II."""

import numpy as np


class SafetyFeedback:
    """Return rejected robot-shape motion to the operator as resistance."""

    def __init__(self, robot_stiffness, human_damping, gain=1.0):
        self.robot_stiffness = np.asarray(robot_stiffness, dtype=float)
        self.human_damping = np.asarray(human_damping, dtype=float)
        self.gain = float(gain)

    @property
    def robot_joint_count(self):
        return self.robot_stiffness.size

    @property
    def human_joint_count(self):
        return self.human_damping.size

    def compute(self, shape_error, mapping_jacobian, human_velocity):
        shape_error = np.asarray(shape_error, dtype=float)
        mapping_jacobian = np.asarray(mapping_jacobian, dtype=float)
        human_velocity = np.asarray(human_velocity, dtype=float)

        expected_mapping = (self.robot_joint_count, self.human_joint_count)
        if shape_error.shape != (self.robot_joint_count,):
            raise ValueError("shape error size does not match robot joints")
        if mapping_jacobian.shape != expected_mapping:
            raise ValueError("mapping Jacobian must have shape %s" % (expected_mapping,))
        if human_velocity.shape != (self.human_joint_count,):
            raise ValueError("human velocity size does not match human joints")
        if not (np.all(np.isfinite(shape_error)) and
                np.all(np.isfinite(mapping_jacobian)) and
                np.all(np.isfinite(human_velocity))):
            raise ValueError("safety feedback input contains non-finite values")

        # shape_error = q_ref - q_feasible.  The minus sign makes the cue resist
        # motion toward the rejected shape rather than push the operator into it.
        robot_resistance = -self.robot_stiffness * shape_error
        spring = mapping_jacobian.T.dot(robot_resistance)
        damping = -self.human_damping * human_velocity
        return self.gain * spring + damping


def combine_torque(contact, safety, external, limits, deadband=0.0):
    contact = np.asarray(contact, dtype=float)
    safety = np.asarray(safety, dtype=float)
    external = np.asarray(external, dtype=float)
    limits = np.abs(np.asarray(limits, dtype=float))
    if not (contact.shape == safety.shape == external.shape == limits.shape):
        raise ValueError("haptic torque vectors must have the same shape")
    if not (np.all(np.isfinite(contact)) and np.all(np.isfinite(safety)) and
            np.all(np.isfinite(external)) and np.all(np.isfinite(limits))):
        raise ValueError("haptic torque vectors must contain finite values")

    total = contact + safety + external
    total[np.abs(total) < abs(float(deadband))] = 0.0
    return np.clip(total, -limits, limits)


def rate_limit(current, previous, limits_per_second, dt):
    current = np.asarray(current, dtype=float)
    previous = np.asarray(previous, dtype=float)
    limits = np.abs(np.asarray(limits_per_second, dtype=float))
    if not (current.shape == previous.shape == limits.shape):
        raise ValueError("torque rate-limit vectors must have the same shape")
    if not (np.all(np.isfinite(current)) and np.all(np.isfinite(previous)) and
            np.all(np.isfinite(limits))):
        raise ValueError("torque rate-limit vectors must contain finite values")
    if dt <= 0.0:
        return previous.copy()
    step = limits * float(dt)
    return previous + np.clip(current - previous, -step, step)
