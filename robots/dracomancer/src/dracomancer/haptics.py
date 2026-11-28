"""Virtual-work reflection, shape constraint feedback and torque limits."""
import numpy as np


def _reflect(mapping, robot_jacobian, wrench, gain):
    return float(gain) * mapping.T.dot(robot_jacobian.T.dot(wrench))


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
        if self.robot_joint_count <= 0 or self.human_joint_count <= 0 or not np.isfinite(self.gain) or self.gain < 0:
            raise ValueError("joint counts must be positive and contact gain nonnegative")

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

        return _reflect(mapping_jacobian, robot_jacobian, wrench, self.gain)


def reflect_wrench(mapping, robot_jacobian, wrench, gain, limits):
    """Return raw and limited device torques using gain * M.T * J.T * w.

    w is the environment-on-robot wrench. Do not negate it again: virtual
    work maps that resisting wrench to a resisting device torque already.
    """
    mapping = np.asarray(mapping, dtype=float)
    robot_jacobian = np.asarray(robot_jacobian, dtype=float)
    wrench = np.asarray(wrench, dtype=float)
    limits = np.asarray(limits, dtype=float)
    if mapping.ndim != 2 or robot_jacobian.shape != (6, mapping.shape[0]):
        raise ValueError("incompatible mapping and robot Jacobian dimensions")
    if wrench.shape != (6,) or limits.shape != (mapping.shape[1],):
        raise ValueError("incompatible wrench or device torque limit dimensions")
    if (not all(np.all(np.isfinite(v)) for v in
                (mapping, robot_jacobian, wrench, limits)) or
            not np.isfinite(gain) or gain < 0 or np.any(limits < 0)):
        raise ValueError("non-finite input or negative gain/torque limit")
    raw = _reflect(mapping, robot_jacobian, wrench, gain)
    if not np.all(np.isfinite(raw)):
        raise ValueError("reflected torque is not finite")
    return raw, np.clip(raw, -limits, limits)


class SafetyFeedback:
    """Return rejected robot-shape motion to the operator as resistance."""

    def __init__(self, robot_stiffness, human_damping, gain=1.0):
        self.robot_stiffness = np.asarray(robot_stiffness, dtype=float)
        self.human_damping = np.asarray(human_damping, dtype=float)
        self.gain = float(gain)
        # Nonnegative gains are required by the thesis virtual potential.
        if (self.robot_stiffness.ndim != 1 or self.human_damping.ndim != 1
                or not self.robot_stiffness.size or not self.human_damping.size
                or not all(np.all(np.isfinite(v)) for v in (self.robot_stiffness, self.human_damping))
                or np.any(self.robot_stiffness < 0) or np.any(self.human_damping < 0)
                or not np.isfinite(self.gain) or self.gain < 0):
            raise ValueError("stiffness, damping, and safety gain must be finite and nonnegative")

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

        # shape_error = q_ref - q_cmd. The minus sign makes the cue resist
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
