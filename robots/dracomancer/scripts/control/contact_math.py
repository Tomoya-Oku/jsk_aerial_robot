#!/usr/bin/env python3
"""ROS-independent virtual-work calculations for contact haptics.

Spatial vectors use [Fx, Fy, Fz, Tx, Ty, Tz], in one common frame and
about the same reference point as the supplied robot Jacobian.
"""

import numpy as np


def finite_jacobian(function, position, step=1e-6):
    """Differentiate the actual mapping, including its angle saturation.

    At a nonsmooth switching boundary this gives the symmetric derivative.
    Callers must freeze neutral references and other mapping state first.
    """
    position = np.asarray(position, dtype=float)
    if position.ndim != 1 or not np.all(np.isfinite(position)):
        raise ValueError("position must be a finite vector")
    if not np.isfinite(step) or step <= 0:
        raise ValueError("step must be positive and finite")
    base = np.asarray(function(position.copy()), dtype=float)
    if base.ndim != 1 or not np.all(np.isfinite(base)):
        raise ValueError("mapping must return a finite vector")
    result = np.empty((base.size, position.size))
    for column in range(position.size):
        plus, minus = position.copy(), position.copy()
        plus[column] += step
        minus[column] -= step
        high = np.asarray(function(plus), dtype=float)
        low = np.asarray(function(minus), dtype=float)
        if high.shape != base.shape or low.shape != base.shape:
            raise ValueError("mapping output size changed")
        result[:, column] = (high - low) / (2.0 * step)
    if not np.all(np.isfinite(result)):
        raise ValueError("mapping derivative is not finite")
    return result


def shift_wrench(force, torque, source, destination):
    """Move a wrench's moment origin; all arguments share one orientation.

    A force at source contributes (source - destination) x force about
    destination. The force itself does not change.
    """
    values = [np.asarray(v, dtype=float) for v in (force, torque, source, destination)]
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in values):
        raise ValueError("wrench and reference points must be finite 3-vectors")
    force, torque, source, destination = values
    return np.r_[force, torque + np.cross(source - destination, force)]


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
    raw = float(gain) * mapping.T.dot(robot_jacobian.T.dot(wrench))
    if not np.all(np.isfinite(raw)):
        raise ValueError("reflected torque is not finite")
    return raw, np.clip(raw, -limits, limits)


def anchor_position(reference, world_rotation, relative_point):
    """Thesis eq:anchor-compensation / eq:bimanual-center-position.

    The same command q_cmd used for the joints must produce relative_point.
    This fixes the point's position; it does not command a fixed orientation.
    """
    reference, relative_point = (np.asarray(v, dtype=float) for v in (reference, relative_point))
    world_rotation = np.asarray(world_rotation, dtype=float)
    if (reference.shape != (3,) or relative_point.shape != (3,) or world_rotation.shape != (3, 3)
            or not all(np.all(np.isfinite(v)) for v in (reference, relative_point, world_rotation))
            or not np.allclose(world_rotation.T @ world_rotation, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(world_rotation), 1., atol=1e-7)):
        raise ValueError("invalid anchor position or rotation")
    return reference - world_rotation @ relative_point


def required_wrench(mass, gravity, contact_wrench, contact_from_cog):
    """Thesis eq:required-static-wrench; all components share one frame.

    contact_wrench is environment-on-robot, about the contact point E.
    """
    mass = float(mass)
    gravity, contact_from_cog = (np.asarray(v, dtype=float) for v in (gravity, contact_from_cog))
    wrench = np.asarray(contact_wrench, dtype=float)
    if (not np.isfinite(mass) or mass <= 0 or gravity.shape != (3,)
            or contact_from_cog.shape != (3,) or wrench.shape != (6,)
            or not all(np.all(np.isfinite(v)) for v in (gravity, contact_from_cog, wrench))):
        raise ValueError("invalid static wrench inputs")
    return np.r_[-mass * gravity - wrench[:3],
                 -wrench[3:] - np.cross(contact_from_cog, wrench[:3])]
