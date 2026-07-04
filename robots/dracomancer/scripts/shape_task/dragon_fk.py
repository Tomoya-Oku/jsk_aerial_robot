#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Minimal DRAGON forward kinematics shared by the shape-task nodes.

Mirrors the fc->link4 chain used by control_joint_angle.py
(dragon_fc_to_link4_matrix): each link is link_length long; a pitch joint
(about +Y) sits at the link end, followed by a 2*inter_joint_x_offset spacer
and the yaw joint (about +Z). All positions are expressed in the link1-root
frame, so the shape comparison is invariant to the flying body pose.

Units: rad / m. Joint order: [j1_pitch, j1_yaw, j2_pitch, j2_yaw, j3_pitch, j3_yaw].
"""

import numpy as np

DEFAULT_LINK_LENGTH = 0.474
DEFAULT_INTER_JOINT_X_OFFSET = 0.02575
JOINT_ORDER = [
    "joint1_pitch", "joint1_yaw",
    "joint2_pitch", "joint2_yaw",
    "joint3_pitch", "joint3_yaw",
]


def _trans_x(d):
    m = np.eye(4)
    m[0, 3] = d
    return m


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    m = np.eye(4)
    m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c
    return m


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    m = np.eye(4)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c
    return m


def total_length(link_length=DEFAULT_LINK_LENGTH,
                 inter_joint_x_offset=DEFAULT_INTER_JOINT_X_OFFSET):
    return 4.0 * link_length + 6.0 * inter_joint_x_offset


def fk_points(q, link_length=DEFAULT_LINK_LENGTH,
              inter_joint_x_offset=DEFAULT_INTER_JOINT_X_OFFSET):
    """Return a (5, 3) array of node positions in the link1-root frame:
    [link1 origin, joint1, joint2, joint3, link4 tip]."""
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    for k in range(3):
        T = T.dot(_trans_x(link_length))
        pts.append(T[:3, 3].copy())  # joint (k+1) pitch axis location
        T = T.dot(_rot_y(q[2 * k]))
        T = T.dot(_trans_x(2.0 * inter_joint_x_offset))
        T = T.dot(_rot_z(q[2 * k + 1]))
    T = T.dot(_trans_x(link_length))
    pts.append(T[:3, 3].copy())  # link4 tip
    return np.array(pts)


def joint_error(q, q_star):
    """RMS joint-angle error E_q [rad]. Returns NaN when inputs are invalid."""
    q = np.asarray(q, dtype=float)
    q_star = np.asarray(q_star, dtype=float)
    if q.shape != q_star.shape or np.any(np.isnan(q)) or np.any(np.isnan(q_star)):
        return float("nan")
    return float(np.sqrt(np.mean((q - q_star) ** 2)))


def shape_error(q, q_star, link_length=DEFAULT_LINK_LENGTH,
                inter_joint_x_offset=DEFAULT_INTER_JOINT_X_OFFSET,
                normalize=False):
    """RMS of the link node position errors E_s [m] between two shapes.

    The link1 origin is common to both shapes, so only the 4 joint/tip nodes
    that actually depend on q are compared. normalize=True divides by the
    robot's total stretched length (dimensionless output).
    """
    q = np.asarray(q, dtype=float)
    q_star = np.asarray(q_star, dtype=float)
    if q.shape != q_star.shape or np.any(np.isnan(q)) or np.any(np.isnan(q_star)):
        return float("nan")
    p = fk_points(q, link_length, inter_joint_x_offset)[1:]
    p_star = fk_points(q_star, link_length, inter_joint_x_offset)[1:]
    err = float(np.sqrt(np.mean(np.sum((p - p_star) ** 2, axis=1))))
    if normalize:
        err /= total_length(link_length, inter_joint_x_offset)
    return err
