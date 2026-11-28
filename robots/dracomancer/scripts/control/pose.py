"""Rotation conversions shared by the DRAGON pose controllers."""

import math

import numpy as np

from .shape import vector


def quat_from_euler(roll, pitch, yaw):
    # intrinsic Rz(yaw) * Ry(pitch) * Rx(roll)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quaternion_matrix(xyzw):
    q=vector(xyzw,4)
    if np.linalg.norm(q)<1e-9: raise ValueError('invalid orientation')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def matrix_rpy(r):
    horizontal = np.hypot(r[0, 0], r[1, 0])
    pitch = np.arctan2(-r[2, 0], horizontal)
    if horizontal < 1e-9:
        return np.array([0., pitch, np.arctan2(-r[0, 1], r[1, 1])])
    return np.array([np.arctan2(r[2, 1], r[2, 2]), pitch,
                     np.arctan2(r[1, 0], r[0, 0])])
