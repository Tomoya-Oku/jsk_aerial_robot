#!/usr/bin/env python3
"""URDF kinematics about the moving CoG with fixed FC orientation."""
import xml.etree.ElementTree as ET
import numpy as np
from contact_math import finite_jacobian


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew.dot(skew)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        transform[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return transform


class ContactModel:
    def __init__(self, xml, fc="fc", tip="link1"):
        root = ET.fromstring(xml)
        self.fc, self.tip = fc, tip
        self.masses, self.joints = [], []
        links = {e.get("name") for e in root.findall("link")}
        children = set()
        for e in root.findall("link"):
            inertial = e.find("inertial")
            if inertial is not None:
                mass = float(inertial.find("mass").get("value"))
                self.masses.append((e.get("name"), mass, origin(inertial.find("origin"))[:3, 3]))
        for e in root.findall("joint"):
            kind = e.get("type")
            if kind not in ("fixed", "revolute", "continuous", "prismatic") or e.find("mimic") is not None:
                raise ValueError("unsupported joint: " + e.get("name"))
            axis = e.find("axis")
            axis = np.fromstring(axis.get("xyz", "1 0 0") if axis is not None else "1 0 0", sep=" ")
            parent, child = e.find("parent").get("link"), e.find("child").get("link")
            self.joints.append((e.get("name"), kind, parent, child, origin(e.find("origin")), axis))
            children.add(child)
        roots = links - children
        if len(roots) != 1 or fc not in links or tip not in links or sum(m for _, m, _ in self.masses) <= 0:
            raise ValueError("invalid URDF roots, mass, FC or tip")
        self.root = roots.pop()
        self.required_joints = {name for name, kind, _, _, _, _ in self.joints
                                if kind != "fixed" and not name.startswith("rotor")}

    def transforms(self, positions):
        frames, pending = {self.root: np.eye(4)}, list(self.joints)
        while pending:
            remaining = []
            for name, kind, parent, child, offset, axis in pending:
                if parent not in frames:
                    remaining.append((name, kind, parent, child, offset, axis))
                    continue
                motion = np.eye(4)
                q = float(positions.get(name, 0.0))
                if not np.isfinite(q):
                    raise ValueError("non-finite robot position")
                if kind in ("continuous", "revolute"):
                    motion[:3, :3] = rotation(axis, q)
                elif kind == "prismatic":
                    motion[:3, 3] = axis * q
                frames[child] = frames[parent] @ offset @ motion
            if len(remaining) == len(pending):
                raise ValueError("disconnected URDF")
            pending = remaining
        return frames

    def tip_pose(self, positions):
        frames = self.transforms(positions)
        fc_inv = np.linalg.inv(frames[self.fc])
        cog = sum(m * (frames[name][:3, :3] @ p + frames[name][:3, 3])
                  for name, m, p in self.masses) / sum(m for _, m, _ in self.masses)
        pose = fc_inv @ frames[self.tip]
        cog_fc = fc_inv[:3, :3] @ cog + fc_inv[:3, 3]
        pose[:3, 3] -= cog_fc
        return pose

    def jacobian(self, positions, names):
        missing = self.required_joints - positions.keys()
        if missing:
            raise ValueError("missing robot joints: " + ", ".join(sorted(missing)))
        base_rotation = self.tip_pose(positions)[:3, :3]

        def displacement(q):
            pose = self.tip_pose(dict(positions, **dict(zip(names, q))))
            delta = pose[:3, :3] @ base_rotation.T
            # Infinitesimal spatial rotation expressed in the FC orientation.
            omega = 0.5 * np.array([delta[2, 1] - delta[1, 2],
                                    delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
            return np.r_[pose[:3, 3], omega]

        return finite_jacobian(displacement, [positions[n] for n in names])

    def wrench_in_fc(self, positions, wrench):
        rotation_fc_tip = self.tip_pose(positions)[:3, :3]
        return np.r_[rotation_fc_tip @ wrench[:3], rotation_fc_tip @ wrench[3:]]
