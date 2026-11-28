"""Contact wrench inputs, reference-frame transformations and robot kinematics."""
import xml.etree.ElementTree as ET

import numpy as np

from .shape import finite_jacobian, rotation, transform, vector


def origin(element):
    if element is None:
        return np.eye(4)
    xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    return transform(xyz, rpy)


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

    def point_pose(self, positions, link, point=(0., 0., 0.)):
        """Point pose relative to the CoG, expressed in the fixed FC orientation."""
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("point must contain three finite coordinates")
        frames = self.transforms(positions)
        fc_inv = np.linalg.inv(frames[self.fc])
        cog = sum(m * (frames[name][:3, :3] @ p + frames[name][:3, 3])
                  for name, m, p in self.masses) / sum(m for _, m, _ in self.masses)
        pose = fc_inv @ frames[link]
        pose[:3, 3] += pose[:3, :3] @ point
        cog_fc = fc_inv[:3, :3] @ cog + fc_inv[:3, 3]
        pose[:3, 3] -= cog_fc
        return pose

    def tip_pose(self, positions):
        return self.point_pose(positions, self.tip)

    def jacobian(self, positions, names, point=(0., 0., 0.), anchor=None, link=None):
        """Internal or anchored contact Jacobian, at one nominal configuration.

        Thesis eq:anchored-jacobian: fixed base orientation makes base
        compensation a pure translation, so subtract the anchor's linear
        Jacobian. The angular contact rows are unchanged.
        """
        missing = self.required_joints - positions.keys()
        if missing:
            raise ValueError("missing robot joints: " + ", ".join(sorted(missing)))
        link=self.tip if link is None else link
        base_rotation = self.point_pose(positions, link, point)[:3, :3]

        def displacement(q):
            values = dict(positions, **dict(zip(names, q)))
            pose = self.point_pose(values, link, point)
            if anchor is not None:
                pose[:3, 3] -= self.point_pose(values, anchor['link'], anchor['xyz'])[:3, 3]
            delta = pose[:3, :3] @ base_rotation.T
            # Infinitesimal spatial rotation expressed in the FC orientation.
            omega = 0.5 * np.array([delta[2, 1] - delta[1, 2],
                                    delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
            return np.r_[pose[:3, 3], omega]

        return finite_jacobian(displacement, [positions[n] for n in names])

    def wrench_in_fc(self, positions, wrench, link=None):
        rotation_fc_tip = self.point_pose(positions,self.tip if link is None else link)[:3, :3]
        return np.r_[rotation_fc_tip @ wrench[:3], rotation_fc_tip @ wrench[3:]]


class ContactInputs:
    def __init__(self,specs,robot):
        if not isinstance(specs,list) or not specs:raise ValueError('at least one contact specification required')
        self.specs=[]
        self.robot=robot
        self.latest={}
        for spec in specs:
            link=spec.get('link','link1')
            if not isinstance(link,str) or not link:raise ValueError('contact link required')
            self.specs.append(dict(link=link,point=vector(spec.get('point',[0,0,0]),3).tolist(),
                                   topic=spec.get('topic','/'+robot+'/contact/wrench')))
        if len({s['topic'] for s in self.specs})!=len(self.specs):
            raise ValueError('each contact requires an individual wrench topic')

    def update(self,index,msg,received):
        self.latest[index]=(msg,received)

    def values(self,now,wall):
        contacts=[]
        for i,spec in enumerate(self.specs):
            if i not in self.latest:raise ValueError('contact wrench unavailable')
            msg,received=self.latest[i]
            stamp=msg.header.stamp.to_sec()
            if not stamp>0 or not 0<=now-stamp<=.25 or not 0<=wall-received<=.25:
                raise ValueError('contact wrench stale or future')
            frame=msg.header.frame_id.lstrip('/')
            local=frame in (spec['link'],self.robot+'/'+spec['link'])
            if not local and frame not in ('fc',self.robot+'/fc'):raise ValueError('contact frame mismatch')
            f,m=msg.wrench.force,msg.wrench.torque
            wrench=vector([f.x,f.y,f.z,m.x,m.y,m.z],6).tolist()
            contacts.append((spec['link'],spec['point'],wrench,local))
        return contacts
