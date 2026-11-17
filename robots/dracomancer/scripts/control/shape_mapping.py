#!/usr/bin/env python3
"""URDF centerlines and bounded shape retargeting (no ROS dependency).

All FK is in a fixed mounting frame. Alignment is a calibration constant;
in particular it never cancels a moving shoulder rotation.
"""
import time
import xml.etree.ElementTree as ET

import numpy as np

DRAGON_JOINTS = ["joint%d_%s" % (i, axis)
                 for i in range(1, 4) for axis in ("pitch", "yaw")]


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError("expected %d finite values" % size)
    return result


def rotation(axis, angle):
    axis = vector(axis, 3)
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or not np.isfinite(angle):
        raise ValueError("invalid axis or angle")
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew.dot(skew)


def rpy_matrix(rpy):
    r, p, y = vector(rpy, 3)
    return rotation([0, 0, 1], y).dot(rotation([0, 1, 0], p)).dot(rotation([1, 0, 0], r))


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = rpy_matrix(rpy)
    result[:3, 3] = vector(xyz, 3)
    return result


class UrdfModel:
    """Small strict FK for fixed/revolute/continuous URDF trees."""

    def __init__(self, xml):
        self.xml = xml
        root = ET.fromstring(xml)
        self.links = {link.get("name") for link in root.findall("link")}
        self.joints = {}
        self.parents = {}
        for node in root.findall("joint"):
            name, kind = node.get("name"), node.get("type")
            origin, axis, limit = node.find("origin"), node.find("axis"), node.find("limit")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0]*3
            joint = dict(name=name, kind=kind, parent=node.find("parent").get("link"),
                         child=node.find("child").get("link"), origin=transform(xyz, rpy),
                         axis=vector([float(v) for v in axis.get("xyz").split()], 3)
                         if axis is not None else np.array([1., 0., 0.]), limit=limit)
            axis_norm = np.linalg.norm(joint["axis"])
            if axis_norm < 1e-12:
                raise ValueError("zero URDF joint axis")
            x, y, z = joint["axis"] / axis_norm
            joint["skew"] = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            joint["skew2"] = joint["skew"].dot(joint["skew"])
            self.joints[name] = joint
            self.parents[joint["child"]] = joint
        roots = self.links - self.parents.keys()
        if len(roots) != 1:
            raise ValueError("one URDF root required")
        self.root = roots.pop()

    def chain(self, base, tip):
        chain, seen = [], set()
        while tip != base:
            if tip in seen or tip not in self.parents:
                raise ValueError("no URDF chain from %s to %s" % (base, tip))
            seen.add(tip)
            joint = self.parents[tip]
            chain.append(joint)
            tip = joint["parent"]
        return chain[::-1]

    def pose(self, base, tip, values, cache=None, enforce_limits=True):
        cache = {} if cache is None else cache
        result = np.eye(4)
        for joint in self.chain(base, tip):
            if joint["child"] in cache:
                result = cache[joint["child"]]
                continue
            motion = np.eye(4)
            if joint["kind"] != "fixed":
                if joint["kind"] not in ("revolute", "continuous"):
                    raise ValueError("unsupported URDF joint type")
                angle = float(values[joint["name"]])  # missing names are errors
                if not np.isfinite(angle):
                    raise ValueError("non-finite joint angle")
                motion[:3, :3] = joint_rotation(joint, angle)
                limit = joint["limit"]
                if enforce_limits and joint["kind"] == "revolute" and limit is not None:
                    if not float(limit.get("lower")) - 1e-8 <= angle <= float(limit.get("upper")) + 1e-8:
                        raise ValueError("input joint outside URDF limits")
            result = result.dot(joint["origin"]).dot(motion)
            cache[joint["child"]] = result
        return result

    def limits(self, names):
        lower, upper, velocity = [], [], []
        for name in names:
            joint = self.joints[name]
            limit = joint["limit"]
            if joint["kind"] != "revolute" or limit is None:
                raise ValueError("bounded revolute joint required: " + name)
            lower.append(float(limit.get("lower")))
            upper.append(float(limit.get("upper")))
            velocity.append(float(limit.get("velocity")))
        lower, upper, velocity = (vector(v, len(names)) for v in (lower, upper, velocity))
        if np.any(lower >= upper) or np.any(velocity <= 0):
            raise ValueError("invalid URDF joint limits")
        return lower, upper, velocity


def joint_rotation(joint, angle):
    # The axis skew matrices are constant URDF geometry, reused by all FD calls.
    return np.eye(3) + np.sin(angle)*joint["skew"] + (1-np.cos(angle))*joint["skew2"]


def centerline(points, samples=20, alignment=(0, 0, 0), origin=None):
    """Equal arc-length samples, translated and divided by total polyline length."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.all(np.isfinite(points)):
        raise ValueError("invalid centerline points")
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(lengths <= 1e-10) or int(samples) != samples or samples < 2:
        raise ValueError("zero-length segment or invalid sample count")
    arc = np.r_[0., np.cumsum(lengths)]
    targets = np.linspace(0., arc[-1], samples)
    sampled = np.column_stack([np.interp(targets, arc, points[:, i]) for i in range(3)])
    align = np.asarray(alignment)
    align = align if align.shape == (3, 3) else rpy_matrix(alignment)
    reference = points[0] if origin is None else vector(origin, 3)
    return ((sampled - reference) / arc[-1]).dot(align.T)


class HumanChain:
    def __init__(self, model, config):
        self.model = model
        self.base = config["human_base"]
        self.nodes = config["human_points"]
        self.torso = vector(config.get("torso_point", []), 3)
        self.required = set()
        if len(self.nodes) != 4:
            raise ValueError("shoulder/elbow/wrist/hand points required")
        for node in self.nodes:
            vector(node["xyz"], 3)
            self.required.update(j["name"] for j in model.chain(self.base, node["link"])
                                 if j["kind"] != "fixed")
        if len(self.required) != 7:
            raise ValueError("human centerline must use seven URDF joints")
        self.names = [j['name'] for j in model.chain(self.base, self.nodes[-1]['link']) if j['kind'] != 'fixed']

    def points(self, values):
        points = [self.torso]
        cache = {}
        for node in self.nodes:
            pose = self.model.pose(self.base, node["link"], values, cache)
            points.append(pose[:3, :3].dot(vector(node["xyz"], 3)) + pose[:3, 3])
        return np.asarray(points)


class DragonChain:
    """Physical polyline including pitch/yaw spacers, rooted at link4 tail.

    The old shape_task FK inverts the link1->link4 *tip* transform but skips
    spacer corners in its point list. Here every straight segment comes from
    URDF and the reversed traversal keeps the true pitch/yaw order and signs.
    """
    def __init__(self, model, tail):
        self.model = model
        self.tail = vector(tail, 3)
        self.chain = model.chain("link1", "link4")
        if [j["name"] for j in self.chain if j["kind"] != "fixed"] != DRAGON_JOINTS:
            raise ValueError("unexpected DRAGON joint order")
        self.lower, self.upper, self.velocity = model.limits(DRAGON_JOINTS)

    def points_mid(self, q):
        """Tail-to-head polyline, in fixed FC axes about joint2_pitch."""
        values = dict(zip(DRAGON_JOINTS, vector(q, 6)))
        root_fc = self.model.pose(self.model.root, 'fc', values)
        root_tail = self.model.pose(self.model.root, 'link4', values)
        root_tail[:3, 3] += root_tail[:3, :3] @ self.tail
        fc_tail = np.linalg.inv(root_fc) @ root_tail
        fc_mid = np.linalg.inv(root_fc) @ self.model.pose(self.model.root, 'inter_joint2', values)
        return self.points(q) @ fc_tail[:3, :3].T + fc_tail[:3, 3] - fc_mid[:3, 3]

    def points(self, q):
        q = vector(q, 6)
        pose, points = np.eye(4), [np.zeros(3)]
        for joint in self.chain:
            pose = pose.dot(joint["origin"])
            if np.linalg.norm(pose[:3, 3] - points[-1]) > 1e-10:
                points.append(pose[:3, 3].copy())
            if joint["kind"] != "fixed":
                motion = np.eye(4)
                motion[:3, :3] = joint_rotation(joint, q[DRAGON_JOINTS.index(joint["name"])])
                pose = pose.dot(motion)
        root = pose.copy()
        root[:3, 3] += pose[:3, :3].dot(self.tail)
        points.append(root[:3, 3].copy())
        return (np.asarray(points[::-1]) - root[:3, 3]).dot(root[:3, :3])

    def point_batch(self, angles):
        """Evaluate the six finite-difference chains together to bound Python work."""
        angles = np.asarray(angles, dtype=float)
        if angles.ndim != 2 or angles.shape[1] != 6 or not np.all(np.isfinite(angles)):
            raise ValueError("invalid joint angle batch")
        count = len(angles)
        pose = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
        points = [np.zeros((count, 3))]
        for joint in self.chain:
            pose = pose @ joint["origin"]
            if np.linalg.norm(joint["origin"][:3, 3]) > 1e-10:
                points.append(pose[:, :3, 3].copy())
            if joint["kind"] != "fixed":
                angle = angles[:, DRAGON_JOINTS.index(joint["name"])]
                motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
                motion[:, :3, :3] = (np.eye(3) + np.sin(angle)[:, None, None]*joint["skew"]
                                     + (1-np.cos(angle))[:, None, None]*joint["skew2"])
                pose = pose @ motion
        root = pose[:, :3, 3] + np.einsum("nij,j->ni", pose[:, :3, :3], self.tail)
        points.append(root)
        points = np.stack(points[::-1], axis=1)-root[:, None, :]
        return np.einsum("nki,nij->nkj", points, pose[:, :3, :3])


def limit_step(current, target, velocity, dt, max_step):
    """Limit each axis using elapsed time; no accumulated catch-up motion."""
    current, target, velocity = (vector(v, 6) for v in (current, target, velocity))
    if not np.isfinite(dt) or dt < 0 or not np.isfinite(max_step) or max_step <= 0 or np.any(velocity <= 0):
        raise ValueError("invalid rate limit")
    bound = np.minimum(velocity * dt, max_step)
    return current + np.clip(target - current, -bound, bound)


class ShapeMapper:
    is_shape = True
    def __init__(self, human, dragon, config, rest):
        self.human, self.dragon, self.config = human, dragon, config
        self.samples = int(config.get("samples", 20))
        self.weights = vector(config.get("weights", [1.] * self.samples), self.samples)
        if np.any(self.weights < 0) or self.weights.sum() <= 0:
            raise ValueError("invalid shape weights")
        self.sqrt_w = np.sqrt(np.repeat(self.weights / self.weights.sum(), 3))
        self.rest = vector(config.get("rest", rest), 6)
        self.human_alignment = rpy_matrix(config.get("human_alignment", [0, 0, 0]))
        self.dragon_alignment = rpy_matrix(config.get("dragon_alignment", [0, 0, 0]))
        self.lam, self.rho = float(config.get("continuity", 0.002)), float(config.get("rest_weight", 0.0002))
        self.iterations = int(config.get("max_iterations", 4))
        self.budget = float(config.get("time_budget", 0.020))
        self.epsilon = float(config.get("difference_step", 1e-5))
        self.damping = float(config.get("damping", 1e-5))
        self.tolerance = float(config.get("tolerance", 1e-6))
        if (not np.all(np.isfinite([self.lam, self.rho, self.budget, self.epsilon, self.damping, self.tolerance]))
                or min(self.lam, self.rho) < 0 or min(self.budget, self.epsilon, self.damping, self.tolerance) <= 0
                or self.iterations < 1 or self.samples < 2):
            raise ValueError("invalid solver configuration")
        if np.any(self.rest < dragon.lower) or np.any(self.rest > dragon.upper):
            raise ValueError("rest pose outside DRAGON limits")

    def dragon_shape(self, q):
        if isinstance(self.human, BimanualHumanChain):
            return centerline(self.dragon.points_mid(q), self.samples, self.dragon_alignment, origin=[0, 0, 0])
        return centerline(self.dragon.points(q), self.samples,
                          self.dragon_alignment)

    def human_shape(self, values):
        return centerline(self.human.points(values), self.samples,
                          self.human_alignment,
                          self.human.body(values) if isinstance(self.human, BimanualHumanChain) else None)

    def solve(self, target, previous, initial=None):
        start = time.perf_counter()
        previous = vector(previous, 6)
        q = previous.copy() if initial is None else vector(initial, 6).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (self.samples, 3) or not np.all(np.isfinite(target)):
            raise ValueError("invalid target shape")
        if np.any(q < self.dragon.lower - 1e-8) or np.any(q > self.dragon.upper + 1e-8):
            raise ValueError("previous command outside DRAGON limits")
        q = np.clip(q, self.dragon.lower, self.dragon.upper)

        def residual(q):
            return (self.dragon_shape(q) - target).ravel() * self.sqrt_w

        def cost(q, residual):
            return residual.dot(residual) + self.lam * np.sum((q - previous)**2) + self.rho * np.sum((q - self.rest)**2)

        residual_q = residual(q)
        n = 0
        for n in range(1, self.iterations + 1):
            if time.perf_counter() - start > self.budget:
                break
            steps = np.where(q+self.epsilon <= self.dragon.upper, self.epsilon, -self.epsilon)
            perturbed = np.tile(q, (6, 1))
            perturbed[np.arange(6), np.arange(6)] += steps
            shapes = (np.asarray([self.dragon_shape(v) for v in perturbed])
                      if isinstance(self.human, BimanualHumanChain) else
                      np.asarray([centerline(points, self.samples, self.dragon_alignment)
                                  for points in self.dragon.point_batch(perturbed)]))
            residuals = (shapes-target).reshape(6, -1)*self.sqrt_w
            jac = ((residuals-residual_q)/steps[:, None]).T
            lhs = jac.T.dot(jac) + (self.lam + self.rho + self.damping) * np.eye(6)
            rhs = -jac.T.dot(residual_q) - self.lam*(q-previous) - self.rho*(q-self.rest)
            # An unconstrained GN step followed by clipping can stall at a bound:
            # outward yaw motion can dominate the other useful directions. Solve
            # again on the free axes when an active bound blocks that direction.
            free = np.ones(6, dtype=bool)
            step = np.zeros(6)
            for _ in range(7):
                step[:] = 0.
                if not np.any(free):
                    break
                step[free] = np.linalg.solve(lhs[np.ix_(free, free)], rhs[free])
                blocked = (((q >= self.dragon.upper-1e-10) & (step > 0)) |
                           ((q <= self.dragon.lower+1e-10) & (step < 0))) & free
                if not np.any(blocked):
                    break
                free[blocked] = False
            if not np.all(np.isfinite(step)):
                raise ValueError("non-finite solver step")
            if np.linalg.norm(step) < self.tolerance:
                break
            old_cost = cost(q, residual_q)
            for fraction in (1., .5, .25, .125, .0625, .03125, .015625, .0078125):
                candidate = np.clip(q + fraction*step, self.dragon.lower, self.dragon.upper)
                new_residual = residual(candidate)
                if cost(candidate, new_residual) < old_cost:
                    q, residual_q = candidate, new_residual
                    break
            else:
                break
        elapsed = time.perf_counter() - start
        ok = elapsed <= self.budget
        # An over-budget iterate is never forwarded, even if its cost improved.
        if not ok:
            q = previous.copy()
            residual_q = residual(q)
        return dict(q=q, rmse=float(np.linalg.norm(residual_q)), seconds=elapsed,
                    iterations=n, ok=ok, status="success" if ok else "timeout")

    def mapping_jacobian(self, values, previous, nominal=None):
        """Freeze history/rest and warm-start both perturbations at one branch."""
        from contact_math import finite_jacobian
        names = self.human.names
        if nominal is None:
            result = self.solve(self.human_shape(values), previous)
            if not result['ok']:
                raise ValueError('nominal shape mapping unavailable')
            nominal = result['q']
        def reference(q):
            target = self.human_shape(dict(values, **dict(zip(names, q))))
            result = self.solve(target, previous, initial=nominal)
            if not result['ok']:
                raise ValueError('perturbed shape mapping unavailable')
            return result['q']
        return finite_jacobian(reference, [values[n] for n in names], self.epsilon)


class BimanualHumanChain:
    """eq:bimanual-centerline: left hand -> shoulders -> right hand."""
    def __init__(self, model, config):
        right_config = dict(config, torso_point=[0, 0, 0])
        left_config = dict(right_config, human_points=config['left_human_points'])
        self.left, self.right = HumanChain(model, left_config), HumanChain(model, right_config)
        self.base = self.right.base
        self.required = self.left.required | self.right.required
        self.names = [f'{side}_joint_{i}' for side in ('left', 'right') for i in range(1, 8)]
        if len(self.required) != 14 or set(self.names) != self.required:
            raise ValueError('fourteen CAD arm joints required')

    def points(self, values):
        return np.r_[self.left.points(values)[1:][::-1], self.right.points(values)[1:]]

    def body(self, values):
        points = self.points(values)
        return .5*(points[3]+points[4])
