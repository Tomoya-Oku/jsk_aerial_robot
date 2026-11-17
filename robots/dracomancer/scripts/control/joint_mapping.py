#!/usr/bin/env python3
"""Single/bimanual joint-angle mappings from thesis chapter 4."""
import numpy as np
from shape_mapping import vector
from contact_math import finite_jacobian

HUMAN_COORDINATES = ('shoulder_abd', 'shoulder_flex', 'shoulder_rot',
                     'elbow_flex', 'forearm_pron', 'wrist_flex', 'wrist_dev')


class HumanCoordinates:
    """Explicit CAD-to-anatomical calibration; bimanual order is left, right."""
    def __init__(self, model, config, bimanual=False):
        self.sides = ('left', 'right') if bimanual else ('right',)
        self.names, self.zero, self.scale, self.neutral = [], [], [], []
        for side in self.sides:
            arm = config.get(side, {})
            names = arm.get('names', [f'{side}_joint_{i}' for i in range(1, 8)])
            if len(names) != 7 or len(set(names)) != 7 or any(n not in model.joints for n in names):
                raise ValueError('seven distinct CAD joints required per arm')
            scale = vector(arm.get('scale', [1.]*7), 7)
            if np.any(scale == 0):
                raise ValueError('anatomical coordinate scales must be nonzero')
            self.names.extend(names)
            self.zero.extend(vector(arm.get('zero', [0.]*7), 7))
            self.scale.extend(scale)
            self.neutral.extend(vector(arm.get('neutral', [0.]*7), 7))
        self.zero, self.scale, self.neutral = map(np.asarray, (self.zero, self.scale, self.neutral))
        self.required = set(self.names)
        if len(self.required) != len(self.names):
            raise ValueError('left and right coordinate names must be distinct')

    def values(self, positions):
        return self.scale * (vector([positions[n] for n in self.names], len(self.names)) - self.zero)


class JointMapper:
    """eq:legacy-wrist/elbow/shoulder and eq:bimanual-direct-map."""
    is_shape = False

    def __init__(self, model, dragon, config, rest, bimanual=False):
        cfg = config.get('joint_mapping', {})
        self.human = HumanCoordinates(model, cfg, bimanual)
        self.dragon, self.bimanual = dragon, bimanual
        self.rest = vector(config.get('rest', rest), 6)
        self.budget = float(config.get('time_budget', .02))
        self.single_gains = vector(cfg.get('single_gains', [1.]*5), 5)
        self.gains = {side: {part: vector(cfg.get(side, {}).get(part, [1., 1.]), 2)
                            for part in ('elbow_gain', 'shoulder_gain')} for side in ('left', 'right')}
        if not np.isfinite(self.budget) or self.budget <= 0:
            raise ValueError('mapping time budget must be positive')
        if np.any(self.rest < dragon.lower) or np.any(self.rest > dragon.upper):
            raise ValueError('rest outside robot limits')

    @staticmethod
    def elbow(angle, shoulder_rotation):
        alpha = np.clip(shoulder_rotation, -np.pi/2, np.pi/2)
        return angle * np.array([alpha/(np.pi/2), 1.-abs(alpha)/(np.pi/2)])

    def reference(self, q):
        q = vector(q, len(self.human.names))
        if not self.bimanual:
            abd, flex, rot, elbow, pron, wrist_flex, wrist_dev = q
            wf, wd, ef, sf, sa = self.single_gains
            alpha = np.clip(rot, -np.pi/2, np.pi/2) + np.clip(pron, -np.pi/2, np.pi/2)
            wrist = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]]) @ [wf*wrist_flex, wd*wrist_dev]
            candidate = np.r_[wrist, self.elbow(ef*elbow, rot), sf*flex, sa*abd]
        else:
            left, right = q[:7], q[7:]
            dl, dr = left-self.human.neutral[:7], right-self.human.neutral[7:]
            candidate = self.rest.copy()
            candidate[:2] += self.gains['right']['elbow_gain'] * self.elbow(dr[3], right[2])
            candidate[4:] += self.gains['left']['elbow_gain'] * self.elbow(dl[3], left[2])
            candidate[2:4] += (self.gains['right']['shoulder_gain'] * dr[[1, 0]]
                               - self.gains['left']['shoulder_gain'] * dl[[1, 0]])
        return np.clip(candidate, self.dragon.lower, self.dragon.upper)

    def human_shape(self, values):
        return self.human.values(values)

    def solve(self, target, previous):
        return dict(q=self.reference(target), ok=True, status='success', iterations=0)

    def mapping_jacobian(self, values, previous, nominal=None):
        # Differentiate the saturated map in calibrated anatomical coordinates.
        return finite_jacobian(self.reference, self.human.values(values))
