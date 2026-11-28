#!/usr/bin/env python3
"""View/tilt input plane and SO(3) target integration (thesis chapter 4)."""
import numpy as np
from .shape import vector, rotation, rpy_matrix


def quaternion_matrix(xyzw):
    q=vector(xyzw,4)
    if np.linalg.norm(q)<1e-9: raise ValueError('invalid orientation')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def matrix_rpy(r):
    return np.array([np.arctan2(r[2,1],r[2,2]),
                     np.arctan2(-r[2,0],np.hypot(r[0,0],r[1,0])),
                     np.arctan2(r[1,0],r[0,0])])


class ViewControl:
    def __init__(self,operator_position,linear=(.3,.3),angular=(.3,.3),deadzone=.05,maximum=(None,None)):
        self.operator=vector(operator_position,3)
        self.linear,self.angular=vector(linear,2),vector(angular,2)
        if np.any(self.linear<=0) or np.any(self.angular<=0):raise ValueError('positive speed scales required')
        self.maximum=np.array([self.linear.max() if maximum[0] is None else maximum[0],
                               self.angular.max() if maximum[1] is None else maximum[1]],dtype=float)
        if not np.all(np.isfinite(self.maximum)) or np.any(self.maximum<=0):raise ValueError('positive speed limits required')
        self.deadzone=float(deadzone)
        if not np.isfinite(self.deadzone) or not 0<=self.deadzone<1:raise ValueError('invalid neutral zone')
        self.previous_right=None
        self.rotation_target=None
        self.rotate_mode=self.pressed=self.pending=False

    def basis(self,position,tilt):
        z=self.operator-vector(position,3)
        if np.linalg.norm(z)<1e-6:raise ValueError('operator and robot positions coincide')
        z/=np.linalg.norm(z)
        right=np.cross([0.,0.,1.],z)
        if np.linalg.norm(right)<1e-3:
            seed=self.previous_right if self.previous_right is not None else np.array([1.,0.,0.])
            right=seed-z*np.dot(z,seed)
        if np.linalg.norm(right)<1e-9:raise ValueError('view basis unavailable')
        right/=np.linalg.norm(right)
        if self.previous_right is not None and np.dot(right,self.previous_right)<0:
            right=-right
        self.previous_right=right.copy()
        axes=np.column_stack([right,np.cross(z,right),z])
        tilt=np.asarray(tilt,dtype=float)
        if (tilt.shape!=(3,3) or not np.all(np.isfinite(tilt))
                or not np.allclose(tilt.T@tilt,np.eye(3),atol=1e-7) or np.linalg.det(tilt)<0):
            raise ValueError('invalid tilt rotation')
        return (axes@tilt)[:,:2]

    def button(self,pressed):
        if pressed and not self.pressed:self.pending=True
        self.pressed=bool(pressed)

    def commands(self,position,tilt,right,left=None):
        right=np.clip(vector(right,2),-1.,1.)
        left=None if left is None else np.clip(vector(left,2),-1.,1.)
        if self.pending:
            if np.max(np.abs(right))<=self.deadzone:
                self.rotate_mode=not self.rotate_mode
                self.pending=False
            else:return np.zeros(3),np.zeros(3)
        right[np.abs(right)<self.deadzone]=0.
        if left is not None:left[np.abs(left)<self.deadzone]=0.
        basis=self.basis(position,tilt)
        translation=right if left is not None or not self.rotate_mode else np.zeros(2)
        angular=left if left is not None else (right if self.rotate_mode else np.zeros(2))
        v=basis@(self.linear*translation)
        turn=self.angular*angular
        omega=basis@np.array([-turn[1],turn[0]])
        for value,limit in zip((v,omega),self.maximum):
            norm=np.linalg.norm(value)
            if norm>limit:value*=limit/norm
        return v,omega

    def integrate_rotation(self,measured,omega,dt,limits):
        if self.rotation_target is None:self.rotation_target=np.asarray(measured,dtype=float).copy()
        omega=vector(omega,3)
        if not np.isfinite(dt) or dt<0:raise ValueError('invalid integration time')
        angle=np.linalg.norm(omega)*dt
        if angle>0:self.rotation_target=rotation(omega,angle)@self.rotation_target
        # Integrate as a rotation; Euler components are used only at the
        # command/limit boundary, never as angular-velocity integrators.
        angles=matrix_rpy(self.rotation_target)
        low,high=np.asarray(limits,dtype=float).T
        if low.shape!=(3,) or not np.all(np.isfinite([low,high])) or np.any(low>=high):
            raise ValueError('invalid attitude bounds')
        angles=np.clip(angles,low,high)
        self.rotation_target=rpy_matrix(angles)
        return angles
