#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2

class motion_detection():
    def __init__(self):
        self.device_pos_sub = rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.device_pos_cb)
        # self.trigger_sub = rospy.Subscriber('/twin_hammer/trigger', UInt8, self.trigger_cb)
        self.trigger_event_sub = rospy.Subscriber('/twin_hammer/trigger_event', UInt8, self.trigger_event_cb)

        self.detectionMode = False
        self.posForDetection = []

    def device_pos_cb(self, msg):
        if self.detectionMode:
            pos = msg.pose.position
            # print(f"Device pos: x={pos.x}, y={pos.y}, z={pos.z}")
            self.posForDetection.append([pos.y, pos.z]) # とりあえずy-zで

      # トリガーのイベント (EV_PUSH, EV_DOUBLE, EV_TRIPLE, EV_LONG)
    def trigger_event_cb(self, msg):
        # EV_PUSH: 単押し -> ON/OFF切り替え
        if msg.data == 0:
            self.detectionMode = not self.detectionMode
            print(f"Detection mode: {'ON' if self.detectionMode else 'OFF'}")

            if self.detectionMode:
                print(f"Detection started.")

            # OFFになったときに図形判定・位置情報クリア
            else:
                pts = np.asarray(self.posForDetection, dtype=np.float32)
                result = self.classify_shape(pts)
                print(f"Detected shape: {result}")
                self.posForDetection = []  # クリア

    def polygon_signed_area(poly_yz: np.ndarray) -> float:
        """poly: (M,2) CCW>0, CW<0  （y→x, z→y とみなす）"""
        y = poly_yz[:,0]; z = poly_yz[:,1]
        return 0.5 * np.sum(y*np.roll(z,-1) - z*np.roll(y,-1))

    def classify_shape(self, points: np.ndarray) -> str:
        pts = np.asarray(points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 3:
            return "Unknown"

        v = np.diff(pts, axis=0)
        d = pts[-1] - pts[0]                       # (dy, dz)
        cross = v[:-1,0]*v[1:,1] - v[:-1,1]*v[1:,0]
        rot_sign = np.sign(np.sum(cross))          # -1:右回り, +1:左回り

        # 直線性
        c = pts - pts.mean(axis=0)
        cov = np.cov(c.T)
        eigval, _ = np.linalg.eigh(cov)
        line_ratio = eigval[0] / max(eigval[1], 1e-12)

        # 形状量
        cnt = pts.reshape(-1,1,2)
        hull = cv2.convexHull(cnt)
        peri = cv2.arcLength(hull, True)
        area = abs(cv2.contourArea(hull))
        circularity = 4.0*np.pi*area/(peri*peri + 1e-12)

        # 1) 直線
        LINE_THINNESS = 0.02
        ANGLE_TOL = 15*np.pi/180
        if line_ratio < LINE_THINNESS:
            ang = abs(np.arctan2(d[1], d[0]))
            if ang <= ANGLE_TOL:
                return "Horizontal Line (Left to Right)" if d[0] > 0 else "Horizontal Line (Right to Left)"
            if abs(ang - np.pi/2) <= ANGLE_TOL:
                return "Vertical Line (Bottom to Top)" if d[1] > 0 else "Vertical Line (Top to Bottom)"
            return "Unknown"

        # 2) 円
        if circularity > 0.80 and len(pts) >= 6:
            Y, Z = pts[:,0], pts[:,1]
            A = np.c_[2*Y, 2*Z, np.ones_like(Y)]
            b = Y**2 + Z**2
            try:
                cy, cz, c0 = np.linalg.lstsq(A, b, rcond=None)[0]
                r = np.sqrt(max(c0 + cy**2 + cz**2, 1e-12))
                radial = np.sqrt((Y-cy)**2 + (Z-cz)**2)
                if r > 1e-6 and np.std(radial)/r < 0.15:
                    if rot_sign < 0:  return "Circle (Clockwise)"
                    if rot_sign > 0:  return "Circle (Counter-Clockwise)"
            except np.linalg.LinAlgError:
                pass

        # 3) 多角形（3 or 4）
        eps = 0.02 * peri
        approx = cv2.approxPolyDP(hull, eps, True)
        K = len(approx)

        def by_rot(base):
            if rot_sign < 0:  return f"{base} (Clockwise)"
            if rot_sign > 0:  return f"{base} (Counter-Clockwise)"
            s = self.polygon_signed_area(hull[:,0,:])
            return f"{base} (Counter-Clockwise)" if s > 0 else f"{base} (Clockwise)"

        if K == 3:
            tri = approx[:,0,:].astype(np.float32)
            z = tri[:,1]
            i_top = int(np.argmax(z))
            i_bottom = int(np.argmin(z))
            base = "Triangle" if (i_top != i_bottom and z[i_top]-np.median(z) > np.median(z)-z[i_bottom]) else "Inverted Triangle"
            return by_rot(base)

        if K == 4:
            return by_rot("Rectangle")

        return "Unknown"
        
    def main(self):
        r = rospy.Rate(40)
        rospy.spin()

if __name__ == "__main__":
  rospy.init_node("motion_detection")
  Tracker = motion_detection()
  Tracker.main()