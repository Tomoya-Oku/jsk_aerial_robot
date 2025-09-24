import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2

class motion_detection():
    def __init__(self):
        self.device_pos_sub = rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.device_pos_cb)
        self.detectionMode = False
        self.posForDetection = []

    def device_pos_cb(self, msg):
        pos = msg.pose.position

        if self.detectionMode:
            print(f"Device pos: x={pos.x}, y={pos.y}, z={pos.z}")

      # トリガーのイベント (EV_PUSH, EV_DOUBLE, EV_TRIPLE, EV_LONG)
    def trigger_event_cb(self, msg):
        # EV_PUSH: 単押し -> ON/OFF切り替え
        if msg.data == 0:
            self.detectionMode = not self.detectionMode
            print(f"Detection mode: {'ON' if self.detectionMode else 'OFF'}")

    def classify_shape(points_xy: np.ndarray) -> str:
        """
        points_xy: shape (N,2) のfloat/int配列。描かれた順でなくてもOK（凸包で外形化）
        返り値: 'triangle' | 'rectangle' | 'square' | 'circle' | 'polygon' | 'unknown'
        """
        pts = points_xy.astype(np.float32)
        if len(pts) < 3:
            return 'unknown'

        # OpenCVの輪郭形式（N,1,2）に
        cnt = pts.reshape(-1,1,2)
        # 凸包で外形（ノイズ点があっても外形を拾う）
        hull = cv2.convexHull(cnt)

        # 多角形近似
        peri = cv2.arcLength(hull, True)
        eps = 0.02 * peri  # 近似許容（データ次第で調整）
        approx = cv2.approxPolyDP(hull, eps, True)
        v = len(approx)

        # 円らしさ（円なら 4πA / P^2 ≈ 1）
        area = abs(cv2.contourArea(hull))
        circularity = 4.0 * np.pi * area / (peri * peri + 1e-12)

        # 円判定（しきい値は経験的に0.85前後、データ次第で調整）
        if circularity > 0.85:
            return 'circle'

        if v == 3:
            return 'triangle'
        elif v == 4:
            # 矩形/正方形判定：最小外接矩形の縦横比でざっくり
            rect = cv2.minAreaRect(hull)
            (w, h) = rect[1]
            if w == 0 or h == 0:
                return 'rectangle'
            ratio = max(w, h) / min(w, h)
            return 'square' if ratio < 1.1 else 'rectangle'
        elif v >= 5:
            return 'polygon'
        else:
            return 'unknown'


