import rospy
import numpy as np
import math
import cv2
from enum import Enum, auto
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3Stamped
from integration import X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX

class Shape(Enum):
    UNKNOWN = auto()
    LINE_HORIZONTAL_LEFT_TO_RIGHT = auto()
    LINE_HORIZONTAL_RIGHT_TO_LEFT = auto()
    LINE_VERTICAL_BOTTOM_TO_TOP = auto()
    LINE_VERTICAL_TOP_TO_BOTTOM = auto()
    LINE_DIAGONAL = auto()
    CIRCLE_CLOCKWISE = auto()
    CIRCLE_COUNTER_CLOCKWISE = auto()
    TRIANGLE_CLOCKWISE = auto()
    TRIANGLE_COUNTER_CLOCKWISE = auto()
    TRIANGLE_INVERTED_CLOCKWISE = auto()
    TRIANGLE_INVERTED_COUNTER_CLOCKWISE = auto()
    RECTANGLE_CLOCKWISE = auto()
    RECTANGLE_COUNTER_CLOCKWISE = auto()

class Trajectory_Based_Gesture_Recognition():

    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "gimbalrotor")

        # Publisher
        self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
        self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)

        # Messages
        self.flight_nav = FlightNav()
        self.flight_nav.target = FlightNav.COG
        self.target_att_nav = Vector3Stamped()

        self.trajectory = []

    def circleTask(self, radius=1.0, period=8.0, hz=40):
        """
        現在位置(center)を中心に、zは据え置きで半径radiusの円を1周する。
        period: 1周にかける秒数、hz: 制御周期
        """
        # 中心と高度・初期yawを固定
        cx, cy, cz = self.robot_pos[0], self.robot_pos[1], self.robot_pos[2]
        yaw0 = self.robot_att[2]

        rate = rospy.Rate(hz)
        total = max(1, int(period * hz))

        for k in range(total):
            theta = 2.0 * math.pi * (float(k) / float(total))
            target_x = cx + radius * math.cos(theta)
            target_y = cy + radius * math.sin(theta)
            target_z = cz  # 高度据え置き

            # 0066環境の安全範囲にクリップ（ファイルの下限上限に合わせる）
            target_x = max(min(target_x, X_MAX), X_MIN)
            target_y = max(min(target_y, Y_MAX), Y_MIN)
            target_z = max(min(target_z, Z_MAX), Z_MIN)

            # 目標セット（姿勢は水平・yaw据え置き）
            self.flight_nav.target_pos_x = target_x
            self.flight_nav.target_pos_y = target_y
            self.flight_nav.target_pos_z = target_z
            self.flight_nav.target_roll  = 0.0
            self.flight_nav.target_pitch = 0.0
            self.flight_nav.target_yaw   = yaw0

            self.target_att_nav.vector.x = 0.0
            self.target_att_nav.vector.y = 0.0

            # 即時送信（メインループと重なっても同値なのでOK）
            self.nav_pub.publish(self.flight_nav)
            self.att_pub.publish(self.target_att_nav)

            rate.sleep()

    def lineTask(self):
        pass

class Trajectory_Based_Gesture_Recognition():

    def __init__(self):
        self.trajectory = []

    def saveTrajectory(self):
        pass

    def polygon_signed_area(self, poly_uv: np.ndarray) -> float:
        u, v = poly_uv[:, 0], poly_uv[:, 1]
        return 0.5 * np.sum(u * np.roll(v, -1) - v * np.roll(u, -1))

def classify_shape_2d(self, points: np.ndarray) -> Shape:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 3:
        return Shape.UNKNOWN

    # 回転符号（離散曲率和）
    v = np.diff(pts, axis=0)
    if len(v) >= 2:
        cross = v[:-1, 0] * v[1:, 1] - v[:-1, 1] * v[1:, 0]
        rot_sign = np.sign(np.sum(cross))  # -1: CW, +1: CCW
    else:
        rot_sign = 0
    d = pts[-1] - pts[0]  # 始終点差

    # PCAで直線性
    c = pts - pts.mean(axis=0)
    cov = np.cov(c.T)
    eigval, _ = np.linalg.eigh(cov)         # λ1≤λ2
    line_ratio = eigval[0] / max(eigval[1], 1e-12)

    # 形状量（凸包）
    cnt = pts.reshape(-1, 1, 2)
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    area = abs(cv2.contourArea(hull))
    circularity = 4.0 * np.pi * area / (peri * peri + 1e-12)

    # 1) 直線
    LINE_THINNESS = 0.02
    ANGLE_TOL = 15 * np.pi / 180
    if line_ratio < LINE_THINNESS:
        theta = np.arctan2(d[1], d[0])
        if abs(theta) <= ANGLE_TOL:
            return (Shape.LINE_HORIZONTAL_LEFT_TO_RIGHT
                    if d[0] > 0 else Shape.LINE_HORIZONTAL_RIGHT_TO_LEFT)
        if abs(abs(theta) - np.pi/2) <= ANGLE_TOL:
            return (Shape.LINE_VERTICAL_BOTTOM_TO_TOP
                    if d[1] > 0 else Shape.LINE_VERTICAL_TOP_TO_BOTTOM)
        # 水平/垂直でなければ斜め直線
        return Shape.LINE_DIAGONAL

    # 2) 円
    if circularity > 0.80 and len(pts) >= 6:
        U, V = pts[:, 0], pts[:, 1]
        A = np.c_[2 * U, 2 * V, np.ones_like(U)]
        b = U**2 + V**2
        try:
            cu, cv, c0 = np.linalg.lstsq(A, b, rcond=None)[0]
            r = np.sqrt(max(c0 + cu**2 + cv**2, 1e-12))
            radial = np.sqrt((U - cu)**2 + (V - cv)**2)
            if r > 1e-6 and np.std(radial) / r < 0.15:
                if rot_sign == 0:
                    s = self.polygon_signed_area(hull[:, 0, :])
                    rot_sign = np.sign(s)
                if rot_sign < 0:  return Shape.CIRCLE_CLOCKWISE
                if rot_sign > 0:  return Shape.CIRCLE_COUNTER_CLOCKWISE
        except np.linalg.LinAlgError:
            pass

    # 3) 多角形（3 or 4）
    eps = 0.02 * peri
    approx = cv2.approxPolyDP(hull, eps, True)
    K = len(approx)

    if rot_sign == 0:
        s = self.polygon_signed_area(hull[:, 0, :])
        rot_sign = np.sign(s)

    if K == 3:
        # “正/逆( inverted )”の判定：v座標（上が+）で上頂点が優勢なら正、下が優勢なら逆
        tri = approx[:, 0, :].astype(np.float32)
        vcoords = tri[:, 1]
        i_top = int(np.argmax(vcoords))
        i_bottom = int(np.argmin(vcoords))
        v_med = np.median(vcoords)
        is_inverted = not (i_top != i_bottom and (vcoords[i_top] - v_med) > (v_med - vcoords[i_bottom]))
        if is_inverted:
            return (Shape.TRIANGLE_INVERTED_CLOCKWISE if rot_sign < 0
                    else Shape.TRIANGLE_INVERTED_COUNTER_CLOCKWISE)
        else:
            return (Shape.TRIANGLE_CLOCKWISE if rot_sign < 0
                    else Shape.TRIANGLE_COUNTER_CLOCKWISE)

    if K == 4:
        return Shape.RECTANGLE_CLOCKWISE if rot_sign < 0 else Shape.RECTANGLE_COUNTER_CLOCKWISE

    return Shape.UNKNOWN

# --------- 3D → 2Dラッパ（PCA基底） ---------

def fit_plane_pca(self, P: np.ndarray):
    assert P.ndim == 2 and P.shape[1] == 3 and len(P) >= 3
    o = P.mean(axis=0)
    X = P - o
    C = (X.T @ X) / len(P)
    evals, evecs = np.linalg.eigh(C)  # 昇順
    w = evecs[:, 0]; w /= (np.linalg.norm(w) + 1e-12)
    ref = np.array([1.0, 0.0, 0.0]) if abs(w[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = ref - (ref @ w) * w; u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(w, u); v /= (np.linalg.norm(v) + 1e-12)
    return o, u, v, w, evals

def project_to_plane(self, P: np.ndarray, o: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    X = P - o
    U = X @ u
    V = X @ v
    return np.c_[U, V].astype(np.float32)

def rotation_sign_3d(self, P: np.ndarray, w: np.ndarray) -> int:
    if len(P) < 3:
        return 0
    Q = np.roll(P, -1, axis=0)
    A = 0.5 * np.sum(np.cross(P, Q), axis=0)
    return int(np.sign(A @ w))

def planarity_and_line_checks(self, P: np.ndarray, evals: np.ndarray, planarity_tau=1e-3, line_tau=1e-3):
    mu = P.mean(axis=0)
    rms = np.sqrt(np.mean(np.sum((P - mu)**2, axis=1)))
    scale = max(rms, 1e-6)
    lam1, lam2, lam3 = evals
    if lam1 / (scale**2) > planarity_tau:
        return "nonplanar"
    if lam2 / (scale**2) < line_tau:
        return "line"
    return "ok"

def classify_shape_3d(self, angle_tol_rad=np.deg2rad(15)) -> Shape:
    P = np.asarray(self.trajectory, dtype=np.float64)
    if P.ndim != 2 or P.shape[1] != 3 or len(P) < 3:
        return Shape.UNKNOWN

    o, u, v, w, evals = self.fit_plane_pca(P)
    status = self.planarity_and_line_checks(P, evals)
    if status == "nonplanar":
        return Shape.UNKNOWN

    UV = self.project_to_plane(P, o, u, v)

    if status == "line":
        d = UV[-1] - UV[0]
        theta = np.arctan2(d[1], d[0])
        if abs(theta) <= angle_tol_rad:
            return Shape.LINE_HORIZONTAL_LEFT_TO_RIGHT if d[0] > 0 else Shape.LINE_HORIZONTAL_RIGHT_TO_LEFT
        if abs(abs(theta) - np.pi/2) <= angle_tol_rad:
            return Shape.LINE_VERTICAL_BOTTOM_TO_TOP if d[1] > 0 else Shape.LINE_VERTICAL_TOP_TO_BOTTOM
        return Shape.LINE_DIAGONAL  # ← 斜め直線を返す

    # 2D分類 → 3Dの回転符号で最終上書き
    shape2d = self.classify_shape_2d(UV)
    rot3d = self.rotation_sign_3d(P, w)
    return self.apply_rot_override(shape2d, rot3d)

if __name__ == "__main__":
  rospy.init_node("gesture")
  Tracker = Trajectory_Based_Gesture_Recognition()
  Tracker.main()
