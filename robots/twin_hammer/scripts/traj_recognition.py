import numpy as np
import cv2
from enum import Enum, auto

class Shape(Enum):
    UNKNOWN = auto()
    LINE_HORIZONTAL_LEFT_TO_RIGHT = auto()
    LINE_HORIZONTAL_RIGHT_TO_LEFT = auto()
    LINE_VERTICAL_BOTTOM_TO_TOP = auto()
    LINE_VERTICAL_TOP_TO_BOTTOM = auto()
    LINE_DIAGONAL = auto()
    CIRCLE = auto()
    CIRCLE_CLOCKWISE = auto()
    CIRCLE_COUNTER_CLOCKWISE = auto()
    TRIANGLE = auto()
    TRIANGLE_CLOCKWISE = auto()
    TRIANGLE_COUNTER_CLOCKWISE = auto()
    TRIANGLE_INVERTED_CLOCKWISE = auto()
    TRIANGLE_INVERTED_COUNTER_CLOCKWISE = auto()
    RECTANGLE = auto()
    RECTANGLE_CLOCKWISE = auto()
    RECTANGLE_COUNTER_CLOCKWISE = auto()

def classify_shape(points_xy: np.ndarray) -> str:
    pts = points_xy.astype(np.float32)
    if len(pts) < 3:
        return Shape.UNKNOWN

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
        return Shape.CIRCLE

    if v == 3:
        return Shape.TRIANGLE
    elif v == 4:
        # 矩形/正方形判定：最小外接矩形の縦横比でざっくり
        rect = cv2.minAreaRect(hull)
        (w, h) = rect[1]
        if w == 0 or h == 0:
            return Shape.RECTANGLE
    else:
        return Shape.UNKNOWN
