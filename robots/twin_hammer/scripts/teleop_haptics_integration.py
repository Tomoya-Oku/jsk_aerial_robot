#!/usr/bin/env python

import rospy
import time
import math
import numpy as np
import cv2
import tf.transformations as tf
from std_msgs.msg import UInt8, String, Int8, Empty
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3Stamped
from scipy.spatial.transform import Rotation as R

def exponential(x, base, k_exp):
  return pow(x, base) * k_exp

def logarithm(x, base, k_log):
  return math.log(x, base) * k_log

class teleop_haptics_integration():

  def __init__(self):

    # Parameters
    self.robot_name = rospy.get_param("~robot_name", "gimbalrotor")
    self.control_mode = rospy.get_param("~control_mode", "pos") # "pos" or "vel" -> switch with trigger
    self.convert_method = rospy.get_param("~convert_method", "log") # "prop" or "exp" or "log"
    self.frame = rospy.get_param("~frame", "local") # "local" or "world"
    self.feedback_from_ang = rospy.get_param("~feedback_from_ang", "False")

    self.device_initialize_flag = False
    self.robot_initialize_flag = False

    # Publishers
    self.device_start_pub = rospy.Publisher('/twin_hammer/teleop_command/start', Empty, queue_size=1) # for arming
    self.device_takeoff_pub = rospy.Publisher('/twin_hammer/teleop_command/takeoff', Empty, queue_size=1) # for takeoff
    self.device_land_pub = rospy.Publisher('/twin_hammer/teleop_command/land', Empty, queue_size=1) # for landing

    self.robot_start_pub = rospy.Publisher('/' + self.robot_name + '/teleop_command/start', Empty, queue_size=1) # for arming
    self.robot_takeoff_pub = rospy.Publisher('/' + self.robot_name + '/teleop_command/takeoff', Empty, queue_size=1) # for takeoff
    self.robot_land_pub = rospy.Publisher('/' + self.robot_name + '/teleop_command/land', Empty, queue_size=1) # for landing
    
    self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
    self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)
    self.feedback_pub = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)

    # Subscribers
    self.device_flight_state_sub = rospy.Subscriber('/twin_hammer/flight_state', UInt8, self.device_flight_state_cb)
    self.robot_flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.robot_flight_state_cb)
    self.device_pos_sub = rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.device_pos_cb)
    self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
    self.teleop_mode_sub = rospy.Subscriber('/twin_hammer/teleop_mode', String, self.teleop_mode_cb)
    self.robot_wrench_sub = rospy.Subscriber('/cfs/data', WrenchStamped, self.robot_wrench_cb)

    # State -> Raw Data (0/1)
    # Event -> Judged Data (PUSH / DOUBLE / LONG etc.)
    self.trigger_state_sub = rospy.Subscriber('/twin_hammer/trigger_state', UInt8, self.trigger_state_cb)
    self.trigger_event_sub = rospy.Subscriber('/twin_hammer/trigger_event', String, self.trigger_event_cb)
    self.device_button_state_sub = rospy.Subscriber('/twin_hammer/device_button_state', UInt8, self.device_button_state_cb)
    self.device_button_event_sub = rospy.Subscriber('/twin_hammer/device_button_event', String, self.device_button_event_cb)
    self.robot_button_state_sub = rospy.Subscriber('/twin_hammer/robot_button_state', UInt8, self.robot_button_state_cb)
    self.robot_button_event_sub = rospy.Subscriber('/twin_hammer/robot_button_event', String, self.robot_button_event_cb)

    # Messages
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.roll_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pitch_nav_mode = FlightNav.POS_VEL_MODE
    self.target_att_nav = Vector3Stamped()
    self.haptics_wrench_msg = WrenchStamped()

    # States
    self.device_arm_off = True
    self.device_arm_on = False
    self.device_takeoff = False
    self.device_hovering = False
    self.device_landing = False
    self.device_stop = False

    self.robot_arm_off = True
    self.robot_arm_on = False
    self.robot_takeoff = False
    self.robot_hovering = False
    self.robot_landing = False
    self.robot_stop = False

    self.device_pos = [None]*3
    self.device_pos_traj = []
    self.device_att = [None]*3
    self.robot_pos = [None]*3
    self.robot_att = [None]*3
    self.device_init_pos = [None]*3
    self.device_init_att = [None]*3
    self.robot_init_pos = [None]*3
    self.robot_init_att = [None]*3
    self.robot_vel_mode_fix_pos = [None]*3
    self.robot_vel_mode_fix_att = [None]*3
    self.device_att_unwrapped = [0.0]*3
    self.device_att_prev = [0.0]*3

    self.wait_flag = False
    self.pos_scale = 1.0
    self.vel_scale = 0.3
    self.ang_vel_scale = 0.1
    self.feedback_force_scale = 10.0
    self.feedback_torque_scale = 1.0
    self.robot_wrench = [0.0]*6
    self.filtered_robot_wrench_local = [0.0]*6
    self.Ad_R_robot = np.identity(6)
    self.Ad_R_inv_device = np.identity(6)
    self.moment_arm = np.array([-(0.044 + 0.025), 0, 0])
    self.k_p = 1.0
    self.exp_base = 1.45
    self.log_base = 1.45
    self.k_exp = 0.4
    self.k_log = 1.0
    self.k_att_diff = 1.0

    # Trajectory-Based Gesture Recognition
    self.gestureMode = False # トリガー長押しで切り替え
    self.trajectory = []

  def device_flight_state_cb(self, msg):
    # aerial_robot_base/flight_navigaton.h 参照
    if msg.data == 2:
      self.device_arm_off = False
      self.device_arm_on = True
    elif msg.data == 3:
      self.device_arm_on = False
      self.device_takeoff = True
    elif msg.data == 5:
      self.device_takeoff = False
      self.device_hovering = True
    elif msg.data == 4:
      self.device_hovering = False
      self.device_landing = True
    elif msg.data == 6:
      self.device_landing = False
      self.device_stop = True

  def robot_flight_state_cb(self, msg):
    # aerial_robot_base/flight_navigaton.h 参照
    if msg.data == 2:
      self.robot_arm_off = False
      self.robot_arm_on = True
    elif msg.data == 3:
      self.robot_arm_on = False
      self.robot_takeoff = True
    elif msg.data == 5:
      self.robot_takeoff = False
      self.robot_hovering = True
    elif msg.data == 4:
      self.robot_hovering = False
      self.robot_landing = True
    elif msg.data == 6:
      self.robot_landing = False
      self.robot_stop = True

  def device_pos_cb(self, msg):
    self.device_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    if self.gestureMode:
      self.trajectory.append([self.device_pos[1], self.device_pos[2]]) # とりあえずy-zで
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rot = R.from_quat(q)
    self.device_att = rot.as_euler('xyz')
    for i in range(3):
      current_angle = self.device_att[i]
      delta_angle = (current_angle - self.device_att_prev[i] + np.pi) % (2 * np.pi) - np.pi
      self.device_att_unwrapped[i] += delta_angle
      self.device_att_prev[i] = current_angle
    R_mat = rot.as_matrix()
    self.Ad_R_inv_device = np.block([
      [R_mat.T, np.zeros((3,3))],
      [np.zeros((3,3)), R_mat.T]
    ])
    if not self.device_initialize_flag:
      self.device_init_pos = self.device_pos
      self.device_init_att = self.device_att
      self.device_initialize_flag = True

  def robot_pos_cb(self, msg):
    self.robot_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rot = R.from_quat(q)
    self.robot_att = rot.as_euler('xyz')
    R_mat = rot.as_matrix()
    self.Ad_R_robot = np.block([
      [R_mat, np.zeros((3,3))],
      [np.zeros((3,3)), R_mat]
    ])
    if not self.robot_initialize_flag:
      self.robot_init_pos = self.robot_pos
      self.robot_init_att = self.robot_att
      self.robot_initialize_flag = True

  def teleop_mode_cb(self, msg):
    self.wait_flag = False
    self.device_initialize_flag = False
    self.robot_initialize_flag = False
    if msg.data == "pos":
      self.control_mode = "pos"
    if msg.data == "vel":
      self.control_mode = "vel"

  def robot_wrench_cb(self, msg):
    fx = msg.wrench.force.z
    fy = msg.wrench.force.x
    fz = msg.wrench.force.y
    tx = msg.wrench.torque.z
    ty = msg.wrench.torque.x
    tz = msg.wrench.torque.y
    wrench_local = [fx, fy, fz, tx, ty, tz]
    delay_param = 0.05
    for i in range(6):
      self.filtered_robot_wrench_local[i] = (1 - delay_param) * self.filtered_robot_wrench_local[i] + delay_param * wrench_local[i]
    wrench_world = np.dot(self.Ad_R_robot, self.filtered_robot_wrench_local)
    if self.frame == "local":
      self.robot_wrench = self.filtered_robot_wrench_local
    else:
      self.robot_wrench = wrench_world

  # トリガーのON / OFF 生データ
  def trigger_state_cb(self, msg):
    # トリガーがOFFになったタイミング
    if (msg.data == 0) and self.gestureMode == True:
      pts = np.asarray(self.trajectory, dtype=np.float32)
      shape, shape_info = self.classify_shape(pts)

      print(f"Detected shape: {shape}")

      if shape == "circle": print(f"Radius of Circle: {shape_info}")
      else: pass
      
      self.doTask(self, shape=shape, shape_info=shape_info)
      self.trajectory = []  # クリア
    else:
      self.gestureMode = (msg.data == 1)

  # トリガーのイベント (EV_PUSH, EV_DOUBLE, EV_TRIPLE, EV_LONG)
  def trigger_event_cb(self, msg):
    # EV_DOUBLE: 2回押し -> POS/VEL切り替え
    if msg.data == "double":
      if self.control_mode == "pos" and self.robot_hovering:
        self.control_mode = "vel"
      elif self.control_mode == "vel" and self.robot_hovering:
        self.control_mode = "pos"

      # 現在地を初期位置に
      self.robot_init_pos = self.robot_pos
      self.robot_init_att = self.robot_att
      self.device_init_pos = self.device_pos
      self.device_init_att = self.device_att

  def device_button_state_cb(self, msg):
    pass

  def device_button_event_cb(self, msg):
    # EV_LONG: 長押し -> Twin-Hammerの起動シーケンス
    if msg.data == "long":
      # arm_offであればarm_onする
      if self.device_arm_off:
        self.device_start_pub.publish(Empty())
        print("[Twin-Hammer] Send arming command")
      # arm_onであればTakeoffする
      elif self.device_arm_on:
        self.device_takeoff_pub.publish(Empty())
        print("[Twin-Hammer] Send takeoff command")
      # Takeoff/Hovering中であればLandingする
      elif self.device_takeoff or self.device_hovering:
        self.device_land_pub.publish(Empty())
        print("[Twin-Hammer] Send land command")
      else:
        pass

  def robot_button_state_cb(self, msg):
    pass

  def robot_button_event_cb(self, msg):
    # EV_LONG: 長押し -> Robotの起動シーケンス
    if msg.data == "long":
      # arm_offであればarm_onする
      if self.robot_arm_off:
        self.robot_start_pub.publish(Empty())
        print(f"[{self.robot_name}] Send arming command")
      # arm_onであればTakeoffする
      elif self.robot_arm_on:
        self.robot_takeoff_pub.publish(Empty())
        print(f"[{self.robot_name}] Send takeoff command")
      # Takeoff/Hovering中であればLandingする
      elif self.robot_takeoff or self.robot_hovering:
        self.robot_land_pub.publish(Empty())
        print(f"[{self.robot_name}] Send land command")
      else:
        pass

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
  
  def circleTask(self, radius=1.0, period=8.0, hz=40):
    """
    現在位置(center)を中心に、zは据え置きで半径radiusの円を1周する。
    period: 1周にかける秒数、hz: 制御周期
    """
    # 安全確認
    if not self.robot_hovering or self.robot_pos is None or self.robot_att is None:
      rospy.logwarn("circleTask: robot not ready (hovering/pose).")
      return

    # 中心と高度・初期yawを固定
    cx, cy, cz = self.robot_pos[0], self.robot_pos[1], self.robot_pos[2]
    yaw0 = self.robot_att[2]

    # メインループ側の目標計算を一時停止（上書き防止）
    _dev_init, _rob_init = self.device_initialize_flag, self.robot_initialize_flag
    self.device_initialize_flag = False
    self.robot_initialize_flag = False

    # 一時的にposモードへ（復帰用に保存）
    _mode = self.control_mode
    self.control_mode = "pos"

    rate = rospy.Rate(hz)
    total = max(1, int(period * hz))

    try:
      for k in range(total):
        if not self.robot_hovering or self.robot_landing or rospy.is_shutdown():
          break

        theta = 2.0 * math.pi * (float(k) / float(total))
        x = cx + radius * math.cos(theta)
        y = cy + radius * math.sin(theta)
        z = cz  # 高度据え置き

        # 0066環境の安全範囲にクリップ（ファイルの下限上限に合わせる）
        x = max(min(x, 2.0), -1.3)
        y = max(min(y, 1.6), -1.6)
        z = max(min(z, 1.2), 0.3)

        # 目標セット（姿勢は水平・yaw据え置き）
        self.flight_nav.target_pos_x = x
        self.flight_nav.target_pos_y = y
        self.flight_nav.target_pos_z = z
        self.flight_nav.target_roll  = 0.0
        self.flight_nav.target_pitch = 0.0
        self.flight_nav.target_yaw   = yaw0

        self.target_att_nav.vector.x = 0.0
        self.target_att_nav.vector.y = 0.0

        # 即時送信（メインループと重なっても同値なのでOK）
        self.nav_pub.publish(self.flight_nav)
        self.att_pub.publish(self.target_att_nav)

        rate.sleep()
    finally:
      # 元の状態に戻す
      self.control_mode = _mode
      self.device_initialize_flag = _dev_init
      self.robot_initialize_flag  = _rob_init

  def lineTask(self):
    pass

  def doTask(self, shape="unknown", shape_info=None):
    if shape == "circle":
      radius = shape_info
      self.circleTask(radius)
    elif shape == "line":
      self.lineTask()
    else:
      pass

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      target_pos = [0.0]*3
      target_att = [0.0]*3
      target_vel = [0.0]*3
      target_ang_vel = [0.0]*3
      feedback_wrench = [0.0]*6

      if self.device_init_pos is None or not self.robot_hovering:
        self.device_initialize_flag = False
      if self.robot_init_pos is None or not self.robot_hovering:
        self.robot_initialize_flag = False

      if self.device_initialize_flag and self.robot_initialize_flag:
        """ calc target pos and vel """
        vel_mode_pos_thre = [0.1,0.1,0.1]
        vel_mode_att_thre = [0.05,0.05,0.05]
        for i in range(3):
          device_pos_diff = self.device_pos[i] - self.device_init_pos[i]
          # device_att_diff = self.device_att_unwrapped[i] - self.device_init_att[i]
          device_att_diff = self.device_att[i] - self.device_init_att[i]
          target_pos[i] = (self.robot_init_pos[i] + device_pos_diff) * self.pos_scale
          target_att[i] = self.device_att[i]
          target_vel[i] = self.robot_pos[i] + device_pos_diff * self.vel_scale
          target_ang_vel[i] = self.robot_att[i] + device_att_diff * self.ang_vel_scale
          """ position fix for vel mode """
          if abs(target_vel[i]-self.robot_pos[i]) < vel_mode_pos_thre[i]:
            if self.robot_vel_mode_fix_pos[i] == None:
              self.robot_vel_mode_fix_pos[i] = self.robot_pos[i]
            target_vel[i] = self.robot_vel_mode_fix_pos[i]
          else:
            self.robot_vel_mode_fix_pos[i] = None
          if abs(target_ang_vel[i]-self.robot_att[i]) < vel_mode_att_thre[i]:
            if self.robot_vel_mode_fix_att[i] == None:
              self.robot_vel_mode_fix_att[i] = self.robot_att[i]
            target_ang_vel[i] = self.robot_vel_mode_fix_att[i]
          else:
            self.robot_vel_mode_fix_att[i] = None
          if self.control_mode == "vel":
            feedback_wrench[i] = - device_pos_diff * self.feedback_force_scale
            feedback_wrench[i+3] = - device_att_diff * self.feedback_torque_scale

        """ convert feedback wrench with log """
        k_force = 1.5
        k_torque = 1.0
        log_base = 1.45
        for i in range(6):
          if feedback_wrench[i] >= 0:
            feedback_wrench[i] = logarithm(feedback_wrench[i]+1, log_base, k_force)
          else:
            feedback_wrench[i] = -logarithm(-(feedback_wrench[i]-1), log_base, k_torque)
            
        # feedback_wrench: velocy mode の位置のズレ
        # haptics_wrench: ロボットの力センサから
        """ calc feedback wrench from force sensor """
        haptics_wrench = [0.0]*6
        for i in range(len(self.robot_wrench)):
          wrench_i = self.robot_wrench[i]
          """ propotional conversion """
          if self.convert_method == "prop":
            haptics_wrench[i] = wrench_i * self.k_p
          """ exponential conversion """
          if self.convert_method == "exp":
            if wrench_i >= 0.0:
              haptics_wrench[i] = exponential(wrench_i, self.exp_base, self.k_exp)
            else:
              haptics_wrench[i] = -exponential(-wrench_i, self.exp_base, self.k_exp)
          """ log conversion """
          if self.convert_method == "log":
            '''
            if wrench_i>self.range_log:
              haptics_wrench[i] = logarithm(wrench_i, self.log_base, self.k_log)
            elif wrench_i<-self.range_log:
              haptics_wrench[i] = -logarithm(-wrench_i, self.log_base, self.k_log)
            else:
              haptics_wrench[i] = wrench_i * self.a_log
            '''
            if wrench_i >= 0:
              haptics_wrench[i] = logarithm(wrench_i+1, self.log_base, self.k_log)
            else:
              haptics_wrench[i] = -logarithm(-(wrench_i-1), self.log_base, self.k_log)

        """ force feedback from ang diff """
        if self.feedback_from_ang:
          att_diff = self.robot_att - self.device_att
          for i in range(len(att_diff)):
            if att_diff[i] >= 0:
              wrench_from_pos_diff = logarithm(att_diff[i]+1,self.log_base,self.k_att_diff)
            else:
              wrench_from_pos_diff = logarithm(-(att_diff[i]-1),self.log_base,self.k_att_diff)
            haptics_wrench[i] += wrench_from_pos_diff

        """ convert frame of feedback wrench """
        if self.frame == "world":
          haptics_wrench = np.dot(self.Ad_R_inv_device,haptics_wrench)

        for i in range(6):
          haptics_wrench[i] += feedback_wrench[i]

        """ limitation of feedback wrench for safety """
        force_limit = 10
        torque_limit = 1.5
        for i in range(3):
          haptics_wrench[i] = max(min(haptics_wrench[i], force_limit), -force_limit)
          haptics_wrench[i+3] = max(min(haptics_wrench[i+3], torque_limit), -torque_limit)

        """ limitation of target_pos and att in 0066 """
        """ x """
        if self.robot_pos[0] > 2.0:
          target_pos[0] = 2.0
          target_vel[0] = 2.0
        if self.robot_pos[0] < -1.3:
          target_pos[0] = -1.3
          target_vel[0] = -1.3
        """ y """
        if self.robot_pos[1] > 1.6:
          target_pos[1] = 1.6
          target_vel[1] = 1.6
        if self.robot_pos[1] < -1.6:
          target_pos[1] = -1.6
          target_vel[1] = -1.6
        """ z """
        if self.robot_pos[2] > 1.2:
          target_pos[2] = 1.2
          target_vel[2] = 1.2
        if self.robot_pos[2] < 0.3:
          target_pos[2] = 0.3
          target_vel[2] = 0.3
        """ roll and pitch """
        limit_angle = 0.35
        for i in range(2):
          target_att[i] = max(min(target_att[i], limit_angle), -limit_angle)
          target_ang_vel[i] = max(min(target_ang_vel[i], limit_angle), -limit_angle)


        if self.control_mode == "pos":
          self.flight_nav.target_pos_x = target_pos[0]
          self.flight_nav.target_pos_y = target_pos[1]
          self.flight_nav.target_pos_z = target_pos[2]
          self.flight_nav.target_yaw = target_att[2]
          self.flight_nav.target_roll = target_att[0]
          self.flight_nav.target_pitch = target_att[1]
          self.target_att_nav.vector.x = target_att[0]
          self.target_att_nav.vector.y = target_att[1]

        if self.control_mode == "vel":
          self.flight_nav.target_pos_x = target_vel[0]
          self.flight_nav.target_pos_y = target_vel[1]
          # self.flight_nav.target_vel_x = target_vel[0]
          # self.flight_nav.target_vel_y = target_vel[1]
          self.flight_nav.target_pos_z = target_pos[2] # not use vel for safety
          self.flight_nav.target_yaw = target_att[2]
          # self.flight_nav.target_omega_z = target_ang_vel[2]
          self.flight_nav.target_roll = target_att[0] # not use vel for safety
          self.flight_nav.target_pitch = target_att[1] # not use vel for safety
          self.target_att_nav.vector.x = target_att[0] # not use vel for safety
          self.target_att_nav.vector.y = target_att[1] # not use vel for safety

        self.haptics_wrench_msg.wrench.force.x = haptics_wrench[0]
        self.haptics_wrench_msg.wrench.force.y = haptics_wrench[1]
        self.haptics_wrench_msg.wrench.force.z = haptics_wrench[2]
        self.haptics_wrench_msg.wrench.torque.x = haptics_wrench[3]
        self.haptics_wrench_msg.wrench.torque.y = haptics_wrench[4]
        self.haptics_wrench_msg.wrench.torque.z = haptics_wrench[5]        

      if self.robot_hovering and not self.robot_landing:
        if not self.wait_flag:
          rospy.sleep(3.0)
          self.wait_flag = True

        if not self.gestureMode:
          self.nav_pub.publish(self.flight_nav)
          self.att_pub.publish(self.target_att_nav)
          self.feedback_pub.publish(self.haptics_wrench_msg) 

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("teleop_haptics_integration")
  Tracker = teleop_haptics_integration()
  Tracker.main()
