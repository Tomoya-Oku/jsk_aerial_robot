import rospy
import numpy as np
import gesture
from enum import Enum
from integration import ControlMode
from gesture import Shape
from std_msgs.msg import UInt8, String, Int8, Empty

### Arduino側と一致させること！！ ###
class ButtonEvent(Enum):
  NONE   = 0,
  SINGLE = 1,
  DOUBLE = 2,
  TRIPLE = 3,
  LONG   = 9

class ButtonSystem():

  def __init__(self):
    # Raw -> Raw Data (0/1)
    # Event -> Judged Data (PUSH / DOUBLE / LONG etc.)
    self.trigger_state_sub = rospy.Subscriber('/twin_hammer/button/trigger_raw', UInt8, self.trigger_raw_cb)
    self.trigger_event_sub = rospy.Subscriber('/twin_hammer/button/trigger_event', String, self.trigger_event_cb)
    self.device_button_state_sub = rospy.Subscriber('/twin_hammer/button/device_raw', UInt8, self.device_button_raw_cb)
    self.device_button_event_sub = rospy.Subscriber('/twin_hammer/button/device_event', String, self.device_button_event_cb)
    self.robot_button_state_sub = rospy.Subscriber('/twin_hammer/button/robot_raw', UInt8, self.robot_button_raw_cb)
    self.robot_button_event_sub = rospy.Subscriber('/twin_hammer/button/robot_event', String, self.robot_button_event_cb)

    self.trajectory = []

  # トリガーのON (1) / OFF (0) 生データ
  def trigger_raw_cb(self, msg):
    if msg.data == 0: # OFF
      if self.gestureMode:
        pts = np.asarray(self.trajectory, dtype=np.float32)
        shape = gesture.classify_shape_3d(pts)
        
        print(f"Detected shape: {shape}")

        if shape == Shape.UNKNOWN:
          pass
        elif shape == Shape.LINE_HORIZONTAL_LEFT_TO_RIGHT:
          pass
        elif shape == Shape.LINE_HORIZONTAL_RIGHT_TO_LEFT:
          pass
        elif shape == Shape.LINE_VERTICAL_BOTTOM_TO_TOP:
          pass
        elif shape == Shape.LINE_VERTICAL_TOP_TO_BOTTOM:
          pass
        elif shape == Shape.CIRCLE_CLOCKWISE:
          gesture.circleTask(self, radius=1.0, period=8.0, hz=40)
          # self.circleTask(self, radius=1.0, period=8.0, hz=40)
        elif shape == Shape.CIRCLE_COUNTER_CLOCKWISE:
          gesture.circleTask(self, radius=1.0, period=8.0, hz=40)
          # self.circleTask(self, radius=1.0, period=8.0, hz=40)
        elif shape == Shape.TRIANGLE_CLOCKWISE:
          pass
        elif shape == Shape.TRIANGLE_COUNTER_CLOCKWISE:
          pass
        elif shape == Shape.RECTANGLE_CLOCKWISE:
          pass
        elif shape == Shape.RECTANGLE_COUNTER_CLOCKWISE:
          pass
      
      self.resetInitPos()
      self.trajectory = []  # クリア

    elif msg.data == 1: # ON
      self.trajectory.append(self.device_pos)

    self.gestureMode = (msg.data == 1)
    print(self.gestureMode)

  # トリガーのイベント (EV_PUSH, EV_DOUBLE, EV_TRIPLE, EV_LONG)
  def trigger_event_cb(self, msg):
    # EV_DOUBLE: 2回押し -> POS/VEL切り替え
    if msg.data == ButtonEvent.DOUBLE:
      if self.control_mode == ControlMode.POS:
        self.control_mode = ControlMode.VEL
      elif self.control_mode == ControlMode.VEL:
        self.control_mode = ControlMode.POS

      self.init_robot_pose()
      self.init_device_pose()

  def device_button_raw_cb(self, msg):
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

  def robot_button_raw_cb(self, msg):
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

if __name__ == "__main__":
  rospy.init_node("button_system")
  Tracker = ButtonSystem()
  Tracker.main()