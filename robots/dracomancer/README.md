# Dracomancer

Dracomancer is the upper-limb exoskeleton teleoperation device used to command
DRAGON.  The current setup has two independent parts:

- `dracomancer bringup.launch`: starts the Dracomancer URDF, TF, servo bridge,
  and optional FC connection for the device itself.
- `dracomancer teleoperation.launch`: starts position, attitude, and joint-angle
  teleoperation nodes.
- `dracomancer haptics.launch`: starts haptic feedback from suppressed shape
  command error.

## Teleoperation Flow

Current topic flow from Dracomancer to DRAGON:

```text
M5/joystick serial
  -> /dracomancer/joystick/raw
  -> calibrate_joystick.py
  -> /dracomancer/joystick/calibrated
  -> control_position.py
  -> /dragon/uav/nav

Dracomancer servo states
  -> /dracomancer/servo/states
  -> convert_servo_to_joint_states.py
  -> /dracomancer/joint_states
  -> control_joint_angle.py
  -> /dragon/joints_ctrl
  -> /dracomancer/shape_control_error
  -> haptics.launch / control_haptic_feedback.py
  -> /dracomancer/haptic_torque
```

Default joint mapping treats DRAGON as one serial arm.  DRAGON `link1` is the
head side and `link4` is the tail side, so wrist motion from Dracomancer drives
the head-side DRAGON joint and shoulder motion drives the tail-side joint:

```text
DRAGON joint1_pitch <- Dracomancer wrist_flexion_extension_joint
DRAGON joint1_yaw   <- Dracomancer wrist_supination_joint
DRAGON joint2_pitch <- Dracomancer upper_arm_external_internal_rotation_joint
DRAGON joint2_yaw   <- Dracomancer elbow_flexion_extension_joint, inverted
DRAGON joint3_pitch <- Dracomancer shoulder_flexion_extension_joint
DRAGON joint3_yaw   <- Dracomancer shoulder_abduction_adduction_joint
```

The default scale is 1:1.  DRAGON yaw joints keep their nominal `pi/2` offset
except `joint2_yaw`, where the Dracomancer elbow is inverted with no offset so
elbow extension maps to the straight middle joint.

`control_joint_angle.py` also subscribes to:

- `/dragon/flight_state`
- `/dragon/debug/fc_f_min`
- `/dragon/debug/fc_t_min`

These are used to gate or scale joint commands so that the robot is not forced
into obviously infeasible shapes.  For simulation, these checks can be relaxed.

Teleoperation has three runtime modes:

| Mode | Pose command | Joint command | Purpose |
| --- | --- | --- | --- |
| `startup` | disabled | hold startup pose `[0, pi/2, 0, pi/2, 0, pi/2]` | takeoff / reset posture |
| `precision` | disabled | map Dracomancer arm joints to DRAGON | close-range shape control |
| `wide` | joystick + IMU relative movement | hold startup pose | large-area movement |

Switch modes at runtime:

```bash
rostopic pub -1 /dracomancer/teleop_mode std_msgs/String "data: 'wide'"
rostopic pub -1 /dracomancer/teleop_mode std_msgs/String "data: 'precision'"
rostopic pub -1 /dracomancer/teleop_mode std_msgs/String "data: 'startup'"
```

## Bringup Quick Reference

Use `rm` and `sim` for normal operation.  `real_machine` and `simulation` are
kept as compatibility aliases for older commands.

### Frequently Used Commands

On the PC running DRAGON simulation:

```bash
roslaunch dragon bringup.launch sim:=true headless:=false
```

On the Dracomancer/Khadas side without FC connected:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false connect_fc:=false
```

On the Dracomancer/Khadas side with FC connected:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false fc_serial_port:=/dev/flight_controller
```

On the Dracomancer/Khadas side with FC connected while showing RViz on another
GUI PC:

```bash
roslaunch dracomancer bringup.launch \
  rm:=true \
  sim:=true \
  headless:=true \
  fc_serial_port:=/dev/flight_controller
```

Use the GUI PC as the fixed ROS master. On the Dracomancer/Khadas side, point
`ROS_MASTER_URI` to the GUI PC before launching bringup. On the GUI PC, keep
`ROS_MASTER_URI` pointed at itself and launch only RViz:

```bash
roslaunch dracomancer rviz.launch
```

If the FC appears as a USB serial device, use the actual device name:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false fc_serial_port:=/dev/ttyUSB0
```

Khadas usually has no display.  Do not start RViz there.  Use
`launch_rviz:=true headless:=false` only on a GUI machine.

RViz-only Dracomancer joint slider mode, without FC or simulator:

```bash
roslaunch dracomancer bringup.launch \
  rm:=false \
  sim:=true \
  launch_rviz:=true \
  headless:=false
```

### Bringup Arguments

| Group | Argument | Default | Purpose |
| --- | --- | --- | --- |
| Mode | `rm` | `True` | Shortcut for real-machine mode. |
| Mode | `sim` | `False` | Shortcut for visualization/simulation-oriented launch behavior. |
| Mode | `real_machine` | `rm` | Compatibility alias used by included launch files. |
| Mode | `simulation` | `sim` | Compatibility alias used by included launch files. |
| FC | `connect_fc` | `real_machine` | Starts the flight-controller serial bridge. |
| FC | `fc_serial_port` | `/dev/ttyUSB0` | Serial device for the FC. |
| FC | `fc_serial_baud` | `921600` | Serial baud rate for the FC. |
| FC | `run_servo_rough_calib` | `False` | Runs rough servo calibration node from the sensor include. |
| Model/RViz | `headless` | `True` | Hides RViz/model visualization unless overridden. |
| Model/RViz | `launch_rviz` | `sim` | Allows the model launch to show RViz. |
| Model/RViz | `rviz.launch` | - | Starts only RViz on a GUI PC connected to the same ROS master. |
| Model/RViz | `robot_ns` | `dracomancer` | ROS namespace. |
| Servo | `servo_topic` | `/dracomancer/servo/states` | Input topic for `convert_servo_to_joint_states.py`. |
| Servo | `joint_states_topic` | `/dracomancer/joint_states` | Output topic from `convert_servo_to_joint_states.py`. |
| Web | `web` | `False` | Starts the mobile web console. |
| Web | `web_console_port` | `8080` | Web console port. |

## Teleoperation Launch

Normal teleoperation:

```bash
roslaunch dracomancer teleoperation.launch
```

This starts in `startup` mode.  Switch to `wide` for joystick/IMU movement, or
`precision` for arm-to-DRAGON shape mapping.

For simulation where falling is acceptable and infeasible-pose rejection is too
strong:

```bash
roslaunch dracomancer teleoperation.launch \
  enable_shape_safety:=false \
  publish_joints_only_when_hovering:=true \
  publish_joints_before_device_ready:=false \
  max_step:=0.03
```

For partial safety instead of disabling it completely:

```bash
roslaunch dracomancer teleoperation.launch \
  min_safety_scale:=0.5 \
  missing_inradius_scale:=0.5 \
  force_inradius_min:=0.05 \
  torque_inradius_min:=0.005 \
  max_step:=0.03
```

To compute haptic feedback torque without sending current to the device, start
the haptics launch beside teleoperation:

```bash
roslaunch dracomancer haptics.launch
```

Publishing actual Spinal current commands requires an explicitly calibrated
conversion:

```bash
roslaunch dracomancer haptics.launch \
  enable_haptic_current_command:=true \
  haptic_current_per_nm:=100.0
```

## Useful Switches

`teleoperation.launch` arguments:

- `enable_position_control`: enables `/dragon/uav/nav` velocity commands.
- `enable_attitude_control`: enables attitude command publishing.
- `enable_joint_angle_control`: enables `/dragon/joints_ctrl` shape commands.
- `teleop_mode`: initial mode, one of `startup`, `precision`, or `wide`.
- `mode_topic`: runtime mode switch topic. Default: `/dracomancer/teleop_mode`.
- `enable_servo_to_joint_states`: converts Dracomancer servo state to joint
  state.
- `servo_topic`: input servo state topic. Default:
  `/dracomancer/servo/states`.
- `device_joint_topic`: Dracomancer joint state topic. Default:
  `/dracomancer/joint_states`.
- `enable_shape_safety`: enables feasible-control inradius based joint command
  scaling.
- `missing_inradius_scale`: command scale used before `/dragon/debug/fc_*` is
  available.
- `min_safety_scale`: lower bound for joint command scale when inradius is low.
- `max_step`: max joint target change per control cycle.
- `publish_joints_only_when_hovering`: publishes `/dragon/joints_ctrl` only when
  DRAGON is in hover or later flight states.
- `publish_joints_before_device_ready`: allows publishing before Dracomancer
  joint state is received. Default is `false`.
- `axis_x`, `axis_y`, `axis_z`: joystick axes. Defaults: `0`, `1`, `2`.
- `xy_vel`, `z_vel`: joystick velocity scale. Defaults: `0.3`, `0.2`.

## Debugging

Check whether teleoperation is sending pose commands:

```bash
rostopic echo /dragon/uav/nav
```

Check whether teleoperation is sending shape commands:

```bash
rostopic echo /dragon/joints_ctrl
```

Check the shape safety state:

```bash
rostopic echo /dracomancer/dragon_shape_safety
```

The safety message contains:

```text
[force_inradius, torque_inradius, safety_scale]
```

Check haptic feedback torque computed from suppressed shape input:

```bash
rostopic echo /dracomancer/haptic_torque
```

`haptics.launch` arguments:

- `enable_haptic_current_command`: publishes haptic torque as
  `/servo/target_current`. Default is `false`.
- `haptic_current_per_nm`: calibrated conversion from Nm to Spinal current
  command units. Must be positive before current commands are sent.

If DRAGON falls immediately, first isolate the command path:

```bash
roslaunch dracomancer teleoperation.launch enable_joint_angle_control:=false
```

If it still falls, disable both DRAGON command publishers from teleoperation:

```bash
roslaunch dracomancer teleoperation.launch \
  enable_joint_angle_control:=false \
  enable_position_control:=false
```

If the robot only falls when `enable_joint_angle_control:=true`, inspect
`/dragon/joints_ctrl`, reduce `max_step`, or enable more conservative shape
safety.

## RViz-Only Dracomancer Simulation

To move only the Dracomancer URDF in RViz using sliders, use the RViz-only
bringup command above:

```bash
roslaunch dracomancer bringup.launch \
  rm:=false sim:=true launch_rviz:=true headless:=false
```

This does not command DRAGON.

## Build Notes

After changing C++ code such as `aerial_robot_model/src/servo_bridge`, rebuild:

```bash
catkin build aerial_robot_model dracomancer
source devel/setup.bash
```
