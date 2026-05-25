# Dracomancer

Dracomancer is the upper-limb exoskeleton teleoperation device used to command
DRAGON.  The current setup has two independent parts:

- `dracomancer bringup.launch`: starts the Dracomancer URDF, TF, servo bridge,
  and optional FC connection for the device itself.
- `dracomancer teleoperation.launch`: converts Dracomancer joystick and joint
  states into DRAGON navigation and joint commands.

## Teleoperation Flow

Current topic flow from Dracomancer to DRAGON:

```text
M5/joystick serial
  -> /dracomancer/joystick/raw
  -> joystick_calib_publisher.py
  -> /dracomancer/joystick/calibrated
  -> control_pose.py
  -> /dragon/uav/nav

Dracomancer servo states
  -> /dracomancer/servo/states
  -> servo_to_joint_states.py
  -> /dracomancer/joint_states
  -> control_joints.py
  -> /dragon/joints_ctrl
```

`control_joints.py` also subscribes to:

- `/dragon/flight_state`
- `/dragon/debug/fc_f_min`
- `/dragon/debug/fc_t_min`

These are used to gate or scale joint commands so that the robot is not forced
into obviously infeasible shapes.  For simulation, these checks can be relaxed.

## Recommended Bringup

On the PC running DRAGON simulation:

```bash
roslaunch dragon bringup.launch sim:=true rm:=true headless:=false
```

On the Dracomancer/Khadas side without FC connected:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false connect_fc:=false
```

On the Dracomancer/Khadas side with FC connected:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false fc_serial_port:=/dev/flight_controller
```

If the FC appears as a USB serial device, use the actual device name:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false fc_serial_port:=/dev/ttyUSB0
```

Khadas usually has no display.  Do not start RViz there.  Use
`launch_rviz:=true headless:=false` only on a GUI machine.

## Teleoperation Launch

Normal teleoperation:

```bash
roslaunch dracomancer teleoperation.launch
```

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

## Useful Switches

`teleoperation.launch` arguments:

- `enable_control_pose`: enables `/dragon/uav/nav` velocity commands.
- `enable_control_joints`: enables `/dragon/joints_ctrl` shape commands.
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

If DRAGON falls immediately, first isolate the command path:

```bash
roslaunch dracomancer teleoperation.launch enable_control_joints:=false
```

If it still falls, disable both DRAGON command publishers from teleoperation:

```bash
roslaunch dracomancer teleoperation.launch \
  enable_control_joints:=false \
  enable_control_pose:=false
```

If the robot only falls when `enable_control_joints:=true`, inspect
`/dragon/joints_ctrl`, reduce `max_step`, or enable more conservative shape
safety.

## RViz-Only Dracomancer Simulation

To move only the Dracomancer URDF in RViz using sliders:

```bash
roslaunch dracomancer bringup.launch \
  rm:=false sim:=false gui:=true launch_rviz:=true headless:=false
```

This does not command DRAGON.

## Build Notes

After changing C++ code such as `aerial_robot_model/src/servo_bridge`, rebuild:

```bash
catkin build aerial_robot_model dracomancer
source devel/setup.bash
```
