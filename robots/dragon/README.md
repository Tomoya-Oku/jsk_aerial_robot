# DRAGON: 

**Definiation**: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation 

<img src="images/dragon_clip.jpg" width="410" height="300"> <a href="https://www.youtube.com/embed/ZDYU22qNI_Q" target="_blank"><img src="http://img.youtube.com/vi/ZDYU22qNI_Q/0.jpg"  alt="euroc" width="400" height="300" border="10" /></a>

**Related Papers**:
-  M. Zhao, T. Anzai, F. Shi, X. Chen, K. Okada and M. Inaba, "Design, Modeling, and Control of an Aerial Robot DRAGON: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation," in IEEE Robotics and Automation Letters, vol. 3, no. 2, pp. 1176-1183, April 2018.
# How to use

## 0. calibration: 
    Calibration is via rosserial between on-board processor and spinal. Please check the wiki.

## 1. Bringup quick reference

Use `rm` and `sim` for normal operation.  `real_machine` and `simulation` are
kept as compatibility aliases for older commands.

### Frequently used commands

Real robot, default current model:

```bash
roslaunch dragon bringup.launch
```

Real robot, explicit estimation mode:

```bash
roslaunch dragon bringup.launch estimate_mode:=1
```

Gazebo simulation with RViz:

```bash
roslaunch dragon bringup.launch sim:=true headless:=false
```

Gazebo simulation without GUI:

```bash
roslaunch dragon bringup.launch sim:=true
```

Mujoco simulation:

```bash
roslaunch dragon bringup.launch sim:=true mujoco:=true
```

RViz joint-control only, without FC or simulator:

```bash
roslaunch dragon bringup.launch rm:=false sim:=false rviz_joint_ctrl:=true headless:=false
```

Legacy simulation commands such as
`real_machine:=false simulation:=true` still work, but new notes should prefer
`sim:=true`.

**Note**: before takeoff, check the servo angles with:

```bash
rostopic echo -c /servo/states
```

Make sure no angle exceeds the normal range, for example `5 -> 4095`.

### Bringup arguments

| Group | Argument | Default | Purpose |
| --- | --- | --- | --- |
| Mode | `rm` | `True` | Shortcut for real-machine mode. |
| Mode | `sim` | `False` | Shortcut for simulation mode.  When true, `real_machine` becomes false. |
| Mode | `real_machine` | `rm && !sim` | Compatibility alias used by included launch files. |
| Mode | `simulation` | `sim` | Compatibility alias used by included launch files. |
| Robot config | `new_model` | `true` | Selects `v1_5_202601`; false selects `vim4_202311`. |
| Robot config | `model_name` | derived | Internal model directory name. |
| Robot config | `link_num` | `quad` | Robot configuration directory under `robots/dragon/config`. |
| Robot config | `battery` | `true` | Selects the URDF/Mujoco model with or without batteries. |
| Robot config | `full_vectoring_mode` | `true` | Uses full-vectoring model/control; false uses hydrus-like gimbal control. |
| Estimation | `estimate_mode` | `1` | Real-robot estimator mode.  `1` is experiment estimate. |
| Estimation | `sim_estimate_mode` | `2` | Simulation estimator mode.  `2` is ground truth. |
| Navigation | `takeoff_height` | `0` | Overrides navigation takeoff height when greater than zero. |
| Simulation | `headless` | `True` | Hides RViz/Gazebo GUI when possible. |
| Simulation | `launch_gazebo` | `True` | Starts Gazebo when `sim:=true mujoco:=false`. |
| Simulation | `worldtype` | empty world | Gazebo world file. |
| Simulation | `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` | `0.0` | Gazebo spawn pose. |
| Simulation | `mujoco` | `False` | Uses Mujoco instead of Gazebo when `sim:=true`. |
| Namespacing | `robot_id` | empty | Appended to `dragon` to build `robot_ns`. |
| Namespacing | `robot_ns` | `dragon$(arg robot_id)` | ROS namespace. |
| Advanced | `direct_model` | `False` | Uses `direct_model_name` instead of the packaged model path. |
| Advanced | `direct_model_name` | unset | Explicit URDF/Xacro path when `direct_model:=true`. |
| Advanced | `config_dir` | derived | Config directory. Override only for custom configs. |
| Advanced | `preflight_joint_control` | `simulation` | Enables preflight joint control. |
| Advanced | `rviz_joint_ctrl` | derived | Enables RViz joint-control wrapper. |
| Advanced | `rviz_spawn_*` | `(0, 0, 2, 0)` | Static root TF for RViz joint-control mode. |
| Demo | `demo` | `False` | Starts `simple_demo.py`. |

Estimator mode values used in the launch file:

- `0`: egomotion estimate
- `1`: experiment estimate, useful with unstable mocap
- `2`: ground truth

## 2. tele-operation
   **note**: after the robot completely transforms to normal form

   - **keyboard**: please check instruction [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki/keyboard_operation)
   - **joystick**: please check instruction [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki/joystick_operation)
   
## 3. transformation demostration
   **note**: after the robot completely hovering with the message `Hovering!`
   
   - defualt dragon pose: ``` $ rosrun dragon transformation_demo.py  _mode:=0```
   - spiral pose: ``` $ rosrun dragon transformation_demo.py  _mode:=1```
   - m-like pose: ``` $ rosrun dragon transformation_demo.py  _mode:=2```
   - normal pose: ``` $ rosrun dragon transformation_demo.py  _reset:=1```
   - reverse normal pose: ``` $ rosrun dragon transformation_demo.py  _reverse_reset:=1```
