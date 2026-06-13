# aerial_robot_web

**DRAGON Lab Aerial Robot Web Console** — a mobile-first web UI shared by all aerial robot
bringup launches. It serves a static React single-page application from a ROS node and talks
to the robot through `rosbridge_server`, so a phone, tablet, or laptop browser can inspect
the ROS graph, watch the robot model move, record rosbags, and send teleop commands —
**no app install, no pip, just `roslaunch` and a QR code**.

![Web console overview](docs/web_console.png)

## Quick start

```bash
# one-time dependency install (all available via apt / rosdep, no pip required)
sudo apt install ros-one-rosbridge-server python3-rospkg python3-qrcode
```

Enable the console from any robot bringup (it is **disabled by default**):

```bash
roslaunch dragon bringup.launch web:=true
```

The longer `launch_web_console:=true` argument is still accepted for compatibility.

or launch it standalone:

```bash
roslaunch aerial_robot_web web_console.launch robot_ns:=dragon robot_type:=dragon
```

Near the end of the launch output (after `banner_delay`, default 8 s) the console prints its
URLs and a QR code — scan it from a phone on the robot network:

```text
============================================================
 DRAGON Lab Aerial Robot Web Console
------------------------------------------------------------
 Local browser : http://localhost:8080?robot_ns=dragon&...
 Phone / LAN   : http://192.168.0.10:8080?robot_ns=dragon&...
------------------------------------------------------------
 Scan this QR code from a phone on the robot network:
   ▄▄▄▄▄▄▄ ▄  ▄▄ ▄▄▄▄▄▄▄
   █ ▄▄▄ █ ▀▄█▀▄ █ ▄▄▄ █   (compact half-block QR)
   █▄▄▄▄▄█ ▄▀▄ ▀ █▄▄▄▄▄█
============================================================
```

## UI tour

### Graph inspection & topic publishing

![Topic info and publish box](docs/topic_info_publish.png)

- **Nodes / Topics** cards list the live graph via `rosapi`, each with its own
  case-insensitive filter and a collapse toggle (`▾` / `▸`) that shrinks the card to its
  title bar.
- Selecting a topic shows its **type, publishers, subscribers, and a throttled live message
  preview** in the Info card; selecting a node shows its publications, subscriptions, and
  services.
- The **Publish box** below the topic info is pre-filled with a JSON template generated from
  the topic's message type (`rosapi/message_details`), so you can edit values and publish
  through rosbridge immediately.

### Rosbag recording

![Rosbag settings popup](docs/rosbag_settings.png)

- **● Record rosbag** starts `rosbag record -a` on the robot right away; the button turns
  into **■ Stop recording** while a bag is being written.
- The **⚙ gear** opens the settings popup where you can switch from *all topics* to a
  checked subset (with filter / select-shown / clear helpers) and set the **bag folder on
  the robot**. Settings persist in the browser (localStorage).
- Bags are saved as `web_console_<timestamp>.bag` under `rosbag_dir`
  (default `$HOME/rosbags`, falling back to `/tmp/rosbags` when `HOME` is unset).
  Control flows over `/aerial_robot_web/rosbag/{start,stop,status}`; the latched status
  topic restores the recording state after a page reload.

### Live robot model & flight control

The **Live Robot Model (URDF + Odometry)** panel renders the robot URDF with Three.js,
driven by the live `joint_states` and the odometry pose topic — a lightweight Gazebo-style
pose viewer in the browser (touch-drag to orbit). `package://` meshes are served by the
console itself under `/pkg/<package>/<path>`.

The **Flight Control** pad next to the viewer mirrors
`aerial_robot_base/scripts/keyboard_command.py`:

| Button | Topic | Message |
| --- | --- | --- |
| Arm / Takeoff / Land / F.Land / Halt | `<robot_ns>/teleop_command/{start,takeoff,land,force_landing,halt}` | `std_msgs/Empty` |
| ↑↓←→ / ▲▼ / ↺↻ velocity nudges (0.2 m/s, 0.2 rad/s) | `<robot_ns>/uav/nav` | `aerial_robot_msgs/FlightNav` (VEL_MODE) |

### Mobile layout

<img src="docs/web_console_mobile.png" width="340" alt="Mobile layout" />

The layout collapses to a single column on phones, with touch-sized buttons and
safe-area-aware padding.

## Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `robot_ns` | `""` | Robot namespace shown in the UI and used for teleop/URDF topics |
| `robot_type` | `generic` | Robot label shown in the header |
| `pose_topic` | `/<robot_ns>/ground_truth` | `nav_msgs/Odometry` topic that moves the URDF model. For estimator output use e.g. `/<robot_ns>/uav/baselink/odom` |
| `web_port` | `8080` | HTTP port of the console |
| `rosbridge_port` | `9090` | rosbridge websocket port |
| `launch_rosbridge` | `true` | Disable when another rosbridge is already running |
| `rosbridge_output` | `log` | Set to `screen` when debugging rosbridge itself |
| `auto_install_qr_dependency` | `true` | Try `sudo -n apt-get install python3-qrcode`, then `pip --user`, when QR support is missing (never waits for a sudo password; QR is optional either way) |
| `banner_delay` | `8.0` | Seconds to delay the URL/QR banner so it lands near the end of the launch output |
| `rosbag_dir` | `$(optenv HOME /tmp)/rosbags` | Where browser-triggered rosbags are saved |

## Architecture

```mermaid
flowchart LR
    subgraph Browser["Browser (phone / laptop)"]
        UI["React SPA<br/>app.js + urdf-viewer.js"]
    end
    subgraph Robot["Robot PC (roslaunch)"]
        HTTP["aerial_robot_web_server.py<br/>static files + /pkg meshes + QR banner"]
        RB["rosbridge_websocket + rosapi"]
        BAG["rosbag record (subprocess)"]
        ROS[("ROS graph")]
    end
    UI -- "HTTP :8080" --> HTTP
    UI -- "WebSocket :9090" --> RB
    RB --- ROS
    HTTP -- "/aerial_robot_web/rosbag/*" --> BAG
```

## Limitations

- React, roslib, and Three.js are loaded from a CDN, so the **browser** needs internet
  access at least once to cache them (the robot itself does not). On a fully offline robot
  LAN the console shows a static notice instead of the interface.
- The terminal QR code needs `python3-qrcode`; without it the console still works and only
  the QR is skipped.
