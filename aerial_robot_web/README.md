# aerial_robot_web

`aerial_robot_web` is a mobile-first ROS web console shared by all aerial robot bringup launches.
It serves a static React single-page application and starts `rosbridge_server` so phones, tablets,
or a laptop browser can inspect the ROS graph without RViz/rqt.

## Launch

Install the runtime dependencies before launching the console:

```bash
sudo apt install ros-one-rosbridge-server python3-rospkg python3-qrcode
```

If installing QR support with pip instead of apt, use the Python package name:

```bash
python3 -m pip install --user qrcode
```

Required packages:

- `ros-one-rosbridge-server` provides `rosbridge_server`, `rosbridge_websocket`, and `rosapi`.
- `python3-rospkg` is used by the web server to resolve `package://` mesh resources.
- `python3-qrcode` (apt) / `qrcode` (pip) is used to print a terminal QR code for the
  phone/LAN URL.

Each robot `bringup.launch` can include this package, but it is disabled by default; enable it
explicitly:

```bash
roslaunch hydrus bringup.launch launch_web_console:=true
```

When enabled, the console URL and a QR code are printed near the end of the launch output
(after a short `banner_delay`, default 8 s, so they are not buried by other startup logs):

```text
============================================================
 DRAGON Lab Aerial Robot Web Console
------------------------------------------------------------
 Local browser : http://localhost:8080?robot_ns=hydrus&robot_type=hydrus&rosbridge_port=9090
 Phone / LAN   : http://192.168.0.10:8080?robot_ns=hydrus&robot_type=hydrus&rosbridge_port=9090
------------------------------------------------------------
 Scan this QR code from a phone on the robot network:
...
============================================================
```

The console can also be launched directly:

```bash
roslaunch aerial_robot_web web_console.launch robot_ns:=hydrus robot_type:=hydrus
```

Useful arguments:

- `web_port` (default: `8080`) changes the HTTP port.
- `rosbridge_port` (default: `9090`) changes the websocket port.
- `pose_topic` (default: `/<robot_ns>/ground_truth`) sets the `nav_msgs/Odometry` topic used
  to move the whole URDF model in the viewer. For estimator output, use a topic such as
  `/<robot_ns>/uav/baselink/odom`.
- `launch_rosbridge` (default: `true`) can be disabled when another rosbridge is already running.
- `rosbridge_output` (default: `log`) changes rosbridge/rosapi output; set it to `screen`
  when debugging rosbridge itself.
- `auto_install_qr_dependency` (default: `true`) tries `sudo -n apt-get install -y python3-qrcode`
  first, then `python3 -m pip install --user qrcode` when QR support is missing. It never waits
  for a sudo password; if automatic installation is not allowed, install it manually with apt or pip.
- `banner_delay` (default: `8.0`) is how many seconds the URL/QR banner is delayed so it appears
  near the end of the roslaunch startup output.

If roslaunch reports that `rosbridge_server` or `rosbridge_websocket` cannot be found, install
`ros-one-rosbridge-server` and source the ROS environment again before relaunching.

## Features

- Live node/topic lists using `rosapi`, with case-insensitive filtering.
- Per-node publications, subscriptions, and services.
- Per-topic type, publishers, subscribers, and a throttled live message preview.
- A publish box below the topic info that pre-fills a JSON template generated from the
  topic's message type (via `rosapi/message_details`) and publishes it through rosbridge.
- Touch-friendly URDF viewer that mirrors live `/joint_states`, reframes the camera once
  meshes finish loading, and reports mesh load failures.
- `package://` mesh resources are served by the console HTTP server under `/pkg/<package>/<path>`.
- Automatic rosbridge reconnection every few seconds after a dropped or failed connection.

## Limitations

- React, roslib, and Three.js are loaded from a CDN, so the browser needs internet access at
  least once to cache them. On a fully offline robot LAN the console shows a static notice
  instead of the interface.
