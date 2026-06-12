# aerial_robot_web

`aerial_robot_web` is a mobile-first ROS web console shared by all aerial robot bringup launches.
It serves a static React single-page application and starts `rosbridge_server` so phones, tablets,
or a laptop browser can inspect the ROS graph without RViz/rqt.

## Launch

Each robot `bringup.launch` includes this package by default and prints a URL similar to:

```text
Aerial Robot Web Console ready: http://localhost:8080?robot_ns=dracomancer&robot_type=dracomancer&rosbridge_port=9090
```

The console can also be launched directly:

```bash
roslaunch aerial_robot_web web_console.launch robot_ns:=dracomancer robot_type:=dracomancer
```

Useful arguments:

- `web_port` (default: `8080`) changes the HTTP port.
- `rosbridge_port` (default: `9090`) changes the websocket port.
- `launch_rosbridge` (default: `true`) can be disabled when another rosbridge is already running.

## Features

- Live node/topic lists using `rosapi`, with case-insensitive filtering.
- Per-node publications, subscriptions, and services.
- Per-topic type, publishers, subscribers, and a throttled live message preview.
- Dracomancer joint sliders generated from `robot_description` (mimic joints excluded) and
  published as `sensor_msgs/JointState`, throttled, with sync-from-robot and reset-to-zero buttons.
- Touch-friendly URDF viewer that mirrors live `/joint_states` as well as the sliders,
  reframes the camera once meshes finish loading, and reports mesh load failures.
- `package://` mesh resources are served by the console HTTP server under `/pkg/<package>/<path>`.
- Automatic rosbridge reconnection every few seconds after a dropped or failed connection.

## Limitations

- React, roslib, and Three.js are loaded from a CDN, so the browser needs internet access at
  least once to cache them. On a fully offline robot LAN the console shows a static notice
  instead of the interface.
