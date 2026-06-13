#!/usr/bin/env python3
"""Serve the aerial robot web console as a ROS node."""

import datetime
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import posixpath
import re
import signal
import subprocess
import socket
import sys
import threading
import time
from urllib.parse import urlencode, unquote, urlparse

import rospy
import rospkg
from std_msgs.msg import Empty as EmptyMsg, String as StringMsg


ROSPACK = rospkg.RosPack()


def resolve_package_resource(url_path):
    """Map /pkg/<package>/<relpath> to a file inside the package share directory.

    Returns None when the package is unknown or the path escapes the package.
    """
    parts = [unquote(part) for part in posixpath.normpath(url_path).split("/") if part]
    if len(parts) < 3 or parts[0] != "pkg":
        return None
    try:
        package_root = os.path.realpath(ROSPACK.get_path(parts[1]))
    except rospkg.ResourceNotFound:
        return None
    candidate = os.path.realpath(os.path.join(package_root, *parts[2:]))
    if candidate != package_root and not candidate.startswith(package_root + os.sep):
        return None
    return candidate


class ConsoleHandler(SimpleHTTPRequestHandler):
    """Static file handler with SPA fallback and package:// mesh serving."""

    def end_headers(self):
        if self.path.startswith("/pkg/"):
            self.send_header("Cache-Control", "max-age=3600")
        else:
            self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        rospy.logdebug("web console: " + fmt, *args)

    def translate_path(self, path):
        url_path = urlparse(path).path
        if url_path.startswith("/pkg/"):
            resolved = resolve_package_resource(url_path)
            if resolved:
                return resolved
            # Force a 404 instead of falling back to the SPA page.
            return os.path.join(self.directory, "pkg-not-found")
        return super().translate_path(path)

    def send_head(self):
        path = urlparse(self.path).path
        if path not in ("/", "/index.html") and not path.startswith("/pkg/"):
            candidate = os.path.join(self.directory, path.lstrip("/"))
            # Only application routes (no file extension) fall back to the SPA
            # page; missing assets must return 404 so loaders can detect them.
            if not os.path.exists(candidate) and "." not in posixpath.basename(path):
                self.path = "/index.html"
        return super().send_head()


def get_host_label(configured_host):
    if configured_host not in ("", "0.0.0.0", "::"):
        return configured_host
    lan_host = get_lan_host()
    if lan_host:
        return lan_host
    try:
        return socket.gethostname() or "localhost"
    except socket.error:
        return "localhost"


def get_lan_host():
    """Return the outbound LAN address when the robot is on a network."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        address = probe.getsockname()[0]
        if address and not address.startswith("127."):
            return address
    except socket.error:
        return None
    finally:
        probe.close()
    return None


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() not in ("0", "false", "no", "off")
    return bool(value)


def ns_join(ns, name):
    clean_name = name[1:] if name.startswith("/") else name
    if not ns:
        return "/" + clean_name
    clean_ns = ns if ns.startswith("/") else "/" + ns
    return clean_ns.rstrip("/") + "/" + clean_name


def import_qrcode(auto_install):
    try:
        import qrcode
        return qrcode
    except ImportError:
        pass

    if not auto_install:
        return None

    install_commands = [
        ("apt", ["sudo", "-n", "apt-get", "install", "-y", "python3-qrcode"]),
        ("pip", [sys.executable, "-m", "pip", "install", "--user", "qrcode"]),
    ]
    rospy.logwarn("[aerial_robot_web] QR module is missing; apt package is python3-qrcode, pip package is qrcode.")
    for label, install_cmd in install_commands:
        rospy.logwarn("[aerial_robot_web] Trying QR dependency install via %s: %s", label, " ".join(install_cmd))
        try:
            result = subprocess.run(
                install_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=120,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            rospy.logwarn("[aerial_robot_web] QR dependency install via %s failed: %s", label, error)
            continue
        if result.returncode != 0:
            rospy.logwarn("[aerial_robot_web] QR dependency install via %s failed with code %s", label, result.returncode)
            if result.stdout:
                rospy.logdebug("[aerial_robot_web] %s output:\n%s", label, result.stdout)
            continue
        try:
            import qrcode
            rospy.loginfo("[aerial_robot_web] Installed QR dependency automatically via %s.", label)
            return qrcode
        except ImportError:
            rospy.logwarn("[aerial_robot_web] QR module still cannot be imported after %s install.", label)
    return None


def format_qr(url, auto_install):
    qrcode = import_qrcode(auto_install)
    if not qrcode:
        return None
    qr = qrcode.QRCode(border=1)
    qr.add_data(url)
    qr.make(fit=True)
    matrix = qr.get_matrix()
    if len(matrix) % 2:
        matrix.append([False] * len(matrix[0]))
    # Half-block rendering packs two QR rows into one terminal line, so the
    # printed code is a quarter of the plain "██" rendering.
    glyphs = {
        (False, False): " ",
        (True, False): "▀",
        (False, True): "▄",
        (True, True): "█",
    }
    lines = []
    for top, bottom in zip(matrix[0::2], matrix[1::2]):
        lines.append("".join(glyphs[(top[i], bottom[i])] for i in range(len(top))))
    return "\n".join(lines)


def print_console_banner(localhost_url, host_url, web_root, bind_host, auto_install_qr):
    qr = format_qr(host_url, auto_install_qr)
    lines = [
        "",
        "============================================================",
        " DRAGON Lab Aerial Robot Web Console",
        "------------------------------------------------------------",
        " Local browser : {}".format(localhost_url),
        " Phone / LAN   : {}".format(host_url),
        " Bind address  : {}".format(bind_host or "0.0.0.0"),
        " Web root      : {}".format(web_root),
    ]
    if qr:
        lines.extend([
            "------------------------------------------------------------",
            " Scan this QR code from a phone on the robot network:",
            qr,
        ])
    else:
        lines.extend([
            "------------------------------------------------------------",
            " QR code unavailable.",
            " Install manually with apt: sudo apt install python3-qrcode",
            " Or with pip: python3 -m pip install --user qrcode",
        ])
    lines.append("============================================================")
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


# Only plain ROS graph names may reach the rosbag command line; this also
# rejects anything that could be parsed as an extra rosbag option.
TOPIC_NAME_RE = re.compile(r"^/[A-Za-z0-9_][A-Za-z0-9_/]*$")


class RosbagRecorder:
    """Drive `rosbag record` from the web console over plain topics.

    The browser publishes std_msgs/String on /aerial_robot_web/rosbag/start
    with JSON {"all": bool, "topics": [...], "bag_dir": "..."} and
    std_msgs/Empty on .../stop; the current state is mirrored on the latched
    .../status topic as JSON.
    """

    def __init__(self, bag_dir):
        self.bag_dir = bag_dir
        self.proc = None
        self.bag_path = None
        self.topics = []
        self.record_all = False
        self.error = ""
        self.lock = threading.Lock()
        self.status_pub = rospy.Publisher(
            "/aerial_robot_web/rosbag/status", StringMsg, queue_size=1, latch=True)
        rospy.Subscriber("/aerial_robot_web/rosbag/start", StringMsg, self.handle_start)
        rospy.Subscriber("/aerial_robot_web/rosbag/stop", EmptyMsg, self.handle_stop)
        self.publish_status()
        monitor = threading.Thread(target=self.watch_process)
        monitor.daemon = True
        monitor.start()

    def recording(self):
        return self.proc is not None and self.proc.poll() is None

    def publish_status(self):
        payload = {
            "recording": self.recording(),
            "bag_path": self.bag_path or "",
            "topics": self.topics,
            "all": self.record_all,
            "error": self.error,
        }
        self.status_pub.publish(StringMsg(data=json.dumps(payload)))

    def watch_process(self):
        """Publish the final status once rosbag exits (stop or crash)."""
        while not rospy.is_shutdown():
            with self.lock:
                if self.proc is not None and self.proc.poll() is not None:
                    if self.proc.returncode not in (0, -signal.SIGINT):
                        self.error = "rosbag exited with code {}".format(self.proc.returncode)
                        rospy.logwarn("[aerial_robot_web] %s", self.error)
                    self.proc = None
                    self.publish_status()
            time.sleep(0.5)

    def handle_start(self, msg):
        try:
            request = json.loads(msg.data)
        except ValueError:
            request = {"topics": msg.data.split()}
        if not isinstance(request, dict):
            request = {}
        record_all = bool(request.get("all"))
        requested = request.get("topics", [])
        topics = [t for t in requested if isinstance(t, str) and TOPIC_NAME_RE.match(t)]
        bag_dir = request.get("bag_dir") or ""
        bag_dir = os.path.expanduser(bag_dir) if isinstance(bag_dir, str) else ""
        with self.lock:
            self.error = ""
            if self.recording():
                self.error = "already recording"
            elif not record_all and not topics:
                self.error = "no valid topics requested"
            elif bag_dir and not os.path.isabs(bag_dir):
                self.error = "bag folder must be an absolute path"
            else:
                target_dir = bag_dir or self.bag_dir
                try:
                    os.makedirs(target_dir, exist_ok=True)
                    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    self.bag_path = os.path.join(target_dir, "web_console_{}.bag".format(stamp))
                    args = ["-a"] if record_all else topics
                    self.proc = subprocess.Popen(["rosbag", "record", "-O", self.bag_path] + args)
                    self.topics = [] if record_all else topics
                    self.record_all = record_all
                    rospy.loginfo("[aerial_robot_web] rosbag record started: %s (%s)",
                                  self.bag_path, "all topics" if record_all else "{} topics".format(len(topics)))
                except OSError as error:
                    self.error = "failed to start rosbag: {}".format(error)
            if self.error:
                rospy.logwarn("[aerial_robot_web] rosbag start rejected: %s", self.error)
            self.publish_status()

    def handle_stop(self, _msg):
        with self.lock:
            if not self.recording():
                return
            rospy.loginfo("[aerial_robot_web] stopping rosbag record (%s)", self.bag_path)
            # SIGINT lets rosbag close the bag file cleanly; watch_process
            # publishes the final status once it exits.
            self.proc.send_signal(signal.SIGINT)

    def shutdown(self):
        with self.lock:
            if not self.recording():
                return
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def main():
    rospy.init_node("aerial_robot_web_server")
    host = rospy.get_param("~host", "")
    port = int(rospy.get_param("~port", 8080))
    rosbridge_port = int(rospy.get_param("~rosbridge_port", 9090))
    auto_install_qr = get_bool_param("~auto_install_qr_dependency", True)
    banner_delay = float(rospy.get_param("~banner_delay", 8.0))
    robot_ns = rospy.get_param("~robot_ns", "")
    robot_type = rospy.get_param("~robot_type", "generic")
    pose_topic = rospy.get_param("~pose_topic", "") or ns_join(robot_ns, "ground_truth")
    package_path = ROSPACK.get_path("aerial_robot_web")
    web_root = rospy.get_param("~web_root", os.path.join(package_path, "www"))
    rosbag_dir = rospy.get_param("~rosbag_dir", os.path.expanduser("~/rosbags"))

    rospy.loginfo("[aerial_robot_web] Starting web console")
    rospy.loginfo("[aerial_robot_web] robot_type=%s robot_ns=%s", robot_type, robot_ns or "/")
    rospy.loginfo("[aerial_robot_web] odometry pose topic=%s", pose_topic)
    rospy.loginfo("[aerial_robot_web] HTTP port=%s rosbridge_port=%s", port, rosbridge_port)

    recorder = RosbagRecorder(rosbag_dir)

    handler = partial(ConsoleHandler, directory=web_root)
    bind_host = "" if host in ("0.0.0.0", "::") else host
    httpd = ThreadingHTTPServer((bind_host, port), handler)
    thread = threading.Thread(target=httpd.serve_forever)
    thread.daemon = True
    thread.start()

    query = "?" + urlencode({
        "robot_ns": robot_ns,
        "robot_type": robot_type,
        "rosbridge_port": rosbridge_port,
        "pose_topic": pose_topic,
    })
    localhost_url = "http://localhost:{}{}".format(port, query)
    host_url = "http://{}:{}{}".format(get_host_label(host), port, query)
    rospy.loginfo("[aerial_robot_web] Local URL: %s", localhost_url)
    rospy.loginfo("[aerial_robot_web] Phone/LAN URL: %s", host_url)
    rospy.loginfo("[aerial_robot_web] ROS bridge URL used by browser: ws://<browser-host>:%s", rosbridge_port)

    # Delay the banner so the URL and QR code land near the end of the
    # roslaunch startup output instead of being buried by other nodes' logs.
    def banner():
        if rospy.is_shutdown():
            return
        print_console_banner(localhost_url, host_url, web_root, bind_host, auto_install_qr)

    banner_timer = threading.Timer(max(banner_delay, 0.0), banner)
    banner_timer.daemon = True
    banner_timer.start()

    def shutdown():
        banner_timer.cancel()
        recorder.shutdown()
        httpd.shutdown()

    rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
