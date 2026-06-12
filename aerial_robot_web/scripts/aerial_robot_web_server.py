#!/usr/bin/env python3
"""Serve the aerial robot web console as a ROS node."""

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os
import posixpath
import socket
import threading
from urllib.parse import unquote, urlparse

import rospy
import rospkg


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
    try:
        return socket.gethostname() or "localhost"
    except socket.error:
        return "localhost"


def main():
    rospy.init_node("aerial_robot_web_server")
    host = rospy.get_param("~host", "")
    port = int(rospy.get_param("~port", 8080))
    rosbridge_port = int(rospy.get_param("~rosbridge_port", 9090))
    robot_ns = rospy.get_param("~robot_ns", "")
    robot_type = rospy.get_param("~robot_type", "generic")
    package_path = ROSPACK.get_path("aerial_robot_web")
    web_root = rospy.get_param("~web_root", os.path.join(package_path, "www"))

    handler = partial(ConsoleHandler, directory=web_root)
    bind_host = "" if host in ("0.0.0.0", "::") else host
    httpd = ThreadingHTTPServer((bind_host, port), handler)
    thread = threading.Thread(target=httpd.serve_forever)
    thread.daemon = True
    thread.start()

    query = "?robot_ns={}&robot_type={}&rosbridge_port={}".format(robot_ns, robot_type, rosbridge_port)
    localhost_url = "http://localhost:{}{}".format(port, query)
    host_url = "http://{}:{}{}".format(get_host_label(host), port, query)
    rospy.loginfo("Aerial Robot Web Console ready: %s", localhost_url)
    rospy.loginfo("Aerial Robot Web Console LAN hint: %s", host_url)

    rospy.on_shutdown(httpd.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
