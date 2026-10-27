#!/usr/bin/env python3
"""Spawn the configured fixed wall into the existing Gazebo ODE world."""
import math
import xml.etree.ElementTree as ET
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def wall_sdf(position, size, friction):
    if (len(position) != 3 or len(size) != 3 or
            not all(math.isfinite(float(v)) for v in list(position) + list(size)) or
            any(float(v) <= 0 for v in size) or not math.isfinite(friction) or friction < 0):
        raise ValueError("invalid wall geometry/friction")
    root = ET.Element("sdf", version="1.6")
    model = ET.SubElement(root, "model", name="contact_wall")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(map(str, position)) + " 0 0 0"
    link = ET.SubElement(model, "link", name="wall")
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, name=tag)
        geom = ET.SubElement(item, "geometry")
        ET.SubElement(ET.SubElement(geom, "box"), "size").text = " ".join(map(str, size))
        if tag == "collision":
            ode = ET.SubElement(ET.SubElement(ET.SubElement(item, "surface"), "friction"), "ode")
            for key in ("mu", "mu2"):
                ET.SubElement(ode, key).text = str(friction)
    sensor = ET.SubElement(link, "sensor", name="wall_contacts", type="contact")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = "collision"
    plugin = ET.SubElement(sensor, "plugin", name="wall_contact", filename="libwall_contact.so")
    ET.SubElement(plugin, "robot").text = "dragon"
    ET.SubElement(plugin, "link").text = "link1"
    return ET.tostring(root, encoding="unicode")


if __name__ == "__main__":
    rospy.init_node("contact_wall")
    settings = rospy.get_param("/dracomancer/contact/wall")
    xml = wall_sdf(settings["position"], settings["size"], float(settings["friction"]))
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    initial = Pose()
    initial.orientation.w = 1.0
    response = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)(
        "contact_wall", xml, "", initial, "world")
    if not response.success:
        raise RuntimeError(response.status_message)
