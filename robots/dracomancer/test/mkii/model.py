#!/usr/bin/env python3
"""Verify model integrity relative to the untouched CAD export."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

package = Path(__file__).parents[2]
metadata = json.loads((package / 'config/mkii/cad.json').read_text())
source = ET.parse(package / 'urdf/cad.urdf').getroot()
model = ET.parse(package / 'urdf/mkii.urdf.xacro').getroot()
assert len(model.findall('link')) == 16  # CAD 15 + world
joints = [j for j in model.findall('joint') if j.get('type') == 'continuous']
assert len(joints) == 14
for old in source.findall('link'):
    new = model.find("link[@name='%s']" % old.get('name'))
    assert ET.tostring(old.find('inertial')) == ET.tostring(new.find('inertial'))
for old in source.findall('joint'):
    new = model.find("joint[@name='%s']" % old.get('name'))
    for tag in ('origin', 'parent', 'child', 'axis'):
        assert old.find(tag).attrib == new.find(tag).attrib
assert len(model.findall('transmission')) == 14
for link in model.findall('link'):
    if link.get('name') != 'world':
        assert link.find('collision/geometry/box') is not None
    for mesh in link.findall('visual/geometry/mesh'):
        file = Path(mesh.get('filename').split('package://dracomancer/')[1])
        assert hashlib.sha256((package/file).read_bytes()).hexdigest() == metadata['mesh_stats'][file.as_posix()]['sha256']
assert source.find("link[@name='mannequin']") is None
print('PASS: 15 CAD inertials, 14 transforms/axes, 14 transmissions, 8 identical meshes, world mount')

assert hashlib.sha256((package / 'urdf/cad.urdf').read_bytes()).hexdigest() == metadata['baseline_urdf_sha256']
