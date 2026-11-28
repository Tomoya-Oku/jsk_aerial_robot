#!/usr/bin/env python3
"""Verify the standard model, meshes and joint table against the CAD baseline."""
from pathlib import Path
import csv
import hashlib
import json
import math
import xml.etree.ElementTree as ET

package = Path(__file__).parents[2]
metadata = json.loads((package / 'config/model/cad.json').read_text())
cad_file = package / 'urdf/dracomancer.urdf'
source = ET.parse(cad_file).getroot()
model = ET.parse(package / 'urdf/dracomancer.urdf.xacro').getroot()
links = {link.get('name'): link for link in source.findall('link')}
joints = {joint.get('name'): joint for joint in source.findall('joint')}
with (package / 'config/model/joint_mapping.csv').open(encoding='utf-8-sig', newline='') as stream:
    joint_table = {row['joint']: row for row in csv.DictReader(stream)}

assert len(links) == metadata['links'] == 15
assert set(links) | {'world'} == {link.get('name') for link in model.findall('link')}
assert len(joints) == metadata['continuous_joints'] == 14
assert set(joint_table) == set(joints)
assert {joint.get('name') for joint in model.findall('transmission/joint')} == set(joints)
assert model.find("joint[@name='mount'][@type='fixed']/child").get('link') == 'base_link'
assert 'mannequin' not in links


def visual_origin(link):
    origin = link.find('visual/origin')
    return dict(xyz='0 0 0', rpy='0 0 0') if origin is None else origin.attrib


def mesh_path(link):
    filename = link.find('visual/geometry/mesh').get('filename')
    return filename.removeprefix('package://dracomancer/')


referenced = set()
for name, cad in links.items():
    standard = model.find(f"link[@name='{name}']")
    assert ET.tostring(cad.find('inertial')) == ET.tostring(standard.find('inertial'))
    assert visual_origin(cad) == visual_origin(standard)
    file = mesh_path(cad)
    assert file == mesh_path(standard)
    assert cad.find('collision/geometry/mesh').get('filename') == cad.find('visual/geometry/mesh').get('filename')
    referenced.add(file)
    bounds = metadata['mesh_stats'][file]
    dimensions = map(float, standard.find('collision/geometry/box').get('size').split())
    for size, low, high in zip(dimensions, bounds['min_m'], bounds['max_m']):
        assert size > 0 and math.isclose(size, high-low, rel_tol=1e-9), name

files = {mesh.relative_to(package).as_posix() for mesh in (package/'meshes').rglob('*.stl')}
assert files == referenced == set(metadata['mesh_stats'])
assert len(files) == metadata['unique_meshes'] == 10
for file in files:
    assert hashlib.sha256((package/file).read_bytes()).hexdigest() == metadata['mesh_stats'][file]['sha256']

for name, cad in joints.items():
    standard = model.find(f"joint[@name='{name}']")
    assert standard.get('type') == cad.get('type') == 'continuous'
    for tag in ('origin', 'parent', 'child', 'axis'):
        assert cad.find(tag).attrib == standard.find(tag).attrib
    row = joint_table[name]
    assert row['parent'] == cad.find('parent').get('link')
    assert row['child'] == cad.find('child').get('link')
    assert row['mesh'] == mesh_path(links[row['child']])
    assert [float(row[f'axis {axis}']) for axis in 'xyz'] == list(map(float, cad.find('axis').get('xyz').split()))

assert hashlib.sha256(cad_file.read_bytes()).hexdigest() == metadata['baseline_urdf_sha256']
print('PASS: 15 CAD links, 14 joint frames and axes, 14 transmissions, 10 shared meshes, collision bounds')
