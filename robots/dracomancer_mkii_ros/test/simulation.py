#!/usr/bin/env python3
"""Integration test against a running, isolated gazebo.launch (no hardware)."""
import importlib.util
import json
import math
import os
from pathlib import Path
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ['ROS_NAMESPACE'] = '/dracomancer_mkii'
import rospy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from std_srvs.srv import SetBool, Trigger, Empty

print('Initializing test node', flush=True)
rospy.init_node('mkii_test', disable_signals=True)
unpaced = os.environ.get('MKII_TEST_UNPACED') == '1'
if unpaced:
    # Optional CI/WSL profile: avoid wall-clock pacing, keep dt/gravity/ODE unchanged.
    rospy.wait_for_service('/gazebo/get_physics_properties', timeout=120)
    physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)()
    assert physics.success, physics.status_message
    changed = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)(
        physics.time_step, 0.0, physics.gravity, physics.ode_config)
    assert changed.success, changed.status_message
print('Waiting for control services', flush=True)
latest, commands = {}, {}


def state(msg):
    latest['joint'] = msg


def status(msg):
    latest['status'] = json.loads(msg.data)


def until(predicate, timeout=8):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.03)
    raise AssertionError('Timed out waiting for '+str(predicate))


subs = [rospy.Subscriber('joint_states', JointState, state),
        rospy.Subscriber('control/status', String, status),
        rospy.Subscriber('right_joint_7_effort/command', Float64, lambda m: commands.update(effort=m.data))]
rospy.wait_for_service('control/right_joint_7/torque_mode', timeout=120)
print('Control services ready', flush=True)
until(lambda: 'joint' in latest and 'status' in latest)
assert len(latest['joint'].name) == 14
names = list(latest['joint'].name)
angle = rospy.Publisher('joint_cmd', JointState, queue_size=1)
torque = rospy.Publisher('torque_cmd', JointState, queue_size=1)
until(lambda: angle.get_num_connections() > 0 and torque.get_num_connections() > 0)
target = 'right_joint_7'


def q():
    m = latest['joint']
    return m.position[m.name.index(target)]


msg = JointState(name=names, position=[0.25 if n == target else 0.0 for n in names])
for _ in range(30):
    angle.publish(msg)
    time.sleep(0.05)
until(lambda: abs(q()-0.25) < 0.07)
print('Position response verified', flush=True)
position_error = abs(q()-0.25)
mode = rospy.ServiceProxy('control/'+target+'/torque_mode', SetBool)


def set_mode(enabled):
    # Wait for genuine feedback after startup/unpause. Never bypass the controller's guard.
    deadline = time.monotonic()+15
    while time.monotonic() < deadline:
        response = mode(enabled)
        if response.success:
            return response
        if response.message != 'Joint feedback is stale or missing':
            raise AssertionError(response.message)
        time.sleep(0.05)
    raise AssertionError('Feedback did not recover within 15 wall-clock seconds')

set_mode(True)
print('Torque mode enabled', flush=True)
listing = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
controllers = {c.name: c.state for c in listing().controller}
assert controllers[target+'_effort'] == 'running'
assert controllers[target+'_position'] == 'stopped'
before = q()
for _ in range(12):
    torque.publish(JointState(name=[target], effort=[0.1]))
    time.sleep(0.05)
until(lambda: abs(latest['status']['commanded_torque'][target]-0.1) < 1e-8)
motion = abs(q()-before)
assert motion > 0.01, motion
time.sleep(0.6)
until(lambda: latest['status']['commanded_torque'][target] == 0.0)
until(lambda: commands.get('effort') == 0.0)

# A paused simulation must not pause the watchdog.
for _ in range(3):
    torque.publish(JointState(name=[target], effort=[0.1]))
    time.sleep(0.05)
rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
time.sleep(0.6)
assert latest['status']['commanded_torque'][target] == 0.0
assert commands['effort'] == 0.0
assert not mode(False).success, 'Stale state should refuse mode change'
rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
time.sleep(0.3)

# Out-of-range, NaN and malformed packets zero the last effort.
for bad in (JointState(name=[target], effort=[2.1]),
            JointState(name=[target], effort=[float('nan')]),
            JointState(name=[target], effort=[])):
    torque.publish(bad)
    time.sleep(0.1)
    assert latest['status']['commanded_torque'][target] == 0.0
set_mode(False)
controllers = {c.name: c.state for c in listing().controller}
assert controllers[target+'_position'] == 'running'
assert controllers[target+'_effort'] == 'stopped'

# Real Qt widget, actual ROS services and publishers, with offscreen rendering.
from python_qt_binding.QtWidgets import QApplication
panel_file = Path(__file__).parents[1] / 'scripts/panel.py'
spec = importlib.util.spec_from_file_location('mkii_panel', panel_file)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
app = QApplication([])
panel = module.Panel()
panel.show()


def events(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.01)


events(0.6)
assert len(panel.angles.joint_map) == 14
panel.angles.joint_map[target]['slider'].setValue(5500)
events(1.0)
assert abs(q()-math.pi*0.1) < 0.09, q()
panel.checks[target].setChecked(True)
events(0.5)
assert panel.mode[target]
assert not panel.angles.joint_map[target]['slider'].isEnabled()
panel.values[target].setValue(-0.08)
panel.apply.setDown(True)
events(0.4)
assert latest['status']['commanded_torque'][target] == -0.08
panel.apply.setDown(False)
panel.zero()
events(0.2)
assert latest['status']['commanded_torque'][target] == 0.0
out = Path(os.environ['MKII_RESULTS'])
panel.grab().save(str(out.parent / 'panel-test.png'))
result = {'joints': len(names), 'position_error_rad': position_error,
          'effort_motion_rad': motion, 'exclusive_controller_switch': True,
          'watchdog_wall_clock': True, 'paused_watchdog': True,
          'invalid_commands_rejected': True, 'qt_angle_slider': True,
          'qt_torque_mode_and_nm': True, 'qt_zero': True}
result['unpaced_physics'] = unpaced
out.write_text(json.dumps(result, indent=2))
panel.close()
print(json.dumps(result))
