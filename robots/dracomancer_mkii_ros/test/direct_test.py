#!/usr/bin/env python3
"""Real ROS/Qt adapter test against a fake Spinal; never a serial device."""
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
import subprocess
import signal
os.environ['ROS_NAMESPACE'] = '/dracomancer_mkii'
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from spinal.msg import DirectServoCurrent, ServoStates, ServoState
from spinal.srv import DirectServoMode, DirectServoModeRequest as Mode, DirectServoModeResponse as Response
from std_srvs.srv import SetBool, Trigger
from python_qt_binding.QtWidgets import QApplication

rospy.init_node('direct_test', disable_signals=True)
PACKAGE = Path(__file__).parents[1]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, PACKAGE/'scripts'/filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def until(test, seconds=4):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        app.processEvents()
        if test():
            return
        time.sleep(0.01)
    raise AssertionError('Timed out waiting for '+str(test))


app = QApplication([])
joint = 'right_joint_1'
cfg = {joint: {'index': 2, 'id': 7, 'direction': -1, 'zero_tick': 2000,
               'radians_per_tick': 0.001, 'joint_nm_per_amp': 2.0,
               'max_current_amp': 0.5, 'max_torque_nm': 0.8}}
rospy.set_param('robot_description', (PACKAGE/'urdf/mkii.urdf.xacro').read_text())
rospy.set_param('joints', ['%s_joint_%d' % (side, i) for side in ('right', 'left') for i in range(1, 8)])
rospy.set_param('~actuators', cfg)
rospy.set_param('~calibrated', True)
rospy.set_param('~mode_service', '/mock/direct_servo_mode')
rospy.set_param('~state_topic', '/mock/servo/states')
rospy.set_param('~current_topic', '/mock/servo/direct_current')
fake = {'token': 0, 'armed': False, 'mode': 3, 'generation': 0, 'raw': 0,
        'sequence': 0, 'last': 0, 'state_on': True, 'delay_arm': False}
calls = []


def service(req):
    assert req.index == 2 and req.servo_id == 7
    calls.append(req.action)
    if req.action == Mode.PREPARE:
        assert req.session != 0
        assert 0 < req.current_limit <= 185
        fake.update(armed=False, mode=0, raw=0, sequence=0)
        fake['generation'] += 1
        fake['token'] = fake['generation']
    elif req.action == Mode.ARM:
        if fake['delay_arm']:
            time.sleep(0.4)
        if req.session != fake['token'] or req.session == 0:
            return Response(success=False, status=2, servo_id=7, model_number=1020)
        fake.update(armed=True, raw=0, last=time.monotonic())
    elif req.action == Mode.STOP:
        fake.update(armed=False, token=0, raw=0)
    elif req.action == Mode.RESTORE:
        fake.update(armed=False, token=0, mode=3, raw=0)
    return Response(success=True, status=0, servo_id=7, model_number=1020,
                    firmware_version=46, operating_mode=fake['mode'], current_limit=1193,
                    torque_enabled=fake['armed'], position=2050, session=fake['token'], timeout_ms=250)


def current(msg):
    for index, token, sequence, raw in zip(msg.index, msg.session, msg.sequence, msg.current):
        assert index == 2
        if fake['armed'] and token == fake['token'] and sequence > fake['sequence']:
            fake.update(sequence=sequence, raw=raw, last=time.monotonic())


srv = rospy.Service('/mock/direct_servo_mode', DirectServoMode, service)
pub = rospy.Publisher('/mock/servo/states', ServoStates, queue_size=1)
sub = rospy.Subscriber('/mock/servo/direct_current', DirectServoCurrent, current)
alive = True


def device():
    while alive and not rospy.is_shutdown():
        if fake['state_on']:
            pub.publish(ServoStates(stamp=rospy.Time.now(), servos=[ServoState(index=2, angle=2050, error=0)]))
        if fake['armed'] and time.monotonic()-fake['last'] >= 0.25:
            fake.update(armed=False, token=0, raw=0)
        time.sleep(0.02)


threading.Thread(target=device, daemon=True).start()
adapter = load('mkii_direct_adapter', 'direct.py').SpinalControl()
panel = load('mkii_direct_panel', 'panel.py').Panel()
panel.show()
until(lambda: panel.checks[joint].isEnabled())
assert not panel.checks['left_joint_1'].isEnabled()
assert not panel.angles.joint_map[joint]['slider'].isEnabled()
until(lambda: abs(adapter.position[joint]+0.05) < 1e-8)
until(lambda: abs(panel.jsp.free_joints[joint]['position']+0.05) < 0.001)
panel.values[joint].setValue(0.538)
panel.checks[joint].setChecked(True)
until(lambda: panel.mode[joint])
assert fake['mode'] == 0 and not fake['armed']
panel.apply.setDown(True)
until(lambda: fake['raw'] == -100)
assert fake['armed'] and adapter.armed[joint]
panel.apply.setDown(False)
panel.zero()
until(lambda: not fake['armed'] and fake['raw'] == 0 and not panel.mode[joint])

# Loss of host command: adapter stops, MCU independently also has a deadline.
assert adapter.change_mode(joint, True).success
command = JointState(name=[joint], effort=[0.538])
command.header.stamp = rospy.Time.now()
adapter.torques(command)
until(lambda: fake['armed'])
until(lambda: not fake['armed'] and not adapter.mode[joint])

# Stale feedback refuses PREPARE and turns an active session OFF.
assert adapter.change_mode(joint, True).success
panel.values[joint].setValue(0.1)
until(lambda: panel.mode[joint])
panel.apply.setDown(True)
until(lambda: fake['armed'])
fake['state_on'] = False
until(lambda: not fake['armed'] and not adapter.mode[joint])
assert not adapter.change_mode(joint, True).success
panel.apply.setDown(False)
fake['state_on'] = True
until(lambda: adapter.fresh(joint))

assert adapter.change_mode(joint, True).success
stale = JointState(name=[joint], effort=[0.1])
stale.header.stamp = rospy.Time.now()-rospy.Duration(0.5)
adapter.torques(stale)
assert not adapter.mode[joint]

# Late ARM completion after user cancellation must not send a nonzero command.
fake['delay_arm'] = True
assert adapter.change_mode(joint, True).success
command = JointState(name=[joint], effort=[0.1])
command.header.stamp = rospy.Time.now()
adapter.torques(command)
adapter.stop(None)
time.sleep(0.8)
assert not fake['armed'] and fake['raw'] == 0
fake['delay_arm'] = False

assert adapter.restore(joint).success
assert fake['mode'] == 3 and not fake['armed']
adapter.calibrated = False
assert not adapter.change_mode(joint, True).success

# Exercise the actual catkin/rosrun entrypoint without a scripts PYTHONPATH shortcut.
rospy.set_param('/entry/joints', rospy.get_param('joints'))
environment = dict(os.environ, ROS_NAMESPACE='/entry')
runner = ['rosrun'] + (['--debug'] if os.environ.get('MKII_TEST_DEBUG') else [])
child = subprocess.Popen(runner + ['dracomancer_mkii_ros', 'direct.py',
    '_mode_service:=/mock/direct_servo_mode', '_state_topic:=/mock/servo/states',
    '_current_topic:=/mock/servo/direct_current'], env=environment)
try:
    response = rospy.wait_for_message('/entry/control/status', String, timeout=30)
    assert json.loads(response.data)['backend'] == 'spinal_direct'
    assert not rospy.ServiceProxy('/entry/control/'+joint+'/torque_mode', SetBool)(True).success
finally:
    child.send_signal(signal.SIGINT)
    child.wait(timeout=5)
assert child.returncode == 0
result = {'fake_spinal_only': True, 'qt_prepare_no_torque': True, 'qt_nm_to_signed_current': True,
          'qt_release_off': True, 'command_timeout_off': True, 'stale_state_off': True,
          'late_arm_canceled': True, 'restore_position_off': True, 'calibration_gate': True}
result['stale_torque_rejected'] = True
result['rosrun_entrypoint'] = True
Path(os.environ['DIRECT_RESULTS']).write_text(json.dumps(result, indent=2))
alive = False
panel.close()
print(json.dumps(result))
