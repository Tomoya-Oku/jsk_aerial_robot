#!/usr/bin/env python3
"""Direct-UART Spinal adapter. Never connects to or flashes a serial port."""
import concurrent.futures
import json
import math
import threading
import time
import queue
import secrets

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from spinal.msg import DirectServoCurrent, ServoStates
from spinal.srv import DirectServoMode, DirectServoModeRequest as Mode, GetBoardInfo
from dracomancer.xm430 import CurrentScale
from dracomancer.output_lock import claim_output


# Match the restored Spinal MAX_SERVO_NUM for one direct-UART handler.
MAX_DIRECT_SERVOS = 10
ARMS = ("right", "left")
MAX_ARM_SKEW = 0.05


class DaemonExecutor:
    """One bounded RPC worker; a disconnected rosserial call cannot hold process exit."""
    def __init__(self):
        self.queue = queue.Queue(maxsize=32)
        threading.Thread(target=self.work, daemon=True).start()

    def submit(self, function, *args):
        future = concurrent.futures.Future()
        try:
            self.queue.put_nowait((future, function, args))
        except queue.Full:
            future.set_exception(RuntimeError('Spinal RPC queue is full'))
        return future

    def work(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            future, function, args = item
            if not future.set_running_or_notify_cancel():
                continue
            try:
                future.set_result(function(*args))
            except Exception as error:
                future.set_exception(error)

    def shutdown(self):
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            pass


class SpinalControl:
    def __init__(self):
        if rospy.get_param('/use_sim_time', False):
            raise ValueError('Hardware requires wall/real ROS time; use a separate master from Gazebo')
        self.names = rospy.get_param('joints')
        self.calibrated = rospy.get_param('~calibrated', False) is True
        self.config = rospy.get_param('~actuators', {})
        self.servo_ids = rospy.get_param('~servo_ids', {})
        self.dual = rospy.get_param('~dual_spinal', False)
        self.buses = ARMS if self.dual else ('legacy',)
        if not isinstance(self.config, dict) or (not self.dual and len(self.config) > MAX_DIRECT_SERVOS):
            raise ValueError('actuators must map at most 10 direct-UART servo indexes per bus')
        if not isinstance(self.servo_ids, dict):
            raise ValueError('servo_ids must map CAD joint names to physical bus IDs')
        indexes, ids = set(), set()
        self.joints_by_bus = {bus: [] for bus in self.buses}
        self.scales, self.limits, self.device_limits = {}, {}, {}
        for name, cfg in self.config.items():
            if name not in self.names:
                raise ValueError('Unknown URDF joint '+name)
            # The physical ID and the scan index are independently verified on each bus.
            cfg = dict(cfg)
            cfg.setdefault('id', self.servo_ids.get(name))
            self.config[name] = cfg
            bus = name.split('_', 1)[0] if self.dual else 'legacy'
            if bus not in self.joints_by_bus:
                raise ValueError('Dual Spinal requires a right or left CAD joint: '+name)
            cfg['bus'] = bus
            index, servo_id = cfg['index'], cfg['id']
            max_index = 7 if self.dual else MAX_DIRECT_SERVOS
            expected = self.servo_ids.get(name) if self.dual else servo_id
            if (type(index) is not int or type(servo_id) is not int or not 0 <= index < max_index
                    or not 0 <= servo_id <= 252 or servo_id != expected
                    or (bus, index) in indexes or (bus, servo_id) in ids):
                raise ValueError('Invalid or duplicate servo index/ID')
            indexes.add((bus, index))
            ids.add((bus, servo_id))
            self.joints_by_bus[bus].append(name)
            if (not math.isfinite(cfg['zero_tick']) or not math.isfinite(cfg['radians_per_tick'])
                    or cfg['radians_per_tick'] <= 0 or not math.isfinite(cfg['max_torque_nm'])
                    or cfg['max_torque_nm'] <= 0):
                raise ValueError('Calibrated encoder and torque limits are required')
            self.scales[name] = CurrentScale(cfg['joint_nm_per_amp'], cfg['direction'], cfg['max_current_amp'])
            self.device_limits[name] = self.scales[name].limit
            self.limits[name] = min(cfg['max_torque_nm'], self.scales[name].limit*0.00269*cfg['joint_nm_per_amp'])
        self.lock = threading.RLock()
        self.position = dict.fromkeys(self.names, 0.0)
        self.last_state = dict.fromkeys(self.names, 0.0)
        self.measurement_stamp = dict.fromkeys(self.names, rospy.Time(0))
        self.error = dict.fromkeys(self.names, 255)
        self.mode = dict.fromkeys(self.names, False)
        self.armed = dict.fromkeys(self.names, False)
        self.token = dict.fromkeys(self.names, 0)
        self.sequence = dict.fromkeys(self.names, 0)
        self.raw = dict.fromkeys(self.names, 0)
        self.commanded = dict.fromkeys(self.names, 0.0)
        self.last_command = dict.fromkeys(self.names, 0.0)
        self.busy = set()
        self.alive = True
        self.timeout = 0.15  # Shorter than the independent MCU's fixed 250ms deadline.
        if self.dual:
            prefix = rospy.get_param('~spinal_ns', '/dracomancer').rstrip('/')
            active = [bus for bus in self.buses if self.joints_by_bus[bus]]
            mode_paths = {bus: prefix+'/'+bus+'/direct_servo_mode' for bus in active}
            current_paths = {bus: prefix+'/'+bus+'/servo/direct_current' for bus in active}
            state_paths = {bus: prefix+'/'+bus+'/servo/states' for bus in active}
            for bus in active:
                self.check_board(prefix+'/'+bus+'/get_board_info', bus)
        else:
            mode_paths = {'legacy': rospy.get_param('~mode_service', '/dracomancer/direct_servo_mode')}
            current_paths = {'legacy': rospy.get_param('~current_topic', '/dracomancer/servo/direct_current')}
            state_paths = {'legacy': rospy.get_param('~state_topic', '/dracomancer/servo/states')}

        # Own all active outputs before creating publishers or shutdown callbacks.
        paths = [*current_paths.values(), *mode_paths.values()]
        self.output_lock = claim_output(*paths) if self.calibrated and self.config else None
        self.rpc, self.executor, self.current_pub = {}, {}, {}
        for bus, service in mode_paths.items():
            rospy.wait_for_service(service, timeout=30)
            self.rpc[bus] = rospy.ServiceProxy(service, DirectServoMode)
            self.executor[bus] = DaemonExecutor()
            self.current_pub[bus] = rospy.Publisher(current_paths[bus], DirectServoCurrent, queue_size=1)
        self.state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.measured_pub = rospy.Publisher('measured_joint_states', JointState, queue_size=1)
        self.status_pub = rospy.Publisher('control/status', String, queue_size=1, latch=True)
        self.state_sub = [rospy.Subscriber(topic, ServoStates, self.states, callback_args=bus, queue_size=1)
                          if self.dual else rospy.Subscriber(topic, ServoStates, self.states, queue_size=1)
                          for bus, topic in state_paths.items()]
        self.torque_sub = rospy.Subscriber('torque_cmd', JointState, self.torques, queue_size=1)
        self.services = [rospy.Service('control/'+n+'/torque_mode', SetBool,
                         lambda req, name=n: self.change_mode(name, req.data)) for n in self.names]
        self.stop_service = rospy.Service('control/stop', Trigger, self.stop)
        self.restore_services = [rospy.Service('control/'+n+'/restore_position', Trigger,
                                  lambda req, name=n: self.restore(name)) for n in self.config]
        rospy.on_shutdown(self.shutdown)
        threading.Thread(target=self.loop, daemon=True).start()
        threading.Thread(target=self.monitor, daemon=True).start()

    def check_board(self, service, bus):
        rospy.wait_for_service(service, timeout=10)
        boards = rospy.ServiceProxy(service, GetBoardInfo)().boards
        if len(boards) != 1 or boards[0].slave_id != 0:
            raise ValueError('Expected one direct-UART board on '+bus)
        ids = [servo.id for servo in boards[0].servos]
        first = 1 if bus == 'right' else 11
        expected_ids = set(range(first, first+7))
        if len(ids) != 7 or set(ids) != expected_ids:
            raise ValueError('Unexpected DYNAMIXEL IDs on '+bus)
        for name in self.joints_by_bus[bus]:
            cfg = self.config[name]
            if ids[cfg['index']] != cfg['id']:
                raise ValueError('Spinal scan index and physical ID disagree for '+name)

    def request(self, name, action, session=0, cap=0):
        cfg = self.config[name]
        bus = cfg['bus']
        future = self.executor[bus].submit(self.rpc[bus], Mode(index=cfg['index'], servo_id=cfg['id'],
                                      action=action, session=session, current_limit=cap))
        try:
            result = future.result(timeout=2.0)
        except (concurrent.futures.TimeoutError, rospy.ROSException) as error:
            future.cancel()
            raise RuntimeError('Spinal service timeout/error') from error
        if (not result.success or result.servo_id != cfg['id'] or result.model_number != 1020):
            raise RuntimeError('Spinal refused action %s: status %s' % (action, result.status))
        return result

    def states(self, msg, bus=None):
        age = (rospy.Time.now()-msg.stamp).to_sec()
        if msg.stamp.to_sec() <= 0 or not -0.1 <= age <= 0.5:
            return
        indexes = [s.index for s in msg.servos]
        if len(indexes) != len(set(indexes)):
            return
        by_index = {s.index: s for s in msg.servos}
        with self.lock:
            for name, cfg in self.config.items():
                servo = by_index.get(cfg['index'])
                if (servo is None or (bus is not None and cfg['bus'] != bus) or
                        msg.stamp.to_sec() <= self.measurement_stamp[name].to_sec()):
                    continue
                self.position[name] = (servo.angle-cfg['zero_tick'])*cfg['radians_per_tick']*cfg['direction']
                self.error[name] = servo.error
                self.last_state[name] = time.monotonic()
                self.measurement_stamp[name] = msg.stamp

    def measured_state(self):
        """Only calibrated, fresh measured axes may command DRAGON morphology."""
        msg = JointState()
        msg.name = [n for n in self.config if self.calibrated and self.fresh(n)]
        if getattr(self, 'dual', False) and len({self.config[n]['bus'] for n in msg.name}) == 2:
            newest = max(self.measurement_stamp[n] for n in msg.name)
            msg.name = [n for n in msg.name
                        if (newest-self.measurement_stamp[n]).to_sec() <= MAX_ARM_SKEW]
        msg.position = [self.position[n] for n in msg.name]
        # GUI placeholders and cache republishes must not look like new input.
        msg.header.stamp = min((self.measurement_stamp[n] for n in msg.name), default=rospy.Time(0))
        return msg

    def fresh(self, name):
        return (self.error[name] == 0 and time.monotonic()-self.last_state[name] < self.timeout
                and -0.1 <= (rospy.Time.now()-self.measurement_stamp[name]).to_sec() < self.timeout)

    def change_mode(self, name, enabled):
        if name not in self.config:
            return SetBoolResponse(False, 'Unmapped joint (displayed at CAD q=0)')
        if not enabled:
            self.stop_joints([name])
            return SetBoolResponse(True, 'Stopped; current mode stays selected on the servo, torque OFF')
        with self.lock:
            if not self.calibrated or not self.fresh(name) or name in self.busy:
                return SetBoolResponse(False, 'Calibration, fresh error-free state, and idle service required')
            if self.mode[name]:
                return SetBoolResponse(True, 'Already prepared')
            self.busy.add(name)
        try:
            query = self.request(name, Mode.QUERY)
            effective_cap = min(self.scales[name].limit, query.current_limit)
            result = self.request(name, Mode.PREPARE, session=secrets.randbits(32) or 1,
                                  cap=effective_cap)
            if (result.operating_mode != 0 or result.torque_enabled or not result.session
                    or result.timeout_ms != 250 or result.firmware_version < 38):
                raise RuntimeError('Unexpected prepare readback')
            with self.lock:
                if name not in self.busy:  # A concurrent stop canceled this preparation.
                    raise RuntimeError('Preparation canceled')
                self.token[name] = result.session
                self.device_limits[name] = effective_cap
                self.limits[name] = min(self.config[name]['max_torque_nm'],
                    effective_cap*CurrentScale.UNIT_A*self.config[name]['joint_nm_per_amp'])
                self.mode[name] = True
                self.sequence[name] = 0
            return SetBoolResponse(True, 'Mode 0 verified, torque OFF. HOLD Apply to arm and command.')
        except RuntimeError as error:
            self.stop_joints([name])
            return SetBoolResponse(False, str(error))
        finally:
            with self.lock:
                self.busy.discard(name)

    def begin_arm(self, name, token):
        try:
            result = self.request(name, Mode.ARM, session=token)
            with self.lock:
                if (self.token[name] != token or not self.mode[name] or not self.fresh(name)
                        or time.monotonic()-self.last_command[name] >= self.timeout
                        or result.session != token or not result.torque_enabled or result.operating_mode != 0):
                    raise RuntimeError('ARM canceled or expired')
                self.armed[name] = True
        except RuntimeError:
            self.stop_joints([name])
        finally:
            with self.lock:
                self.busy.discard(name)

    def torques(self, msg):
        age = (rospy.Time.now()-msg.header.stamp).to_sec()
        if msg.header.stamp.to_sec() <= 0 or not -0.05 <= age < self.timeout:
            self.stop(None)
            return
        if (len(msg.name) != len(msg.effort) or len(msg.name) != len(set(msg.name))
                or any(n not in self.config for n in msg.name)):
            self.stop(None)
            return
        try:
            raw = {}
            for name, value in zip(msg.name, msg.effort):
                if not math.isfinite(value) or abs(value) > self.limits[name]:
                    raise ValueError('Torque exceeds calibrated limit')
                raw[name] = self.scales[name].to_raw(value, self.device_limits[name])
        except ValueError:
            self.stop(None)
            return
        with self.lock:
            for name, value in zip(msg.name, msg.effort):
                if not self.mode[name] or not self.fresh(name):
                    continue
                self.raw[name] = raw[name]
                self.commanded[name] = value
                self.last_command[name] = time.monotonic()
                if raw[name] != 0 and not self.armed[name] and name not in self.busy:
                    self.busy.add(name)
                    threading.Thread(target=self.begin_arm, args=(name, self.token[name]), daemon=True).start()

    def packet(self, names, zero=False):
        msg = DirectServoCurrent(index=[], session=[], sequence=[], current=[])
        for name in names:
            if not self.token[name]:
                continue
            self.sequence[name] = (self.sequence[name]+1) & 0xffffffff
            msg.index.append(self.config[name]['index'])
            msg.session.append(self.token[name])
            msg.sequence.append(self.sequence[name])
            msg.current.append(0 if zero else self.raw[name])
        return msg

    def publish_current(self, names, zero=False):
        for bus, publisher in self.current_pub.items():
            message = self.packet([n for n in names if self.config[n]['bus'] == bus], zero=zero)
            if message.index:
                publisher.publish(message)

    def stop_joints(self, names):
        with self.lock:
            self.publish_current(names, zero=True)
            for name in names:
                self.mode[name] = self.armed[name] = False
                self.token[name] = self.raw[name] = 0
                self.commanded[name] = 0.0
                self.busy.discard(name)
                # STOP is queued after any in-flight ARM; the MCU deadline is independent.
                cfg = self.config[name]
                bus = cfg['bus']
                self.executor[bus].submit(
                    self.rpc[bus], Mode(index=cfg['index'], servo_id=cfg['id'], action=Mode.STOP))

    def stop(self, _request):
        self.stop_joints(list(self.config))
        return TriggerResponse(True, 'Zero sent and OFF requested; MCU independently expires old sessions')

    def restore(self, name):
        self.stop_joints([name])
        try:
            self.request(name, Mode.RESTORE)
            return TriggerResponse(True, 'Original position setup restored; torque remains OFF')
        except RuntimeError as error:
            return TriggerResponse(False, str(error))

    def loop(self):
        while self.alive and not rospy.is_shutdown():
            with self.lock:
                expire = [n for n in self.config if self.mode[n] and
                          (not self.fresh(n) or (self.armed[n] and time.monotonic()-self.last_command[n] >= self.timeout))]
                if expire:
                    self.stop_joints(expire)
                self.publish_current([n for n in self.config if self.armed[n]])
                state = JointState()
                state.header.stamp = rospy.Time.now()
                state.name = self.names
                state.position = [self.position[n] for n in self.names]
                self.state_pub.publish(state)
                self.measured_pub.publish(self.measured_state())
                available = {n: self.calibrated and n in self.config and self.fresh(n) for n in self.names}
                self.status_pub.publish(json.dumps({'backend': 'spinal_direct', 'torque_mode': self.mode,
                    'position': self.position, 'commanded_torque': self.commanded,
                    'available': available, 'max_torque_by_joint': self.limits,
                    'armed': self.armed, 'measured_joints': list(self.config)}))
            time.sleep(0.025)

    def monitor(self):
        while self.alive and not rospy.is_shutdown():
            for name in self.config:
                with self.lock:
                    check = self.mode[name] and name not in self.busy
                    token = self.token[name]
                    armed = self.armed[name]
                if check:
                    try:
                        result = self.request(name, Mode.QUERY)
                        with self.lock:
                            if self.token[name] == token and (result.session != token or
                                    result.operating_mode != 0 or (armed and not result.torque_enabled)):
                                self.stop_joints([name])
                    except RuntimeError:
                        self.stop_joints([name])
                time.sleep(0.2)
            time.sleep(0.2)

    def shutdown(self):
        self.alive = False
        self.stop(None)
        for worker in self.executor.values():
            worker.shutdown()


if __name__ == '__main__':
    rospy.init_node('spinal_control')
    try:
        node = SpinalControl()
        rospy.spin()
    except (rospy.ROSException, RuntimeError, ValueError, KeyError) as error:
        rospy.logfatal(str(error))
        raise SystemExit(1)
