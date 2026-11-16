#!/usr/bin/env python3
"""CAD joint angle sliders with a separate torque tab."""
import json
import sys
import threading
import time

import rospy
from joint_state_publisher import JointStatePublisher
from joint_state_publisher_gui import JointStatePublisherGui
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel,
    QPushButton, QTabWidget, QTableWidget, QTableWidgetItem, QCheckBox, QDoubleSpinBox)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class Panel(QWidget):
    feedback = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Dracomancer Mk-II — joint parameters')
        self.jsp = JointStatePublisher()
        # JSP normally publishes joint_states. As in Mk-I, route its output to joint_cmd.
        # Do this before starting its loop so viewer/physics feedback has one owner.
        self.jsp.pub.unregister()
        self.jsp.pub = rospy.Publisher('joint_cmd', JointState, queue_size=1)
        self.angles = JointStatePublisherGui('Angles [rad]', self.jsp, num_rows=7)
        # Randomizing every joint is inappropriate for a mounted device.
        self.angles.randbutton.hide()
        self.names = list(self.jsp.joint_list)
        self.mode = dict.fromkeys(self.names, False)
        self.pending = set()
        self.last_status = 0.0
        self.torque_backend = False
        self.available = dict.fromkeys(self.names, True)
        self.torque_pub = rospy.Publisher('torque_cmd', JointState, queue_size=1)
        self.status_sub = rospy.Subscriber('control/status', String, lambda m: self.feedback.emit(m.data), queue_size=1)
        self.feedback.connect(self.status)
        layout = QVBoxLayout(self)
        self.notice = QLabel('Waiting for controller status. Angles: rad; torque: Nm.')
        layout.addWidget(self.notice)
        self.error_notice = QLabel('')
        layout.addWidget(self.error_notice)
        tabs = QTabWidget()
        tabs.addTab(self.angles, 'Angles [rad] — Mk-I sliders')
        torque = QWidget()
        torque_layout = QVBoxLayout(torque)
        torque_layout.addWidget(QLabel('Select torque mode, enter Nm, then HOLD Apply. Release sends zero.\n'
                                      'Zero torque does not hold the arm against gravity.'))
        self.table = QTableWidget(len(self.names), 3)
        self.table.setHorizontalHeaderLabels(['Joint', 'Torque mode', 'Command [Nm]'])
        self.checks, self.values = {}, {}
        limit = float(rospy.get_param('max_torque', 2.0))
        for row, name in enumerate(self.names):
            self.table.setItem(row, 0, QTableWidgetItem(name))
            check = QCheckBox()
            check.setEnabled(False)
            check.toggled.connect(lambda enabled, n=name: self.request_mode(n, enabled))
            value = QDoubleSpinBox()
            value.setDecimals(3)
            value.setRange(-limit, limit)
            value.setSingleStep(0.01)
            self.table.setCellWidget(row, 1, check)
            self.table.setCellWidget(row, 2, value)
            self.checks[name], self.values[name] = check, value
        self.table.resizeColumnsToContents()
        torque_layout.addWidget(self.table)
        self.apply = QPushButton('HOLD to apply selected torque')
        self.apply.setEnabled(False)
        self.apply.released.connect(self.zero)
        torque_layout.addWidget(self.apply)
        self.stop_button = QPushButton('ZERO ALL TORQUE')
        self.stop_button.clicked.connect(self.zero)
        torque_layout.addWidget(self.stop_button)
        tabs.addTab(torque, 'Torque [Nm]')
        layout.addWidget(tabs)
        self.resize(950, 760)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        # Reuse the existing ROS angle publisher without blocking the Qt event loop.
        threading.Thread(target=self.publish_angles, daemon=True).start()

    def publish_angles(self):
        try:
            self.jsp.loop()
        except rospy.ROSException:
            # Shutdown can close the publisher between the JSP loop's check and publish.
            if not rospy.is_shutdown():
                raise

    def request_mode(self, name, enabled):
        self.pending.add(name)
        self.checks[name].setEnabled(False)
        self.zero(request_stop=False)

        def call():
            try:
                service = 'control/'+name+'/torque_mode'
                rospy.wait_for_service(service, timeout=2)
                result = rospy.ServiceProxy(service, SetBool)(enabled)
                self.feedback.emit(json.dumps({'reply': name, 'success': result.success, 'message': result.message}))
            except rospy.ROSException as error:
                self.feedback.emit(json.dumps({'reply': name, 'success': False, 'message': str(error)}))
        threading.Thread(target=call, daemon=True).start()

    def status(self, raw):
        data = json.loads(raw)
        if 'reply' in data:
            self.pending.discard(data['reply'])
            if not data['success']:
                self.error_notice.setText('Mode switch refused: '+data['message'])
            else:
                self.error_notice.setText('')
            return
        self.last_status = time.monotonic()
        hardware = data['backend'] == 'spinal_direct'
        self.torque_backend = data['backend'] in ('gazebo', 'spinal_direct')
        self.available = data.get('available', dict.fromkeys(self.names, True))
        if hardware:
            self.notice.setText('Spinal direct UART: angles read-only. Checkbox prepares mode 0; HOLD arms.\n'
                                'Unmapped joints are shown at CAD q=0. Release/zero requests torque OFF.')
        elif self.torque_backend:
            self.notice.setText('Gazebo: simulated joint torque [Nm].')
        else:
            self.notice.setText('RViz viewer: angles only. Torque requires a physics/hardware backend.')
        for name in self.names:
            enabled = data['torque_mode'][name]
            if hardware or self.mode[name] or (enabled != self.mode[name]):
                # Track measured position in torque mode and at the return to position.
                self.jsp.free_joints[name]['position'] = data['position'][name]
            self.mode[name] = enabled
            if name not in self.pending:
                check = self.checks[name]
                check.blockSignals(True)
                check.setChecked(enabled)
                check.blockSignals(False)
            self.angles.joint_map[name]['slider'].setEnabled(not enabled and not hardware)
            if hardware and name in data.get('max_torque_by_joint', {}):
                limit = data['max_torque_by_joint'][name]
                self.values[name].setRange(-limit, limit)
        self.angles.update_sliders()

    def tick(self):
        healthy = time.monotonic()-self.last_status < 0.4
        for name in self.names:
            self.checks[name].setEnabled(healthy and self.torque_backend and self.available[name] and name not in self.pending)
        self.apply.setEnabled(healthy and self.torque_backend and not self.pending)
        if not healthy:
            self.notice.setText('Controller status missing; torque commands stopped.')
        if healthy and self.torque_backend and self.apply.isDown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = [n for n in self.names if self.mode[n]]
            msg.effort = [self.values[n].value() for n in msg.name]
            self.torque_pub.publish(msg)
        if rospy.is_shutdown():
            self.close()

    def zero(self, _checked=False, request_stop=True):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [n for n in self.names if self.mode[n]]
        msg.effort = [0.0]*len(msg.name)
        self.torque_pub.publish(msg)
        # Works even if a different ROS client issued the previous effort command.
        def call():
            try:
                rospy.wait_for_service('control/stop', timeout=0.2)
                rospy.ServiceProxy('control/stop', Trigger)()
            except rospy.ROSException:
                pass
        if request_stop:
            threading.Thread(target=call, daemon=True).start()

    def closeEvent(self, event):
        self.timer.stop()
        self.status_sub.unregister()
        self.zero()
        rospy.signal_shutdown('Parameter window closed')
        event.accept()


if __name__ == '__main__':
    rospy.init_node('panel', disable_signals=True)
    app = QApplication(sys.argv)
    window = Panel()
    window.show()
    sys.exit(app.exec_())
