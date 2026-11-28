#!/usr/bin/env python3
"""Convert one Spinal Unit Joystick2 stream to sensor_msgs/Joy."""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray


def axis_value(count):
    # Spinal subtracts the startup center and saturates to a signed 16-bit count.
    return count / (32768.0 if count < 0 else 32767.0)


def main():
    rospy.init_node('joystick')
    publisher = rospy.Publisher('joy', Joy, queue_size=1)

    def publish(raw):
        if len(raw.data) != 3 or raw.data[2] not in (0, 1):
            rospy.logwarn_throttle(5.0, 'Expected three Joystick2 values and a 0/1 button level')
            return

        x, y, button = raw.data
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = [axis_value(x), axis_value(y)]
        # Unit Joystick2 register 0x20 is active-low. Joy reports pressed as 1.
        msg.buttons = [int(button == 0)]
        publisher.publish(msg)

    subscriber = rospy.Subscriber('raw', Int16MultiArray, publish, queue_size=1)
    rospy.spin()
    subscriber.unregister()


if __name__ == '__main__':
    main()
