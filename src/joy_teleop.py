#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench

class JoyTeleop:
    axes = {'X': 1,
            'Y': 0,
            'Z': 2,
            'YAW': 3}

    buttons = {'STOP': 1,
               'INC_BUOYANCY': -1,
               'DEC_BUOYANCY': -1,
               'SET_NEG_BUOYANCY': -1,
               'SET_POS_BUOYANCY': -1,
               'SET_NEUTRAL_BUOYANCY': -1}

    def __init__(self):
        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.wrench_publisher = rospy.Publisher("command", Wrench, queue_size=3)
        
    def joy_callback(self, msg):
        cmd = Wrench()
        cmd.force.x = msg.axes[self.axes['X']]
        cmd.force.y = msg.axes[self.axes['Y']]
        cmd.force.z = msg.axes[self.axes['Z']]
        
        # cmd.torque.x = 0 # roll
        # cmd.torque.y = 0 # pitch
        cmd.torque.z = msg.axes[self.axes['YAW']]

        self.wrench_publisher.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('surface_side', anonymous=True)
    j = JoyTeleop()
    rospy.spin()

