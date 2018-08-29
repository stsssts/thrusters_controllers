#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench
from akara_msgs.msg import Thruster
from akara_msgs.srv import BCS

X_AXIS = 1
Y_AXIS = 0
Z_AXIS = 2
YAW_AXIS = 3
BUOYANCY_AXIS = 5

STOP_BUTTON = 1
FRONT_BCS_BUTTON = 7
MAX_BUOYANCY_BUTTON = 4
MIN_BUOYANCY_BUTTON = 6
NEUTRAL_BUOYANCY_BUTTON = 5

class JoyTeleop:
    def __init__(self):
        rospy.Subscriber("joy", Joy, self.joy_callback)

        self.data = None
        self.steps = 50

        self.wrench_publisher = rospy.Publisher("command", Wrench, queue_size=3)
        self.buoyancy_publisher = rospy.Publisher("buoyancy", Thruster, queue_size=3)
        self.buoyancy_client = rospy.ServiceProxy('bcs_service', BCS)

    def joy_callback(self, msg):
        self.data = msg
        
    def apply_data(self):
        thrust_cmd = self.get_thrust_command()
        self.wrench_publisher.publish(thrust_cmd)
        
        stepper_cmd = self.get_stepper_command()
        self.buoyancy_publisher.publish(stepper_cmd)

        self.set_buoyancy()
        
    def get_thrust_command(self):
        cmd = Wrench()
        cmd.force.x = self.data.axes[X_AXIS]
        cmd.force.y = self.data.axes[Y_AXIS]
        cmd.force.z = self.data.axes[Z_AXIS]
        cmd.torque.z = self.data.axes[YAW_AXIS]
        return cmd
    
    def get_stepper_command(self):
        cmd = Thruster()
        s = -self.data.axes[BUOYANCY_AXIS] * self.steps
        if self.data.buttons[FRONT_BCS_BUTTON]:
            cmd.power = [s, 0]
        else:
            cmd.power = [0, s]
        return cmd
        
    def set_buoyancy(self):
        if self.data.buttons[MIN_BUOYANCY_BUTTON]:
            self.buoyancy_client.call('negative')
        elif self.data.buttons[MAX_BUOYANCY_BUTTON]:
            self.buoyancy_client.call('positive')
        elif self.data.buttons[NEUTRAL_BUOYANCY_BUTTON]:
            self.buoyancy_client.call('neutral')
        elif self.data.buttons[STOP_BUTTON]:
            self.buoyancy_client.call('stop')

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.data is not None:
                self.apply_data()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('surface_side', anonymous=True)
    j = JoyTeleop()
    j.loop()

