#!/usr/bin/env python
import rospy
from thrusters_controller import ThrustersController


class ThrustersController1x2x1(ThrustersController):
    def update(self):
        f = self.thrusters_['sway_front'].distance
        b = self.thrusters_['sway_rear'].distance
        c = self.thrusters_['front'].distance

        front_cmd = self.commands_['surge']
        vertical_cmd = self.commands_['heave']

        sway_front_cmd = (self.commands_['yaw'] - b * self.commands_['sway'] + 
                          c * self.commands_['surge']) / (f - b) 
        sway_rear_cmd =  -(self.commands_['yaw'] - f * self.commands_['sway'] + 
                           c * self.commands_['surge']) / (f - b)

        self.thrusters_['front'].set_power(front_cmd)
        self.thrusters_['vertical'].set_power(vertical_cmd)
        self.thrusters_['sway_front'].set_power(sway_front_cmd)
        self.thrusters_['sway_rear'].set_power(sway_rear_cmd)
        

if __name__ == '__main__':
    rospy.init_node('thrusters_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    tc = ThrustersController1x2x1(['surge', 'heave', 'sway', 'yaw'])

    while not rospy.is_shutdown():
        tc.update()
        rate.sleep()
