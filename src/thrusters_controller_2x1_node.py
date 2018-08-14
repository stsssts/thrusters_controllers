#!/usr/bin/env python
import rospy
from thrusters_controller import ThrustersController


class ThrustersController2x1(ThrustersController):
    def update(self):
        port_cmd = self.commands_['surge'] + self.commands_['yaw']
        vertical_cmd = self.commands_['heave']
        starboard_cmd = self.commands_['surge'] - self.commands_['yaw']

        self.thrusters_['port'].set_power(port_cmd)
        self.thrusters_['vertical'].set_power(vertical_cmd)
        self.thrusters_['starboard'].set_power(starboard_cmd)


if __name__ == '__main__':
    rospy.init_node('thrusters_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    tc = ThrustersController2x1(['surge', 'heave', 'yaw'])

    while not rospy.is_shutdown():
        tc.update()
        rate.sleep()
