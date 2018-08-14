#!/usr/bin/env python
import rospy
from thrusters_controller import ThrustersController


class ThrustersController2x2(ThrustersController):
    def update(self):
        port_cmd = self.commands_['surge'] + self.commands_['yaw']
        starboard_cmd = self.commands_['surge'] - self.commands_['yaw']
        sway_cmd = self.commands_['sway']

        self.thrusters_['port'].set_power(port_cmd)
        self.thrusters_['starboard'].set_power(starboard_cmd)
        self.thrusters_['sway_front'].set_power(sway_cmd)
        self.thrusters_['sway_rear'].set_power(sway_cmd)


if __name__ == '__main__':
    rospy.init_node('thrusters_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    tc = ThrustersController2x2(['surge', 'sway', 'yaw'])

    while not rospy.is_shutdown():
        tc.update()
        rate.sleep()
