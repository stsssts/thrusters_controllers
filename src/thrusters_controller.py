#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


class Thruster:
    # TODO: add curve interpolation
    def __init__(self, name, settings):
        self.reversed_ = settings['reversed']
        self.forward_modifier_ = settings['forward_modifier']
        self.reverse_modifier_ = settings['reverse_modifier']
        if 'distance' in settings:
            self.distance = settings['distance']

        self.publisher_ = rospy.Publisher('thrusters/%s' % name, Float64, queue_size=1)

    def _get_effort(self, power):
        if self.reversed_:
            power *= -1
        power *= self.forward_modifier_ if power > 0 else self.reverse_modifier_
        return power

    def set_power(self, power):
        self.publisher_.publish(self._get_effort(power))


class ThrustersController:
    def __init__(self, commands):
        if not rospy.has_param('~thrusters'):
            rospy.logerr("Didn't find thrusters configuration")
            raise rospy.ROSException

        self.thrusters_ = {}
        settings = rospy.get_param('~thrusters')
        for thr in settings:
            self.thrusters_[thr] = Thruster(thr, settings[thr])

        self.commands_ = {}
        for cmd in commands:
            self.commands_[cmd] = 0.0
            rospy.Subscriber('thrust_command/%s' % cmd, Float64, self.command_callback, cmd)

    def command_callback(self, msg, cmd):
        self.commands_[cmd] = msg.data

    def update(self):
        raise NotImplementedError
