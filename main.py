#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from geometry_msgs.msg import Twist
from sound_play.libsoundplay import SoundClient

BEEP_SOUND_FILE = "/home/gordon/ros/beeper/bb8 - beep beep.wav"


def absolute_sum(vector3):
    return abs(vector3.x) + abs(vector3.y) + abs(vector3.z)


def nonzero_velocity(twist_data):
    return (absolute_sum(twist_data.linear) + absolute_sum(twist_data.angular)) > 0


class Beeper:
    def __init__(self, beep_sound_file):
        self.sound_client = SoundClient()
        self.beep = self.sound_client.waveSound(beep_sound_file)

    def listen(self, data):
        if nonzero_velocity(data):
            rospy.loginfo("Motion detected; beeping!")
            self.beep.play()


def main():
    rospy.init_node('beeper')

    beeper = Beeper(BEEP_SOUND_FILE)
    rospy.Subscriber("cmd_vel", Twist, beeper.listen)

    rospy.loginfo("beeper is now listening for nonzero velocity on /cmd_vel")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
