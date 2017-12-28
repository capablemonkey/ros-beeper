#!/usr/bin/env python

import rospy
from os.path import join, abspath, dirname, exists
from std_msgs.msg import String

from geometry_msgs.msg import Twist
from sound_play.libsoundplay import SoundClient

DEFAULT_BEEP_SOUND_FILE = join(dirname(abspath(__file__)), '../sounds/sample.wav')


def absolute_sum(vector3):
    return abs(vector3.x) + abs(vector3.y) + abs(vector3.z)


def zero_velocity(twist_data):
    return (absolute_sum(twist_data.linear) + absolute_sum(twist_data.angular)) == 0


class Beeper:
    def __init__(self, beep_sound_file, interval_seconds):
        if not exists(beep_sound_file):
            raise ValueError("Beep sound file doesn't exist!")

        self.sound_client = SoundClient()
        self.beep_sound = self.sound_client.waveSound(beep_sound_file)

        self.interval_seconds = rospy.Duration(interval_seconds)
        self.last_beep = rospy.Time.now() - self.interval_seconds

    def beep(self):
        self.beep_sound.play()
        self.last_beep = rospy.Time.now()

    def beeped_recently(self):
        return (rospy.Time.now() - self.last_beep) < self.interval_seconds

    def listen(self, data):
        if zero_velocity(data):
            return

        if not self.beeped_recently():
            rospy.loginfo("Motion detected; beeping!")
            self.beep()
        else:
            rospy.loginfo("Motion detected; already beeped recently")


def main():
    rospy.init_node('beeper')
    beep_sound_file = rospy.get_param("beeper/sound_file", DEFAULT_BEEP_SOUND_FILE)
    beep_interval_seconds = rospy.get_param("beeper/interval", 2)

    # Listen to cmd_vel messages and pass Beeper's listener as callback
    beeper = Beeper(beep_sound_file, beep_interval_seconds)
    rospy.Subscriber("/cmd_vel", Twist, beeper.listen)

    rospy.loginfo("beeper is now listening for nonzero velocity on /cmd_vel")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
