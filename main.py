#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest

BEEP_SOUND_FILE = "/home/gordon/ros/beeper/bb8 - beep beep.wav"


def main():
    rospy.init_node('beeper')
    soundhandle = SoundClient()
    rospy.sleep(0.5)  # Ensure publisher connection is successful.

    sound_beep = soundhandle.waveSound(BEEP_SOUND_FILE)
    sound_beep.play()

    rospy.loginfo('Finished')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
