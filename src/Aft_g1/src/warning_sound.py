#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from Aft_g1.msg import SoundMsg
import os

def chatter_callback(msg: SoundMsg) -> None:
    speech = ""
    if msg.op == 1:
        speech = "warning, fire detected!"
    elif msg.op == 2:
        speech = "warning, smoke detected!"
    elif msg.op == 3:
        speech = "warning, stranger detected!"

    if msg.op != 0 and speech:
        command = f'rosrun sound_play say.py "{speech}"'
        os.system(command)

if __name__ == '__main__':
    try:
        rospy.init_node('voice_node', anonymous=False)
        rospy.Subscriber("/sound_detect", SoundMsg, chatter_callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print(f"[ERROR] warning_sound start error!")
