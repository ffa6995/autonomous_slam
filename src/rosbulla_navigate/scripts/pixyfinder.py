#!/usr/bin/env python
import rospy
from rosbulla_navigate.msg import PixyData
from turtlebot3_msgs.msg import Sound
import time

def callback(data):
    if len(data.blocks) > 0:
        # sound_msg = Sound()
        # sound_msg.value = 0
        # pub.publish(sound_msg)
        rospy.loginfo('Found ')
        # rospy.loginfo(data.blocks)
        # time.sleep(0.5) # wait for 0.5 seconds
        # sound_msg.value = 1
        # pub.publish(sound_msg)

rospy.init_node('pixy_sound_node')
sub = rospy.Subscriber("/my_pixy/block_data", PixyData, callback)
pub = rospy.Publisher("/sound", Sound, queue_size=10)
rospy.spin()

