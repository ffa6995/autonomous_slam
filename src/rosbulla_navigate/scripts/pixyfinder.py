#!/usr/bin/env python
import rospy
from rosbulla_navigate.msg import PixyData
from turtlebot3_msgs.msg import Sound
from std_msgs.msg import Bool
import time

def callback(data):
    if len(data.blocks) > 0:
        # sound_msg = Sound()
        # sound_msg.value = 0
        # pub.publish(sound_msg)
        rospy.loginfo('Found ')
        foundpublisher2.publish(True)
        # rospy.loginfo(data.blocks)
        rospy.sleep(2)
        # sound_msg.value = 1
        # pub.publish(sound_msg)

rospy.init_node('pixy_sound_node')
sub = rospy.Subscriber("/my_pixy/block_data", PixyData, callback)
foundpublisher2 = rospy.Publisher("/found", Bool, queue_size=10)
foundpublisher2.publish(False)
rospy.spin()

