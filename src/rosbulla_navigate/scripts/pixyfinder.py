#!/usr/bin/env python
import math
import rospy
from rosbulla_navigate.msg import PixyData
from turtlebot3_msgs.msg import Sound
from std_msgs.msg import Bool, String
import time

class PixyFinder():
    def __init__(self) -> None:
        # Define the publisher
        self.explorationDone: bool = False
        self.sub2 = rospy.Subscriber("/explore/ExplorationDone", Bool, self.callbackExplorationDone)
        self.foundpublisher2 = rospy.Publisher("/found", Bool, queue_size=2)
        self.foundpublisher2.publish(False)
        self.foundone = False
        self.consecutive_non_detections = math.inf # Counter for letting intermediate blocks that don't recognize the tag fly through

    def callbackExplorationDone(self, data):
        self.explorationDone = data.data


    def callback(self, data):
        if self.found_tag(data):
            self.foundone = True
            self.consecutive_non_detections = 0
            rospy.loginfo('Found')
            self.foundpublisher2.publish(True)
            self.foundone = True
        else:
            self.foundone = False
            self.consecutive_non_detections += 1

    def found_tag(self, data: PixyData) -> bool:
        return data.header.stamp.secs != 0

if __name__ == '__main__':
    rospy.init_node('pixy_sound_node')
    pixyfinder = PixyFinder()
    sub = rospy.Subscriber("/my_pixy/block_data", PixyData, pixyfinder.callback)
    sub2 = rospy.Subscriber("/explore/ExplorationDone", Bool, pixyfinder.callbackExplorationDone)
    rospy.spin()

