#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from typing import Optional
import rospy
from rosbulla_navigate.msg import coordinate
from csv import reader


def talker():

    #topic name
    pub = rospy.Publisher('coordinates', coordinate, queue_size=10)
    rospy.init_node('coord_topic', anonymous=True)

    coord = coordinate()
    coord.x = 12
    coord.y = 14
    coord.tagnr = 1
    coord.found: bool = True

   
    read_coord_file("/home/parallels/catkin_ws/src/rosbulla_navigate/scripts/coords.csv")

    # TODO read in file (where coordinates of found post-it are)
    # and write them out


    # Publish coordinate as long as the coordinate exists in file as not found
    while not rospy.is_shutdown():

        coordinates = coord 
        rospy.loginfo(coordinates)
        pub.publish(coordinates)
        rospy.Rate(1).sleep()

def read_coord_file(path: str) -> None:
    # skip first line i.e. read header first and then iterate over each row od csv as a list
    with open(path, 'r') as read_obj:
        csv_reader = reader(read_obj)
        header = next(csv_reader)
        # Check file as empty
        if header != None:
            # Iterate over each row after the header in the csv
            for row in csv_reader:
                # row variable is a list that represents a row in csv
                rospy.loginfo(row)

        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
