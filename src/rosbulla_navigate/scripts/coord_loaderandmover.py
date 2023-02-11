#!/usr/bin/env python
#https://prod.liveshare.vsengsaas.visualstudio.com/join?999F2D41262B8102BE0E34C82EBABB9BE0F6
import rospy
import csv
import os
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray

def callback(data):
    rospy.loginfo(data.data)

    # Get the row number from the message
    searched_postit = data.data
    path = os.path.expanduser('~/Documents/position_data.csv')
    with open(path, 'r') as file:
        # Read the CSV file
        reader = csv.reader(file)
        # Get all rows in a list
        rows = [row for row in reader]
        # Get the specific row
        for row in rows:
            # Compare the first value of the row
            if row[0] == '"':                     
                row[0] = ''                     
                row[-1] = ''
            
        
            row = ','.join(row)
            rospy.loginfo(row)
            if row[0] != "x" and int(row[0]) == searched_postit:
                # Print the row
                rospy.loginfo(row)
                try:
                    mover(row)
                except:
                    mover(row.split(','))
                    rospy.loginfo(row.split(','))
                break
                
def mover(data):
    # Create a action client to send goal requests to the move_base server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait until the action server is available
    client.wait_for_server()

    # Define the goal pose
    goal_pose = Pose(Point(float(data[1]),float(data[2]),float(data[3])), Quaternion(0.0, 0.0, 0.0, 1.0))
    # goal_pose = Pose(Point(-1.88,-0.05,0.01), Quaternion(0.0, 0.0, 0.0, 1.0))

    # Create a goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose = goal_pose

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the action server to complete the goal
    client.wait_for_result()

def status_callback(msg):
    if len(msg.status_list) > 0:
        status = msg.status_list[0].status
        if status == 3: # goal reached
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = -0.8
                cmd_vel_pub.publish(twist)

        
def csv_reader():
    # Initialize the node
    rospy.init_node('csv_reader', anonymous=True)
    # Subscribe to the topics
    rospy.Subscriber("/postit_num", Int32, callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)
    # Initialize the publisher for the cmd_vel topic
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # Start spinning
    rospy.spin()


if __name__ == '__main__':
    csv_reader()