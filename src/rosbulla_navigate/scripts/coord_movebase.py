# Import necessary modules
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# Initialize ROS node
rospy.init_node('move_base_node')

print("start movebase test")
# Create a action client to send goal requests to the move_base server
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# Wait until the action server is available
client.wait_for_server()

# Define the goal pose
goal_pose = Pose(Point(0.24,5.99,2.01), Quaternion(0.0, 0.0, 0.0, 1.0))
# goal_pose = Pose(Point(-1.88,-0.05,0.01), Quaternion(0.0, 0.0, 0.0, 1.0))

# Create a goal message
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
goal.target_pose.pose = goal_pose

# Send the goal to the action server
client.send_goal(goal)

# Wait for the action server to complete the goal
client.wait_for_result()

# Print the result of the action
print(client.get_result())
