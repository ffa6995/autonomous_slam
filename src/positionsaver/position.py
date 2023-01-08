import rospy
import csv
from geometry_msgs.msg import Pose

def save_position(pose_msg):
    # Extract the x, y, and z coordinates from the pose message
    x = pose_msg.position.x
    y = pose_msg.position.y
    z = pose_msg.position.z
    # Open the CSV file in append mode
    with open("/path/to/save/positions.csv", "a") as csvfile:
        # Create a CSV writer
        writer = csv.writer(csvfile)
        # Write the position to the CSV file
        writer.writerow([x, y, z])

def main():
    rospy.init_node("save_position_node")
    # Subscribe to the position topic
    rospy.Subscriber("/turtlebot2/pose", Pose, save_position)
    rospy.spin()

if __name__ == "__main__":
    main()
