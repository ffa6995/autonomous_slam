import rospy
from geometry_msgs.msg import Pose
import tf
from geometry_msgs.msg import Pose, Point, Quaternion


def get_current_position():
    # Create a transform listener
    listener = tf.TransformListener()
    # Wait for the /odom to /base_footprint transform to be available
    listener.waitForTransform("/odom", "/base_footprint", rospy.Time(), rospy.Duration(4.0))
    # Get the current transform from /odom to /base_footprint
    (trans, rot) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time())
    # Create a Pose message with the position and orientation from the transform
    position = Point(*trans)
    orientation = Quaternion(*rot)
    pose = Pose(position, orientation)
    return pose

def main():
    rospy.init_node("get_position_node")
    get_position()

if __name__ == "__main__":
    main()
