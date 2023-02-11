import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped

"""
Goal: Improve the initial estimate by listening to imu message once at the beginning

"""

def imu_callback():
    pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
    msg = rospy.wait_for_message("imu", Imu)
    rate = rospy.Rate(10) # 10hz

    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.pose.orientation = msg.orientation
    pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

    pub.publish(pose)
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node("update_pose_estimation")
    imu_callback()