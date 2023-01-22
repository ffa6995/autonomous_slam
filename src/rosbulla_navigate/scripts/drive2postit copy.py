import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class RedPostItFinder:

    def __init__(self):
        image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.bridge = CvBridge()
        rospy.loginfo("started")

    def callback(self, data):
        # rospy.loginfo("callback called")
        np_arr = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = self.getmask(cv_image)

        res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            rospy.loginfo("red post-it found!")
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                rospy.loginfo("red post-it found!")
                twist = Twist()
                if cX < cv_image.shape[1] / 3:
                    twist.angular.z = 0.25
                elif cX > cv_image.shape[1] * 2 / 3:
                    twist.angular.z = -0.25
                else:
                    twist.linear.x = 0.25
                self.cmd_vel_pub.publish(twist)

        # cv2.imshow("Image window", res)
        # cv2.waitKey(3)

    def getmask(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        token_color_lower_1 = np.array([0, 50, 100]) 
        token_color_upper_1 = np.array([15, 255, 255])
        token_color_lower_2 = np.array([165, 50, 100])
        token_color_upper_2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, token_color_lower_1, token_color_upper_1)
        mask2 = cv2.inRange(hsv, token_color_lower_2, token_color_upper_2)
        mask = np.zeros(mask1.shape)
        mask = cv2.bitwise_or(mask1, mask2, mask)
        return mask

    def scan_callback(self, data):
        # check for obstacles in front of the robot
        ranges = data.ranges
        min_range = min(ranges[0:90]) # check the 90 degree field in front of the robot
        if min_range < 0.10: # if the minimum range is less than 0.5 meters
            rospy.logwarn("Obstacle detected! Stopping the robot.")
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('drive2postit', anonymous=True)
    drive2postit = RedPostItFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()