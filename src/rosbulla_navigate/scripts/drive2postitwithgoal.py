import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class RedPostItFinder:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.bridge = CvBridge()
        rospy.loginfo("RedPostItFinder started")

    def callback(self, data):
        rospy.loginfo("Callback started")
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = self.getmask(cv_image)

        res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                goal = PoseStamped()
                goal.header.frame_id = "base_link"
                goal.pose.position.x = cX
                goal.pose.position.y = cY
                goal.pose.orientation.w = 1
                rospy.loginfo(goal.pose)
                self.move_base_pub.publish(goal)

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


if __name__ == '__main__':
    rospy.init_node('drive2postit', anonymous=True)
    drive2postit = RedPostItFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()