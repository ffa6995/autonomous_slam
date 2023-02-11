import csv
import os
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import GoalStatusArray
import cv2
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import time
import random
import actionlib
from math import hypot
import tf
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Quaternion
class RedPostItFinder:
    def __init__(self):
        self.sub2 = rospy.Subscriber("/explore/ExplorationDone", Bool, self.callbackExplorationDone)
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.foundsub = rospy.Subscriber("found",Bool,self.found)
        self.bridge = CvBridge()
        self.plansend = False
        self.explorationDone = False
        self.rotate_count = 0
        self.goalsub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.stopwallpub = rospy.Publisher('/stopwall', Bool, queue_size=1)
        self.stopwallpub.publish(True)
        rospy.loginfo("RedPostItFinder started")

    def callbackExplorationDone(self, data):
            self.explorationDone = True
            self.plansend = False
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0.8
            self.cmd_vel_pub.publish(twist)
            self.rotate_count = self.rotate_count + 1
            if(self.rotate_count %400 == 0):
                    self.plansend = False
                    self.rotate_count = 1
            rospy.loginfo("callbackExplorationDone invoced")

    def status_callback(self,msg):
        if len(msg.status_list) > 0:
            status = msg.status_list[0].status
            if status == 3: # goal reached
                if self.plansend:
                    twist = Twist()
                    twist.linear.x = 0
                    twist.angular.z = -0.8
                    self.cmd_vel_pub.publish(twist)
                    self.rotate_count = self.rotate_count + 1
                    if(self.rotate_count %400 == 0):
                        self.plansend = False
                        self.rotate_count = 1
                        self.stopwallpub.publish(False)
            # 9 = abort 4= canchel
            elif status == 9 or status == 4:
                self.plansend = False

    def found(self,data):
        self.plansend = not data
        self.stopwallpub.publish(False)



    def get_current_position(self):
        try:
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo('Failed %s', e)
            return
        current_x = round(trans[0], 2)
        current_y = round(trans[1], 2)
        return (current_x, current_y)


      
    @staticmethod
    def nearest_neighbors(grid_x, grid_y, value_x, value_y, target_x, target_y, k=4):
        """
        Get the weighted average of the k nearest neighbors
        grid_x, grid_y: arrays of x and y coordinates of the grid points
        value_x, value_y: arrays of x and y values of the grid points
        target_x, target_y: x and y coordinates of the target point
        k: number of nearest neighbors to use (default 4)
        """
        # Calculate the distances between the target point and each grid point
        distances = np.sqrt((grid_x - target_x) ** 2 + (grid_y - target_y) ** 2)
        # Add a small value to avoid division by zero
        distances += 1e-10
        # Get the indices of the k nearest grid points
        nearest_indices = np.argsort(distances)[:k]
        # Get the x and y values of the k nearest grid points
        nearest_x = value_x[nearest_indices]
        nearest_y = value_y[nearest_indices]
        # Calculate the weights as the inverse of the distances
        weights = np.reciprocal(distances[nearest_indices])
        # Normalize the weights to sum to 1
        weights /= weights.sum()
        # Calculate the weighted average of the x and y values
        weighted_x = (nearest_x * weights).sum()
        weighted_y = (nearest_y * weights).sum()
        return weighted_x+5, weighted_y

    @staticmethod
    def transform(x_pix, y_pix):
        grid_x = np.array(
            [205, 2, 97, 202, 306, 404, 26, 114, 201, 282, 364, 2, 56, 130, 198, 264, 334, 395, 16, 76, 138, 197, 252, 308,
            362, 409, 36, 91, 145, 194, 244, 292, 340, 392, 2, 40, 104, 151, 193, 236, 279, 322, 363, 399, 4, 57, 113, 154,
            193, 232, 267, 306, 341, 375, 409, 19, 69, 120, 157, 190, 227, 258, 294, 327, 361, 397, 1, 21, 69, 105, 139,
            173, 203, 234, 264, 296, 340, 368, 4, 34, 78, 110, 140, 172, 201, 228, 255, 284, 326, 352, 379, 5, 46, 84, 114,
            141, 171, 195, 223, 247, 277, 316, 340, 366, 15, 43, 92, 118, 143, 169, 193, 219, 242, 268, 304, 328, 351, 3,
            28, 72, 100, 126, 149, 173, 197, 220, 243, 266, 297, 319, 341, 11, 37, 77, 101, 125, 149, 172, 193, 215, 236,
            268, 291, 311, 331, 351, 370, 387, 410, 18, 43, 80, 104, 127, 148, 170, 191, 212, 232, 263, 283, 301, 321, 340,
            360, 376, 397, 4, 28, 48, 86, 106, 129, 148, 171, 189, 210, 230, 257, 277, 294, 312, 332, 350, 366, 387, 404,
            16, 38, 60, 91, 111, 130, 151, 171, 189, 207, 226, 256, 272, 291, 308, 324, 343, 359, 376, 391, 408, 24, 43,
            64, 90, 113, 133, 152, 171, 187, 207, 224, 252, 268, 284, 302, 318, 335, 352, 368, 383, 399, 29, 50, 69, 96,
            116, 132, 152, 170, 187, 204, 221, 246, 263, 280, 296, 310, 327, 342, 359, 373, 388, 35, 54, 72, 96, 118, 135,
            151, 169, 185, 202, 219, 241, 259, 275, 289, 305, 320, 335, 351, 366, 379, 7, 25, 43, 60, 77, 99, 119, 137,
            154, 170, 185, 202, 217, 240, 255, 270, 285, 300, 315, 329, 344, 358, 372, 7, 23, 39, 55, 71, 86, 105, 125,
            140, 156, 170, 184, 198, 213, 230, 247, 260, 275, 287, 301, 317, 331, 344, 357, 18, 34, 48, 63, 77, 90, 112,
            129, 143, 155, 170, 183, 196, 209, 227, 240, 268, 280, 292, 307, 29, 43, 57, 71, 84, 97, 114, 131, 143, 156,
            169, 183, 196, 208, 221, 238, 250, 261, 273, 285, 295, 39, 51, 64, 76, 89, 101, 121, 133, 145, 159, 171, 182,
            193, 205, 219, 233, 245, 256, 266, 277, 289, 48, 58, 71, 83, 95, 106, 122, 134, 148, 158, 168, 180, 192, 203,
            213, 225, 236, 247, 257, 267, 278, 54, 67, 77, 88, 99, 110, 125, 138, 149, 160, 170, 181, 192, 201, 212, 224,
            234, 242, 253, 261, 270])
        grid_y = np.array(
            [308, 307, 309, 308, 308, 308, 252, 251, 250, 251, 248, 201, 201, 203, 203, 202, 202, 199, 166, 168, 167, 169,
            168, 168, 170, 164, 143, 145, 143, 145, 144, 145, 146, 147, 120, 122, 123, 124, 124, 124, 125, 125, 124, 124,
            105, 109, 107, 110, 109, 110, 110, 110, 110, 110, 110, 95, 96, 98, 98, 98, 100, 98, 99, 98, 97, 95, 86, 88, 87,
            89, 89, 89, 89, 89, 90, 89, 89, 90, 78, 78, 80, 80, 80, 81, 81, 81, 81, 81, 81, 81, 83, 70, 71, 71, 72, 72, 75,
            73, 74, 74, 73, 75, 75, 75, 64, 64, 65, 64, 66, 66, 66, 68, 68, 68, 69, 70, 70, 59, 59, 60, 59, 60, 62, 63, 64,
            65, 64, 65, 64, 63, 65, 54, 54, 57, 56, 57, 58, 58, 59, 59, 59, 60, 60, 60, 59, 58, 59, 61, 61, 50, 52, 52, 53,
            53, 52, 53, 53, 54, 55, 55, 56, 55, 55, 55, 55, 57, 57, 43, 47, 47, 49, 48, 49, 49, 49, 50, 51, 51, 51, 52, 53,
            52, 53, 53, 54, 54, 53, 43, 44, 43, 44, 45, 46, 46, 46, 47, 48, 50, 49, 50, 49, 51, 51, 51, 51, 51, 51, 51, 39,
            40, 41, 41, 42, 41, 43, 43, 45, 45, 45, 45, 47, 47, 45, 46, 46, 47, 47, 49, 49, 38, 38, 38, 38, 39, 40, 40, 40,
            42, 41, 41, 43, 43, 43, 44, 44, 44, 45, 45, 46, 46, 34, 34, 35, 35, 37, 37, 37, 37, 40, 40, 39, 41, 42, 42, 42,
            43, 42, 42, 44, 44, 45, 32, 32, 31, 32, 33, 34, 35, 35, 36, 35, 36, 38, 38, 38, 39, 39, 39, 41, 42, 41, 41, 41,
            41, 28, 28, 29, 30, 28, 30, 30, 30, 31, 33, 33, 33, 33, 34, 36, 35, 36, 37, 36, 37, 38, 38, 37, 39, 24, 24, 25,
            25, 26, 26, 26, 28, 29, 28, 29, 30, 30, 31, 31, 30, 31, 33, 33, 34, 22, 22, 23, 23, 23, 23, 24, 24, 25, 25, 26,
            27, 27, 29, 27, 29, 28, 30, 30, 31, 31, 21, 20, 21, 21, 21, 22, 22, 23, 25, 23, 25, 26, 25, 25, 26, 26, 27, 26,
            28, 28, 28, 17, 18, 18, 19, 19, 20, 20, 20, 22, 22, 23, 23, 24, 24, 24, 23, 24, 25, 24, 24, 24, 15, 15, 14, 16,
            17, 17, 17, 18, 19, 19, 20, 20, 20, 21, 22, 21, 23, 23, 23, 24, 23])
        value_x = np.array(
            [0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10, 10, 10, 15, 15, 15, 15, 15, 15, 15, 15, 20, 20, 20, 20,
            20, 20, 20, 20, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 35, 35, 35,
            35, 35, 35, 35, 35, 35, 35, 35, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 45, 45, 45, 45, 45, 45, 45, 45,
            45, 45, 45, 45, 45, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55,
            55, 55, 55, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65,
            65, 65, 65, 65, 65, 65, 65, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 75, 75, 75,
            75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
            80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85,
            85, 85, 85, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 95, 95, 95, 95,
            95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 95, 100, 100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 110, 110, 110, 110, 110, 110, 110,
            110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 120, 120, 120, 120, 120,
            120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 130, 130, 130, 130, 130, 130, 130,
            130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 140, 140, 140, 140, 140, 140, 140, 140,
            140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 140, 150, 150, 150, 150, 150, 150, 150, 150, 150,
            150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160,
            160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160])
        value_y = np.array(
            [0, -10, -5, 0, 5, 10, -10, -5, 0, 5, 10, -15, -10, -5, 0, 5, 10, 15, -15, -10, -5, 0, 5, 10, 15, 20, -15, -10,
            -5, 0, 5, 10, 15, 20, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
            -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, -22.5, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, -25, -20, -15,
            -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -25, -20, -15,
            -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -30, -25,
            -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
            25, 30, 35, 40, 45, 50, 55, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60,
            -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, -35, -30, -25, -20,
            -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10,
            15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40,
            45, 50, 55, 60, 65, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55,
            60, 65, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65,
            -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, -50, -45, -40, -35, -30,
            -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, -50, -45, -40, -35, -30, -25, -20, -15, -10,
            -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15,
            20, 25, 30, 35, 40, 45, 50, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40,
            45, 50, ])
        weighted_x, weighted_y = RedPostItFinder.nearest_neighbors(grid_x, grid_y, value_x, value_y, x_pix, y_pix)

        return weighted_x, weighted_y


    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            self.image_detection(cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)
        

    def image_detection(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define the two ranges of colors in HSV
        lower_range1 = np.array([0, 100, 180])
        upper_range1 = np.array([10, 255, 255])
        lower_range2 = np.array([165, 50, 100])
        upper_range2 = np.array([180, 255, 255])

        # Threshold the HSV image to get only the colors in both ranges
        mask1 = cv2.inRange(hsv, lower_range1, upper_range1)
        mask2 = cv2.inRange(hsv, lower_range2, upper_range2)
        mask = mask2

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_contour = 1
        try:
                # Find the contour with the largest area
                max_contour = max(contours, key=cv2.contourArea)
        except:
            contours, _ = cv2.findContours(mask1+mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours != () and contours != []:
                max_contour = max(contours, key=cv2.contourArea)
        if contours != () and contours != []:
            self.stopwallpub.publish(True)
            # Find the bounding rectangle of the largest contour
            x, y, w, h = cv2.boundingRect(max_contour)
            rospy.logdebug(( x, y ))
            if w > 10 and h > 5:
                self.goal_flag = False
                #STOP THE BOT
                twist = Twist()
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                # Find the corners of the square
                corners = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])

                # Draw the corners on the image
                for corner in corners:
                    x, y = corner

                # convert corners list to numpy array
                corners = np.array(corners)

                # calculate mean of x and y coordinates
                x_mean = np.mean(corners[:, 0])
                y_mean = np.mean(corners[:, 1])

                x_cm, y_cm = RedPostItFinder.transform(int(x_mean), int(y_mean))
                
                if not self.plansend :
                    goal = PoseStamped()
                    goal.header.frame_id = "base_link"
                    goal.pose.position.x = x_cm/100
                    goal.pose.position.y = y_cm/100
                    goal.pose.orientation.w = 1
                    self.goal_flag = True
                    self.move_base_pub.publish(goal)
                    rospy.loginfo("({} pixels, {} pixels) in centimeters is ({:.2f} cm, {:.2f} cm)".format(x_mean, y_mean, x_cm, y_cm))
                    self.plansend = True
        else:
            self.goal_flag = False

if __name__ == '__main__':
    rospy.init_node('drive2postit', anonymous=True)
    global listener
    listener = tf.TransformListener()
    drive2postit = RedPostItFinder()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()