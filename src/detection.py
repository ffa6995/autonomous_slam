import cv2
import numpy as np
from PIL import Image as im
import rospy
from sensor_msgs.msg import Image, CompressedImage


class Detector:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

    def detection(self):
        cap = cv2.VideoCapture(0)
        count = 0

        ret, frame = cap.read()
        # ret will return a true value if the frame exists otherwise False
        # TODO -- check convert

        test = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        into_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # changing the color format from BGr to HSV
        # This will be used to create the mask
        # TODO -- Adjust limits
        L_limit = np.array([50, 134, 134])  # setting the blue lower limit
        U_limit = np.array([179, 255, 255])  # setting the blue upper limit

        b_mask = cv2.inRange(into_hsv, L_limit, U_limit)
        # creating the mask using inRange() function
        # this will produce an image where the color of the objects
        # falling in the range will turn white and rest will be black
        green = cv2.bitwise_and(frame, frame, mask=b_mask)
        # this will give the color to mask.
        cv2.imshow('Green Detector', green)  # to display the blue object output
        data = im.fromarray(green)
        mask = np.zeros_like(green, dtype=np.uint8)

        # Set all green pixels to 1
        mask[(green > 80) & (green < 100)] = 1
        # Now print percentage of green pixels
        print(mask.mean() * 100)
        if (mask.mean() * 100) > 0.8:
            print("gotit")
            break
        data.save('green_detected.png')
        img_green = im.open('green_detected.png')

        if cv2.waitKey(1) == 27:
            print: "lol"
            break
        # this function will be triggered when the ESC key is pressed
        # and the while loop will terminate and so will the program
    cap.release()

    cv2.destroyAllWindows()

    img_green = cv2.imread('green_detected.png')
    gray = cv2.cvtColor(img_green, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (13, 13), 0)
    thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    # Two pass dilate with horizontal and vertical kernel
    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 5))
    dilate = cv2.dilate(thresh, horizontal_kernel, iterations=2)
    vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 9))
    dilate = cv2.dilate(dilate, vertical_kernel, iterations=2)

    # Find contours, filter using contour threshold area, and draw rectangle
    cnts = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        area = cv2.contourArea(c)
        if area > 20000:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(img_green, (x, y), (x + w, y + h), (36, 255, 12), 3)
    print("x: " + str(x) + " " + "y: " + str(y) + " w: " + str(w) + " h: " + str(h))
    cv2.imshow('image', img_green)
    cv2.waitKey()



