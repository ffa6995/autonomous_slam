import cv2
import numpy as np

# Load the image
img = cv2.imread("C:\\dev\\autonomus_slam\\src\\camera\\src\\live4.jpeg")

# Convert to HSV color space
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define the two ranges of colors in HSV
lower_range1 = np.array([0, 50, 100])
upper_range1 = np.array([15, 255, 255])
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
    max_contour = max(contours, key=cv2.contourArea)




# Find the bounding rectangle of the largest contour
x, y, w, h = cv2.boundingRect(max_contour)

# Draw the bounding rectangle around the square
cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

# Find the corners of the square
corners = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])

# Draw the corners on the image
for corner in corners:
    x, y = corner
    cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

# Show the image
cv2.namedWindow("Square Detection", cv2.WINDOW_NORMAL)
cv2.imshow("Square Detection", img)
cv2.resizeWindow("Square Detection", img.shape[1], img.shape[0])
cv2.waitKey(0)