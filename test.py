import cv2
import numpy as np

img = cv2.imread("test.png", cv2.IMREAD_COLOR)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255])) + cv2.inRange(hsv, np.array([170, 50, 50]),
                                                                                       np.array([180, 255, 255]))

cv2.imshow('color0', mask)
cv2.waitKey()

circles = cv2.HoughCircles(
    mask, cv2.HOUGH_GRADIENT, 1, 30,
    param1 = None, param2 = 30,
    minRadius = 10, maxRadius = 0
)
count = 0
circle = [0, 0, 0]

# !!! SHOULD THE FOLLOWING CODE BE IN THE WHILE LOOP???????
circles = list(circles)
# find the circle with maximum radius
circles.sort(key = lambda x:x[0][2], reverse = True)
circle += circles[0][0]
count += 1

# !!! WHAT ARE YOU FUCKING DOING
# trying to get the average circle
if (count >= 10):
    circle = circle / count

    # for circle in circles[0]:
# UNUSED
x = int(circle[0])
y = int(circle[1])
r = int(circle[2])
rgb_pic = cv2.circle(img, (x, y), r, (0, 0, 255), 3)  # 显示圆
rgb_pic = cv2.circle(rgb_pic, (x, y), 2, (255, 255, 0), -1)  # 显示圆心
cv2.imshow('new', rgb_pic)
cv2.waitKey()
cv2.destroyAllWindows()
