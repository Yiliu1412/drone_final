import robomaster
from robomaster.robot import Drone
import cv2
import cv2.aruco

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
drone = Drone()
drone.initialize()
frame = robomaster.camera.read_cv2_image(timeout=3, strategy="newest")


image = cv2.imread('test2.jpg', cv2.IMREAD_COLOR)
height, width = image.shape[:2]
size = (1280, 720)
image = cv2.resize(image, size)
corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)


if len(corners) > 0:
    ids = ids.flatten()
    for (markerCorner, markerID) in zip(corners, ids):
        # TOP-LEFT, TOP-RIGHT, BOTTOM-RIGHT, BOTTOM-LEFT
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        # Draw margin
        cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
        # Draw ArUCo center (x, y)
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
        # Mark ArUco ID
        cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("[INFO] ArUco marker ID: {}".format(markerID))
        cv2.imshow("Image", image)
        cv2.waitKey(0)
