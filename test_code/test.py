import time
import recognition
from robomaster import config
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera
import cv2
import cv2.aruco
import numpy as np

DEBUG = True


def find_circle_position(scene_image: np.ndarray):
    hsv = cv2.cvtColor(scene_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255])) + cv2.inRange(hsv, np.array([170, 50, 50]),
                                                                                           np.array([180, 255, 255]))
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 30, param1=None, param2=30, minRadius=30, maxRadius=300)
    x, y, r = circles[0][0]
    return x, y, r


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAM = cv2.aruco.DetectorParameters()


class ArucoResult:
    def __init__(self, corner):
        self.top_left = corner[0]
        self.top_right = corner[1]
        self.bottom_right = corner[2]
        self.bottom_left = corner[3]

    def get_center(self):
        return ((self.top_left[0] + self.bottom_right[0]) / 2, (self.top_left[1] + self.bottom_right[1]) / 2)

    def get_height(self):
        top_center = ((self.top_left[0] + self.top_right[0]) / 2, (self.top_left[1] + self.top_right[1]) / 2)
        bottom_center = (
            (self.bottom_right[0] + self.bottom_left[0]) / 2, (self.bottom_right[1] + self.bottom_left[1]) / 2)
        return bottom_center[1] - top_center[1]


def recognize_aruco(image, code):
    corners, ids, _ = cv2.aruco.detectMarkers(image, ARUCO_DICT, parameters=ARUCO_PARAM)
    if len(corners) <= 0:
        return None
    ids = ids.flatten()

    result = []
    for corner, id in zip(corners, ids):
        corner = corner.reshape((4, 2))
        if code != None and id == code:
            return [ArucoResult(corner)]
    #     result.append(ArucoResult(corner))
    # return result
    return None


code_scale = 2.5
camera_center = (480, 360)

config.LOCAL_IP_STR = '192.168.10.2'

current_count = 0
test_count = 0

recognizer = recognition.GestureRecognizer(mode=recognition.VIDEO, display=True, debug=True)

drone = Drone()
drone.initialize()
print(drone.get_drone_version())
print(drone.get_sn())

flight: Flight = drone.flight
camera: Camera = drone.camera

camera.start_video_stream(display=True)

flight.takeoff().wait_for_completed()

# wait or gesture
while True:
    image = camera.read_cv2_image(strategy='newest')
    gesture, count = recognizer.step(image)  # // 3: right and left, % 3 :1, 2, 3
    cv2.imshow('video', image)
    cv2.waitKey(1)

    flight.stop()

    test_count = test_count + 1
    if test_count == 500:
        flight.land().wait_for_completed()
        exit(0)

    print(gesture, count)
    if gesture is not None and count >= 2:
        break
cv2.destroyWindow('video')
# recognizer.release()

print('[info] step 1')
while True:
    image = camera.read_cv2_image(strategy='newest')
    result = recognize_aruco(image, 10 * (gesture // 3 + 1))
    if result is None:
        continue
    result = result[0]
    cx, cy = result.get_center()

    distance_square = (cx - camera_center[0]) ** 2 + (camera_center[1] - cy + code_scale * result.get_height()) ** 2
    if distance_square <= 100 ** 2:
        if DEBUG: print('[debug] forward')
        flight.rc(0, 40, 0)  ##
        while True:
            image = camera.read_cv2_image(strategy='newest')
            result = recognize_aruco(image, 10 * (gesture // 3 + 1))
            if result is None:
                break
        time.sleep(2.5)
        flight.stop()
        if gesture // 3 == 0:
            flight.left(150).wait_for_completed()
            flight.stop()
        break
    else:
        if DEBUG: print('[debug] calibrate ', distance_square)
        flight.rc((cx - camera_center[0]) * 45 / 480, 0,
                  (camera_center[1] - cy + code_scale * result.get_height()) * 45 / 360)
        time.sleep(1)

print('[info] step 2')
flight.down(50).wait_for_completed()
while True:
    image = camera.read_cv2_image(strategy='newest')
    result = recognize_aruco(image, 30)
    if result is None:
        continue
    result = result[0]
    cx, cy = result.get_center()
    distance_square = (cx - camera_center[0]) ** 2 + (camera_center[1] - cy + code_scale * result.get_height()) ** 2

    # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, np.array([11, 43, 46]), np.array([25, 255, 255]))
    # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 30, param1=None, param2=30, minRadius=30, maxRadius=300)
    # cx, cy, r = circles[0][0]
    if distance_square <= 100 ** 2:
    # if abs(cx - camera_center[0]) <= 100:
        if DEBUG: print('[debug] forward')
        flight.rc(0, 50, 15)  ##
        while True:
            image = camera.read_cv2_image(strategy='newest')
            result = recognize_aruco(image, 30)
            if result is None:
                break
        time.sleep(2)
        flight.stop()
        break
    else:
        if DEBUG: print('[debug] calibrate ', distance_square)
        flight.rc((cx - camera_center[0]) * 45 / 480, 0,
                  (camera_center[1] - cy + code_scale * result.get_height()) * 45 / 360)
        time.sleep(1)
        # flight.rc((cx - camera_center[0]) * 45 / 480, 0, 0)
        # time.sleep(1)

print('[info] step 3')

flight.left(180).wait_for_completed()
flight.rotate(35)
while True:
    image = camera.read_cv2_image(strategy='newest')
    x, y, r = find_circle_position(image)

    if abs(x - camera_center[0]) <= 100:
        flight.rc(0, 35, 0)  ##
        time.sleep(7)  ##

        flight.rotate(45).wait_for_completed()
        flight.stop()
        image = camera.read_cv2_image(strategy='newest')
        cv2.imwrite('land.jpg', image)
        break
    else:
        flight.rc((x - camera_center[0]) * 35 / 480, 0, 0)

print('[info] land')
while True:
    image = camera.read_cv2_image(strategy='newest')
    result = recognize_aruco(image, 30)[0]
    cx, cy = result.get_center()
    break

flight.land()
drone.close()
