import time
import recognition
from robomaster import config
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera
import cv2
import cv2.aruco
import numpy as np
from robomaster.protocol import TextProtoDrone, TextMsg

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
    def __init__(self, corner, aruco_id):
        self.top_left = corner[0]
        self.top_right = corner[1]
        self.bottom_right = corner[2]
        self.bottom_left = corner[3]
        self.aruco_id = aruco_id

    def get_center(self):
        return ((self.top_left[0] + self.bottom_right[0]) / 2, (self.top_left[1] + self.bottom_right[1]) / 2)

    def get_height(self):
        top_center = ((self.top_left[0] + self.top_right[0]) / 2, (self.top_left[1] + self.top_right[1]) / 2)
        bottom_center = (
            (self.bottom_right[0] + self.bottom_left[0]) / 2, (self.bottom_right[1] + self.bottom_left[1]) / 2)
        return bottom_center[1] - top_center[1]


def recognize_aruco(image, code=None):
    corners, ids, _ = cv2.aruco.detectMarkers(image, ARUCO_DICT, parameters=ARUCO_PARAM)
    if len(corners) <= 0:
        return None
    ids = ids.flatten()

    if code != None:
        for corner, id in zip(corners, ids):
            corner = corner.reshape((4, 2))
            if id == code:
                return ArucoResult(corner, id)
    else:
        result = []
        for corner, id in zip(corners, ids):
            corner = corner.reshape((4, 2))
            result.append(ArucoResult(corner, id))
        return result


code_scale = 2.5
camera_center = (480, 360)
bottom_camera_center = (160, 120)
aruco_distance = 60

config.LOCAL_IP_STR = '192.168.10.2'

current_count = 0
test_count = 0

recognizer = recognition.GestureRecognizer(mode=recognition.VIDEO, debug=True)

drone = Drone()
drone.initialize()
print(drone.get_drone_version())
print(drone.get_sn())

flight: Flight = drone.flight
camera: Camera = drone.camera

camera.start_video_stream(display=True)

flight.takeoff().wait_for_completed(timeout=5)

# wait or gesture

proto = TextProtoDrone()
proto.text_cmd = 'downvision 0'
msg = TextMsg(proto)
drone._client.send_sync_msg(msg)
while True:
    image = camera.read_cv2_image(strategy='newest')
    gesture, count = recognizer.step(image)  # // 3: right and left, % 3 :1, 2, 3
    cv2.imshow('video', image)
    cv2.waitKey(1)

    flight.stop()

    test_count = test_count + 1
    if test_count == 500:
        flight.land().wait_for_completed(timeout=5)
        exit(0)

    print(gesture, count)
    if gesture is not None and count >= 2:
        break
cv2.destroyWindow('video')
# recognizer.release()

print('[info] step 1')
while True:
    image = camera.read_cv2_image(strategy='newest')
    result = recognize_aruco(image, 30 - 10 * (gesture // 3 + 1))
    if result is None:
        continue
    cx, cy = result.get_center()

    distance_square = (cx - camera_center[0]) ** 2 + (camera_center[1] - cy + code_scale * result.get_height()) ** 2
    if distance_square <= 100 ** 2:
        if DEBUG: print('[debug] forward')
        flight.rc(0, 40, 0)  ##
        while True:
            image = camera.read_cv2_image(strategy='newest')
            result = recognize_aruco(image, 30 - 10 * (gesture // 3 + 1))
            if result is None:
                break
        time.sleep(3)
        flight.stop()
        if gesture // 3 == 0:
            flight.left(120).wait_for_completed(timeout=5)
            flight.stop()
        break
    else:
        if DEBUG: print('[debug] calibrate ', distance_square)
        flight.rc((cx - camera_center[0]) * 45 / 480, 0,
                  (camera_center[1] - cy + code_scale * result.get_height()) * 45 / 360)
        time.sleep(1)

print('[info] step 2')
flight.down(50).wait_for_completed(timeout=5)
while True:
    image = camera.read_cv2_image(strategy='newest')
    result = recognize_aruco(image, 30)
    if result is None:
        continue
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
flight.left(180).wait_for_completed(timeout=5)
flight.rotate(35).wait_for_completed(timeout=5)
while True:
    image = camera.read_cv2_image(strategy='newest')
    x, y, r = find_circle_position(image)
    if x is None:
        continue
    if abs(x - camera_center[0]) <= 100:
        flight.rc(0, 45, 0)  ##
        # while True:
        #     image = camera.read_cv2_image(strategy='newest')
        #     x, y, r = find_circle_position(image)
        #     if r >= 260 or r is None:
        #         break
        break
    else:
        flight.rc((x - camera_center[0]) * 35 / 480, 0, 0)

print('[info] land')
gesture = 2
flight.rc(0, 0, 30)
time.sleep(5.5)
flight.stop()
flight.rc(10, 45, 0)
proto = TextProtoDrone()
proto.text_cmd = 'downvision 1'
msg = TextMsg(proto)
drone._client.send_sync_msg(msg)

while True:
    image = camera.read_cv2_image(strategy='newest')
    result = recognize_aruco(image)
    if result is not None:
        flight.stop().wait_for_completed(timeout=2)
        time.sleep(1)
        flight.backward(40).wait_for_completed(timeout=5)
        flight.rotate(45).wait_for_completed(timeout=3)
        break

while True:
    image = camera.read_cv2_image(strategy='newest')
    result = recognize_aruco(image)
    # if result is None:
    #     flight.forward(30).wait_for_completed(timeout=5)
    #     continue
    #
    distance_square = 9999999
    for i in result:
        if distance_square > (i.get_center()[0] - bottom_camera_center[0]) ** 2 + (i.get_center()[1] - bottom_camera_center[1]) ** 2:
            cx, cy = i.get_center()
            aruco_id = i.aruco_id
            distance_square = (i.get_center()[0] - bottom_camera_center[0]) ** 2 + (i.get_center()[1] - bottom_camera_center[1]) ** 2

    distance_square = (cx - bottom_camera_center[0]) ** 2 + (bottom_camera_center[1] - cy) ** 2
    if distance_square <= 20 ** 2:
        if aruco_id == gesture % 3 + 1:
            flight.land()
            break
        elif aruco_id < gesture % 3 + 1:
            flight.right(aruco_distance).wait_for_completed(timeout=3)
        else:
            flight.left(aruco_distance).wait_for_completed(timeout=3)
        continue
    else:
        flight.rc((bottom_camera_center[1] - cy) * 20 / 120, (bottom_camera_center[0] - cx) * 20 / 160, 0)
        time.sleep(0.5)
flight.land()
drone.close()
