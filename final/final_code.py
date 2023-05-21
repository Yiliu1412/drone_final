import time
import recognition
from robomaster import config
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera
import robomaster.action
import cv2
import cv2.aruco
import numpy as np
from robomaster.protocol import TextProtoDrone, TextMsg
from math import sqrt
from simple_pid import PID

DEBUG = True

code_scale = 2.5
camera_center = (480, 360)
IR_CAMERA_CENTER = (160, 120)
ARUCO_DISTANCE = 40

config.LOCAL_IP_STR = '192.168.10.3'
config.DEFAULT_PROTO_TYPE = 'tcp'

current_count = 0
test_count = 0

def find_circle_position(scene_image: np.ndarray):
    hsv = cv2.cvtColor(scene_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255])) + cv2.inRange(hsv, np.array([170, 50, 50]),
                                                                                           np.array([180, 255, 255]))
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 30, param1=None, param2=30, minRadius=30, maxRadius=300)
    if circles is None:
        return None, None, None
    x, y, r = circles[0][0]
    return x, y, r


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAM = cv2.aruco.DetectorParameters()

def enable_ir_camera(drone: Drone, enable):
    '''
    enable or disable the ir camera
    @param {Drone} drone - the drone object
    @param {bool} enable - true if use the ir camera
    '''
    proto = TextProtoDrone()
    proto.text_cmd = 'downvision 1' if enable else 'downvision 0'
    msg = TextMsg(proto)
    drone._client.send_sync_msg(msg)


class Timer:
    '''
    the timer in milliseconds
    '''

    def __init__(self):
        '''
        @constructor Timer
        '''
        self.time_begin = 0
        self._stop = True

    def begin(self):
        '''
        start the timer
        '''
        self.time_begin = time.time()
        self._stop = False

    def stop(self):
        self._stop = True

    def is_stop(self):
        return self._stop

    def get_ms(self):
        '''
        get current time in milliseconds
        @return {int}
        '''
        if self._stop:
            return -1
        current_time = time.time()
        return int((current_time - self.time_begin) * 1000)


class ArucoResult:
    '''
    the result of aruco recognition
    '''

    def __init__(self, corner, aruco_id):
        '''
        @constructor ArucoResult
        @param {(float, float)[4]} corner - a list of 4 points represents the 4 corners
        @param {str} aruco_id - the aruco id
        '''
        self.top_left = corner[0]
        self.top_right = corner[1]
        self.bottom_right = corner[2]
        self.bottom_left = corner[3]
        self.aruco_id = aruco_id

    def get_center(self):
        '''
        get the center position
        @return {(float, float)} (x, y)
        '''
        return ((self.top_left[0] + self.bottom_right[0]) / 2, (self.top_left[1] + self.bottom_right[1]) / 2)

    def get_height(self):
        '''
        get the height
        @return {float}
        '''
        top_center = ((self.top_left[0] + self.top_right[0]) / 2, (self.top_left[1] + self.top_right[1]) / 2)
        bottom_center = (
        (self.bottom_right[0] + self.bottom_left[0]) / 2, (self.bottom_right[1] + self.bottom_left[1]) / 2)
        return bottom_center[1] - top_center[1]

    def get_height2(self):
        '''
        get the height
        @return {float}
        '''
        top_center = ((self.top_left[0] + self.top_right[0]) / 2, (self.top_left[1] + self.top_right[1]) / 2)
        bottom_center = (
        (self.bottom_right[0] + self.bottom_left[0]) / 2, (self.bottom_right[1] + self.bottom_left[1]) / 2)
        return bottom_center[0] - top_center[0]


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAM = cv2.aruco.DetectorParameters()

def recognize_aruco(image, code=None):
    '''
    recognize all the aruco codes or a specific aruco code in the image
    @param {np.ndarray} image - the image data
    @param {None | int} [code] - None to get all the codes, or a specific code, defaults to None
    @return {ArucoResult[] | ArucoResult} - the array of result if code is None, otherwise return the result
    '''
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

        if len(result) == 0:
            return None
        return result


def outer_bound(x, k, d):
    if x <= d and x >= -d: return 0
    if x >= d and x < k: return k
    if x <= -d and x > -k: return -k
    return x

def calibrate(flight: Flight, aruco_id, threshold, down=False):
    x_pid = PID(0.55, 0.10, 0.25, setpoint=0, output_limits=(-30, 30))
    y_pid = PID(0.55, 0.10, 0.25, setpoint=0, output_limits=(-30, 30))

    z_speed = 0
    flag = False
    timer = Timer()
    timer_down = Timer()
    counter = 0
    while True:
        image = camera.read_cv2_image(strategy='newest')
        result = recognize_aruco(image, aruco_id)
        if result is None:
            print('not found ', counter)
            counter += 1
            if counter >= 10:
                break
            continue

        cx, cy = result.get_center()

        distance_square = (cx - IR_CAMERA_CENTER[0]) ** 2 + (cy - IR_CAMERA_CENTER[1]) ** 2
        if distance_square <= threshold ** 2:
            # z_speed = -10
            print('[ddddd] get height', result.get_height(), result.get_height2(), distance_square)
            if timer.is_stop():
                print('waiting')
                timer.begin()
            elif timer.get_ms() >= 2000:
                if not flag and down:
                    z_speed = -20
                    timer_down.begin()
                    flag = True
                    continue
                if timer_down.is_stop():
                    print('break')
                    break
                else:
                    if timer_down.get_ms() >= 6000:
                        timer_down.stop()
        else:
            print('timer stop')
            timer.stop()

        if timer_down.get_ms() >= 5000: ##
            z_speed = 0

        dx = x_pid(cx - IR_CAMERA_CENTER[0])
        dy = y_pid(cy - IR_CAMERA_CENTER[1])
        dx = outer_bound(dx, 15, 9)
        dy = outer_bound(dy, 15, 9)

        cv2.circle(image, (int(cx), int(cy)), 3, (255, 0, 0), -1)
        cv2.circle(image, (int(IR_CAMERA_CENTER[0]), int(IR_CAMERA_CENTER[1])), 3, (0, 255, 0), -1)
        cv2.circle(image, (int(cx + dx), int(cy + dy)), 3, (0, 0, 255), -1)
        cv2.imshow('video', image)
        cv2.waitKey(1)
        print('dcx={} dcy={} dx={} dy={}'.format(cx - IR_CAMERA_CENTER[0], cy - IR_CAMERA_CENTER[1], dx, dy))

        flight.rc(dy, dx, z_speed)


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
enable_ir_camera(drone, False)
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
    gesture_aruco_id = 10 * (gesture // 3 + 1)
    another_gesture_aruco_id = 30 - 10 * (gesture // 3 + 1)
    result = recognize_aruco(image, gesture_aruco_id)
    if result is None:
        another_result = recognize_aruco(image, another_gesture_aruco_id)
        if another_result is None:
            print('没救了')
        elif another_gesture_aruco_id == 10:
            flight.stop().wait_for_completed(timeout=1)
            flight.right(80).wait_for_completed(timeout=5)
            time.sleep(1)
        else:
            flight.stop().wait_for_completed(timeout=1)
            flight.left(80).wait_for_completed(timeout=5)
            time.sleep(1)
        continue
    cx, cy = result.get_center()

    distance_square = (cx - camera_center[0]) ** 2 + (camera_center[1] - cy + code_scale * result.get_height()) ** 2
    if distance_square <= 100 ** 2:
        if DEBUG: print('[debug] forward')
        flight.rc(0, 40, 0)  ##
        while True:
            image = camera.read_cv2_image(strategy='newest')
            result = recognize_aruco(image, gesture_aruco_id)
            if result is None:
                break
        time.sleep(3)
        flight.stop()
        # if gesture // 3 == 1: ##!!!!!!!!!!!!!!!!!!!!!
        #     flight.stop().wait_for_completed(timeout=1)
        #     time.sleep(1)
        #     flight.left(100).wait_for_completed(timeout=5)
        #     flight.stop()
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
    if result is None: # ?
        flight.stop().wait_for_completed(timeout=1)
        time.sleep(1)
        flight.left(30).wait_for_completed(timeout=3)
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
        flight.rc(0, 50, 14)  ##
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
flight.rotate(30).wait_for_completed(timeout=5)

while True:
    image = camera.read_cv2_image(strategy='newest')
    x, y, r = find_circle_position(image)
    if x is None:
        continue
    circle_image = cv2.circle(image, (int(x), int(y)), int(r), (0, 0, 255), 5)
    cv2.imshow('six', circle_image)
    cv2.waitKey(1)
    if abs(x - camera_center[0]) <= 80:
        flight.stop()
        time.sleep(2)
        flight.rc(0, 45, 0)  ##
        # while True:
        #     image = camera.read_cv2_image(strategy='newest')
        #     x, y, r = find_circle_position(image)
        #     if r is None or r >= 260:
        #         break
        # time.sleep(3)
        # flight.rc(10, 45, 0)
        break
    else:
        flight.rc((x - camera_center[0]) * 35 / 480, 0, 0)


print('[info] land')

enable_ir_camera(drone, True)

# gesture = 1
# # flight.up(80, retry=True).wait_for_completed(timeout=5)
# flight.rc(0, 0, 45)
# time.sleep(4.5)
# flight.stop()
# flight.rc(15, 40, 0)

while True:
    image = camera.read_cv2_image(strategy='newest')
    results = recognize_aruco(image)
    if results is not None:
        flight.stop().wait_for_completed(timeout=1)
        # time.sleep(1)
        # flight.backward(30).wait_for_completed(timeout=5)
        break

# image = camera.read_cv2_image(strategy='newest')
# image = cv2.transpose(image)
# image = cv2.flip(image, 1)
# aruco_results = recognize_aruco(image)
# aruco_result = None
# min_d2 = 0x3f3f3f3f
# for result in aruco_results:
#     cx, cy = result.get_center()
#     current_d2 = (cx - IR_CAMERA_CENTER[0]) ** 2 + (cy - IR_CAMERA_CENTER[1]) ** 2
#     if current_d2 < min_d2:
#         aruco_result = result
#         min_d2 = current_d2
# cx, cy = aruco_result.get_center()
# vector_x, vector_y = normalize(cx, cy, 4)
# print(vector_x, vector_y)
# flight.rc(vector_x, vector_y, 0)
#
# while True:
#     if (cx - IR_CAMERA_CENTER[0]) ** 2 + (cy - IR_CAMERA_CENTER[1]) ** 2 <= 24 ** 2:
#         flight.stop()
#         break


flight.stop().wait_for_completed(timeout=1)
time.sleep(1)
flight.rotate(47).wait_for_completed(timeout=3)
# time.sleep(10)
flight.stop()

while True:
    image = camera.read_cv2_image(strategy='newest')
    aruco_results = recognize_aruco(image)
    if aruco_results is None:
        flight.stop().wait_for_completed(timeout=1)
        time.sleep(1)
        flight.left(30).wait_for_completed(timeout=3)
        continue
    break


# get the nearest aruco code
aruco_result = None
min_d2 = 0x3f3f3f3f
for result in aruco_results:
    cx, cy = result.get_center()
    current_d2 = (cx - IR_CAMERA_CENTER[0]) ** 2 + (cy - IR_CAMERA_CENTER[1]) ** 2
    if current_d2 < min_d2:
        aruco_result = result
        min_d2 = current_d2

# fit to the nearest aruco code
aruco_id = aruco_result.aruco_id
# flag = False
# first PID
calibrate(flight, aruco_id, 30)

while True:
    if aruco_id == gesture % 3 + 1:
        print('last calibrate')
        calibrate(flight, aruco_id, 12, True)
        flight.rc(0, 0, 0)
        flight.stop().wait_for_completed(timeout=1)
        time.sleep(1)
        flight.land()
        print('FOUNDED NOW LOAND')
        flight.rc(0, 0, 0)
        flight.stop().wait_for_completed(timeout=1)
        time.sleep(2)  # !important
        flight.land().wait_for_completed(timeout=5)
        break

    elif aruco_id < gesture % 3 + 1:
        # change to the correct aruco
        print('LEFT 60')
        flight.stop().wait_for_completed(timeout=1)
        time.sleep(1)
        flight.left(ARUCO_DISTANCE).wait_for_completed(timeout=3)
        aruco_id += 1
        calibrate(flight, aruco_id, 30)
    else:
        print('RIGHT 60')
        flight.stop().wait_for_completed(timeout=1)
        time.sleep(1)
        flight.right(ARUCO_DISTANCE).wait_for_completed(timeout=3)
        aruco_id -= 1
        calibrate(flight, aruco_id, 30)
    # else:
    #     # print(IR_CAMERA_CENTER[0] - cx, (IR_CAMERA_CENTER[1] - cy), (-(cy - IR_CAMERA_CENTER[1]) * 20 / 120), (-(cx - IR_CAMERA_CENTER[0]) * 20 / 120))
    #     # flight.stop().wait_for_completed(timeout=1)
    #     #flight.rc(-10, 0, 0)
    #     print((cx - IR_CAMERA_CENTER[0]) * 15 / 120, (IR_CAMERA_CENTER[1] - cy) * 15 / 120, aruco_id)
    #
    #     time.sleep(0.5)

camera.stop_video_stream()
drone.close()
