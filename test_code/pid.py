import time
from robomaster import config
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera
from robomaster.protocol import TextProtoDrone, TextMsg
from simple_pid import PID
import cv2
import cv2.aruco
import numpy as np

DEBUG = True
ARUCO_SCALE = 2.5
ARUCO_DISTANCE = 60
CAMERA_CENTER = (480, 360)
IR_CAMERA_CENTER = (160, 120)

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
        bottom_center = ((self.bottom_right[0] + self.bottom_left[0]) / 2, (self.bottom_right[1] + self.bottom_left[1]) / 2)
        return bottom_center[1] - top_center[1]
    
    def get_height2(self):
        '''
        get the height
        @return {float}
        '''
        top_center = ((self.top_left[0] + self.top_right[0]) / 2, (self.top_left[1] + self.top_right[1]) / 2)
        bottom_center = ((self.bottom_right[0] + self.bottom_left[0]) / 2, (self.bottom_right[1] + self.bottom_left[1]) / 2)
        return bottom_center[0] - top_center[0]

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAM = cv2.aruco.DetectorParameters()

def recognize_aruco(image, code = None):
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
        return result

def outer_bound(x, k):
    if x >= 0 and x < k: return k
    if x <= 0 and x > -k: return -k
    return x

if __name__ == '__main__':
    drone = Drone()
    drone.initialize()
    
    flight: Flight = drone.flight
    camera: Camera = drone.camera

    # print info
    print("Drone SDK Version: {0}".format(drone.get_sdk_version()))
    print("Drone SN: {0}".format(drone.get_sn()))

    camera.start_video_stream(display=True)

    flight.takeoff().wait_for_completed()
    enable_ir_camera(drone, True)

    flight.up(70).wait_for_completed()
    flight.stop().wait_for_completed(timeout = 1)

    x_pid = PID(0.5, 0.04, 0.3, setpoint = 0, output_limits = (-30, 30))
    y_pid = PID(0.5, 0.04, 0.3, setpoint = 0, output_limits = (-30, 30))

    z_speed = 0
    flag = False
    timer = Timer()
    counter = 0
    while True:
        image = camera.read_cv2_image(strategy = 'newest')
        result = recognize_aruco(image, 3)
        if result is None:
            print('not found ', counter)
            counter += 1
            if counter >= 10:
                break
            continue

        cx, cy = result.get_center()

        distance_square = (cx - IR_CAMERA_CENTER[0]) ** 2 + (cy - IR_CAMERA_CENTER[1]) ** 2
        if distance_square <= 12 ** 2:
            #z_speed = -10
            print('[ddddd] get height', result.get_height(), result.get_height2(), distance_square)
            if timer.is_stop():
                timer.begin()
            elif timer.get_ms() >= 1500:
                if not flag:
                    flight.down(70).wait_for_completed() ##
                    flight.stop().wait_for_completed(timeout = 1)
                    flag = True
                    continue
                print('break')
                break
            #if distance_square <= 4 ** 2:
            #    print('break')
            #    break
        else:
            print('timer stop')
            timer.stop()
        
        dx = x_pid(cx - IR_CAMERA_CENTER[0])
        dy = y_pid(cy - IR_CAMERA_CENTER[1])
        dx = outer_bound(dx, 5)
        dy = outer_bound(dy, 5)

        cv2.circle(image, (int(cx), int(cy)), 3, (255, 0, 0), -1)
        cv2.circle(image, (int(IR_CAMERA_CENTER[0]), int(IR_CAMERA_CENTER[1])), 3, (0, 255, 0), -1)
        cv2.circle(image, (int(cx + dx), int(cy + dy)), 3, (0, 0, 255), -1)
        cv2.imshow('video', image)
        cv2.waitKey(1)
        print('dcx={} dcy={} dx={} dy={}'.format(cx - IR_CAMERA_CENTER[0], cy - IR_CAMERA_CENTER[1], dx, dy))

        flight.rc(dy, dx, 0)
    
    print('???')
    flight.rc(0, 0, 0)
    flight.stop().wait_for_completed(timeout = 1)
    time.sleep(2)
    flight.land().wait_for_completed(timeout = 5)
    camera.stop_video_stream()
    drone.close()
    cv2.waitKey()
