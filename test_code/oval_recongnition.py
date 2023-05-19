import robomaster
from robomaster import config
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera
import cv2
import numpy as np

config.LOCAL_IP_STR = '192.168.10.2'
drone = Drone()
drone.initialize()

camera: Camera = drone.camera
flight: Flight = drone.flight
camera.start_video_stream(display = True)

while True:
    frame = camera.read_cv2_image(strategy = "newest")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 红色
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    mask = mask1 + mask2
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 椭圆识别
    contours, _ = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ellipse = None
    area = []
    for i in range(len(contours)):
        if len(contours[i]) >= 5:
            ell = cv2.fitEllipse(contours[i])
            (center, axis, angle) = ell
            area.append((axis[0]*axis[1], ell))
    max_area = 0
    for i in range(len(area)):
        if area[i][0] > max_area:
            max_area = area[i][0]
            ellipse = area[i][1]
    if ellipse is not None:
        cv2.ellipse(frame, ellipse, (0, 255, 0), 4)
        # 判断椭圆中心在图像中心附近
        center_x, center_y = int(ellipse[0][0]), int(ellipse[0][1])
        if abs(center_x - frame.shape[1] // 2) < 50 and abs(center_y - frame.shape[0] // 2) < 50:
            # 计算旋转角度
            angle = ellipse[2]
            if angle > 90:
                angle -= 180

            # 将无人机飞向椭圆中心并旋转
            if center_x < frame.shape[1] // 2:
                flight.rotate(-20)
            elif center_x > frame.shape[1] // 2:
                flight.rotate(20)
            # if center_y < frame.shape[0]//2:
            #     flight.move_up(20)
            # elif center_y > frame.shape[0]//2:
            #     flight.move_down(20)
            if abs(angle) > 5:
                if angle > 0:
                    flight.rotate(10)
                else:
                    flight.rotate(-10)
            else:
                # 调整无人机位置，使椭圆变成圆形
                axis_x, axis_y = ellipse[1]
                if axis_x > axis_y:
                    flight.left(10)
                elif axis_x < axis_y:
                    flight.right(10)
                if axis_x - axis_y > 50:
                    flight.forward(10)
                elif axis_y - axis_x > 50:
                    flight.backward(10)

    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.stop_video_stream()
drone.close()
cv2.destroyAllWindows()
