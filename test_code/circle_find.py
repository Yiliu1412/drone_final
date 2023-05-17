import robomaster
from robomaster.robot import Drone
from robomaster.camera import Camera
import cv2
import numpy as np

drone = Drone()
drone.initialize()

camera: Camera = drone.camera

# 开始视频流
camera.start_video_stream(display=True)

while True:
    # 读取视频帧
    frame = drone.read_video_frame()
    # 转换颜色空间
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 圆圈识别
    circles = cv2.HoughCircles(frame_gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    # 绘制圆圈
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            # 计算圆心坐标和半径
            center_x, center_y, radius = int(x), int(y), int(r)
            # 判断圆心在图像中心附近
            if abs(center_x - frame.shape[1]//2) < 50 and abs(center_y - frame.shape[0]//2) < 50:
                # 将无人机飞向圆心
                if center_x < frame.shape[1]//2:
                    drone.rotate_ccw(20)
                elif center_x > frame.shape[1]//2:
                    drone.rotate_cw(20)
                if center_y < frame.shape[0]//2:
                    drone.move_up(20)
                elif center_y > frame.shape[0]//2:
                    drone.move_down(20)
                # 判断无人机是否进入圆圈内部
                if radius > 50:
                    drone.forward(20)
                else:
                    drone.back(20)
    
    # 显示视频帧
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 关闭视频流
camera.stop_video_stream()
drone.close
cv2.destroyAllWindows()
