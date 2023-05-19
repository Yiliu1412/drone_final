import recognition
from robomaster import config
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera

if __name__ == '__main__':
    drone = Drone()
    drone.initialize()
    print(drone.get_drone_version())
    print(drone.get_sn())

    flight: Flight = drone.flight
    camera: Camera = drone.camera

    recognizer = recognition.GestureRecognizer(mode=recognition.VIDEO, display=True, debug=True)
    camera.start_video_stream(display=False)

    while True:
        frame = camera.read_cv2_image(strategy='newest')

        gesture, count = re.step(frame)
        print('none' if gesture == None else '{} {} {}'.format(count, 'right' if gesture // 3 == 0 else 'left',
                                                               gesture % 3 + 1))

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    camera.stop_video_stream()
    recognizer.release()

    drone.close()
