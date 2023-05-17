import robomaster
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera
import time

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

    flight.forward(distance=50).wait_for_completed()
    flight.backward(distance=50).wait_for_completed()

    flight.rc(a=20, b=0, c=0, d=0)
    time.sleep(3)
    flight.rc(a=-20, b=0, c=0, d=0)
    time.sleep(3)
    flight.rc(a=0, b=0, c=0, d=0)

    flight.land().wait_for_completed()

    camera.stop_video_stream()

    drone.close()
