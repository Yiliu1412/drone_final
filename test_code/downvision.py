from robomaster import config
from robomaster.robot import Drone
from robomaster.flight import Flight
from robomaster.camera import Camera
from robomaster.protocol import TextProtoDrone, TextMsg
import time

if __name__ == '__main__':
    drone = Drone()
    drone.initialize()

    print('Drone SDK Version: {0}'.format(drone.get_sdk_version()))
    print('Drone SN: {0}'.format(drone.get_sn()))

    camera: Camera = drone.camera
    camera.start_video_stream(display = True)
    proto = TextProtoDrone()
    proto.text_cmd = 'downvision 1'
    msg = TextMsg(proto)
    drone._client.send_sync_msg(msg)
    time.sleep(5)
    camera.stop_video_stream()

    drone.close()
