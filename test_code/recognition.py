import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import cv2
import time

MODEL_PATH = './gesture_recognizer.task'

class Timer:
    def __init__(self):
        self.time_begin = 0
    
    def begin(self):
        self.time_begin = time.time()

    def get_ms(self):
        current_time = time.time()
        return int((current_time - self.time_begin) * 1000)

if __name__ == '__main__':
    options = mp.tasks.vision.GestureRecognizerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=2
    )

    with mp.tasks.vision.GestureRecognizer.create_from_options(options) as recognizer:
        video = cv2.VideoCapture(0)
        width = video.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = video.get(cv2.CAP_PROP_FRAME_HEIGHT)

        timer = Timer()
        
        while True:
            is_grabbed, frame = video.read()
            if not is_grabbed:
                continue

            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

            result = recognizer.recognize_for_video(image, timer.get_ms())
            
            for landmarks in result.hand_landmarks:
                landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                landmarks_proto.landmark.extend([landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in landmarks])
                mp.solutions.drawing_utils.draw_landmarks(
                    frame, landmarks_proto, mp.solutions.hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                    mp.solutions.drawing_styles.get_default_hand_connections_style()
                )
            
            cv2.imshow('video', frame)

            if cv2.waitKey(1) & 0xff == ord('q'):
                break
                
        video.release()
        cv2.destroyAllWindows()
