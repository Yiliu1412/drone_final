import mediapipe as mp
import cv2
import time

DEBUG = True

MODEL_PATH = './gesture_recognizer.task'

HAND_COLOR = [
    (0xff, 0xcc, 0x66),
    (0x1b, 0xb1, 0xff),
    (20, 20, 20)
]

FINGER_TIP = [4, 8, 12, 16, 20]

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

        gesture_counter = [0 for _ in range(7)]
        current_gesture = None
        stable_gesture = 0
        last_time = timer.get_ms()
        fps_last_time = timer.get_ms() #DEBUG
        
        while True:
            is_grabbed, frame = video.read()
            if not is_grabbed:
                continue

            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

            result = recognizer.recognize_for_video(image, timer.get_ms())
            
            hand_count = len(result.handedness)
            finger_state = [None, None]

            for i in range(hand_count):
                landmarks = result.hand_landmarks[i]
                hand_index = result.handedness[i][0].index
                positions = [(int(landmark.x * width), int(landmark.y * height)) for landmark in landmarks]
                finger_state[hand_index] = [landmarks[tip].y < landmarks[tip - 2].y for tip in FINGER_TIP[1:]]

                if DEBUG:
                    for point1, point2 in mp.solutions.hands.HAND_CONNECTIONS:
                        cv2.line(frame, positions[point1], positions[point2], HAND_COLOR[hand_index], 3)
                    for position in positions:
                        cv2.circle(frame, position, 5, HAND_COLOR[hand_index], -1)
            
            if DEBUG:
                for i in range(2):
                    if finger_state[i] == None:
                        continue
                    for finger_index in range(4):
                        point = [(10 + i * 100 + finger_index * 20, 10), (25 + i * 100 + finger_index * 20, 30)]
                        cv2.rectangle(frame, point[0], point[1], HAND_COLOR[i] if finger_state[i][finger_index] else HAND_COLOR[-1], -1)

            for i in range(2):
                if finger_state[i] == None:
                    continue
                if finger_state[i][0]:
                    if finger_state[i][1]:
                        if finger_state[i][2]:
                            if finger_state[i][3]:
                                gesture_counter[-1] += 1
                            else:
                                gesture_counter[i * 3 + 2] += 1
                        else:
                            gesture_counter[i * 3 + 1] += 1
                    else:
                        gesture_counter[i * 3 + 0] += 1
            
            if timer.get_ms() - last_time > 1500:
                last_time = timer.get_ms()
                possible_gesture = 0
                for i in range(7):
                    if gesture_counter[i] > gesture_counter[possible_gesture]:
                        possible_gesture = i
                if current_gesture == possible_gesture:
                    stable_gesture += 1
                else:
                    stable_gesture = 0
                current_gesture = possible_gesture if possible_gesture < 6 and gesture_counter[possible_gesture] > 0 else None
                gesture_counter = [0 for _ in range(7)]
            
            if DEBUG:
                text = str(stable_gesture)
                text += 'none' if current_gesture == None else '{}{}'.format('right' if current_gesture // 3 == 0 else 'left', current_gesture % 3 + 1)
                cv2.putText(frame, text, (40, 200), cv2.FONT_HERSHEY_PLAIN, 2, (255, 102, 102), 2)

            if DEBUG:
                cur_time = timer.get_ms()
                cv2.putText(frame, '{} fps'.format(round(1000 / (cur_time - fps_last_time), 2)), (200, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 102, 102), 2)
                fps_last_time = cur_time

            cv2.imshow('video', frame)

            if cv2.waitKey(1) & 0xff == ord('q'):
                break
                
        video.release()
        cv2.destroyAllWindows()
