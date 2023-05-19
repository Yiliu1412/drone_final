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

INTERVAL_MS = 700

class Timer:
    def __init__(self):
        self.time_begin = 0
    
    def begin(self):
        self.time_begin = time.time()

    def get_ms(self):
        current_time = time.time()
        return int((current_time - self.time_begin) * 1000)

class Recognizer:
    def __init__(self, model=MODEL_PATH, interval=INTERVAL_MS, debug=False, display=False):
        self.debug = debug
        self.display = display
        self.interval = interval
        
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=2
        )

        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        self.video = cv2.VideoCapture(0)
        self.width = self.video.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.video.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.timer = Timer()
        self.last_time = self.timer.get_ms()

        if self.debug:
            self.fps_last_time = self.timer.get_ms()

        self.gesture_counter = [0 for _ in range(7)]
        self.current_gesture = None
        self.stable_count = 0
    
    def step(self):
        is_grabbed, frame = self.video.read()
        if not is_grabbed:
            return

        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

        result = self.recognizer.recognize_for_video(image, self.timer.get_ms())
        
        hand_count = len(result.handedness)
        finger_state = [None, None]

        for i in range(hand_count):
            landmarks = result.hand_landmarks[i]
            hand_index = result.handedness[i][0].index
            positions = [(int(landmark.x * self.width), int(landmark.y * self.height)) for landmark in landmarks]
            finger_state[hand_index] = [landmarks[tip].y < landmarks[tip - 2].y for tip in FINGER_TIP[1:]]

            if self.display:
                for point1, point2 in mp.solutions.hands.HAND_CONNECTIONS:
                    cv2.line(frame, positions[point1], positions[point2], HAND_COLOR[hand_index], 3)
                for position in positions:
                    cv2.circle(frame, position, 5, HAND_COLOR[hand_index], -1)
        
        if self.debug and self.display:
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
                            self.gesture_counter[-1] += 1
                        else:
                            self.gesture_counter[i * 3 + 2] += 1
                    else:
                        self.gesture_counter[i * 3 + 1] += 1
                else:
                    self.gesture_counter[i * 3 + 0] += 1
        
        if self.timer.get_ms() - self.last_time > self.interval:
            self.last_time = self.timer.get_ms()
            possible_gesture = 0
            for i in range(7):
                if self.gesture_counter[i] > self.gesture_counter[possible_gesture]:
                    possible_gesture = i
            if self.current_gesture == possible_gesture:
                self.stable_count += 1
            else:
                self.stable_count = 0
            self.current_gesture = possible_gesture if possible_gesture < 6 and self.gesture_counter[possible_gesture] > 0 else None
            self.gesture_counter = [0 for _ in range(7)]
        
        if self.debug and self.display:
            text = str(self.stable_count)
            text += 'none' if self.current_gesture == None else '{}{}'.format('right' if self.current_gesture // 3 == 0 else 'left', self.current_gesture % 3 + 1)
            cv2.putText(frame, text, (40, 200), cv2.FONT_HERSHEY_PLAIN, 2, (255, 102, 102), 2)

            cur_time = self.timer.get_ms()
            cv2.putText(frame, '{} fps'.format(round(1000 / (cur_time - self.fps_last_time), 2)), (200, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 102, 102), 2)
            self.fps_last_time = cur_time
            
        if self.display:
            cv2.imshow('video', frame)

        return self.current_gesture, self.stable_count

    def release(self):
        self.release()

        if self.display:
            cv2.destroyWindow('video')

if __name__ == '__main__':
    re = Recognizer(MODEL_PATH, INTERVAL_MS, False, True)
    while True:
        gesture, count = re.step()
        print('none' if gesture == None else '{} {} {}'.format(count, 'right' if gesture // 3 == 0 else 'left', gesture % 3 + 1))

        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    re.release()
