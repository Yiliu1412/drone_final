import mediapipe as mp
import cv2
import time

MODEL_PATH = './gesture_recognizer.task'

HAND_COLOR = [
    (0xff, 0xcc, 0x66),
    (0x1b, 0xb1, 0xff),
    (20, 20, 20)
]

FINGER_TIP = [4, 8, 12, 16, 20]

INTERVAL_MS = 700

class Timer:
    '''
    the timer in milliseconds
    '''
    
    def __init__(self):
        '''
        @constructor Timer
        '''
        self.time_begin = 0
    
    def begin(self):
        '''
        start the timer
        '''
        self.time_begin = time.time()

    def get_ms(self):
        '''
        get current time in milliseconds
        @return {int}
        '''
        current_time = time.time()
        return int((current_time - self.time_begin) * 1000)

IMAGE = mp.tasks.vision.RunningMode.IMAGE
VIDEO = mp.tasks.vision.RunningMode.VIDEO

class GestureRecognizer:
    '''
    the class to recognize the gesture
    '''

    def __init__(self, mode = VIDEO, max_hand = 2, model = MODEL_PATH, interval = INTERVAL_MS, debug = False):
        '''
        @constructor GestureRecognizer
        @param {VIDEO | IMAGE} [mode] - recognize the gesture in image or video mode, defaults to VIDEO
        @param {int} [max_hand] - the maximum hand allowed in the result, defaults to 2
        @param {str} [model] - the path of recognizer model, defaults to MODEL_PATH
        @param {int} [interval] - the recognize interval in milliseconds, defaults to 700
        @param {bool} [debug] - whether paint debug info to the frame, defaults to False
        '''
        self.debug = debug
        self.interval = interval
        self.mode = mode
        
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options = mp.tasks.BaseOptions(model_asset_path = model),
            running_mode = mode,
            num_hands = max_hand,
            min_hand_detection_confidence = 0.5,
            min_hand_presence_confidence = 0.5,
            min_tracking_confidence = 0.5
        )

        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)

        self.timer = Timer()
        self.last_time = self.timer.get_ms()

        if self.debug:
            self.fps_last_time = self.timer.get_ms()

        self.gesture_counter = [0 for _ in range(7)]
        self.current_gesture = None
        self.stable_count = 0

    def step(self, frame):
        '''
        recognize the gesture in the frame
        @param {np.ndarray} frame - the frame data
        @return {(int, int)} (gesture, count) - the gesture id and its continuous count
        '''
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

        if self.mode == VIDEO:
            result = self.recognizer.recognize_for_video(image, self.timer.get_ms())
        else:
            result = self.recognizer.recognize(image)

        hand_count = len(result.handedness)
        finger_state = [None, None]

        for i in range(hand_count):
            landmarks = result.hand_landmarks[i]
            hand_index = result.handedness[i][0].index
            finger_state[hand_index] = [landmarks[tip].y < landmarks[tip - 2].y for tip in FINGER_TIP[1:]]

            if self.debug:
                height, width = frame.shape[:2]
                positions = [(int(landmark.x * width), int(landmark.y * height)) for landmark in landmarks]
                for point1, point2 in mp.solutions.hands.HAND_CONNECTIONS:
                    cv2.line(frame, positions[point1], positions[point2], HAND_COLOR[hand_index], 3)
                for position in positions:
                    cv2.circle(frame, position, 5, HAND_COLOR[hand_index], -1)
        
        if self.debug:
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
        
        if self.debug:
            text = str(self.stable_count)
            text += 'none' if self.current_gesture == None else '{}{}'.format('right' if self.current_gesture // 3 == 0 else 'left', self.current_gesture % 3 + 1)
            cv2.putText(frame, text, (40, 200), cv2.FONT_HERSHEY_PLAIN, 2, (255, 102, 102), 2)

            cur_time = self.timer.get_ms()
            cv2.putText(frame, '{} fps'.format(round(1000 / (cur_time - self.fps_last_time), 2)), (200, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 102, 102), 2)
            self.fps_last_time = cur_time

        return self.current_gesture, self.stable_count
    
    def release(self):
        '''
        release the recognizer
        '''
        self.recognizer.close()

if __name__ == '__main__':
    re = GestureRecognizer(display=True, debug=True, mode=VIDEO)
    video = cv2.VideoCapture(0)
    while True:
        is_grabbed, frame = video.read()
        if not is_grabbed:
            continue
        
        gesture, count = re.step(frame)
        cv2.imshow('video', frame)
        print('none' if gesture == None else '{} {} {}'.format(count, 'right' if gesture // 3 == 0 else 'left', gesture % 3 + 1))

        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cv2.destroyWindow('video')
    re.release()

