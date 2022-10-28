import cv2
import time
import mediapipe as mp
import numpy as np
from djitellopy import Tello

# tello = Tello()

# tello.connect()
# tello.takeoff()

# tello.move_left(100)
# tello.rotate_counter_clockwise(90)
# tello.move_forward(100)

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose


# 웹캠, 영상 파일의 경우 이것을 사용하세요.:
# cap = cv2.VideoCapture(0)


def calculate_angle(a, b, c):
    a = np.array(a)  # First
    b = np.array(b)  # Mid
    c = np.array(c)  # End

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle

if __name__ == "__main__":
    myDrone = Tello()
    myDrone.connect()
    myDrone.takeoff()
    time.sleep(1)
    myDrone.streamon()
    # cv2.namedWindow("drone")
    # frame_read = myDrone.get_frame_read()
    # cap = myDrone.get_frame_read
    time.sleep(2)
    with mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:
        while True:
            # time.sleep(1/30)
            # success, image = myDrone.get_frame_read().frame
            image = myDrone.get_frame_read()

            if image is None:
                print("no camera!!")
                continue
            image = image.frame

            # 필요에 따라 성능 향상을 위해 이미지 작성을 불가능함으로 기본 설정합니다.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image)

            # 포즈 주석을 이미지 위에 그립니다.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
            # 보기 편하게 이미지를 좌우 반전합니다.
            cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
            # print(results.pose_landmarks)
            # Extract landmarks
            try:
                landmarks = results.pose_landmarks.landmark
                # print(landmarks)
            except:
                pass


            shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            # print(calculate_angle(shoulder, elbow, wrist))

            if cv2.waitKey(5) & 0xFF == 27:
                myDrone.streamoff()
                break
    # cap.release()
    



