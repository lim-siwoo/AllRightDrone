from enum import Enum
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

#set points (center of the frame coordinates in pixels)
rifX = 960/2
rifY = 720/2
rifZ = 200

#PI constant
Kp_X = 0.1
Ki_X = 0.0
Kp_Y = 0.2
Ki_Y = 0.0
Kp_Z = 0.4
Ki_Z = 0.0

#Loop time
Tc = 0.05

#PI terms initialized
integral_X = 0
error_X = 0
previous_error_X = 0
integral_Y = 0
error_Y = 0
previous_error_Y = 0
integral_Z = 0
error_Z = 0
previous_error_Z = 0

centroX_pre = rifX
centroY_pre = rifY
centroY_pre = rifZ

photoCount = 150
 

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose


State = Enum('State', ['IDLE', 'FORWARD', 'BACKWARD','MOVE_LEFT','MOVE_RIGHT','SHOT','STOP','LAND'])
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

def detectPose(landmarks):
    leftShoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
    leftElbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
    leftWrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]

    rightShoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
    rightElbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
    rightWrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]


    leftAngle = calculate_angle(leftShoulder, leftElbow, leftWrist)
    rightAngle = calculate_angle(rightShoulder, rightElbow, rightWrist)

    leftShoulderAngle =  calculate_angle(rightShoulder, leftShoulder, leftElbow)
    rightShoulderAngle =  calculate_angle(leftShoulder, rightShoulder, rightElbow)

    # print(calculate_angle(leftShoulder, leftElbow, leftWrist))
    # print(calculate_angle(rightShoulder, rightElbow, rightWrist))

    # print(" leftAngle: ", leftAngle)
    # print("rightAngle: ", rightAngle)

    if rightAngle > 70 and rightAngle < 110 and leftAngle > 150 and leftShoulderAngle > 150:
        return 25, 0 #왼쪽이동
    elif leftAngle > 70 and leftAngle < 110 and rightAngle > 150 and rightShoulderAngle > 150:
        return -25, 0 #오른쪽이동
    elif rightAngle > 80 and rightAngle < 100 and leftAngle > 80 and leftAngle < 100 and leftShoulderAngle > 150 and rightShoulderAngle > 150:
        return 0, -1 #착륙
    elif rightAngle > 150 and leftAngle > 150 and leftShoulderAngle > 150 and rightShoulderAngle > 150:
        return 0, 1 #사진 촬영
    else:
        return 0, 0


    

def doCommand(state):
    return 0



if __name__ == "__main__":
    myDrone = Tello()
    myDrone.connect()
    print(myDrone.get_battery())
    myDrone.takeoff()
    myDrone.streamon()
    # cv2.namedWindow("drone")
    # frame_read = myDrone.get_frame_read()
    # cap = myDrone.get_frame_read
    # time.sleep(2)
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
            h,w,channels = image.shape

            if photoCount == 149:
                cv2.imwrite("test.jpg", image)
            elif photoCount < 150:
                photoCount += 1

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
            cv2.circle(image, (int(rifX), int(rifY-100)), 1, (0,0,255), 10)
            # print(results.pose_landmarks)
            # Extract landmarks
            try:
                landmarks = results.pose_landmarks.landmark

                nose = [landmarks[mp_pose.PoseLandmark.NOSE.value].x, landmarks[mp_pose.PoseLandmark.NOSE.value].y, landmarks[mp_pose.PoseLandmark.NOSE.value].z]
                # print(nose)

                #draw the center of the person detected
                if nose[0] < 0:
                    nose[0] = 0
                elif nose[0] > 1:
                    nose[0] = 1

                if nose[1] < 0:
                    nose[1] = 0
                elif nose[1] > 1:
                    nose[1] = 1

                centroX = nose[0] * w
                centroY = nose[1] * h + 100
                centroZ = nose[2] * rifZ*2

                centroX_pre = centroX
                centroY_pre = centroY
                centroZ_pre = centroZ


                cv2.circle(image, (int(centroX), int(centroY-100)), 1, (0,0,255), 10)

                error_X = -(rifX - centroX)
                error_Y = rifY - centroY
                error_Z = rifZ + centroZ

                cv2.line(image, (int(rifX),int(rifY-100)), (int(centroX),int(centroY-100)), (0,255,255),5 )

                cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))


                #PI controller
                integral_X = integral_X + error_X*Tc # updating integral PID term
                uX = Kp_X*error_X + Ki_X*integral_X # updating control variable uX
                previous_error_X = error_X # update previous error variable
                
                integral_Y = integral_Y + error_Y*Tc # updating integral PID term
                uY = Kp_Y*error_Y + Ki_Y*integral_Y
                previous_error_Y = error_Y
                
                integral_Z = integral_Z + error_Z*Tc # updating integral PID term
                uZ = Kp_Z*error_Z + Ki_Z*integral_Z
                previous_error_Z = error_Y

                
                p, action = detectPose(landmarks)
                myDrone.send_rc_control(p,round(uZ),round(uY),round(uX))
                if action == -1:
                    myDrone.streamoff()
                    myDrone.land()
                    myDrone.end()
                    break
                elif action == 1:
                    cv2.imwrite("shot.jpg", image)
                    photoCount = 0
                    print("PHOTE!!!!!!!!!")

                # print(landmarks)
            except:
                pass

            
            

            if cv2.waitKey(5) & 0xFF == 27:
                myDrone.streamoff()
                myDrone.land()
                myDrone.end()
                break
    # cap.release()
    



