
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random


# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
# =============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
bridge = CvBridge()
motor = None  # 모터 토픽을 담을 변수

# =============================================
# 프로그램에서 사용할 상수 선언부
# =============================================
CAM_FPS = 30  # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기


# =============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
# =============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")


# =============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
# =============================================
def drive(angle, speed):
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = int(speed)

    motor.publish(motor_msg)


# =============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
# =============================================

# -------변수 선언부------------------------------------------------
Offset = 380  # 이미지의 y축 roi 오프셋 선언
pos_l, pos_r = 125, 525  # 좌우 차선의 중점 좌표 초기 값 선언
pathwidth = pos_r - pos_l  # pathwidth는 우 차선 좌표 - 좌 차선 좌표를 하면 이미지 상에서 도로 폭이 나오게 됨.
Width = 640  # 카메라 이미지 가로 크기 선언
Height = 480  # 카메라 이미지 세로 크기 선언


# -----------------------------------------------------------------

# -------라인 검출 및 차선 좌표 반환 함수--------------------------
# detectLine함수에서는 캐니 에지 검출, ROI 지정 및 추출, 기울기 조건을 추가하여 원하는 차선 검출, 검출 차선 평균계산으로 차선위치를 리턴함.
def detectLine(frame):
    global pos_l, pos_r, pathwidth  # 전역 변수 선언
    src = frame  # src 변수에 frame 복사
    w, h = 130, 60  # ROI를 결정하기 위한 변수 선언

    threshold, minLineLength, maxLineGap = 50, 5, 100  # 허프 변환 함수를 위한 인자 값들 선언
    # HoughLinesP를 사용하여 각각의 ROI 내에서 검출된 직선의 갯수를 num_detect_L과 R에 리스트로 저장한다.
    num_detect_L, num_detect_R = 0, 0  # 검출 직선 갯수를 위한 초기화
    # 허프라인 변환 후에는 같은 직선이어도 많이 검출하게 됨. 이를 막기위해 통일된 좌표 값이 필요한데, 이를 평균 값으로 지정함. 아래 변수들은 각 평균들을 계산하기 위한 변수들의 초기화 부분임.
    l_line_x1, l_line_y1 = 0, 0
    l_line_x2, l_line_y2 = 0, 0
    r_line_x1, r_line_y1 = 0, 0
    r_line_x2, r_line_y2 = 0, 0

    # 캐니 에지 검출 코드, camview영상에서 edge를 추출함
    frame = cv2.Canny(frame, 400, 200, None, 3)
    cv2.imshow("canny", frame)  # 에지 이미지 화면 띄우기

    # ROI 추출 부분
    # ROI를 왼쪽과 오른쪽 차선을 나누는 것이 안정적이고 부드러운 주행이 가능함. 이를 위해 ROI 영역 설정부터 왼쪽, 오른쪽을 나누기로 결정함.
    # 변수 설명: x_L = 왼쪽 ROI에서 검출되는 차선의 가장 왼쪽 좌표
    # x_R은 반대로 오른쪽 ROI에서 검출되는 차선의 가장 왼쪽 좌표
    x_L = pos_l - w / 2  # w가 ROI의 width이기 때문에 왼쪽 차선의 중점에서 w/2를 빼면 x_L일 것임.
    x_R = pos_r - w / 2  # x_L과 마찬가지
    # 제약조건 :  x_L과 x_R이 카메라 가로 프레임을 벗어나면 안되므로, 각각 0보다 작거나 640보다 큰 경우 최대 프레임 좌표로 설정해준다
    # 또한, 프레임의 중간점인 320점을 x_L이 넘어가게 될 경우 오른쪽 차선으로 잘못 인지할 수 있으므로 그런 경우를 배제함. x_R도 마찬가지.
    if x_L < 0:
        x_L = 0
    if x_L > 320:
        x_L = 0
    if x_R > 640:
        x_R = 640
    if x_R < 320:
        x_R = 520

    # 이미지 상에서 왼쪽 ROI와 오른쪽 ROI의 x좌표 y좌표 범위를 지정해줌, 구체적인 ROI영역 설정
    roi_L = frame[int(Offset):int(Offset + h), int(x_L):int(x_L + w)]
    roi_R = frame[int(Offset):int(Offset + h), int(x_R):int(x_R + w)]

    # 왼쪽 ROI와 오른쪽 ROI에 대해 모두 허프 변환 실행, 위에서 선언한 인자값들을 활용.
    linesP_L_1 = cv2.HoughLinesP(roi_L, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)
    linesP_R = cv2.HoughLinesP(roi_R, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)

    # 수평 방향 직선 검출 방지 코드 (의미 없는 차선을 인식하지 않기 위한 조건 제시)
    linesP_L = []  # 왼쪽ROI에서 검출된 직선을 담기위한 리스트 선언
    if linesP_L_1 is not None:  # 왼쪽에서 검출된 모든 Line에 대해 검사
        for i in range(0, len(linesP_L_1)):
            slope = (linesP_L_1[i][0][3] - linesP_L_1[i][0][1])  # y2 - y1 계산
            if np.abs(slope) > 5:  # slope가 5보다 크면
                linesP_L.append(linesP_L_1[i])  # 실제 차선이라고 판단하고 linesP_L List에 넣어줌
    # 마찬가지로 Right line에도 기울기 조건을 추가할 수 있지만 굳이 추가하지 않아도 주행에 큰 문제가 없었음.
    # 아래는 right line에도 적용하는 코드임.
    # linesP_R = []
    # if linesP_R_1 is not None:
    # for i in range(0, len(linesP_R_1)):
    #   slope = (linesP_R_1[i][0][3] - linesP_R_1[i][0][1])
    #  if np.abs(slope) < 5:
    #     linesP_R.append(linesP_R_1[i])
    # 검출된 라인의 평균값 계산
    if linesP_L is not None:  # line L의 평균값 계산, 누적 SUM
        for i in range(0, len(linesP_L)):  # 검출된 left 라인 수에 따라 반복문 수행
            l = linesP_L[i][0]
            l_line_x1 = l_line_x1 + x_L + l[0]
            l_line_y1 = l_line_y1 + Offset + l[1]
            l_line_x2 = l_line_x2 + x_L + l[2]
            l_line_y2 = l_line_y2 + Offset + l[3]
            num_detect_L = num_detect_L + 1  # 검출된 left line 개수 누적

    if linesP_R is not None:  # line R의 평균값 계산, 누적 SUM 위와 마찬가지
        for i in range(0, len(linesP_R)):  # 검출된 right 라인 수에 따라 반복문 수행
            l = linesP_R[i][0]
            r_line_x1 = r_line_x1 + x_R + l[0]
            r_line_y1 = r_line_y1 + Offset + l[1]
            r_line_x2 = r_line_x2 + x_R + l[2]
            r_line_y2 = r_line_y2 + Offset + l[3]
            num_detect_R = num_detect_R + 1  # 검출된 right line 개수 누적

    # 과도하게 많은 직선의 검출, 좌표 값의 큰 변동을 방지하기 위해 left와 right에서 각각 3개이하로 직선이 검출될 때만 pos_l과 pos_r을 업데이트함.
    # 만약 직선의 수가 3개 이상일 때는 업데이트 하지않고, 정확도를 위해 이전에서 사용되었던 차선을 사용.
    if (num_detect_L > 0) & (num_detect_L < 3):  # 3개 이하일 때 조건을 나타내는 if문
        l_line_x1, l_line_y1 = l_line_x1 / num_detect_L, l_line_y1 / num_detect_L  # (x1,y1)의 평균 계산
        l_line_x2, l_line_y2 = l_line_x2 / num_detect_L, l_line_y2 / num_detect_L  # (x2,y2)의 평균 계산
        cv2.line(src, (int(l_line_x1), int(l_line_y1)), (int(l_line_x2), int(l_line_y2)), (0, 0, 255), 3)
        # 원본 이미지의 복사본인 src에 line 함수를 사용하여 차선을 이미지에 그림
        pos_l = (Offset + h / 2 - l_line_y1) * (l_line_x1 - l_line_x2) / (l_line_y1 - l_line_y2) + l_line_x1
        # pos_l에 왼쪽 차선의 평균 좌표를 지나가는 직선 위에 존재하고 y좌표가 ROI의 중심 즉, 처음에 설정한 Offset 변수와 같은 지점의 x좌표를 저장함.

    if (num_detect_R > 0) & (num_detect_R < 3):  # 3개 이하일 때 조건을 나타내는 if문
        r_line_x1, r_line_y1 = r_line_x1 / num_detect_R, r_line_y1 / num_detect_R  # (x1,y1)의 평균 계산
        r_line_x2, r_line_y2 = r_line_x2 / num_detect_R, r_line_y2 / num_detect_R  # (x2,y2)의 평균 계산
        cv2.line(src, (int(r_line_x1), int(r_line_y1)), (int(r_line_x2), int(r_line_y2)), (0, 0, 255), 3)
        # 원본 이미지의 복사본인 src에 line 함수를 사용하여 차선을 이미지에 그림
        pos_r = (Offset + h / 2 - r_line_y1) * (r_line_x1 - r_line_x2) / (r_line_y1 - r_line_y2) + r_line_x1
        # pos_r에 오른쪽 차선의 평균 좌표를 지나가는 직선 위에 존재하고 y좌표가 ROI의 중심 즉, 처음에 설정한 Offset 변수와 같은 지점의 x좌표를 저장함.

    # pos_l과 pos_r은 조향각 제어에 직접적으로 영향을 받게 됨. x_l, x_R과 마찬가지로 이도 제약 조건이 필요함.
    if (pos_l > 320):  # pos_l이 이미지 가로 프레임의 중간을 넘어가면 프레임 가로 중간점으로 설정
        pos_l = 320
    if (pos_l < 0):  # pos_l이 이미지 가로 프레임 왼쪽 끝을 벗어나지 않도록 만드는 조건문
        pos_l = 0
    if (pos_r < 320):  # pos_r이 이미지 가로 프레임의 중간을 넘어가면 프레임 가로 중간점으로 설정
        pos_r = 320
    if (pos_r > 640):  # pos_r이 이미지 가로 프레임 왼쪽 끝을 벗어나지 않도록 만드는 조건문
        pos_r = 640
    ##########################################################################################
    # case1 오른쪽 차선과 왼쪽 차선에서 모두 라인이 검출되었을 경우 pathwidth를 pos_r - pos_l로 저장한다.
    # case2 오른쪽과 왼쪽 중 한 곳에서만 차선이 검출되었을 경우
    # case3 양쪽 차선에서 모두 라인이 검출되지 않았을 경우는 아무것도 하지 않으므로 코드 작성을 하지 않는다.
    #########################################################################################
    if (num_detect_L != 0) & (num_detect_R != 0):  # case1에 대한 경우
        pathwidth = pos_r - pos_l  # pathwidth를 새롭게 저장
    elif (num_detect_L != 0) & (num_detect_R == 0):  # case2에 대한 경우 중 왼쪽만 검출되는 경우
        pos_r = pos_l + pathwidth  # 가장 최근에 둘 다 검출 됐을 때의 pathwidth를 사용하여 pos_r을 계산함. 왼쪽 차선만 감지되었기 때문
    elif (num_detect_L == 0) & (num_detect_R != 0):  # case2에 대한 경우 중 오른쪽만 검출되는 경우
        pos_l = pos_r - pathwidth  # 가장 최근에 pathwidth를 사용하여 pos_l을 계산함. 오른쪽 차선만 감지되었기 때문

    return src, pos_l, pos_r  # detectLine함수에서 3가지 값을 리턴함


def getSteerAng(pos):  # degree를 리턴하여 조향각을 계산하기 위한 함수 선언

    k = 0.21  # 설정한 속도에 적합한 조향 gain값을 실험적으로 선정
    pos_l = pos[0]  # pos 변수의 형식이(pos_l,pos_r)이므로 각 변수에 할당
    pos_r = pos[1]
    center = (pos_l + pos_r) / 2  # 중앙 값을 pos값을 이용하여 계산
    dif = 315 - center  # dif = Width-center인데 좀 더 정확한 주행을 위해 width값을 이 경우 실험적으로 정해줌.
    degree = dif * k  # dif에 조향 gain 값 k를 곱해서 리턴하기 위함
    degree = int(degree)  # degree를 int로 변환
    return degree


# 디버깅을 편안하게 하기 위해 pos_l, pos_r, offset을 사용하여 상자를 쳐줌.
def rectangle(img, pos_l, pos_r, offset=0):  # 상자를 그리는 함수
    # 네개의 상자를 cv2.rectangle을 이용하여 그려줌
    cv2.rectangle(img, (int(pos_l - 5), int(15 + offset)), (int(pos_l + 5), int(25 + offset)), (0, 255, 0), 2) # pos_l지점 표시
    cv2.rectangle(img, (int(pos_r - 5), int(15 + offset)), (int(pos_r + 5), int(25 + offset)), (0, 255, 0), 2)  # pos_r지점 표시
    cv2.rectangle(img, (int((pos_r + pos_l) / 2 - 5),int( 15 + offset)), (int((pos_r + pos_l) / 2 + 5), int(25 + offset)), (0, 255, 0), 2)  # pos_l과 pos_r의 중점 표시
    cv2.rectangle(img, (320 - 5, int(offset + 15)), (320 + 5, int(offset + 25)), (0, 0, 255), 2)  # 이미지 데이터의 가로 좌표의 중점에 중점 표시
    return img


def process_image(frame):  # 이미지를 처리하는 함수 선언
    global Offset
    frame, pos_l, pos_r = detectLine(frame)  # detectLine함수를 이용하여 canny변환에서 얻은 엣지에서 차선을 검출
    if pos_l < 0:  # 위와 마찬가지로 pos_l과 pos_r이 각각 좌우 차선의 좌표 이므로 카메라 프레임 최소, 최대나 중앙을 넘는 경우를 제한해줌
        pos_l = 0
    if pos_r > 640:
        pos_r = 640
    if pos_l > 340:
        pos_r = 0
    if pos_r < 340:
        pos_r = 640
    frame = rectangle(frame, pos_l, pos_r, offset=Offset)  # frame에 상자를 치는 함수를 거쳐 상자친 img를 리턴함

    return (pos_l, pos_r), frame


def start():
    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    # =========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    # =========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    print("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    # =========================================
    # 메인 루프
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행"
    # 작업을 반복적으로 수행함.
    # =========================================
    while not rospy.is_shutdown():

        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장

        img = image.copy()
        pos, frame = process_image(img)  # 위에 선언한 함수를 실행하여 pos, frame 값 할당(차선과, 이미지)
        steer_angle = getSteerAng(pos)  # pos값을 getSteerAng함수에 넣어 steer_angle 변수에 저장 조향각 결정
        if steer_angle > 50:  # degree가 50보다 넘을 경우 50으로 조정, 조향각 제한조건
            steer_angle = 50
        elif steer_angle < -50:  # degree가 -50보다 작을 경우 -50으로 조정
            steer_angle = -50
            # 디버깅을 위해 모니터에 이미지를 디스플레이
        cv2.imshow("CAM View", frame)
        cv2.waitKey(1)

        # 주행속도 설정
        if abs(steer_angle) < 3:  # 직진시 속도 12로 설정. 조향각이 3도보다 작으면 직진으로 간주.
            speed = 12

        else:
            speed = 12 - abs(steer_angle) * 0.2  # 조향각이 3도이상일 경우, 조향각이 커질수록 더 많이 감속하도록 속도 설정.
        
        print(speed)

        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(-steer_angle, speed)


# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임.
# =============================================
if __name__ == '__main__':
    start()
