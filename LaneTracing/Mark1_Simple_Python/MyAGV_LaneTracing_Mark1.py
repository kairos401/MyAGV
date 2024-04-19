# 파이썬 코드를 이용하여 레인 트레이싱을 처음으로 성공한 코드 (성규님 작성)
# 기존의 파이썬 API를 사용하여 timeout 때문에 매우 짧은 간격으로 틱틱거리며 주행

import cv2
import numpy as np
from pymycobot.myagv import MyAgv
import time

agv = MyAgv(port="/dev/ttyAMA2", baudrate=115200) # AGV 객체 생성
cap = cv2.VideoCapture(0) # 0번(AGV 정면) 카메라 사용

# 카메라 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # 프레임 너비 640으로 설정
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # 프레임 높이 480으로 설정

# Frame Rate 설정
# 설정 이유는 레인 트레이싱을 위해 프레임을 1초에 10번만 받아오기 위함 (1초에 10프레임)
# 라즈베리 파이 CPU가 화면을 감당을 못할수도 있다.

frame_rate = 10
prev = 0 # 이전 시간 초기화

def process_image(img):
    height = img.shape[0] # 이미지 높이
    width = img.shape[1] # 이미지 너비

    roi = img[int(3*height/4):height, 0:width] # 관심 영역 설정 : 밑에서 1/4만큼의 영역

    # HSV 색공간으로 변환
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV) # BGR -> HSV
    lower_yellow = np.array([28,125,98])
    upper_yellow = np.array([42,255,239]) # 준수님께서 제공해 주신 HSV 범위

    # 이진화, 이진화의 이유는 필요한 부분만 추출하기 위함
    # 불필요한 정보로 인한 오류를 줄이며 계산 효율성을 높일 수 있음

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # HSV 이미지에서 노란색만 추출
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8)) # 노이즈 제거

    # 노란색 라인의 중심점을 찾기 위한 코드
    M = cv2.moments(mask) # 노란색 라인의 중심점을 찾기 위한 코드
    if M['moo'] > 0:
        cx = int(M['m10']/M['m00']) # 노란색 라인의 중심점 x좌표
        cy = int(M['m01']/M['m00']) # 노란색 라인의 중심점 y좌표
        return (cx, cy), mask
    return None, mask

while True:
    time_elapsed = time.time() - prev # 현재 시간 - 이전 시간
    ret, frame = cap.read() # 프레임 읽기
    if ret and time_elapsed > 1./frame_rate: # 프레임이 정상적으로 읽어지고, 시간이 1초/프레임레이트보다 크면
        prev = time.time() # 이전 시간을 현재 시간으로 업데이트
        center, process_image = process_image(frame) # 노란색 라인의 중심점과 이진화 이미지를 반환
        if center:
            cx,cy = center
            img_center = frame.shape[1]//2 # 이미지의 중심점
            if abs(cx-img_center) < 150:
                agv.go_ahead(go_speed=20, timeout=0.1) # AGV 전진 (경험적인 속도)
                print("Go Ahead")
            elif cx < img_center:
                agv.counterclockwise_rotation(rotate_left_speed=20, timeout=0.2) # AGV 좌회전 (경험적인 속도)
                print("Turn Left")
            else:
                agv.clockwise_rotation(rotate_right_speed=20, timeout=0.2) # AGV 우회전 (경험적인 속도)
                print("Turn Right")
            print(f"Center: {cx}, {cy}, Image Center : {img_center}")
            if cv2.waitKey(1) & 0xFF == 27:
                break

agv.stop()
cap.release() 
cv2.destroyAllWindows()
