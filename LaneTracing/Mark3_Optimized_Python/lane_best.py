import cv2
import numpy as np
from pymycobot.myagv import MyAgv
import time

class CustomAgv(MyAgv):
    def __init__(self, port="/dev/ttyAMA2", baudrate="115200", timeout=0.1):
        super().__init__(port, baudrate, timeout)

    def stop(self):
        super().stop()

agv = CustomAgv(port="/dev/ttyAMA2", baudrate="115200", timeout=0.05)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
prev_command = 0    # 이전 명령 상태 저장
_f = False          # 좌회전, 우회전 플래그

def process_image(img):
    roi = img[int(2 * img.shape[0] / 3):, :]  #웹캠의 경우 roi 하단부 2/3까지   // agv캠의 경우 roi 하단부 1/4까지 
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([28, 125, 98]), np.array([42, 255, 239]))
    M = cv2.moments(mask)
    return int(M['m10'] / M['m00']) if M['m00'] > 0 else None

while True:
    ret, frame = cap.read()
    cx = process_image(frame)
    current_command = 0
    if cx is not None:
        img_center = frame.shape[1] // 2
        if abs(cx - img_center) < 30:
            current_command = 1  # forward
        elif cx < img_center:
            current_command = 2  # left
        elif cx > img_center:
            current_command = 3  # right
    else:
        current_command = 4  # back

    if current_command != prev_command:
        if current_command == 1:
            agv._mesg(128, 128, 128)
            agv._mesg(250, 128, 128)  # forward
            print(1)
        elif current_command == 2:
            agv._mesg(128, 128, 128)
            agv._mesg(170,168,255)  # left (5, 3, 5)
            print(2)
        elif current_command == 3:
            agv._mesg(128, 128, 128)
            agv._mesg(170,88,1)  # right (5, -3, -5)
            print(3)
        elif current_command == 4:
            agv._mesg(128, 128, 128)
            agv._mesg(127, 128, 128)  # back
            print(4)
        prev_command = current_command  # 현재 명령을 이전 명령으로 저장

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

agv.stop()
cap.release()
cv2.destroyAllWindows()