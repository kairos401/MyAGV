#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from pymycobot.myagv import MyAgv
import time

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower', anonymous=True)

        # AGV 연결
        self.agv = MyAgv(port="/dev/ttyAMA2", baudrate=115200)
        
        # 이미지 구독 설정
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)
        self.bridge = CvBridge()

        # 프레임 레이트 제어 변수
        self.frame_rate = 10
        self.prev = 0

        # 차선을 찾지 못했을 때 대비 검색 상태 변수
        self.searching = False

    def process_image(self, img):
        # HSV 색공간으로 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([28, 125, 98])
        upper_yellow = np.array([42, 255, 239])

        # 이진 마스크 생성
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # 중심 좌표 계산
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy), mask
        return None, mask

    def search_for_lane(self):
        # 왼쪽으로 3초 동안 회전
        for _ in range(3):
            self.agv.counterclockwise_rotation(rotate_left_speed=20, timeout=1)
            rospy.sleep(1)  # 1초 대기
            if not self.searching:
                return

        # 오른쪽으로 6초 동안 회전
        for _ in range(6):
            self.agv.clockwise_rotation(rotate_right_speed=20, timeout=1)
            rospy.sleep(1)  # 1초 대기
            if not self.searching:
                return

    def callback(self, data):
        time_elapsed = time.time() - self.prev
        if time_elapsed > 1./self.frame_rate:
            self.prev = time.time()
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
                return
            
            center, processed_img = self.process_image(cv_image)
            if center and self.searching:
                # 차선을 다시 찾았을 때
                self.searching = False
                rospy.loginfo("Lane detected, resuming normal operation")
            elif not center and not self.searching:
                # 차선을 찾지 못하고 검색 상태가 아닐 때
                rospy.loginfo("Lane not detected, starting search pattern")
                self.searching = True
                self.search_for_lane()
            elif center:
                # 정상적으로 차선을 따라감
                cx, cy = center
                img_center = cv_image.shape[1] / 2
                if abs(cx - img_center) < 150:
                    self.agv.go_ahead(go_speed=20, timeout=0.1)
                    rospy.loginfo("forward")
                elif cx < img_center:
                    self.agv.counterclockwise_rotation(rotate_left_speed=20, timeout=0.2)
                    rospy.loginfo("left")
                else:
                    self.agv.clockwise_rotation(rotate_right_speed=30, timeout=0.2)
                    rospy.loginfo("right")
                rospy.loginfo(f"Center: {center}, Image Center: {img_center}")

if __name__ == '__main__':
    lf = LaneFollower()
    rospy.spin()
