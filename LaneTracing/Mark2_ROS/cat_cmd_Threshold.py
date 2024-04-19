#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower', anonymous=True)
        
        # 이미지 구독 설정
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)
        self.bridge = CvBridge()

        # cmd_vel 퍼블리셔 설정
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 프레임 레이트 제어 변수
        self.frame_rate = 10
        self.prev = 0

    def process_image(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([28, 125, 98])
        upper_yellow = np.array([42, 255, 239])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy), mask
        return None, mask

    def callback(self, data):
        time_elapsed = time.time() - self.prev
        if time_elapsed > 1./self.frame_rate:
            self.prev = time.time()
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
                return
            
            center, _ = self.process_image(cv_image)
            twist = Twist()
            if center:
                cx, cy = center
                img_center = cv_image.shape[1] / 2
                if abs(cx - img_center) < 150:
                    twist.linear.x = 0.2  # 전진
                elif cx < img_center:
                    twist.angular.z = 0.1  # 왼쪽 회전
                else:
                    twist.angular.z = -0.1  # 오른쪽 회전

                self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    lf = LaneFollower()
    rospy.spin()
