#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import atexit

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower', anonymous=True)

        # 이미지 구독 설정
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)
        self.bridge = CvBridge()

        # cmd_vel 퍼블리셔 설정
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 프로그램 종료 시 호출될 함수 등록
        atexit.register(self.stop_agv)

    def process_image(self, img):
        # HSV 색공간으로 변환
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
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            result = self.process_image(cv_image)
            twist = Twist()
            if result:
                cx, cy = result[0]
                img_center = cv_image.shape[1] / 2
                deviation = cx - img_center
                twist.linear.x = 0.3  # 기본 전진 속도
                twist.angular.z = -float(deviation) / 300  # 각속도 조절
            else:
                twist.angular.z = 0.1  # 차선을 찾지 못했을 때 오른쪽으로 회전
            self.cmd_vel_pub.publish(twist)
        except CvBridgeError as e:
            rospy.logerr(e)

    def stop_agv(self):
        print("Stopping AGV...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        lf = LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
