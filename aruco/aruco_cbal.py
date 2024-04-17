import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time

class ArucoTracker:
    def __init__(self):
        rospy.init_node('aruco_tracker', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('/processed_image/compressed', CompressedImage, queue_size=1)
        self.board_type = cv2.aruco.DICT_6X6_250
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.board_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.cmtx = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
        self.dist = np.zeros(5)
        self.last_known_position = None  # 마지막 알려진 위치 저장

    def image_callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        twist = Twist()  # Twist 초기화

        if corners:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.cmtx, self.dist)
            img_center_x = cv_image.shape[1] / 2
            img_center_y = cv_image.shape[0] / 2

            for i, corner in enumerate(corners):
                marker_center_x = np.mean(corner[0], axis=0)[0]
                marker_center_y = np.mean(corner[0], axis=0)[1]

                error_x = marker_center_x - img_center_x
                error_y = marker_center_y - img_center_y

                # x, y 선속도 계산
                twist.linear.x = np.clip(0.001 * error_y, -0.2, 0.2)
                twist.linear.y = np.clip(0.001 * error_x, -0.2, 0.2)
                time.sleep(0.1)

                self.last_known_position = twist  # 마지막 위치 저장
        else:
            if self.last_known_position:
                twist = self.last_known_position  # 마지막 알려진 위치로 이동
            else:
                twist.linear.x = 0
                twist.linear.y = 0  # 정지

        self.cmd_vel_pub.publish(twist)  # 명령 발행

        # 처리된 이미지를 CompressedImage로 변환하고 발행
        try:
            compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
            self.image_pub.publish(compressed_img_msg)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    at = ArucoTracker()
    rospy.spin()
