import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class ArucoTracker:
    def __init__(self):
        rospy.init_node('aruco_tracker', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('/processed_image/compressed', CompressedImage, queue_size=1)
        board_type = cv2.aruco.DICT_6X6_250
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(board_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.cmtx = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
        self.dist = np.zeros(5)
        self.error_buffer = np.zeros(3)  # 이동 평균 필터를 위한 버퍼
        self.Kp = 0.0003 # 비례 상수

    def image_callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if corners:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.cmtx, self.dist)
            img_center_x = cv_image.shape[1] / 2
            twist = Twist()
            for i, corner in enumerate(corners):
                marker_center_x = np.mean(corner[0], axis=0)[0]
                current_error = marker_center_x - img_center_x
                # 이동 평균 필터 업데이트
                self.error_buffer = np.roll(self.error_buffer, -1)
                self.error_buffer[-1] = current_error
                filtered_error = np.mean(self.error_buffer)

                # 필터링된 에러로 각속도 계산
                angular_speed = -self.Kp * filtered_error
                distance = np.linalg.norm(tvec[i][0])

                if distance > 0.5:
                    linear_speed = 0.5
                else:
                    linear_speed = 0

                twist.angular.z = angular_speed
                twist.linear.x = linear_speed
                self.cmd_vel_pub.publish(twist)
        else:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

        # 처리된 이미지를 CompressedImage로 변환하고 발행
        try:
            compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
            self.image_pub.publish(compressed_img_msg)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    at = ArucoTracker()
    rospy.spin()
