import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class ArucoTracker:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('aruco_tracker', anonymous=True)
        
        # cv_bridge와 이미지 구독자 설정
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=2**24)
        
        # 명령 속도 발행자 설정
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # ArUco 마커 설정
        board_type = cv2.aruco.DICT_6X6_250
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(board_type)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # 카메라 행렬과 왜곡 계수
        self.cmtx = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
        self.dist = np.zeros(5)
        
        # 제어 상수
        self.Kp = 0.001  # 비례 상수
        self.threshold_angle = 10  # 각도 오차 임계값

    def image_callback(self, data):
        try:
            # 이미지를 OpenCV 형식으로 변환
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # 마커 검출
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if corners:
            # 마커 그리기
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.cmtx, self.dist)
            img_center_x = cv_image.shape[1] / 2
            twist = Twist()
            for i, corner in enumerate(corners):
                marker_center_x = np.mean(corner[0], axis=0)[0]
                error_x = marker_center_x - img_center_x
                if abs(error_x) > self.threshold_angle:
                    twist.angular.z = -self.Kp * error_x
                else:
                    twist.angular.z = 0
                distance = np.linalg.norm(tvec[i][0])
                distance_display = f"{distance:.2f} cm"
                cv2.putText(cv_image, distance_display, (int(corner[0][0, 0]), int(corner[0][0, 1]) + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                print(distance)
                if distance > 1:
                    twist.linear.x = 0.3
                else:
                    twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
        else:
            # 마커가 검출되지 않았을 때 로봇 정지
            print("No markers detected. Stopping.")
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

        # 이미지 표시
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    at = ArucoTracker()
    rospy.spin()
