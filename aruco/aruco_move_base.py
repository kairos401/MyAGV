# import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import PoseStamped
# from tf.transformations import quaternion_from_euler
# from tf2_ros import TransformBroadcaster
# import geometry_msgs.msg

# class ArucoMarkerNavigation:
#     def __init__(self):
#         rospy.init_node('aruco_marker_navigation', anonymous=True)
        
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1)
#         self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
#         self.tf_broadcaster = TransformBroadcaster()
        
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#         self.parameters = cv2.aruco.DetectorParameters()
        
#         self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
#         self.dist_coeffs = np.zeros(5)

#     def publish_transforms(self):
#         t = geometry_msgs.msg.TransformStamped()
#         t.header.stamp = rospy.Time.now()
#         t.header.frame_id = "base_link"
#         t.child_frame_id = "camera_link"
#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.1  # 카메라가 base_link에서 10cm 앞에 위치
#         q = quaternion_from_euler(0, 0, 0)  # 카메라가 정면을 바라봄
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
        
#         self.tf_broadcaster.sendTransform(t)

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
#             return
        
#         corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

#         if corners:
#             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
#             rvec = rvec[0][0]
#             tvec = tvec[0][0]
            
#             pose_msg = PoseStamped()
#             pose_msg.header.frame_id = "camera_link"
#             pose_msg.header.stamp = rospy.Time.now()
            
#             pose_msg.pose.position.x = tvec[0]
#             pose_msg.pose.position.y = tvec[1]
#             pose_msg.pose.position.z = tvec[2]
            
#             quaternion = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
#             pose_msg.pose.orientation.x = quaternion[0]
#             pose_msg.pose.orientation.y = quaternion[1]
#             pose_msg.pose.orientation.z = quaternion[2]
#             pose_msg.pose.orientation.w = quaternion[3]
            
#             self.goal_pub.publish(pose_msg)
#             self.publish_transforms()
#         else:
#             rospy.loginfo("No markers detected.")

# if __name__ == '__main__':
#     try:
#         amn = ArucoMarkerNavigation()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
# import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import PoseStamped
# from tf.transformations import quaternion_from_euler
# from tf2_ros import TransformBroadcaster
# import geometry_msgs.msg

# class ArucoMarkerNavigation:
#     def __init__(self):
#         rospy.init_node('aruco_marker_navigation', anonymous=True)
        
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1)
#         self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
#         self.tf_broadcaster = TransformBroadcaster()
        
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#         self.parameters = cv2.aruco.DetectorParameters()
        
#         self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
#         self.dist_coeffs = np.zeros(5)

#     def publish_transforms(self):
#         t = geometry_msgs.msg.TransformStamped()
#         t.header.stamp = rospy.Time.now()
#         t.header.frame_id = "base_link"
#         t.child_frame_id = "camera_link"
#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.1  # 카메라가 base_link에서 10cm 앞에 위치
#         q = quaternion_from_euler(0, 0, 0)  # 카메라가 정면을 바라봄
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
        
#         self.tf_broadcaster.sendTransform(t)

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
#             return
        
#         corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

#         if corners:
#             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
#             rvec = rvec[0][0]
#             tvec = tvec[0][0]
            
#             pose_msg = PoseStamped()
#             pose_msg.header.frame_id = "camera_link"
#             pose_msg.header.stamp = rospy.Time.now()
            
#             pose_msg.pose.position.x = tvec[0]
#             pose_msg.pose.position.y = tvec[1]
#             pose_msg.pose.position.z = tvec[2]
            
#             quaternion = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
#             pose_msg.pose.orientation.x = quaternion[0]
#             pose_msg.pose.orientation.y = quaternion[1]
#             pose_msg.pose.orientation.z = quaternion[2]
#             pose_msg.pose.orientation.w = quaternion[3]
            
#             self.goal_pub.publish(pose_msg)
#             self.publish_transforms()
#         else:
#             rospy.loginfo("No markers detected.")

# if __name__ == '__main__':
#     try:
#         amn = ArucoMarkerNavigation()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

# import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import PoseStamped
# from tf.transformations import quaternion_from_euler
# from tf2_ros import TransformBroadcaster, Buffer, TransformListener
# import geometry_msgs.msg

# class ArucoMarkerNavigation:
#     def __init__(self):
#         rospy.init_node('aruco_marker_navigation', anonymous=True)
        
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1)
#         self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
#         self.tf_broadcaster = TransformBroadcaster()
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer)

#         # 아루코 마커 딕셔너리를 초기화합니다.
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#         self.parameters = cv2.aruco.DetectorParameters()
        
#         self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
#         self.dist_coeffs = np.zeros(5)

#         rospy.Timer(rospy.Duration(0.1), self.publish_transforms)

#     def publish_transforms(self, event=None):
#         t = geometry_msgs.msg.TransformStamped()
#         t.header.stamp = rospy.Time.now()
#         t.header.frame_id = "base_link"
#         t.child_frame_id = "camera_link"
#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.1
#         q = quaternion_from_euler(0, 0, 0)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]

#         self.tf_broadcaster.sendTransform(t)

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
#             return
        
#         corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

#         if corners:
#             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
#             for i in range(len(tvec)):
#                 pose_msg = PoseStamped()
#                 pose_msg.header.frame_id = "camera_link"
#                 pose_msg.header.stamp = rospy.Time.now()

#                 pose_msg.pose.position.x = tvec[i][0][0]
#                 pose_msg.pose.position.y = tvec[i][0][1]
#                 pose_msg.pose.position.z = tvec[i][0][2]

#                 quaternion = quaternion_from_euler(rvec[i][0][0], rvec[i][0][1], rvec[i][0][2])
#                 pose_msg.pose.orientation.x = quaternion[0]
#                 pose_msg.pose.orientation.y = quaternion[1]
#                 pose_msg.pose.orientation.z = quaternion[2]
#                 pose_msg.pose.orientation.w = quaternion[3]

#                 self.goal_pub.publish(pose_msg)
#         else:
#             rospy.loginfo("No markers detected.")

# if __name__ == '__main__':
#     try:
#         amn = ArucoMarkerNavigation()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import geometry_msgs.msg

class ArucoMarkerNavigation:
    def __init__(self):
        rospy.init_node('aruco_marker_navigation', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        self.tf_broadcaster = TransformBroadcaster()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        
        self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros(5)

    def publish_transforms(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        if corners:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
            for i in range(len(tvec)):
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "camera_link"
                pose_msg.header.stamp = rospy.Time.now()

                # Translate the marker position forward by 1 meter to create a goal in front of the marker
                pose_msg.pose.position.x = tvec[i][0][0] + 1.0
                pose_msg.pose.position.y = tvec[i][0][1]
                pose_msg.pose.position.z = tvec[i][0][2]

                quaternion = quaternion_from_euler(0, 0, 0)  # Keep the orientation simple for this example
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

                self.goal_pub.publish(pose_msg)
        else:
            rospy.loginfo("No markers detected.")

if __name__ == '__main__':
    try:
        amn = ArucoMarkerNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
