# import rospy
# import tf2_ros
# import geometry_msgs.msg
# from tf.transformations import quaternion_from_euler

# class ArucoMarkerNavigation:
#     def __init__(self):
#         rospy.init_node('aruco_marker_navigation', anonymous=True)
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster()
#         self.publish_static_transforms()

#     def publish_static_transforms(self):
#         t = geometry_msgs.msg.TransformStamped()
#         t.header.stamp = rospy.Time.now()
#         t.header.frame_id = "base_footprint"  # 또는 base_link
#         t.child_frame_id = "camera_link"
#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.1  # 카메라가 base로부터 10cm 위에 위치
#         q = quaternion_from_euler(0, 0, 0)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]

#         # 주기적으로 변환 정보를 발행
#         rate = rospy.Rate(10)  # 10Hz
#         while not rospy.is_shutdown():
#             t.header.stamp = rospy.Time.now()
#             self.tf_broadcaster.sendTransform(t)
#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         node = ArucoMarkerNavigation()
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
from tf2_ros import TransformBroadcaster
import geometry_msgs.msg
from visualization_msgs.msg import Marker

class ArucoMarkerNavigation:
    def __init__(self):
        rospy.init_node('aruco_marker_navigation', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/marker', Marker, queue_size=10)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        
        self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros(5)

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
                rvec_i = rvec[i][0]
                tvec_i = tvec[i][0]
                marker = Marker()
                marker.header.frame_id = "camera_link"
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = tvec_i[0]
                marker.pose.position.y = tvec_i[1]
                marker.pose.position.z = tvec_i[2]
                quaternion = quaternion_from_euler(rvec_i[0], rvec_i[1], rvec_i[2])
                marker.pose.orientation.x = quaternion[0]
                marker.pose.orientation.y = quaternion[1]
                marker.pose.orientation.z = quaternion[2]
                marker.pose.orientation.w = quaternion[3]
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                self.marker_pub.publish(marker)
        else:
            rospy.loginfo("No markers detected.")

if __name__ == '__main__':
    try:
        amn = ArucoMarkerNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
