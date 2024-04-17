#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

class ArucoTracker:
    def __init__(self):
        self.marker_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.pose_callback)
        self.marker_positions = []

    def pose_callback(self, msg):
        self.marker_positions.append(msg.pose.position)
        if len(self.marker_positions) > 5:
            self.marker_positions.pop(0)

if __name__ == '__main__':
    rospy.init_node('aruco_tracker')
    tracker = ArucoTracker()
    rospy.spin()
