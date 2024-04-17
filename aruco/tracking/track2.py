#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from math import atan2, sqrt

class ArucoTracker:
    def __init__(self):
        self.marker_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.pose_callback)
        self.marker_positions = []

    def pose_callback(self, msg):
        self.marker_positions.append(msg.pose.position)
        if len(self.marker_positions) > 5:
            self.marker_positions.pop(0)

class MoveToMiddle:
    def __init__(self, tracker):
        self.tracker = tracker
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)  # 10 Hz

    def move_to_middle(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            if len(self.tracker.marker_positions) > 0:
                last_position = self.tracker.marker_positions[-1]
                mid_x = (trans[0] + last_position.x) / 2
                mid_y = (trans[1] + last_position.y) / 2

                angle_to_goal = atan2(mid_y - trans[1], mid_x - trans[0])
                distance = sqrt((mid_x - trans[0])**2 + (mid_y - trans[1])**2)

                twist = Twist()
                while abs(angle_to_goal - tf.transformations.euler_from_quaternion(rot)[2]) > 0.1:
                    twist.angular.z = 1.5 * (angle_to_goal - tf.transformations.euler_from_quaternion(rot)[2])
                    self.cmd_vel_pub.publish(twist)
                    self.rate.sleep()
                    (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                
                twist.angular.z = 0
                while distance > 0.1:
                    twist.linear.x = 0.5 * distance
                    self.cmd_vel_pub.publish(twist)
                    self.rate.sleep()
                    (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    distance = sqrt((mid_x - trans[0])**2 + (mid_y - trans[1])**2)

                twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    rospy.init_node('move_to_middle')
    tracker = ArucoTracker()
    mover = MoveToMiddle(tracker)
    r = rospy.Rate(0.2)  # 5초마다 실행
    while not rospy.is_shutdown():
        mover.move_to_middle()
        r.sleep()
