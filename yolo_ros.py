#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

class ImageProcessor:
    def __init__(self):
        self.node_name = "image_processor"
        rospy.init_node(self.node_name)

        # Initialize YOLO model
        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()

        # Subscriber to compressed image topic
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)
        
        # Example camera parameters (must calibrate your camera to get these values)
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
        self.object_real_size = 0.1  # in meters

    def callback(self, data):
        try:
            # Convert compressed image to OpenCV format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Process image with YOLO model
        results = self.model(cv_image)

        # Iterate over detection results
        for r in results:
            for box in r.boxes:
                if box.cls.item() == 0:
                    x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                    object_size_pixels = max(x2 - x1, y2 - y1)
                    distance = self.estimate_distance(object_size_pixels)

                    # Draw bounding box and distance
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Distance: {distance:.2f}m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the result
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)

    def estimate_distance(self, object_size_pixels):
        object_size_real = object_size_pixels * self.object_real_size / self.object_real_size
        distance = (self.camera_matrix[0, 0] * self.object_real_size) / object_size_real
        return distance

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    image_processor = ImageProcessor()
    image_processor.run()
