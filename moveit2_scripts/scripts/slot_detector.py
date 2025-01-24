#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class TableSlotDetector(Node):
    def __init__(self):
        super().__init__('table_slot_detector')
        
        # Subscriptions
        self.color_sub = self.create_subscription(
            Image,
            '/D415/color/image_raw',
            self.color_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/D415/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # Data
        self.color_image = None
        self.camera_info = None
        
    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        if self.camera_info is not None:
            self.process_images()
    
    def camera_info_callback(self, msg):
        self.camera_info = msg

    def process_images(self):
        color_img = self.color_image.copy()

        # Pre processing
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Finding contours
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Fitting ellipses
        for contour in contours:
            if len(contour) < 5:
                continue

            ellipse = cv2.fitEllipse(contour)
            (x, y), (major_axis, minor_axis), angle = ellipse

            # Filtering out unwanted ellipses
            if minor_axis < 20 or major_axis < 20 or minor_axis > 30 or major_axis > 30:
                continue

            # Drawing ellipses and logging detectiom
            cv2.ellipse(color_img, ellipse, (0, 255, 0), 2)
            self.get_logger().info(
                f"Detected ellipse center=({x:.2f}, {y:.2f}), axes=({major_axis:.2f}, {minor_axis:.2f}), angle={angle:.2f}"
            )

        # Visualization
        cv2.imshow("Ellipse Detection", color_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TableSlotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
