#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class TableSlotDetector(Node):
    def __init__(self):
        super().__init__('table_slot_detector')
        
        # Subscriptions
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/D415/color/camera_info',
            self.camera_info_callback,
            10
        )
        self.color_sub = self.create_subscription(
            Image,
            '/D415/color/image_raw',
            self.color_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/D415/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.bridge = CvBridge()

        # Data
        self.camera_info = None
        self.color_image = None
        self.depth_image = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def camera_info_callback(self, msg):
        """
        Store camera intrinsic parameters and process images
        (only if both color_image and depth_image are available).
        """
        self.camera_info = msg

        # Extracting intrinsics
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        # Processing images if all data is ready
        if (self.color_image is not None and 
            self.depth_image is not None and 
            self.camera_info is not None):
            self.process_images()

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error (Color): {e}")
            return

    def depth_callback(self, msg):
        try:
            # Depending on your camera output, might be 16UC1 or 32FC1
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error (Depth): {e}")
            return

    def process_images(self):
        color_img = self.color_image.copy()

        # Pre processing
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Finding contours and fitting ellipses
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if len(contour) < 5:
                continue

            ellipse = cv2.fitEllipse(contour)
            (x, y), (major_axis, minor_axis), angle = ellipse

            # Filtering out unwanted ellipses by size
            if minor_axis < 20 or major_axis < 20 or minor_axis > 30 or major_axis > 30:
                continue

            # Drawing detected ellipses
            cv2.ellipse(color_img, ellipse, (0, 255, 0), 2)

            # Find median depth around the scaled ellipse
            scale_factor = 1.10  # 10% bigger ellipse
            a = (major_axis * scale_factor) / 2.0
            b = (minor_axis * scale_factor) / 2.0
            angle_rad = np.deg2rad(angle)

            # Sampling N points on the perimeter
            N = 360
            depth_values = []
            for i in range(N):
                theta = 2.0 * math.pi * i / N
                # Parametric ellipse point (before rotation):
                ex = a * math.cos(theta)
                ey = b * math.sin(theta)
                
                # Rotating by angle_rad
                x_rot = ex * math.cos(angle_rad) - ey * math.sin(angle_rad)
                y_rot = ex * math.sin(angle_rad) + ey * math.cos(angle_rad)
                
                # Translating to pixel location
                px = int(round(x + x_rot))
                py = int(round(y + y_rot))

                # Checking image resolution boundaries
                if (0 <= px < self.depth_image.shape[1] and 
                    0 <= py < self.depth_image.shape[0]):
                    Z = self.depth_image[py, px]

                    # Checking if depth is valid
                    if Z > 0 and not np.isnan(Z):
                        depth_values.append(Z)

            if len(depth_values) < 1:
                continue # Skip if empty

            median_depth = np.median(depth_values)

            # Converting the 2D center to 3D
            Z = float(median_depth)
            X = (x - self.cx) * Z / self.fx
            Y = (y - self.cy) * Z / self.fy

            # Logging results
            self.get_logger().info(
                f"Detected ellipse @ center=({x:.2f}, {y:.2f}), "
                f"axes=({major_axis:.2f}, {minor_axis:.2f}), angle={angle:.2f} deg "
                f"-> 3D coords=({X:.3f}, {Y:.3f}, {Z:.3f})"
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
