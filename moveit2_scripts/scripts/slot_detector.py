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
        
        # Creating subscriptions and calling callbacks
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # Placeholders to store images/info
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        
    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        # Calling process method if we have valid depth image and camera_info
        if self.depth_image is not None and self.camera_info is not None:
            self.process_images()
    
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
    
    def camera_info_callback(self, msg):
        self.camera_info = msg

    def process_images(self):
        # color_image, depth_image, and camera_info variables
        color_img = self.color_image.copy()
        depth_img = self.depth_image.copy()

        # Converting color to grayscale for circle detection
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # Detecting the large circle
        table_circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=1000,
            param1=100,
            param2=100,
            minRadius=100,
            maxRadius=2000
        )
        
        if table_circles is not None:
            table_circles = np.round(table_circles[0, :]).astype("int")
            table_x, table_y, table_r = table_circles[0] # Only one circle
            
            # Drawing circle
            cv2.circle(color_img, (table_x, table_y), table_r, (0, 255, 0), 2)
        else:
            table_x, table_y, table_r = None, None, None

        # Detecting smaller circles for the slot holes
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.0,
            minDist=50,
            param1=50,
            param2=30,
            minRadius=5,
            maxRadius=50
        )

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            # Emptiness check + coordinates
            for (x, y, r) in circles:
                # Checking emptiness by average color in the circle
                # Creating a mask for the circle
                mask = np.zeros_like(gray, dtype=np.uint8)
                cv2.circle(mask, (x, y), r, 255, -1)
                # Extracting the region of interest
                circle_vals = color_img[mask == 255]
                # Computing mean intensity in BGR
                mean_b = np.mean(circle_vals[:, 0])
                mean_g = np.mean(circle_vals[:, 1])
                mean_r = np.mean(circle_vals[:, 2])
                # Simple average (might change to grayscale average)
                mean_intensity = (mean_b + mean_g + mean_r) / 3.0

                # Threshold
                emptiness_threshold = 120 
                is_empty = (mean_intensity < emptiness_threshold)

                # Getting depth for 3D coordinates 
                # Using an annulus around the slot circle for depth
                annulus_mask = np.zeros_like(depth_img, dtype=np.uint8)
                cv2.circle(annulus_mask, (x, y), int(r*1.2), 255, -1)  # outer boundary
                cv2.circle(annulus_mask, (x, y), r, 0, -1)             # inner boundary
                annulus_depth_vals = depth_img[annulus_mask == 255]

                # Filtering out invalid depth values
                annulus_depth_vals = annulus_depth_vals[annulus_depth_vals > 0]
                if len(annulus_depth_vals) > 0:
                    # Using median depth
                    table_depth = np.median(annulus_depth_vals)
                else:
                    table_depth = 0.0 # Exception

                # Getting 3D coordinates (x, y, z) in camera frame using camera intrinsic parameters from camera_info
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx = self.camera_info.k[2]
                cy = self.camera_info.k[5]

                Z = table_depth
                X = (x - cx) * Z / fx
                Y = (y - cy) * Z / fy

                # Logging results
                self.get_logger().info(
                    f"Slot circle center=({x}, {y}), radius={r}, empty={is_empty}, "
                    f"3D (X, Y, Z)=({X:.2f}, {Y:.2f}, {Z:.2f})"
                )

                # Drawing each circle
                color = (0, 0, 255) if is_empty else (0, 255, 255)
                cv2.circle(color_img, (x, y), r, color, 2)
        
        # Debug visualization
        # cv2.imshow("Slots Detection", color_img)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TableSlotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
