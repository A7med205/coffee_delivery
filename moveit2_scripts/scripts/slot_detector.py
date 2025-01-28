#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class TableSlotDetector(Node):
    def __init__(self):
        super().__init__('table_slot_detector')

        # For the TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
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
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error (Color): {e}")
            return

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error (Depth): {e}")
            return

    def process_images(self):
        color_img = self.color_image.copy()

        # Convert to grayscale for brightness analysis and edge detection
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Find contours and fit ellipses
        edges = cv2.Canny(blurred, 80, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if len(contour) < 5:
                continue

            ellipse = cv2.fitEllipse(contour)
            (x, y), (major_axis, minor_axis), angle = ellipse

            # Compute brightness
            mask = np.zeros_like(gray)
            center_int = (int(round(x)), int(round(y)))
            axes_int = (int(round(major_axis / 2.0)), int(round(minor_axis / 2.0)))

            # Draw a filled ellipse on the mask
            cv2.ellipse(
                mask,
                center_int,
                axes_int,
                angle,  # angle in degrees
                0, 360,
                color=255,
                thickness=-1
            )

            # Extract all the grayscale values within that mask
            ellipse_pixels = gray[mask == 255]
            if len(ellipse_pixels) == 0:
                # No valid pixels; skip
                continue

            mean_brightness = np.mean(ellipse_pixels)

            # Decide size range
            scale_factor = None

            if (20 <= minor_axis <= 30 and 20 <= major_axis <= 30):
                # Small range: 10% *larger* ellipse for depth
                scale_factor = 1.10
                
                # Brightness filter
                # if not (50 <= mean_brightness <= 100):
                #     continue

            elif (120 <= minor_axis <= 140 and 120 <= major_axis <= 140):
                # Large range: 10% *smaller* ellipse for depth
                scale_factor = 0.90

                # Brightness filter
                # if not (30 <= mean_brightness <= 80):
                #     continue

            # Size filter
            if scale_factor is None:
                continue

            # Logging brightness
            self.get_logger().info(f"Ellipse brightness (grayscale) = {mean_brightness:.2f}")

            # Drawing the ellipse
            cv2.ellipse(color_img, ellipse, (0, 255, 0), 2)

            # Collecting depth values
            a = (major_axis * scale_factor) / 2.0
            b = (minor_axis * scale_factor) / 2.0
            angle_rad = np.deg2rad(angle)

            N = 360
            depth_values = []
            for i in range(N):
                theta = 2.0 * math.pi * i / N
                # Parametric ellipse point (before rotation):
                ex = a * math.cos(theta)
                ey = b * math.sin(theta)
                
                # Rotate by angle_rad
                x_rot = ex * math.cos(angle_rad) - ey * math.sin(angle_rad)
                y_rot = ex * math.sin(angle_rad) + ey * math.cos(angle_rad)
                
                # Translate to pixel location
                px = int(round(x + x_rot))
                py = int(round(y + y_rot))

                # Check image boundaries
                if (0 <= px < self.depth_image.shape[1] and 
                    0 <= py < self.depth_image.shape[0]):
                    Z = self.depth_image[py, px]
                    Z = float(Z) / 1000.0  # Convert mm to meters

                    # Append valid (non-zero, non-NaN) depth
                    if Z > 0 and not np.isnan(Z):
                        depth_values.append(Z)

            if len(depth_values) < 1:
                continue  # no valid depth

            median_depth = np.median(depth_values)

            # Convert 2D center (x,y) to 3D
            Z = float(median_depth)
            X = (x - self.cx) * Z / self.fx
            Y = (y - self.cy) * Z / self.fy

            # Log the ellipse info
            self.get_logger().info(
                f"Detected ellipse @ center=({x:.2f}, {y:.2f}), "
                f"axes=({major_axis:.2f}, {minor_axis:.2f}), angle={angle:.2f} deg, "
                f"scale_factor={scale_factor} -> "
                f"3D coords=({X:.3f}, {Y:.3f}, {Z:.3f})"
            )

            # Transform to the arm's frame
            camera_point = PointStamped()
            camera_point.header.frame_id = 'D415_color_optical_frame'
            camera_point.header.stamp = self.get_clock().now().to_msg()
            camera_point.point.x = X
            camera_point.point.y = Y
            camera_point.point.z = Z

            try:
                base_point = self.tf_buffer.transform(camera_point, 'base_link', timeout=Duration(seconds=1.0))
                X_base = base_point.point.x
                Y_base = base_point.point.y
                Z_base = base_point.point.z

                self.get_logger().info(
                    f"-> base_link coords=({X_base:.3f}, {Y_base:.3f}, {Z_base:.3f})"
                )
            except Exception as e:
                self.get_logger().warn(f"Transform from D415_link to base_link failed: {e}")

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
