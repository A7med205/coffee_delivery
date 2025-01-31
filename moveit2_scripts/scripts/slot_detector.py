#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PointStamped
from scipy.linalg import orthogonal_procrustes
import tf2_geometry_msgs
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
from moveit2_scripts.msg import IntCommand, DisplayPos


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

        # New subscriber for /command_topic
        self.command_sub = self.create_subscription(
            IntCommand,
            '/command_topic',
            self.command_callback,
            10
        )

        # Publisher for /display_
        self.display_pub = self.create_publisher(
            DisplayPos,
            '/display_',
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

        # Store the last integer command received
        self.command_val = None

    def camera_info_callback(self, msg):
        """
        Store camera intrinsic parameters but do NOT process images here.
        """
        self.camera_info = msg

        # Extracting intrinsics
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

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

    def command_callback(self, msg):
        """
        When a command is received on /command_topic, only process images
        if the integer is 0 AND all the required data (color, depth, camera_info) is ready.
        """
        self.command_val = msg.data
        self.get_logger().info(f"Received command: {self.command_val}")

        if self.command_val == 0:
            if (self.color_image is not None and
                self.depth_image is not None and
                self.camera_info is not None):
                # Now we do the image processing
                self.process_images()

    def transform_point(self, x, y):
        # Defining the measured and true points
        measured_points = np.array([
            [-0.385, -0.059],  # Point A
            [-0.516, -0.072],  # Point C
            [-0.530, 0.034],   # Point D
        ])

        true_points = np.array([
            [-0.340, -0.043],  # Point A2
            [-0.467, -0.039],  # Point C2
            [-0.468, 0.067],   # Point D2
        ])

        # Calculating centroids
        measured_centroid = np.mean(measured_points, axis=0)
        true_centroid = np.mean(true_points, axis=0)

        # Centering the points
        measured_centered = measured_points - measured_centroid
        true_centered = true_points - true_centroid

        # Finding optimal rotation
        rotation_matrix, _ = orthogonal_procrustes(measured_centered, true_centered)

        # Calculating translation
        translation = true_centroid - (measured_centroid @ rotation_matrix)

        # Transforming the input point
        point = np.array([x, y])
        transformed = (point @ rotation_matrix) + translation

        return transformed[0], transformed[1]

    def process_images(self):
        """
        Process color/depth images to find ellipses,
        then publish the results to /display_ (DisplayPos).
        This method is called only after receiving a 0 on /command_topic,
        """

        color_img = self.color_image.copy()

        # Convert to grayscale for brightness analysis and edge detection
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Finding contours and fitting ellipses
        edges = cv2.Canny(blurred, 80, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Publisher data variables
        last_large_ellipse_coords = None
        small_ellipse_coords = []

        for contour in contours:
            if len(contour) < 5:
                continue

            ellipse = cv2.fitEllipse(contour)
            (x, y), (major_axis, minor_axis), angle = ellipse

            # Compute brightness
            mask = np.zeros_like(gray)
            center_int = (int(round(x)), int(round(y)))
            axes_int = (int(round(major_axis / 2.0)), int(round(minor_axis / 2.0)))

            # Drawing a filled ellipse on the mask
            cv2.ellipse(
                mask,
                center_int,
                axes_int,
                angle,  # angle in degrees
                0, 360,
                color=255,
                thickness=-1
            )

            ellipse_pixels = gray[mask == 255]
            if len(ellipse_pixels) == 0:
                continue

            mean_brightness = np.mean(ellipse_pixels)

            scale_factor = None
            is_small = False
            is_large = False

            # Check size ranges
            if 20 <= minor_axis <= 30 and 20 <= major_axis <= 30:
                # Small ellipse
                scale_factor = 1.10
                is_small = True
                #Brightness filter
                if not (30 <= mean_brightness <= 100):
                    continue
            elif 120 <= minor_axis <= 140 and 120 <= major_axis <= 140:
                # Large ellipse
                scale_factor = 0.90
                is_large = True
                #Brightness filter
                if not (100 <= mean_brightness <= 200):
                    continue
            if scale_factor is None:
                continue  # not in either size range

            # Log brightness
            self.get_logger().info(f"Ellipse brightness (grayscale) = {mean_brightness:.2f}")

            # Draw ellipse outline
            cv2.ellipse(color_img, ellipse, (0, 255, 0), 2)

            # Collect depth values along the scaled ellipse
            a = (major_axis * scale_factor) / 2.0
            b = (minor_axis * scale_factor) / 2.0
            angle_rad = np.deg2rad(angle)

            N = 360
            depth_values = []
            for i in range(N):
                theta = 2.0 * math.pi * i / N
                ex = a * math.cos(theta)
                ey = b * math.sin(theta)

                # Rotate
                x_rot = ex * math.cos(angle_rad) - ey * math.sin(angle_rad)
                y_rot = ex * math.sin(angle_rad) + ey * math.cos(angle_rad)

                # Translate to pixel coords
                px = int(round(x + x_rot))
                py = int(round(y + y_rot))

                # Check image boundaries
                if (0 <= px < self.depth_image.shape[1] and
                    0 <= py < self.depth_image.shape[0]):
                    Z = float(self.depth_image[py, px]) / 1000.0  # mm to meters
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

            # Transform to base_link
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

                # Apply final calibration transform
                X_base, Y_base = self.transform_point(X_base, Y_base)

                # Log coords in base_link
                self.get_logger().info(
                    f"-> base_link coords=({X_base:.3f}, {Y_base:.3f}, {Z_base:.3f})"
                )

                # Storing publish data
                if is_large:
                    last_large_ellipse_coords = (X_base, Y_base)
                if is_small and len(small_ellipse_coords) < 4:
                    small_ellipse_coords.append((X_base, Y_base))

            except Exception as e:
                self.get_logger().warn(f"Transform from D415_color_optical_frame to base_link failed: {e}")

        # Visualization
        cv2.imshow("Ellipse Detection", color_img)
        cv2.waitKey(1)

        # Publish to /display_
        display_msg = DisplayPos()
        display_msg.data = [0.0] * 10

        # Large ellipse coords
        if last_large_ellipse_coords is not None:
            display_msg.data[0] = float(last_large_ellipse_coords[0])
            display_msg.data[1] = float(last_large_ellipse_coords[1])
        else:
            pass

        # Small ellipse coords
        for i, coords in enumerate(small_ellipse_coords):
            if i >= 4:
                break
            base_index = 2 + 2*i
            display_msg.data[base_index]   = float(coords[0])  # x
            display_msg.data[base_index+1] = float(coords[1])  # y

        # Publish
        self.display_pub.publish(display_msg)
        self.get_logger().info("Published DisplayPos message to /display_")


def main(args=None):
    rclpy.init(args=args)
    node = TableSlotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
