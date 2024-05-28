import rclpy
from rclpy.node import Node

from custom_interfaces.msg import StringStamped
from nav_msgs.msg import OccupancyGrid, Odometry

from object_detection.ply2jpg import ply2jpg
from object_detection.instance_segmentation import segmentation

from ultralytics import YOLO
import cv2
import numpy as np

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        self.default_model_file = 'model/GMRModel.pt'

        self.declare_parameter('model_file', self.default_model_file)
        self.model_file = self.get_parameter('model_file')

        self.model_subscription = self.create_subscription(StringStamped, 'model/path', self.model_callback, 10)
        self.model_subscription

        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.robot_position_publisher = self.create_publisher(Odometry, 'odom_real', 10)

        self.detection_model = YOLO(self.model_file.get_parameter_value().string_value)

        self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.write_costmap, 10)

    def write_costmap(self, msg):
        image = np.array(msg.data)
        image = image.reshape((int(np.sqrt(image.shape[0])), int(np.sqrt(image.shape[0]))))
        cv2.imwrite('/home/adriangt2001/Pictures/costmap.jpg', image)

    def model_callback(self, msg):
        header = msg.header
        ply_path = msg.data
        image = ply2jpg(ply_path)
        robot_mask, obstacle_mask = segmentation(self.detection_model, image)

        # Publish map
        if not np.all(obstacle_mask == 0):
            self.get_logger().info('Map detected')
            self.publish_occupancy_grid(obstacle_mask, header)

        # Publish robot position
        if not np.all(robot_mask == 0):
            self.get_logger().info('Robot Detected')
            real_position, meters_per_pixel = self.estimate_scale_and_transform(robot_mask)
            self.publish_robot_position(real_position, header)


    def publish_occupancy_grid(self, binary_image, header):
        occupancy_grid = OccupancyGrid()
        
        occupancy_grid.header.stamp = header.stamp
        occupancy_grid.header.frame_id = 'map'

        occupancy_grid.info.resolution = 0.05
        occupancy_grid.info.width = binary_image.shape[0]
        occupancy_grid.info.height = binary_image.shape[1]
        occupancy_grid.info.origin.position.x = - binary_image.shape[0] / 2 * 0.05
        occupancy_grid.info.origin.position.y = - binary_image.shape[1] / 2 * 0.05
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        occupancy_values = np.where(binary_image == 255, 100, 0).astype(np.int8)
        occupancy_grid.data = occupancy_values.flatten().tolist()

        self.map_publisher.publish(occupancy_grid)


    def publish_robot_position(self, real_position, header):
        odom_msg = Odometry()
        odom_msg.header.stamp = header.stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = real_position[1]
        odom_msg.pose.pose.position.y = real_position[0]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 0.0
        odom_msg.pose.covariance = self.compute_covariance_matrix(0.1, 2 * np.pi)

        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = np.zeros(36).tolist()

        self.robot_position_publisher.publish(odom_msg)


    def estimate_scale_and_transform(self, robot_mask):
        real_radius = 1
        contours, _ = cv2.findContours(robot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        meters_per_pixel = real_radius / radius
        real_world_coords = (int(x) * meters_per_pixel, int(y) * meters_per_pixel)
        return real_world_coords, meters_per_pixel

    def compute_covariance_matrix(self, std_dev_position=0.1, std_dev_orientation=2*np.pi):
        covariance_matrix = np.zeros((6,6))

        # Set the variances (squared standard deviations)
        covariance_matrix[0, 0] = std_dev_position ** 2  # Variance in x
        covariance_matrix[1, 1] = std_dev_position ** 2  # Variance in y
        covariance_matrix[2, 2] = 1e-6  # Variance in z, small value as it's a 2D plane
        covariance_matrix[3, 3] = 1e-6  # Variance in rotation around x-axis, small value for 2D
        covariance_matrix[4, 4] = 1e-6  # Variance in rotation around y-axis, small value for 2D
        covariance_matrix[5, 5] = std_dev_orientation ** 2  # Variance in yaw

        return covariance_matrix.flatten().tolist()


def main():
    rclpy.init()

    detection_node = ObjectDetection()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()