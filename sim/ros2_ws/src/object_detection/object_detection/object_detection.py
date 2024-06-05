import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

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

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Callback Groups
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = ReentrantCallbackGroup()

        # Subscribers
        self.model_subscription = self.create_subscription(StringStamped, 'reconstruction/publishers/path', self.model_callback, qos_profile=qos)
        # self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.write_costmap, qos_profile=qos)
     
        # Publishers
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', qos_profile=qos)
        self.robot_position_publisher = self.create_publisher(Odometry, 'object_detection/publishers/odom_real', qos_profile=qos)

        self.detection_model = YOLO(self.model_file.get_parameter_value().string_value)
        

    def write_costmap(self, msg):
        image = np.array(msg.data)
        image = image.reshape((int(np.sqrt(image.shape[0])), int(np.sqrt(image.shape[0]))))
        cv2.imwrite('/home/adriangt2001/Pictures/costmap.jpg', image)

    def model_callback(self, msg):
        header = msg.header
        ply_path = msg.data+'.ply'
        image = ply2jpg(ply_path)
        robot_mask, obstacle_mask = segmentation(self.detection_model, image)

        # Publish robot position
        if not np.all(robot_mask == 0):
            self.get_logger().info('Robot Detected')
            real_position, meters_per_pixel = self.estimate_scale_and_transform(robot_mask)
            self.publish_robot_position(real_position, header)

        # Publish map
        # Need to do some transformation to really adjust map size based on the meter_per_pixel calculated earlier
        if not np.all(obstacle_mask == 0):
            self.get_logger().info('Map detected')
            self.publish_occupancy_grid(obstacle_mask, header, meters_per_pixel)


    def publish_occupancy_grid(self, binary_image, header, resolution):
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

        occupancy_values = np.where(binary_image == 255, 100, 0).astype(np.int8).transpose()
        occupancy_grid.data = occupancy_values.flatten().tolist()

        self.map_publisher.publish(occupancy_grid)


    def publish_robot_position(self, real_position, header):
        odom_msg = Odometry()
        odom_msg.header.stamp = header.stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = float(real_position[0])
        odom_msg.pose.pose.position.y = float(real_position[1])
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.pose.covariance = self.compute_covariance_matrix(0.1, 2 * np.pi)

        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = np.zeros(36).tolist()

        self.robot_position_publisher.publish(odom_msg)


    def estimate_scale_and_transform(self, robot_mask):
        real_radius = 1
        contours, _ = cv2.findContours(robot_mask.transpose(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        meters_per_pixel = real_radius / radius
        real_world_coords = ((int(x) - robot_mask.shape[0] / 2) * 0.05, (int(y) - robot_mask.shape[1] / 2) * 0.05)
        self.get_logger().info(f"Robot position: ({real_world_coords})")
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
    executor = MultiThreadedExecutor()

    detection_node = ObjectDetection()
    executor.add_node(detection_node)
    executor.spin()

    detection_node.destroy_node()
    rclpy.shutdown()
