import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from custom_interfaces.msg import StringStamped
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

import math

def euler_from_quaternion(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = 2.0 * (w * y + x * y)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class Map2OdomNode(Node):
    def __init__(self):
        super().__init__('map2odom_replicator_node')


        self.check_odometry = False
        self.robot_odometry_pose = PoseStamped()

        # Initialize the time

        self.stamp = Time()
        self.stamp.nanosec = 0
        self.stamp.sec = 0

        # Initialize the map->odom transform message
        self.map2odom_transform = TransformStamped()
        self.map2odom_transform.header.stamp = self.stamp
        self.map2odom_transform.header.frame_id = 'map'
        self.map2odom_transform.child_frame_id = 'odom'

        self.map2odom_transform.transform.translation.x = 0.0
        self.map2odom_transform.transform.translation.y = 0.0
        self.map2odom_transform.transform.translation.z = 0.0

        self.map2odom_transform.transform.rotation.x = 0.0
        self.map2odom_transform.transform.rotation.y = 0.0
        self.map2odom_transform.transform.rotation.z = 0.0
        self.map2odom_transform.transform.rotation.w = 1.0
        
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Callback Groups
        group1 = ReentrantCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=10, callback_group=group1)
        self.reconstruction_start_subscriber = self.create_subscription(StringStamped, "reconstruction/publishers/start", self.reconstruction_callback, qos_profile=qos, callback_group=group1)
        self.robot_odom_subscriber = self.create_subscription(Odometry, "gmr/odom", self.robot_odom_callback, qos_profile=10, callback_group=group2)
        self.robot_map_subscriber = self.create_subscription(Odometry, "object_detection/publishers/odom_real", self.robot_map_callback, qos_profile=qos, callback_group=group2)

        # Broadcasters
        self.map2odom_broadcaster = TransformBroadcaster(self, qos=qos)

        # Veremos...
        self.timer = self.create_timer(0.05, self.timer_callback, callback_group=group2)

    def clock_callback(self, msg):
        self.stamp = msg.clock
    
    def reconstruction_callback(self, _):
        self.check_odometry = True

    def robot_odom_callback(self, msg):
        if self.check_odometry:
            self.robot_odometry_pose.header = msg.header
            self.robot_odometry_pose.pose = msg.pose.pose
            self.check_odometry = False

    def robot_map_callback(self, msg: Odometry):
        self.map2odom_transform.transform.translation.x = msg.pose.pose.position.x - self.robot_odometry_pose.pose.position.x
        self.map2odom_transform.transform.translation.y = msg.pose.pose.position.y - self.robot_odometry_pose.pose.position.y
        # self.map2odom_transform.transform.rotation = msg.pose.pose.orientation

    def transform_callback(self, msg):
        self.map2odom_transform.transform = msg.transforms[0]

    def timer_callback(self):
        self.map2odom_transform.header.stamp = self.stamp
        self.map2odom_broadcaster.sendTransform(self.map2odom_transform)


def main():
    rclpy.init()
    map2odom_node = Map2OdomNode()
    executor = MultiThreadedExecutor()
    executor.add_node(map2odom_node)
    try:
        executor.spin()
    except:
        pass
    map2odom_node.destroy_node()
    rclpy.shutdown()