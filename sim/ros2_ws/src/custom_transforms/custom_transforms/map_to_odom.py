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
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=qos, callback_group=group1)
        self.reconstruction_subscriber = self.create_subscription(StringStamped, "reconstruction/publishers/path", self.reconstruction_callback, qos_profile=qos, callback_group=group1)
        self.robot_odom_subscriber = self.create_subscription(Odometry, "odometry/filtered", self.robot_odom_callback, qos_profile=qos, callback_group=group1)
        self.robot_map_subscriber = self.create_subscription(Odometry, "object_detection/publishers/odom_real", self.robot_map_callback, qos_profile=qos, callback_group=group1)
        
        # Posición del robot respecto odometría
        # Posición del robot respecto mapa (sale de object_detection)
        # Ambas posiciones en el mismo Time Stamp => instante de inicio de reconstrucción para conseguir la posición del robot respecto odometría
        #
        # self.map2odom_subscriber = self.create_subscription(TFMessage, '/tf/update', self.transform_callback, qos_profile=qos)

        # Broadcasters
        self.map2odom_broadcaster = TransformBroadcaster(self, qos_profile=qos)

        # Veremos...
        self.timer = self.create_timer(0.05, self.timer_callback)

    def clock_callback(self, msg):
        self.stamp = msg.clock
    
    def reconstruction_callback(self, msg):
        self.check_odometry = True

    def robot_odom_callback(self, msg):
        if self.check_odometry:
            self.robot_odometry_pose.header = msg.header
            self.robot_odometry_pose.pose = msg.pose.pose
            self.check_odometry = False

    def robot_map_callback(self, msg: Odometry):
        self.map2odom_transform.header.stamp = msg.header.stamp
        self.map2odom_transform.transform.translation.x = msg.pose.pose.position.x - self.robot_odometry_pose.pose.position.x
        self.map2odom_transform.transform.translation.x = msg.pose.pose.position.y - self.robot_odometry_pose.pose.position.y
        # HOW CAN I SUBSTRACT TWO QUATERNIONS CONSIDERING ONLY THE X-Y PLANE????????
        self.map2odom_broadcaster.sendTransform(self.map2odom_transform)

    def transform_callback(self, msg):
        self.map2odom_transform.transform = msg.transforms[0]


    


    def timer_callback(self):
        self.map2odom_transform.header.stamp = self.stamp
        self.map2odom_broadcaster.sendTransform(self.map2odom_transform)


def main():
    rclpy.init()
    map2odom_replicator_node = Map2OdomNode()
    try:
        rclpy.spin(map2odom_replicator_node)
    except:
        pass
    map2odom_replicator_node.destroy_node()
    rclpy.shutdown()