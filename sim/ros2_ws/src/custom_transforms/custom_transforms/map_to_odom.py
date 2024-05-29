import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

class Map2OdomReplicatorNode(Node):
    def __init__(self):
        super().__init__('map2odom_replicator_node')

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
        
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.sim_time_callback, 10)
        self.map2odom_broadcaster = TransformBroadcaster(self)
        self.map2odom_subscriber = self.create_subscription(TFMessage, '/tf/update', self.transform_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback, )


    def transform_callback(self, msg):
        self.map2odom_transform.transform = msg.transforms[0]


    def sim_time_callback(self, msg):
        self.stamp = msg.clock


    def timer_callback(self):
        self.map2odom_transform.header.stamp = self.stamp
        self.map2odom_broadcaster.sendTransform(self.map2odom_transform)


def main():
    rclpy.init()
    map2odom_replicator_node = Map2OdomReplicatorNode()
    try:
        rclpy.spin(map2odom_replicator_node)
    except:
        pass
    map2odom_replicator_node.destroy_node()
    rclpy.shutdown()