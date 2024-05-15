import rclpy
import rclpy.logging
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomTransformFramePublisher(Node):
    def __init__(self):
        super().__init__('odom_to_base_link_frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/gmr/odom',
            self.handle_odom_transform,
            1
        )

        self.subscription
    

    def handle_odom_transform(self, msg):
        t = TransformStamped()

        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomTransformFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()