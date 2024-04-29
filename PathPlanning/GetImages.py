import rclpy
from rclpy.node import Node

class ImagesSubscriber(Node):
    def __init__(self):
        super().__init__('images_subscriber')
        self.images_list = [i for i in range(4)]
        self.subscriber = []