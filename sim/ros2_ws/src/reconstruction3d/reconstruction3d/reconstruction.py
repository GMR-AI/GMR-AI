import cv2
import numpy as np
import matplotlib.pyplot as plt
import rclpy
import rclpy.logging
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from custom_interfaces.msg import ImageCamInfoGroup
    

class Reconstruction(Node):
    def __init__(self, num_camera):
        super().__init__('reconstruction_node')
        self.num_camera = num_camera

        self.image_group_subscriber = self.create_subscription(ImageCamInfoGroup, 'cams/image_group', self.callback, 10)
        self.image_group_subscriber

    def callback(self, image_cam_info_group):
        # Do reconstruction here
        pass


def main():
    rclpy.init()

    recon_node = Reconstruction(4)
    rclpy.spin(recon_node)
    recon_node.destroy_node()
    rclpy.shutdown()