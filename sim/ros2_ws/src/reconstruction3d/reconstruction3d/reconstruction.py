import cv2
import numpy as np
import matplotlib.pyplot as plt
import rclpy
import rclpy.logging
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from custom_interfaces.msg import ImageCamInfoGroup
from cv_bridge import CvBridge
bridge = CvBridge()
    

class Reconstruction(Node):
    def __init__(self, num_camera):
        super().__init__('reconstruction_node')
        self.num_camera = num_camera

        self.image_group_subscriber = self.create_subscription(ImageCamInfoGroup, 'cams/image_group', self.callback, 10)
        self.image_group_subscriber

        self.counter = 0

    def callback(self, image_cam_info_group):
        if self.counter > 3: return

        for im_cam_info in image_cam_info_group.image_cam_info_list:
            imgmsg = im_cam_info.img
            im = bridge.imgmsg_to_cv2(imgmsg, 'bgr8')
            im = cv2.flip(im, 0)
            cv2.imwrite('/home/adriangt2001/Pictures/robot_cam'+'{:02d}'.format(self.counter)+'.jpg', im)
            self.counter += 1


def main():
    rclpy.init()

    recon_node = Reconstruction(4)
    rclpy.spin(recon_node)
    recon_node.destroy_node()
    rclpy.shutdown()