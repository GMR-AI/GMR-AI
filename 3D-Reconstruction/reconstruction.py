import cv2
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import ImageGroup

class Reconstruction(Node):
    def __init__(self):
        super().__init__('reconstruction_node')
        self.subscription = self.create_subscription(ImageGroup, 'cam_images', self.listener_callback, 10)
    
    def listener_callback(self, msg: ImageGroup):
        # Process the image here
        # msg holds the array in msg.data
        # To get the image as captured, reshape it to (msg.height, msg.width, 3 if RGB)
        for m in msg.image_list:
            img = np.reshape(m.data, (m.height, m.width, 3))
        
            plt.imshow(img) # This opens a new window and execution is stopped until you close it
            plt.show()

if __name__ == '__main__':
    rclpy.init()

    recon_node = Reconstruction()
    rclpy.spin(recon_node)
    recon_node.destroy_node()
    rclpy.shutdown()