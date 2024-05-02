import cv2
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from custom_interfaces.srv import CameraCalibration
from custom_interfaces.msg import ImageGroup

class CamInfoClient(Node):
    def __init__(self):
        super().__init__('cam_info_client_node')
        self.client = self.create_client(
            CameraCalibration,
            'camera_calibration')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = CameraCalibration.Request()
    
    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

class Reconstruction(Node):
    def __init__(self):
        super().__init__('reconstruction_node')
        self.subscription = self.create_subscription(ImageGroup, 'cam_images', self.listener_callback, 10)
        self.cam_info = self.get_cam_info()
        print(self.cam_info)


    def get_cam_info(self):
        cam_info_client = CamInfoClient()
        cam_info = cam_info_client.send_request()
        cam_info_client.destroy_node()
        return cam_info


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