import cv2
import json
import math
import os
import numpy as np
import rclpy
import rclpy.executors
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import subprocess

from std_msgs.msg import Bool
from custom_interfaces.msg import StringStamped
from custom_interfaces.srv import AskModelPath, AskImageCamInfoGroup
from cv_bridge import CvBridge
bridge = CvBridge()
    

class Reconstruction(Node):
    def __init__(self, ):
        super().__init__('reconstruction_node')
        
        self.data_path = 'data'
        self.activated = False

        self.default_instantngp_path = os.path.join(os.path.expanduser('~'), "instant-ngp/scripts/run.py")
        self.default_conda_environment = ''
        self.default_images_path = 'data'
        self.default_model_path = 'model/gmr'

        self.declare_parameter('instantngp_path', self.default_instantngp_path)
        self.instantngp_path = self.get_parameter('instantngp_path')

        self.declare_parameter('conda_environment', self.default_conda_environment)
        self.conda_environment = self.get_parameter('conda_environment')

        self.declare_parameter('images_path', self.default_images_path)
        self.images_path = self.get_parameter('images_path')

        self.declare_parameter('ply_path', self.default_model_path)
        self.model_path = self.get_parameter('ply_path')

        # Callback groups
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        # Publishers
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.model_publisher = self.create_publisher(StringStamped, 'reconstruction/publishers/path', qos_profile=qos)
        
        # Subscribers
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.reconstruction_switch_subscriber = self.create_subscription(Bool, 'robot_manager/publishers/switch_reconstruction', self.reconstruction_switch_callback, qos_profile=qos, callback_group=group2)

        # Services
        self.reconstruction_service = self.create_service(AskModelPath, 'reconstruction/services/path', self.reconstruction_callback, callback_group=group2)

        # Clients
        self.cameras_client = self.create_client(AskImageCamInfoGroup, 'cameras/services/group_info', callback_group=group1)
        self._cameras_request = AskImageCamInfoGroup.Request()

    # Callbacks
    def reconstruction_switch_callback(self, msg):
        self.activated = msg.data
        # Find a way to start a reconstruction loop


    def reconstruction_callback(self, req, response):
        response.path = self._reconstruction()
        return response
    

    # Requests
    def send_cameras_request(self):
        """AskImageCamInfoGroup.srv (from custom_interfaces)
        # Request
        ---
        # Response
        image_cam_info_list: In the custom_interfaces msg folder

        returns the images cams info
        """
        # Connect to the service
        while not self.cameras_client.wait_for_service(timeout_sec=0):
            self.get_logger().info('Cameras service not available, waiting again...')

        self.get_logger().info("Requesting cameras and images...")
        self.future = self.cameras_client.call_async(self._cameras_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Received cameras and images!")
        return self.future.result().image_cam_info_list


    # Publishings    
    def publish_model_path(self, header, path):
        msg = StringStamped()
        msg.header = header
        msg.data = path

        self.model_publisher.publish(msg)
   

    # Other methods
    def _reconstruction(self):
        # Ask for images and cams position
        info_list = self.send_cameras_request()

        # Save images and create json
        self.get_logger().info("Preparing images and cameras...")
        i = 0
        cam_info = []
        
        for im_cam_info in info_list:
            img_msg = im_cam_info.img
            cam_info.append(im_cam_info.cam_info)
            im = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            im = cv2.flip(im, 0)
            cv2.imwrite(os.path.join(self.images_path.get_parameter_value().string_value, 'robot_cam' + '{:02d}'.format(i) + '.jpg'), im)
            i += 1
        self.cam_info_to_json(cam_info)
        self.get_logger().info("Prepared images and cameras!")

        # Build model
        self.get_logger().info("Reconstructing from images...")
        self.build_model()
        self.get_logger().info("Finished reconstruction!")
        return self.model_path.get_parameter_value().string_value
    

    def build_model(self):
        if self.conda_environment.get_parameter_value().string_value == '':
            subprocess.run(['python', self.instantngp_path.get_parameter_value().string_value,
                            '--scene', self.images_path.get_parameter_value().string_value,
                            '--save_mesh', self.model_path.get_parameter_value().string_value,
                            '--n_steps', '5000'],
                            stdout=subprocess.DEVNULL,)
        else:
            subprocess.run(['conda', 'run', '-n', self.conda_environment.get_parameter_value().string_value,
                            'python', self.instantngp_path.get_parameter_value().string_value,
                            '--scene', self.images_path.get_parameter_value().string_value,
                            '--save_mesh', self.model_path.get_parameter_value().string_value,
                            '--n_steps', '5000'],
                            stdout=subprocess.DEVNULL,)
        self.get_logger().info('Finished model')


    def cam_info_to_json(self, cam_info):
        data = {}
        data['w'] = cam_info[0].width
        data['h'] = cam_info[0].height
        data['fl_x'] = cam_info[0].k[0]
        data['fl_y'] = cam_info[0].k[4]
        data['camera_angle_x'] = math.atan(data['w']/(data['fl_x'] * 2)) * 2
        data['camera_angle_y'] = math.atan(data['h']/(data['fl_y'] * 2)) * 2
        data['k1'] = cam_info[0].d[0]
        data['k2'] = cam_info[0].d[1]
        data['k3'] = cam_info[0].d[2]
        data['k4'] = cam_info[0].d[3]
        data['p1'] = cam_info[0].d[3]
        data['p2'] = cam_info[0].d[3]
        data['is_fisheye'] = False
        data['cx'] = cam_info[0].k[2]
        data['cy'] = cam_info[0].k[5]
        data['aabb_scale'] = 128
        data['frames'] = []

        n_frames = 0
        avglen = 0
        for info in cam_info:
            im_data = {}
            im_data['file_path'] = os.path.join('images', 'robot_cam' + '{:02d}'.format(n_frames) + '.jpg')
            im_data['sharpness'] = 300.0
            
            im_data['transform_matrix'] = np.array([[info.p[0], info.p[1], info.p[2], info.p[3],],
                                           [info.p[4], info.p[5], info.p[6], info.p[7],],
                                           [info.p[8], info.p[9], info.p[10], info.p[11],],
                                           [0, 0, 0, 1,]])
            avglen += np.linalg.norm(im_data['transform_matrix'][0:3, 3])
            data['frames'].append(im_data)
            n_frames += 1
        
        avglen /= n_frames

        for frame in data['frames']:
            frame['transform_matrix'][0:3, 3] *= 3./avglen
            frame['transform_matrix'] = frame['transform_matrix'].tolist()

        with open(os.path.join(self.data_path, 'transforms.json'), 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)


def main():
    rclpy.init()

    recon_node = Reconstruction()
    executor = MultiThreadedExecutor()
    executor.add_node(recon_node)

    try:
        recon_node.get_logger().info("Starting Reconstruction Node")
        executor.spin()
    except KeyboardInterrupt:
        recon_node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    recon_node.destroy_node()
    rclpy.shutdown()
