import cv2
import json
import math
import os
import numpy as np
import matplotlib.pyplot as plt
import rclpy
import rclpy.logging
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, CompressedImage
from sensor_msgs.msg import PointCloud2
# from custom_interfaces import SendPath
from custom_interfaces.msg import ImageCamInfoGroup
from cv_bridge import CvBridge
bridge = CvBridge()
    

class Reconstruction(Node):
    def __init__(self, num_camera):
        super().__init__('reconstruction_node')
        self.num_camera = num_camera
        self.data_path = 'data'
        self.available = True

        self.image_group_subscriber = self.create_subscription(ImageCamInfoGroup, 'cams/image_group', self.image_group_callback, 10)
        self.image_group_subscriber

        self.point_cloud_publisher = self.create_publisher(PointCloud2, 'filtered_clouds', 10)

        # TODO: make client for a 3D reconstruction service using instant-ngp
        # self.reconstruct_client = self.create_client(SendPath, 'reconstruct_model', 10)
        # self.req = SendPath.Request()

    def image_group_callback(self, image_cam_info_group):
        if not self.available: return
        self.available = False
        i = 0
        cam_info = []
        for im_cam_info in image_cam_info_group.image_cam_info_list:
            imgmsg = im_cam_info.img
            cam_info.append(im_cam_info.cam_info)
            im = bridge.imgmsg_to_cv2(imgmsg, 'bgr8')
            im = cv2.flip(im, 0)
            cv2.imwrite('data/images/robot_cam'+'{:02d}'.format(i)+'.jpg', im)
            i+=1
        self.cam_info_to_json(cam_info)
        # self.send_reconstruct_request()
        # self.publish_point_cloud()

    def send_reconstruct_request(self):
        self.req.folder = self.data_path
        self.future = self.reconstruct_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
    def publish_point_cloud(self):
        # Process PLY to PointCloud2
        # Header
        # height
        # width
        # fields
        # is_bigendian
        # point_step
        # row_step
        # data
        # is_dense
        pass
    
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

    recon_node = Reconstruction(4)
    rclpy.spin(recon_node)
    recon_node.destroy_node()
    rclpy.shutdown()