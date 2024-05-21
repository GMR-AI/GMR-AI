#!/usr/bin/env python3

import os
import sys
from pathlib import Path
import glob

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import torch

from utils.augmentations import letterbox

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import LoadImages
from utils.general import LOGGER, check_img_size, check_requirements, non_max_suppression, print_args, scale_coords
from utils.torch_utils import select_device, time_sync

import torch
import torch.nn as nn
from torchvision.models import resnet18, vgg11

import numpy as np

from script.Dataset import generate_bins, DetectedObject
from library.Math import *
from library.Plotting import *
from script import Model, ClassAverages
from script.Model import ResNet, ResNet18, VGG11

bridge = CvBridge()

# model factory to choose model
model_factory = {
    'resnet': resnet18(pretrained=True),
    'resnet18': resnet18(pretrained=True),
    # 'vgg11': vgg11(pretrained=True)
}
regressor_factory = {
    'resnet': ResNet,
    'resnet18': ResNet18,
    'vgg11': VGG11
}

class Bbox:
    def __init__(self, box_2d, class_):
        self.box_2d = box_2d
        self.detected_class = class_

def detect3d(
    reg_weights,
    model_select,
    source,
    calib_file,
    show_result,
    save_result,
    output_path
    ):

    # Directory
    imgs_path = sorted(glob.glob(str(source) + '/*'))
    calib = str(calib_file)

    # load model
    base_model = model_factory[model_select]
    regressor = regressor_factory[model_select](model=base_model).cuda()

    # load weight
    checkpoint = torch.load(reg_weights)
    regressor.load_state_dict(checkpoint['model_state_dict'])
    regressor.eval()

    averages = ClassAverages.ClassAverages()
    angle_bins = generate_bins(2)

    # loop images
    for i, img_path in enumerate(imgs_path):
        # read image
        img = cv2.imread(img_path)
        
        # Run detection 2d
        dets = detect2d(
            weights='yolov5s.pt',
            source=img_path,
            data='data/coco128.yaml',
            imgsz=[640, 640],
            device=0,
            classes=[0, 2, 3, 5]
        )

        for det in dets:
            if not averages.recognized_class(det.detected_class):
                continue
            try: 
                detectedObject = DetectedObject(img, det.detected_class, det.box_2d, calib)
            except:
                continue

            theta_ray = detectedObject.theta_ray
            input_img = detectedObject.img
            proj_matrix = detectedObject.proj_matrix
            box_2d = det.box_2d
            detected_class = det.detected_class

            input_tensor = torch.zeros([1,3,224,224]).cuda()
            input_tensor[0,:,:,:] = input_img

            # predict orient, conf, and dim
            [orient, conf, dim] = regressor(input_tensor)
            orient = orient.cpu().data.numpy()[0, :, :]
            conf = conf.cpu().data.numpy()[0, :]
            dim = dim.cpu().data.numpy()[0, :]

            dim += averages.get_item(detected_class)

            argmax = np.argmax(conf)
            orient = orient[argmax, :]
            cos = orient[0]
            sin = orient[1]
            alpha = np.arctan2(sin, cos)
            alpha += angle_bins[argmax]
            alpha -= np.pi

            # plot 3d detection
            plot3d(img, proj_matrix, box_2d, dim, alpha, theta_ray)

        if show_result:
            cv2.imshow('3d detection', img)
            cv2.waitKey(0)

        if save_result and output_path is not None:
            try:
                os.mkdir(output_path)
            except:
                pass
            cv2.imwrite(f'{output_path}/{i:03d}.png', img)

@torch.no_grad()
def detect2d(
    img,
    imgsz,
    device,
    classes,
    model,
    names
    ):

    # array for boundingbox detection
    bbox_list = []

    dt, seen = [0.0, 0.0, 0.0], 0
    t1 = time_sync()
    img0 = img[np.newaxis, :, :, :]        
    img0 = np.stack(img0, 0)
    img0 = img0[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
    img0 = np.ascontiguousarray(img0)
    img0 = img0[0, :, :, :]
    im = torch.from_numpy(img0).to(device)
    im = im.float()
    im /= 255  # 0 - 255 to 0.0 - 1.0
    if len(im.shape) == 3:
        im = im[None]  # expand for batch dim
    t2 = time_sync()
    dt[0] += t2 - t1

    # Inference
    pred = model(im, augment=False, visualize=False)
    t3 = time_sync()
    dt[1] += t3 - t2

    # NMS
    pred = non_max_suppression(prediction=pred, classes=classes)
    dt[2] += time_sync() - t3

    # Process predictions
    for i, det in enumerate(pred):  # per image
        seen += 1

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], img.shape).round()
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class

            # Write results
            for *xyxy, conf, cls in reversed(det):
                xyxy_ = (torch.tensor(xyxy).view(1,4)).view(-1).tolist()
                xyxy_ = [int(x) for x in xyxy_]
                top_left, bottom_right = (xyxy_[0], xyxy_[1]), (xyxy_[2], xyxy_[3])
                bbox = [top_left, bottom_right]
                c = int(cls)  # integer class
                label = names[c]
                bbox_list.append(Bbox(bbox, label))

        # Print time (inference-only)
        LOGGER.info(f'Done. ({t3 - t2:.3f}s)')

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)

    return bbox_list

def plot3d(
    img,
    proj_matrix,
    box_2d,
    dimensions,
    alpha,
    theta_ray,
    img_2d=None
    ):

    # the math! returns X, the corners used for constraint
    location, X = calc_location(dimensions, proj_matrix, box_2d, alpha, theta_ray)

    orient = alpha + theta_ray

    if img_2d is not None:
        plot_2d_box(img_2d, box_2d)

    plot_3d_box(img, proj_matrix, orient, dimensions, location) # 3d boxes

    return location

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        weights='yolov5s.pt'  # model.pt path(s)
        reg_weights = os.getcwd() + '/yolo3D_ros/src/yolo3d/scripts/weights/resnet18.pkl'
        calib_file = os.getcwd() + '/yolo3D_ros/src/yolo3d/scripts/eval/camera_cal/calib_cam_to_cam.txt'

        model_select = "resnet18"
        # Directory
        self.calib = str(calib_file)

        # load model
        base_model = model_factory[model_select]
        self.regressor = regressor_factory[model_select](model=base_model).cuda()

        # load weight
        checkpoint = torch.load(reg_weights)
        self.regressor.load_state_dict(checkpoint['model_state_dict'])
        self.regressor.eval()

        self.averages = ClassAverages.ClassAverages()
        self.angle_bins = generate_bins(2)

        # Load model
        device_num = 0
        imgsz=[640, 640]
        weights='yolov5s.pt'
        data='data/coco128.yaml'
        self.device = select_device(device_num)
        self.model_2d = DetectMultiBackend(weights, device=self.device, dnn=False, data=data)
        stride, self.names, pt, jit, onnx, engine = self.model_2d.stride, self.model_2d.names, self.model_2d.pt, self.model_2d.jit, self.model_2d.onnx, self.model_2d.engine
        self.imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Run inference
        self.model_2d.warmup(imgsz=(1, 3, *imgsz), half=False)  # warmup

        self.subscription = self.create_subscription(
            Image,
            'front/image_raw',
            self.camera_callback,
            10)
        self.subscription

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # Run detection 2d

        dets = detect2d(
            img=img,
            imgsz=[640, 640],
            device=self.device,
            classes=[0, 2, 3, 5],
            model = self.model_2d,
            names = self.names
        )

        for det in dets:
            if not self.averages.recognized_class(det.detected_class):
                continue
            try: 
                detectedObject = DetectedObject(img, det.detected_class, det.box_2d, self.calib)
            except:
                import traceback
                print("exception while DetectedObject!")
                print(traceback.format_exc())
                continue

            theta_ray = detectedObject.theta_ray
            input_img = detectedObject.img
            proj_matrix = detectedObject.proj_matrix
            box_2d = det.box_2d
            detected_class = det.detected_class

            input_tensor = torch.zeros([1,3,224,224]).cuda()
            input_tensor[0,:,:,:] = input_img

            # predict orient, conf, and dim
            [orient, conf, dim] = self.regressor(input_tensor)
            orient = orient.cpu().data.numpy()[0, :, :]
            conf = conf.cpu().data.numpy()[0, :]
            dim = dim.cpu().data.numpy()[0, :]

            dim += self.averages.get_item(detected_class)

            argmax = np.argmax(conf)
            orient = orient[argmax, :]
            cos = orient[0]
            sin = orient[1]
            alpha = np.arctan2(sin, cos)
            alpha += self.angle_bins[argmax]
            alpha -= np.pi

            location, X = calc_location(dim, proj_matrix, box_2d, alpha, theta_ray)

            orient = alpha + theta_ray

            img = plot_3d_box(img, proj_matrix, orient, dim, location) # 3d boxes

        cv2.imshow("IMAGE", img)
        cv2.waitKey(4)

def main():
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

# if __name__ == '__main__':
#     rclpy.init(args=None)
#     camera_subscriber = Camera_subscriber()
#     rclpy.spin(camera_subscriber)
#     rclpy.shutdown()