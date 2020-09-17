#!/usr/bin/python3
import rospy
from boat_msgs.msg import BridgeHoleInfo
from std_msgs.msg import Float64MultiArray
# from cv_bridge import CvBridge
import torch
from torchvision.models.detection import maskrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from torchvision.transforms.functional import to_tensor
from torch import jit
import cv2
import numpy as np
import os
import sys
from time import time
import datetime


class BridgeHoleCommunicator:
    def __init__(self, predictor, verbose=False):
        self.verbose = verbose
        rospy.init_node('bridge_hole', anonymous=True)
        self.predictor = predictor
        self.predictor.verbose = verbose
        self.subscriber = rospy.Subscriber("/origin_data", Float64MultiArray, self.make_callback())
        self.publisher = rospy.Publisher("bridge_hole_info", BridgeHoleInfo, queue_size=1)

    def run(self):
        self.print("started")
        rospy.spin()

    def make_callback(self):
        def callback(msg):
            self.print("Received msg")
            start = time()
            cv_image = np.array(msg.data).astype(np.uint8).reshape((400, 640, 3))
            res = self.predictor(cv_image)
            info = BridgeHoleInfo()
            if res is None:
                info.existence = False
                info.leftmost = 0
                info.rightmost = 0
            else:
                info.existence = True
                info.leftmost = res[0]
                info.rightmost = res[1]
            end = time()
            self.print(f"prediction latency: {end - start}s")
            self.publisher.publish(info)
        return callback

    def print(self, *args, **kwargs):
        if self.verbose:
            print("BRIDGE: ", end="")
            print(*args, **kwargs)


class BridgeHolePredictor:
    def __init__(self, model_path, detection_score_thre=0.5, mask_score_thre=0.5, verbose=False):
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.model = self.build_model_instance_segmentation(num_classes=2).to(self.device).eval()
        self.model.load_state_dict(torch.load(model_path))
        self.det_thre = detection_score_thre
        self.mask_thre = mask_score_thre
        self.verbose = verbose

    def __call__(self, image):
        return self.predict(image)

    @ torch.no_grad()
    def predict(self, cvimage):
        """
        Detect all bridge holes' masks, choose the largest (widest) one as navigation direction.
        Returns the leftmost and rightmost image coordinate of the mask, normalized within -100 ~ 100.
        If no mask is detected, returns None.
        :param cvimage: default cv2 image format (with shape [H, W, 3] in BGR format)
        :return: (int, int) demonstrating the mask's leftmost and rightmost image coordinate, normalized within -100 ~ 100.
                 returns None if no mask is detected
        """
        W = cvimage.shape[1]
        image = [to_tensor(cv2.cvtColor(cvimage, cv2.COLOR_BGR2RGB)).to(self.device)]
        pred = self.model(image)[0]
        masks = [np.where(pred["masks"][i].cpu().numpy().squeeze() > self.mask_thre, 1, 0)
                 for i in range(len(pred["scores"]))
                 if pred["scores"][i] >= self.det_thre]
        # TODO: test this
        # Eliminate masks which overlapped more than 90% of another mask (not the same logic as NMS).
        # The purpose is to get rid of masks with a pillar in it.
        remained = []
        for i, mask in enumerate(masks):
            ok = True
            for j, m in enumerate(masks):
                if j != i and np.sum(m * mask) / np.sum(m) > 0.9:
                    ok = False
                    break
            if ok:
                remained.append(mask)
        self.print(f"{len(masks)} bridge holes detected, {len(remained)} bridge holes kept.")
        masks = remained
        if len(masks) == 0:
            res = None
        else:
            # Find the widest one among remaining masks
            maxwidth, maxwidthcoords = -1, (None, None)
            for mask in masks:
                for row in mask:
                    cols = np.where(row == 1)[0]
                    if len(cols) > 0:
                        width = np.max(cols) - np.min(cols)
                        if width > maxwidth:
                            maxwidth = width
                            maxwidthcoords = (np.min(cols), np.max(cols))
            res = tuple([int(coord * (200.0 / W) - 100.0) for coord in maxwidthcoords])  # normalization

        # save visible result
        if self.verbose:
            res_dir = os.path.join(os.path.split(sys.argv[0])[0], "bridge/results")
            os.makedirs(res_dir, exist_ok=True)
            if len(masks) == 0:
                res_vis = cvimage
            else:
                res_vis = self.visualize_result(cvimage, masks, maxwidthcoords)
            cv2.imwrite(os.path.join(res_dir, f"[{datetime.datetime.now()}].jpg"), res_vis)

        return res

    @ staticmethod
    def build_model_instance_segmentation(num_classes):
        model = maskrcnn_resnet50_fpn(pretrained=False, pretrained_backbone=False)
        model.roi_heads.box_predictor = FastRCNNPredictor(in_channels=model.roi_heads.box_predictor.cls_score.in_features, num_classes=num_classes)
        model.roi_heads.mask_predictor = MaskRCNNPredictor(in_channels=model.roi_heads.mask_predictor.conv5_mask.in_channels, dim_reduced=256, num_classes=num_classes)
        return model

    @ staticmethod
    def visualize_result(cvimage, masks, coords):
        cvimage = np.copy(cvimage).astype(np.float)
        masks = [np.expand_dims(np.copy(mask).astype(np.float), axis=-1) for mask in masks]
        if len(masks) > 0:
            pasteboard = None
            for mask in masks:
                mask = (mask*255)
                if pasteboard is None:
                    pasteboard = mask
                else:
                    pasteboard = pasteboard + mask
            pasteboard = np.where((pasteboard > 255), 255, pasteboard)
            pasteboard *= (2 / 3)
            other_channel = np.zeros_like(pasteboard)
            pasteboard = np.concatenate([pasteboard, other_channel, other_channel], axis=2)
            image_no_mask = np.where(pasteboard > 0, 0, cvimage) * (2 / 3)
            cvimage = cvimage * (1 / 3) + image_no_mask + pasteboard
            cvimage = cvimage.astype(np.uint8)
            cv2.line(cvimage, (coords[0], 0), (coords[0], cvimage.shape[0]), (0, 0, 255), 2)
            cv2.line(cvimage, (coords[1], 0), (coords[1], cvimage.shape[0]), (0, 0, 255), 2)
        cvimage = cvimage.astype(np.uint8)
        return cvimage

    def print(self, *args, **kwargs):
        if self.verbose:
            print("BRIDGE: ", end="")
            print(*args, **kwargs)


def _test():
    # test behaviour of predictor
    predictor = BridgeHolePredictor("E:/Projects/Python/BridgeHole/work_dir/[state_dict]epoch_18.pth")
    img_dir = "E:/Projects/Python/BridgeHole/data/MergedBridges"
    for f in os.listdir(img_dir):
        f = os.path.join(img_dir, f)
        img = cv2.imread(f)
        print(predictor(img))
        pass
    # test behaviour of ro communicator
    BridgeHoleCommunicator(predictor=BridgeHolePredictor("./weights/[state_dict]epoch_18.pth")).run()


if __name__ == '__main__':
    weight_path = os.path.join(os.path.split(sys.argv[0])[0], "./weights/[state_dict]epoch_18.pth")
    BridgeHoleCommunicator(predictor=BridgeHolePredictor(weight_path), verbose=True).run()
