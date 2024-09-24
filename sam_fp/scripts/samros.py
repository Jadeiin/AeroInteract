import torch
import torchvision
import matplotlib.pyplot as plt
import numpy as np
import rospy

from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
# from sam2.build_sam import build_sam2
# from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from std_msgs.msg import Int16MultiArray
from masks_msgs.msg import maskID
from masks_msgs.msg import singlemask
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
import sys
import os
import cv2
import clip
from typing import List
from PIL import Image, ImageDraw
import time


class SamClipRos:
    def __init__(self):
        rospy.init_node("samros", anonymous=True)
        self.image_topic = rospy.get_param(
            "image_topic", "/camera/color/image_raw"
        )  # Default is image_raw topic of Tiago robot
        self.pub = rospy.Publisher(
            "/sam_mask", maskID, queue_size=10
        )  # TODO: pub np.ndarray related func: maskprocessing() and Pub_mask()
        self.pub_img = rospy.Publisher(
            "/sam_img", SensorImage, queue_size=10
        )
        self.sub = rospy.Subscriber(
            self.image_topic, SensorImage, self.callback
        )  # TODO: find image topic from Tiago!
        self.cropped_boxes = []
        self.search_text = rospy.get_param("search_text", None)
        if len(sys.argv) > 1:
            self.search_text = str(sys.argv[1])
        else:
            self.search_text = None
        self.bridge = CvBridge()
        # Load segment anything model
        sam_checkpoint = "/root/autodl-tmp/sam_vit_h_4b8939.pth"
        model_type = "vit_h"
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        sam.to(device=self.device)
        self.mask_generator = SamAutomaticMaskGenerator(
            model=sam,
            # uncomment below to fine tune parameters
            # points_per_side=10,
            # pred_iou_thresh=0.92,
            # stability_score_thresh=0.92,
            # crop_n_layers=1,
            # crop_n_points_downscale_factor=2,
            # min_mask_region_area=100,  # Requires open-cv to run post-processing)
        )
        # TODO: switch to SAM2
        # sam2_checkpoint = "/root/autodl-tmp/sam2_hiera_large.pt"
        # model_cfg = "/root/autodl-tmp/sam2_hiera_l.yaml"
        # sam2 = build_sam2(
        #     model_cfg, sam2_checkpoint, device=self.device, apply_postprocessing=False
        # )
        # self.mask_generator = SAM2AutomaticMaskGenerator(
        #     model=sam2,
        #     # uncomment below to fine tune parameters
        #     # points_per_side=10,
        #     # pred_iou_thresh=0.92,
        #     # stability_score_thresh=0.92,
        #     # crop_n_layers=1,
        #     # crop_n_points_downscale_factor=2,
        #     # min_mask_region_area=100,  # Requires open-cv to run post-processing)
        # )
        rospy.loginfo("sam model has been loaded.")
        # Load clip model
        self.model, self.preprocess = clip.load("ViT-B/32", self.device)
        rospy.loginfo("clip model has been loaded.")
        rospy.loginfo("Node has been started.")

    # Segment Anything
    def show_anns(self, anns):
        if len(anns) == 0:
            return
        sorted_anns = sorted(anns, key=(lambda x: x["area"]), reverse=True)
        ax = plt.gca()
        ax.set_autoscale_on(False)

        img = np.ones(
            (
                sorted_anns[0]["segmentation"].shape[0],
                sorted_anns[0]["segmentation"].shape[1],
                4,
            )
        )
        img[:, :, 3] = 0
        for ann in sorted_anns:
            m = ann["segmentation"]
            color_mask = np.concatenate([np.random.random(3), [0.35]])
            img[m] = color_mask
        ax.imshow(img)

    # def AutoMaskGen() for Segment Anything
    def AutoMaskGen(self, image):
        rospy.loginfo("AutoMaskGen is triggered.")

        masks = self.mask_generator.generate(image)
        # visualize sam auto mask generator
        # plt.figure(figsize=(10,10))
        # plt.imshow(image)
        # self.show_anns(masks)
        # plt.axis('off')
        # plt.show()
        return masks

    # Process the masks and ready to publish
    def maskprocessing(self, masks, if_singlemask=True):
        rospy.loginfo("maskprocessing is triggered.")
        mask_list = []
        if if_singlemask:
            singlemask_msg = singlemask()
            mask_shape = masks["segmentation"].shape
            segmentation_int = masks["segmentation"].astype(np.int64)
            masks_list = segmentation_int.flatten().tolist()
            point_coords = masks["point_coords"]
            # Flatten the list
            flat_point_coords = [item for sublist in point_coords for item in sublist]

            # Optionally, ensure each element is a float64
            flat_point_coords = np.array(flat_point_coords, dtype=np.float64)
            crop_box = masks["crop_box"]
            crop_box_int16 = np.array(crop_box, dtype=np.int16)

            singlemask_msg.maskid = 0
            singlemask_msg.shape = mask_shape
            singlemask_msg.segmentation = masks_list
            singlemask_msg.area = masks["area"]
            singlemask_msg.bbox = masks["bbox"]
            singlemask_msg.predicted_iou = masks["predicted_iou"]
            singlemask_msg.point_coords = flat_point_coords
            singlemask_msg.stability_score = masks["stability_score"]
            singlemask_msg.crop_box = crop_box_int16
            mask_list.append(singlemask_msg)
            mask_list_msg = maskID()
            mask_list_msg.maskID = mask_list
            return mask_list_msg

        if not if_singlemask:
            for index in range(len(masks)):
                singlemask_msg = singlemask()
                mask_shape = masks[index]["segmentation"].shape
                segmentation_int = masks[index]["segmentation"].astype(np.int64)
                masks_list = segmentation_int.flatten().tolist()
                point_coords = masks[index]["point_coords"]
                # Flatten the list
                flat_point_coords = [
                    item for sublist in point_coords for item in sublist
                ]

                # Optionally, ensure each element is a float64
                flat_point_coords = np.array(flat_point_coords, dtype=np.float64)
                crop_box = masks[index]["crop_box"]
                crop_box_int16 = np.array(crop_box, dtype=np.int16)

                singlemask_msg.maskid = index
                singlemask_msg.shape = mask_shape
                singlemask_msg.segmentation = masks_list
                singlemask_msg.area = masks[index]["area"]
                singlemask_msg.bbox = masks[index]["bbox"]
                singlemask_msg.predicted_iou = masks[index]["predicted_iou"]
                singlemask_msg.point_coords = flat_point_coords
                singlemask_msg.stability_score = masks[index]["stability_score"]
                singlemask_msg.crop_box = crop_box_int16
                mask_list.append(singlemask_msg)
            # print(len(mask_list))
            mask_list_msg = maskID()
            mask_list_msg.maskID = mask_list

            rospy.loginfo("maskID length is \n")
            rospy.loginfo(len(mask_list_msg.maskID))

            return mask_list_msg

    def convert_box_xywh_to_xyxy(self, box):
        x1 = box[0]
        y1 = box[1]
        x2 = box[0] + box[2]
        y2 = box[1] + box[3]
        return [x1, y1, x2, y2]

    # Method to cut out all masks
    def segment_image(self, image, segmentation_mask):
        seg_mask = np.array(
            [segmentation_mask, segmentation_mask, segmentation_mask]
        ).transpose(1, 2, 0)
        return np.multiply(image, seg_mask)

    def crop_masks(self, image, masks):
        self.cropped_boxes = []
        self.cropped_images = []
        for mask in masks:
            # make sure all bbox element are integers
            mask["bbox"] = [int(x) for x in mask["bbox"]]
            x1, y1, x2, y2 = self.convert_box_xywh_to_xyxy(mask["bbox"])
            self.cropped_boxes.append(
                self.segment_image(image, mask["segmentation"]).astype("int")[
                    y1:y2, x1:x2
                ]
            )

    @torch.no_grad()
    def retriev(self, elements, search_text):  # copied from sam text prompt method
        # preprocessed_images = [preprocess(image.astype(dtype=np.uint8)).to(device) for image in elements]
        preprocessed_images = [
            self.preprocess(Image.fromarray(image.astype(np.uint8))).to(self.device)
            for image in elements
        ]
        tokenized_text = clip.tokenize([search_text]).to(self.device)
        stacked_images = torch.stack(preprocessed_images)
        image_features = self.model.encode_image(stacked_images)
        text_features = self.model.encode_text(tokenized_text)
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        probs = 100.0 * image_features @ text_features.T
        return probs[:, 0].softmax(dim=0)

    def get_indices_of_values_above_threshold(self, values, threshold):
        # Pair each value with its index and filter by threshold
        filtered_values_with_indices = []
        filtered_values_with_indices = [
            (i, v) for i, v in enumerate(values) if v > threshold
        ]

        # Sort the filtered pairs by value in descending order, then extract indices
        sorted_indices = []
        sorted_indices = [
            i
            for i, v in sorted(
                filtered_values_with_indices, key=lambda x: x[1], reverse=True
            )
        ]

        return sorted_indices

    # Run sam and clip model
    def run_sam_clip(self, cv_image):
        chosen_masks = []
        masks = []
        # Run sam model
        start_time = time.time()
        masks = self.AutoMaskGen(cv_image)
        end_time = time.time()
        elapsed_time_ms = (end_time - start_time) * 1000
        rospy.loginfo("sam consumes: {:.2f} ms".format(elapsed_time_ms))
        # print("masks:", len(masks))
        rospy.loginfo("raw masks: {}".format(len(masks)))
        # Crop masks
        self.crop_masks(cv_image, masks)
        # Run clip model
        scores = []
        indices = []
        start_time = time.time()
        scores = self.retriev(self.cropped_boxes, self.search_text)
        end_time = time.time()
        elapsed_time_ms = (end_time - start_time) * 1000
        rospy.loginfo("clip consumes: {:.2f} ms".format(elapsed_time_ms))
        indices = self.get_indices_of_values_above_threshold(scores, 0.05)
        if len(indices) == 0:
            rospy.loginfo("No masks found!")
            return -1
        else:
            rows = len(indices)
            # cols = len(indices[0])
            rospy.loginfo("row: {}".format(rows))
            rospy.loginfo("indices: {}".format(indices[0]))
            # TODO: change to multiple masks
            chosen_masks = masks[indices[0]]

        segmentation_mask_image = chosen_masks["segmentation"].astype("uint8") * 255
        seg_image = cv_image.copy()
        seg_image[segmentation_mask_image > 0] = [255, 0, 0]
        self.pub_img.publish(self.cv2rosimg(seg_image))
        # visualize CLIP result
        # plt.imshow(seg_image)
        # plt.axis('off')
        # plt.show()

        exported_masks = self.maskprocessing(chosen_masks, True)
        self.Pub_mask(exported_masks)

    def rosimg2cv(self, image):
        # cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        # the ros image is in bgr8 format
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        # plt.imshow(cv_image)
        # plt.show()
        return cv_image

    def cv2rosimg(self, image):
        # ros_image = bridge.cv2_to_imgmsg(image, encoding="passthrough")

        # the cv image is in rgb8 format
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        return ros_image

    def Pub_mask(self, mask):

        # rate = rospy.Rate(10) #10hz
        self.pub.publish(mask)

    def callback(self, rosimage: SensorImage):
        cv_image = self.rosimg2cv(rosimage)

        if self.search_text is not None:
            self.run_sam_clip(cv_image)
        if self.search_text is None:
            masks = self.AutoMaskGen(cv_image)
            exported_masks = self.maskprocessing(
                masks, False
            )  # TODO: publish the list of masks after processing.
            self.Pub_mask(exported_masks)

        # save image
        # filename = 'saved_img.png'
        # cv2.imwrite(filename, cv_image)


if __name__ == "__main__":
    samclipros = SamClipRos()
    rospy.spin()
