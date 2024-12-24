import numpy as np
import rospy
from nanoowl.owl_predictor import OwlPredictor
from nanosam.utils.predictor import Predictor
from masks_msgs.msg import maskID
from masks_msgs.msg import singlemask
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2
from PIL import Image
import sys
import time


class NanoSAMRos:
    def __init__(self):
        rospy.init_node("sam_node")
        self.image_topic = rospy.get_param(
            "~image_topic", "/camera/color/image_raw"
        )  # Default is image_raw topic of Tiago robot
        self.enable_metrics = rospy.get_param("/enable_metrics", False)
        self.mask_pub = rospy.Publisher(
            "/sam_mask", maskID, queue_size=1
        )  # TODO: pub np.ndarray related func: convert_msg() and Pub_mask()
        self.img_pub = rospy.Publisher("/sam_img", SensorImage, queue_size=1)
        self.img_sub = rospy.Subscriber(
            self.image_topic, SensorImage, self.callback, queue_size=1, buff_size=2**24
        )  # TODO: find image topic from Tiago!
        self.search_text = rospy.get_param("~search_text", "[a door]")
        self.bridge = CvBridge()
        # Load nanoowl and nanosam model
        owl_image_encoder = rospy.get_param(
            "~owl_image_encoder", "/opt/nanoowl/data/owl_image_encoder_patch32.engine"
        )
        sam_image_encoder = rospy.get_param(
            "~sam_image_encoder", "/opt/nanosam/data/resnet18_image_encoder.engine"
        )
        sam_mask_decoder = rospy.get_param(
            "~sam_mask_decoder", "/opt/nanosam/data/mobile_sam_mask_decoder.engine"
        )
        self.owl_predictor = OwlPredictor(image_encoder_engine=owl_image_encoder)
        rospy.loginfo("nanoowl model has been loaded.")
        text = self.search_text.strip("][()")
        self.text = text.split(",")
        self.text_encodings = self.owl_predictor.encode_text(text)
        self.sam_predictor = Predictor(sam_image_encoder, sam_mask_decoder)
        rospy.loginfo("nanosam model has been loaded.")
        rospy.loginfo("Node has been started.")

    def infer(self, cv_image):
        # rospy.loginfo("inference is triggered.")
        image = Image.fromarray(cv_image)
        if self.enable_metrics:
            t0 = time.perf_counter_ns()
        detections = self.owl_predictor.predict(
            image=image,  #! PIL format
            text=self.text,
            text_encodings=self.text_encodings,
            pad_square=False,
        )
        if self.enable_metrics:
            t1 = time.perf_counter_ns()
            dt1 = (t1 - t0) / 1e6
            rospy.loginfo(f"OWL time: {dt1:.3f}ms")

        num_detections = len(detections.labels)
        if num_detections == 0:
            rospy.loginfo("No detections found!")
            return None

        # TODO: use multiple bboxes as prompts
        max_score_idx = detections.scores.max(dim=0)[1].item()
        bbox = detections.boxes[max_score_idx].cpu()  # or choose max score one
        # bbox = detections.boxes[0].cpu() # or choose first one
        # TODO: or choose largest one
        points = np.array([[bbox[0], bbox[1]], [bbox[2], bbox[3]]])
        point_labels = np.array([2, 3])
        if self.enable_metrics:
            t2 = time.perf_counter_ns()
        self.sam_predictor.set_image(image)  #! PIL format
        mask, _, _ = self.sam_predictor.predict(points, point_labels)
        if self.enable_metrics:
            t3 = time.perf_counter_ns()
            dt2 = (t3 - t2) / 1e6
            rospy.loginfo(f"SAM time: {dt2:.3f}ms")
        return (bbox, mask)

    # Process the masks and ready to publish
    def convert_msg(self, results):
        # only single mask in text prompt mode
        # rospy.loginfo("convert_msg is triggered.")
        mask_list = []
        singlemask_msg = singlemask()
        bbox, mask = results
        mask_data = mask[0, 0].detach().cpu().numpy() > 0
        mask_shape = mask_data.shape
        segmentation_int = mask_data.astype(np.int64)
        masks_list = segmentation_int.flatten().tolist()

        singlemask_msg.maskid = 0
        singlemask_msg.shape = mask_shape
        singlemask_msg.segmentation = masks_list
        singlemask_msg.area = 0  # FIXME
        singlemask_msg.bbox = np.clip(
            bbox.numpy().round().astype(np.int32), (0, 0, 0, 0), (640, 480, 640, 480)
        ).tolist()
        mask_list.append(singlemask_msg)
        mask_list_msg = maskID()
        mask_list_msg.maskID = mask_list
        return mask_list_msg

    # Run nanoowl & nanosam model
    def run_nanosam(self, cv_image):
        results = self.infer(cv_image)
        if results is None:
            self.img_pub.publish(self.cv2rosimg(cv_image))
            return -1

        mask_overlay = (results[1][0, 0].detach().cpu().numpy() > 0).astype(
            np.uint8
        ) * 255
        mask_overlay = cv2.applyColorMap(mask_overlay, cv2.COLORMAP_PINK)
        seg_image = cv2.addWeighted(cv_image, 0.7, mask_overlay, 0.3, 0)
        self.img_pub.publish(self.cv2rosimg(seg_image))

        mask_msg = self.convert_msg(results)
        self.Pub_mask(mask_msg)

    def rosimg2cv(self, image):
        # cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        # the ros image is in bgr8 format
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        return cv_image

    def cv2rosimg(self, image):
        # ros_image = bridge.cv2_to_imgmsg(image, encoding="passthrough")

        # the cv image is in rgb8 format
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        return ros_image

    def Pub_mask(self, mask_msg):
        # rate = rospy.Rate(10) #10hz
        self.mask_pub.publish(mask_msg)

    def callback(self, rosimage: SensorImage):
        cv_image = self.rosimg2cv(rosimage)

        if self.search_text is None:
            rospy.loginfo("No search text provided.")
            return -1
        self.run_nanosam(cv_image)


if __name__ == "__main__":
    nanosamros = NanoSAMRos()
    rospy.spin()
