#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import os
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge
from utils.tool import handle_preds
from ultralytics import YOLO
from PIL import Image as PILImage, ImageDraw, ImageFont

target_classes = ["person"]


path_curr = os.path.dirname(__file__)

def msg_to_cv2_fallback(msg):
    """Fallback conversion from ROS Image message to OpenCV image"""
    try:
        if msg.encoding == "bgr8":
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = img_data.reshape((msg.height, msg.width, 3))
            return np.ascontiguousarray(img)
        elif msg.encoding == "rgb8":
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = img_data.reshape((msg.height, msg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return np.ascontiguousarray(img)
        else:
            return None
    except Exception as e:
        print(f"Fallback conversion error: {e}")
        return None
img_topic = "/camera1/rs1/color/image_raw"
device = "cuda" if torch.cuda.is_available() else "cpu"
weight = "weights/yolo11n.pt"
class_names = "config/coco.names"

class yolo_detector(Node):
    def __init__(self):
        super().__init__('yolov11_detector')
        print("[onboardDetector]: yolo detector init...")

        self.img_received = False
        self.img_detected = False

        # init and load
        self.model = YOLO(os.path.join(path_curr, weight))
        self.model.eval()

        # subscriber
        self.br = CvBridge()
        self.img_sub = self.create_subscription(Image, img_topic, self.image_callback, 10)

        # publisher
        self.img_pub = self.create_publisher(Image, 'yolo_detector/detected_image', 10)
        self.bbox_pub = self.create_publisher(Detection2DArray, 'yolo_detector/detected_bounding_boxes', 10)
        self.time_pub = self.create_publisher(Float64, 'yolo_detector/yolo_time', 1)

        # timers
        self.create_timer(0.033, self.detect_callback)
        self.create_timer(0.033, self.vis_callback)
        self.create_timer(0.033, self.bbox_callback)
    
    def image_callback(self, msg):
        try:
            # Direct conversion from raw message data - most reliable for Jazzy
            # frombuffer creates a read-only view, so we must copy
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            # Make a writable copy
            img_array = np.array(img_array, copy=True, dtype=np.uint8)
            # Reshape
            self.img = img_array.reshape((msg.height, msg.width, 3))
            
            # Handle color conversion if needed
            if msg.encoding == "rgb8":
                # Convert RGB to BGR in-place by manual channel swap (faster than cv2.cvtColor)
                self.img = self.img[:, :, ::-1].copy()
            
            # Ensure contiguous and correct type
            self.img = np.ascontiguousarray(self.img, dtype=np.uint8)
            
            # Validation
            assert isinstance(self.img, np.ndarray), f"Not numpy array: {type(self.img)}"
            assert self.img.dtype == np.uint8, f"Wrong dtype: {self.img.dtype}"
            assert self.img.flags['WRITEABLE'], "Array is not writable"
            
            self.img_received = True
            
        except Exception as e:
            self.get_logger().error(f"Image callback error: {type(e).__name__}: {e}")
            self.img_received = False

    def detect_callback(self):
        start_time = time.time()
        if self.img_received and hasattr(self, 'img') and self.img is not None:
            try:
                # Debug logging
                self.get_logger().debug(f"Before inference - type: {type(self.img)}, dtype: {self.img.dtype}, shape: {self.img.shape}, writable: {self.img.flags['WRITEABLE']}, C_CONTIGUOUS: {self.img.flags['C_CONTIGUOUS']}")
                
                output = self.inference(self.img)
                self.detected_img, self.detected_bboxes = self.postprocess(self.img, output)
                self.img_detected = True
            except Exception as e:
                self.get_logger().error(f"Error in inference: {type(e).__name__}: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
        elapsed = time.time() - start_time
        msg = Float64()
        msg.data = float(elapsed)
        self.time_pub.publish(msg)
        

    def vis_callback(self):
        if self.img_detected:
            img_msg = self.br.cv2_to_imgmsg(self.detected_img, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.img_pub.publish(img_msg)

    def bbox_callback(self):
        if self.img_detected:
            bboxes_msg = Detection2DArray()
            for detected_box in self.detected_bboxes:
                if detected_box[4] in target_classes:
                    bbox_msg = Detection2D()
                    # detected_box format: [x1, y1, x2, y2, class]
                    x1 = float(detected_box[0])
                    y1 = float(detected_box[1])
                    x2 = float(detected_box[2])
                    y2 = float(detected_box[3])
                    # Store top-left corner in hypothesis pose and size in bbox
                    from vision_msgs.msg import ObjectHypothesisWithPose
                    hyp = ObjectHypothesisWithPose()
                    hyp.pose.pose.position.x = x1  # top-left x
                    hyp.pose.pose.position.y = y1  # top-left y
                    bbox_msg.results.append(hyp)
                    bbox_msg.bbox.size_x = float(abs(x2 - x1))  # width
                    bbox_msg.bbox.size_y = float(abs(y2 - y1))  # height
                    bboxes_msg.detections.append(bbox_msg)
            bboxes_msg.header.stamp = self.get_clock().now().to_msg()
            self.bbox_pub.publish(bboxes_msg)

    def inference(self, ori_img):
        try:
            # Final safety conversion - reconstruct array completely to ensure it's pure numpy
            ori_img = np.array(ori_img, dtype=np.uint8, copy=True, order='C')
            
            # Verify it's actually a numpy array
            if not isinstance(ori_img, np.ndarray):
                raise TypeError(f"After conversion, still not numpy array: {type(ori_img)}")
            
            if ori_img.dtype != np.uint8:
                ori_img = ori_img.astype(np.uint8)
            
            # Ensure C-contiguous
            if not ori_img.flags['C_CONTIGUOUS']:
                ori_img = np.ascontiguousarray(ori_img, dtype=np.uint8)
            
            # Validate shape
            if len(ori_img.shape) != 3 or ori_img.shape[2] != 3:
                raise ValueError(f"Invalid image shape: {ori_img.shape}. Expected (H, W, 3)")
            
            self.get_logger().debug(f"In inference - final type: {type(ori_img)}, dtype: {ori_img.dtype}, shape: {ori_img.shape}, C_CONTIGUOUS: {ori_img.flags['C_CONTIGUOUS']}")
            
            # Use PIL for image resizing instead of cv2 to avoid numpy/OpenCV compatibility issues
            try:
                pil_img = PILImage.fromarray(ori_img, mode='RGB')
                pil_img = pil_img.resize((352, 352), PILImage.Resampling.BILINEAR)
                res_img = np.array(pil_img, dtype=np.uint8)
            except Exception as e:
                self.get_logger().warning(f"PIL resize failed: {e}. Trying torch...")
                # Fallback to torch resizing
                img_tensor = torch.from_numpy(ori_img.copy()).permute(2, 0, 1).unsqueeze(0).float()
                img_resized = torch.nn.functional.interpolate(img_tensor, size=(352, 352), mode='bilinear', align_corners=False)
                res_img = img_resized.squeeze(0).permute(1, 2, 0).numpy().astype(np.uint8)
            
            # Ensure res_img is a proper numpy array before torch conversion
            res_img = np.array(res_img, dtype=np.uint8, copy=True)
            res_img = np.ascontiguousarray(res_img)
            
            # Reshape and convert to tensor
            img = res_img.reshape(1, 352, 352, 3)
            # Convert to float32 for model input
            img = img.astype(np.float32)
            # Use torch.tensor() instead of torch.from_numpy() to avoid numpy version conflicts
            img_tensor = torch.tensor(img, dtype=torch.float32)
            # Transpose: (batch, height, width, channels) -> (batch, channels, height, width)
            img_tensor = img_tensor.permute(0, 3, 1, 2)
            img_tensor = img_tensor.to(device) / 255.0   

            # inference
            preds = self.model(img_tensor, device=device, half=True)[0]
            return [preds.boxes.xyxyn, preds.boxes.conf, preds.boxes.cls]
            
        except Exception as e:
            self.get_logger().error(f"Inference failed - INPUT type={type(ori_img).__name__}, dtype={getattr(ori_img, 'dtype', 'N/A')}, shape={getattr(ori_img, 'shape', 'N/A')}, error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise

    def postprocess(self, ori_img, output):
        # Ensure ori_img is a proper numpy array
        ori_img = np.array(ori_img, dtype=np.uint8, copy=True)
        ori_img = np.ascontiguousarray(ori_img)
        
        LABEL_NAMES = []
        with open(os.path.join(path_curr, class_names), 'r') as f:
            for line in f.readlines():
                LABEL_NAMES.append(line.strip())
        
        H, W, _ = ori_img.shape

        detected_boxes = []
        
        # Convert to PIL for drawing to avoid cv2 numpy compatibility issues
        pil_img = PILImage.fromarray(ori_img, mode='RGB')
        draw = ImageDraw.Draw(pil_img)
        
        try:
            # Try to use a default font, fall back to default if not available
            font = ImageFont.load_default()
        except:
            font = None
        
        for i, box in enumerate(output[0]):
            box = box.tolist()
           
            obj_score = output[1][i]
            category = LABEL_NAMES[int(output[2][i])]
            x1, y1 = int(box[0] * W), int(box[1] * H)
            x2, y2 = int(box[2] * W), int(box[3] * H)
            detected_box = [x1, y1, x2, y2, category]
            detected_boxes.append(detected_box)

            # Draw rectangle using PIL (yellow)
            draw.rectangle([x1, y1, x2, y2], outline=(255, 255, 0), width=2)
            
            # Draw text using PIL (green)
            score_text = '%.2f' % obj_score
            draw.text((x1, y1 - 25), category, fill=(0, 255, 0), font=font)
            draw.text((x1, y1 - 5), score_text, fill=(0, 255, 0), font=font)
        
        # Convert back to numpy array
        ori_img = np.array(pil_img, dtype=np.uint8)
        return ori_img, detected_boxes
