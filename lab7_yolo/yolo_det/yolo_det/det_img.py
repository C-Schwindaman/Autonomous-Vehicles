import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self, weights_path):
        super().__init__('yolo_detector_node')
        namespace = os.environ.get("ROBOT_NAMESPACE", "")
        if namespace:
            self.topic_name = f"/{namespace}/oakd/rgb/preview/image_raw"
        else:
            self.topic_name = "/oakd/rgb/preview/image_raw"
        self.get_logger().info(f"Attempting to subscribe to topic: {self.topic_name}")
        self.bridge = CvBridge()
        self.topic_name = '/oakd/rgb/preview/image_raw'
        if not os.path.exists(weights_path):
            self.get_logger().error(f"Weights file not found at: {weights_path}")
            rclpy.shutdown()
            return
        self.get_logger().info(f"Loading model from {weights_path}...")
        self.model = YOLO(weights_path)
        self.class_names = self.model.names
        self.get_logger().info("Model loaded successfully.")
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10)
        self.get_logger().info(f"Node started. Subscribing to {self.topic_name}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        results = self.model(cv_image, imgsz=256, conf=0.1, verbose=False)
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_index = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.class_names.get(cls_index, 'Unknown')
                label = f'{class_name}: {confidence:.2f}'
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                font_thickness = 1
                text_color = (0, 0, 0)
                bg_color = (0, 255, 0)
                (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
                padding = 5
                bg_x1 = x1
                bg_y1 = y1 - text_height - baseline - (padding * 2) 
                bg_x2 = x1 + text_width + (padding * 2)
                bg_y2 = y1
                if bg_y1 < 0:
                    bg_y1 = y1
                    bg_y2 = y1 + text_height + baseline + (padding * 2)
                cv2.rectangle(cv_image, (bg_x1, bg_y1), (bg_x2, bg_y2), bg_color, -1)
                text_origin_x = bg_x1 + padding
                text_origin_y = bg_y1 + text_height + padding
                cv2.putText(cv_image, label, (text_origin_x, text_origin_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), bg_color, 2)
        cv2.imshow("YOLO Detections", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='YOLO Detector ROS Node')
    parser.add_argument('--weights_dir', type=str, required=True, help='Path to the directory containing best.pt')
    known_args, _ = parser.parse_known_args()
    weights_file = os.path.join(known_args.weights_dir, 'weights', 'best.pt')
    yolo_detector_node = YoloDetector(weights_path=weights_file)
    try:
        rclpy.spin(yolo_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()