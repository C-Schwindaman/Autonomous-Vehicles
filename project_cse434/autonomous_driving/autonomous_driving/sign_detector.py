
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import argparse


class SignDetectorNode(Node):
    def __init__(self, weights_path, image_topic='/camera/image_raw', 
                 confidence_threshold=0.5, detection_rate=5.0, display=True):
        super().__init__('sign_detector_node')
            
        self.get_logger().info(f'Loading YOLO model from: {weights_path}')
        self.model = YOLO(weights_path)
        self.get_logger().info('YOLO model loaded successfully')
        
        # Parameters
        self.conf_threshold = confidence_threshold
        self.display = display
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscriber to camera images
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.sign_pub = self.create_publisher(
            String,
            '/signs/detected',
            10
        )
        
        self.detection_timer = self.create_timer(
            1.0 / detection_rate,
            self.timer_callback
        )
        
        self.latest_image = None
        self.latest_detection = "none"
        self.detection_confidence = 0.0
        self.frame_count = 0

    def image_callback(self, msg):
        """Store latest image for processing"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

    def timer_callback(self):
        """Process image at specified rate"""
        if self.latest_image is None:
            return
        
        self.frame_count += 1
        
        # Run YOLO detection
        results = self.model(
            self.latest_image,
            imgsz=256,
            conf=self.conf_threshold,
            verbose=False
        )
        
        detected_sign = "none"
        max_confidence = 0.0
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            # Get detection with highest confidence
            boxes = results[0].boxes
            confidences = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()  # Bounding boxes
            
            if len(confidences) > 0:
                max_idx = np.argmax(confidences)
                max_confidence = float(confidences[max_idx])
                class_id = int(classes[max_idx])
                
                # Get class name
                detected_sign = self.model.names[class_id]
                
                self.get_logger().info(
                    f'Detected: {detected_sign} (confidence: {max_confidence:.2f})'
                )
        
        # Update state
        self.latest_detection = detected_sign
        self.detection_confidence = max_confidence
        
        # Publish detection
        msg = String()
        msg.data = detected_sign
        self.sign_pub.publish(msg)
        
        # Display image with detections
        if self.display:
            self.display_results(results[0])

    def display_results(self, result):
        """Display image with bounding boxes and labels using cv2"""
        # Get original image
        img = self.latest_image.copy()
        
        # Get detection results
        boxes = result.boxes
        
        if len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy() 
            confidences = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy()
            
            # Draw each detection
            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, xyxy[i])
                conf = confidences[i]
                cls_id = int(classes[i])
                class_name = self.model.names[cls_id]
                
                # Draw bounding box
                color = (0, 255, 0)  # Green
                thickness = 2
                cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
                
                # Prepare label text
                label = f'{class_name}: {conf:.2f}'
                
                (text_width, text_height), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                
                cv2.rectangle(
                    img,
                    (x1, y1 - text_height - 10),
                    (x1 + text_width, y1),
                    color,
                    -1 
                )
                
                cv2.putText(
                    img,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 0), 
                    2
                )
        
        # Add status text at top
        status_text = f'Current: {self.latest_detection}'
        if self.detection_confidence > 0:
            status_text += f' ({self.detection_confidence:.2f})'
        
        # Add black background for status text
        cv2.rectangle(img, (5, 5), (400, 35), (0, 0, 0), -1)
        cv2.putText(
            img,
            status_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # Add frame counter
        frame_text = f'Frame: {self.frame_count}'
        cv2.putText(
            img,
            frame_text,
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )
        
        # Show image
        cv2.imshow('Sign Detection', img)
        cv2.waitKey(1)


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='YOLO Sign Detector Node')
    parser.add_argument('--weights_dir', type=str, required=True,
                       help='Path to directory containing best.pt weights')
    parser.add_argument('--topic', type=str, default='/camera/image_raw',
                       help='Image topic to subscribe to')
    parser.add_argument('--conf', type=float, default=0.5,
                       help='Confidence threshold (0-1)')
    parser.add_argument('--rate', type=float, default=5.0,
                       help='Detection rate in Hz')
    parser.add_argument('--no-display', action='store_true',
                       help='Disable image display window')
    
    parsed_args, unknown = parser.parse_known_args()
    
    # Construct full path to weights
    weights_path = os.path.join(parsed_args.weights_dir, 'best.pt')
    
    # Initialize ROS
    rclpy.init(args=args)
    
    try:
        node = SignDetectorNode(
            weights_path=weights_path,
            image_topic=parsed_args.topic,
            confidence_threshold=parsed_args.conf,
            detection_rate=parsed_args.rate,
            display=not parsed_args.no_display
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()