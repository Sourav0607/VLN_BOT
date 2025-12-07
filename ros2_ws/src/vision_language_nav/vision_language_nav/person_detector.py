#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLOv8 model (nano version for faster inference)
        self.get_logger().info('Loading YOLOv8 model...')
        self.model = YOLO('yolov8n.pt')  # Downloads automatically if not present
        self.get_logger().info('YOLOv8 model loaded successfully!')
        
        # Subscribe to RGB camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for annotated image
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_detected',
            10
        )
        
        # Publisher for person distance
        self.distance_pub = self.create_publisher(
            Float32,
            '/person/distance',
            10
        )
        
        self.frame_count = 0
        self.get_logger().info('Person detector node started with pixel-based depth estimation!')
    
    def image_callback(self, msg):
        """Process RGB image with YOLO"""
        self.frame_count += 1
        
        # Log every 30 frames (once per second at 30Hz)
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Processing frame {self.frame_count}...')
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model(cv_image, verbose=False)
            
            # Count total detections
            person_count = 0
            total_detections = 0
            
            # Process detections
            for result in results:
                boxes = result.boxes
                total_detections = len(boxes)
                
                for box in boxes:
                    # Get class ID and confidence
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Log all detections for debugging
                    class_name = self.model.names[cls_id]
                    if self.frame_count % 30 == 0:
                        self.get_logger().info(f'Detected: {class_name} (conf: {conf:.2f})')
                    
                    # Check if detected object is a person (class 0 in COCO dataset)
                    if cls_id == 0 and conf > 0.5:
                        person_count += 1
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        # Estimate depth from bounding box height (pixel-based method)
                        # Assuming average person height ~1.7m and camera FOV ~60°
                        bbox_height = y2 - y1
                        # Calibrated depth estimation formula
                        # When person fills ~500 pixels, they're ~0.5m away
                        # When person fills ~250 pixels, they're ~1.5m away
                        estimated_depth = (1.7 * 250) / bbox_height if bbox_height > 0 else 5.0
                        # Apply calibration offset
                        estimated_depth = max(0.0, estimated_depth - 0.8)
                        depth_str = f"~{estimated_depth:.2f}m (h={bbox_height}px)"
                        
                        # Publish distance
                        distance_msg = Float32()
                        distance_msg.data = estimated_depth
                        self.distance_pub.publish(distance_msg)
                        
                        # Draw bounding box
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Add label with confidence and depth
                        label = f'Person: {conf:.2f} | {depth_str}'
                        
                        cv2.putText(cv_image, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # Draw center point
                        cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                        
                        # Log detection
                        self.get_logger().info(
                            f'Person detected at ({center_x}, {center_y}) '
                            f'with confidence {conf:.2f} at {depth_str}'
                        )
            
            # Log detection summary every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Total detections: {total_detections}, Persons: {person_count}')
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
