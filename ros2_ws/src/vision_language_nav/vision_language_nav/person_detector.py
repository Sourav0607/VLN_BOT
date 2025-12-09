#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
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
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8 model loaded successfully!')
        
        # Subscribe to RGB camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to target object from LLM parser
        self.target_object_sub = self.create_subscription(
            String,
            '/target_object',
            self.set_target_object,
            10
        )
        
        # Publisher for annotated image
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_detected',
            10
        )
        
        # Publisher for target distance
        self.distance_pub = self.create_publisher(
            Float32,
            '/target/distance',
            10
        )
        
        # Publisher for detected object class
        self.detected_class_pub = self.create_publisher(
            String,
            '/detected_object',
            10
        )
        
        # Target object (default: person)
        self.target_object = 'person'
        self.target_class_id = None
        self.target_confidence_threshold = 0.5
        
        self.frame_count = 0
        self.get_logger().info('Person detector initialized!')
        self.get_logger().info(f'Current target: {self.target_object}')
    
    def set_target_object(self, msg):
        """Receive target object from LLM parser"""
        new_target = msg.data.lower()
        if new_target != self.target_object:
            self.target_object = new_target
            self.target_class_id = None  # Reset class ID
            self.get_logger().info(f'🎯 Target object changed to: "{self.target_object}"')
    
    def get_class_id_for_object(self, class_name):
        """Get YOLO class ID for a given object name"""
        class_name_lower = class_name.lower()
        
        for cls_id, model_class_name in self.model.names.items():
            if model_class_name.lower() == class_name_lower:
                return cls_id
        
        # Try partial match
        for cls_id, model_class_name in self.model.names.items():
            if class_name_lower in model_class_name.lower():
                return cls_id
        
        self.get_logger().warn(f'Class "{class_name}" not found in YOLO model')
        return None
    
    def image_callback(self, msg):
        """Process RGB image with YOLO"""
        self.frame_count += 1
        
        # Log every 30 frames (once per second at 30Hz)
        log_this_frame = (self.frame_count % 30 == 0)
        
        if log_this_frame:
            self.get_logger().info(f'Processing frame {self.frame_count}...')
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model(cv_image, verbose=False)
            
            # Count detections
            target_count = 0
            total_detections = 0
            closest_distance = None
            closest_box = None
            
            # Process detections
            for result in results:
                boxes = result.boxes
                total_detections = len(boxes)
                
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = self.model.names[cls_id]
                    
                    # Log all detections periodically
                    if log_this_frame:
                        self.get_logger().info(
                            f'Detected: {class_name} (conf: {conf:.2f}, class_id: {cls_id})'
                        )
                    
                    # Check if matches target object
                    if (class_name.lower() == self.target_object and 
                        conf > self.target_confidence_threshold):
                        
                        target_count += 1
                        
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        # Estimate depth from bounding box height
                        bbox_height = y2 - y1
                        estimated_depth = (1.7 * 250) / bbox_height if bbox_height > 0 else 5.0
                        estimated_depth = max(0.0, estimated_depth - 0.8)
                        
                        # Track closest object
                        if closest_distance is None or estimated_depth < closest_distance:
                            closest_distance = estimated_depth
                            closest_box = (x1, y1, x2, y2, center_x, center_y, conf)
                        
                        depth_str = f"~{estimated_depth:.2f}m"
                        
                        # Draw bounding box
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Add label
                        label = f'{self.target_object}: {conf:.2f} | {depth_str}'
                        cv2.putText(cv_image, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # Draw center point
                        cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                        
                        if log_this_frame:
                            self.get_logger().info(
                                f'✓ {self.target_object.upper()} at ({center_x}, {center_y}) '
                                f'confidence: {conf:.2f} distance: {depth_str}'
                            )
            
            # Publish closest target distance and info
            if closest_distance is not None:
                distance_msg = Float32()
                distance_msg.data = closest_distance
                self.distance_pub.publish(distance_msg)
                
                # Publish detected object info
                obj_msg = String()
                obj_msg.data = self.target_object
                self.detected_class_pub.publish(obj_msg)
            
            # Log summary every 30 frames
            if log_this_frame:
                self.get_logger().info(
                    f'Frame {self.frame_count}: Total detections={total_detections}, '
                    f'Target "{self.target_object}"={target_count}'
                )
            
            # Add frame info overlay
            info_text = f'Target: {self.target_object} | Detections: {target_count}'
            cv2.putText(cv_image, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
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