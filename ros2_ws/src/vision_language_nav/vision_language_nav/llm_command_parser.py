#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import time


class LLMCommandParser(Node):
    def __init__(self):
        super().__init__('llm_command_parser')

        # Model name
        self.model_name = "tinyllama"

        # COCO classes mapping for yolov8 (name: class_id)
        self.coco_classes = {
            "person": 0, "bicycle": 1, "car": 2, "motorcycle": 3, "airplane": 4,
            "bus": 5, "train": 6, "truck": 7, "boat": 8, "traffic light": 9,
            "fire hydrant": 10, "stop sign": 11, "parking meter": 12, "bench": 13,
            "bird": 14, "cat": 15, "dog": 16, "horse": 17, "sheep": 18, "cow": 19,
            "elephant": 20, "bear": 21, "zebra": 22, "giraffe": 23, "backpack": 24,
            "umbrella": 25, "handbag": 26, "tie": 27, "suitcase": 28, "frisbee": 29,
            "skis": 30, "snowboard": 31, "sports ball": 32, "kite": 33, "baseball bat": 34,
            "baseball glove": 35, "skateboard": 36, "surfboard": 37, "tennis racket": 38,
            "bottle": 39, "wine glass": 40, "cup": 41, "fork": 42, "knife": 43, "spoon": 44,
            "bowl": 45, "banana": 46, "apple": 47, "sandwich": 48, "orange": 49, "broccoli": 50,
            "carrot": 51, "hot dog": 52, "pizza": 53, "donut": 54, "cake": 55, "chair": 56,
            "couch": 57, "potted plant": 58, "bed": 59, "dining table": 60, "toilet": 61,
            "tv": 62, "laptop": 63, "mouse": 64, "remote": 65, "keyboard": 66, "cell phone": 67,
            "microwave": 68, "oven": 69, "toaster": 70, "sink": 71, "refrigerator": 72,
            "book": 73, "clock": 74, "vase": 75, "scissors": 76, "teddy bear": 77,
            "hair drier": 78, "toothbrush": 79,
        }

        # Reverse mapping for quick lookup
        self.class_id_to_name = {v: k for k, v in self.coco_classes.items()}

        # Subscriber to receive LLM commands
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.process_command,
            10)     
        
        # Publisher to send parsed commands
        self.publisher_ = self.create_publisher(String, '/target_object', 10)

        self.get_logger().info("LLM Command Parser Node has been started.")
        self.get_logger().info(f'Using Ollama model: {self.model_name}')
        self.get_logger().info('Using CPU-only inference for Jetson Orin Nano')

        # Test the ollama connection
        self.test_ollama_connection()

    def test_ollama_connection(self):
        """Test if ollama service is running"""
        try:
            result = subprocess.run(
                ['ollama', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                self.get_logger().info('✓ Connected to Ollama')
            else:
                self.get_logger().error(f'Ollama error: {result.stderr}')
        except Exception as e:
            self.get_logger().error(f'Error connecting to Ollama: {e}')

    def call_ollama(self, prompt):
        """Call Ollama using CPU only (no CUDA)"""
        try:
            start_time = time.time()
            
            # Disable GPU, use only CPU
            env = os.environ.copy()
            env['CUDA_VISIBLE_DEVICES'] = ''  # Disable CUDA
            env['OLLAMA_NUM_GPU'] = '0'  # Disable GPU layers
            
            result = subprocess.run(
                ['ollama', 'run', '--nowordwrap', self.model_name, prompt],
                capture_output=True,
                text=True,
                timeout=120,
                env=env
            )
            
            inference_time = time.time() - start_time
            
            if result.returncode == 0:
                return result.stdout.strip(), inference_time
            else:
                self.get_logger().error(f'Ollama stderr: {result.stderr}')
                return None, inference_time
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Ollama command timed out (>120s)')
            return None, 120.0
        except Exception as e:
            self.get_logger().error(f'Error calling Ollama: {e}')
            return None, 0.0
    
    def keyword_fallback_parser(self, command):
        """Fallback keyword-based parser if LLM fails"""
        command_lower = command.lower()
        
        # Simple keyword matching
        keywords = {
            'person': ['person', 'human', 'guy', 'girl', 'man', 'woman', 'people'],
            'dog': ['dog', 'puppy', 'canine'],
            'cat': ['cat', 'kitten', 'feline'],
            'chair': ['chair', 'seat', 'stool'],
            'car': ['car', 'vehicle', 'automobile'],
            'bicycle': ['bicycle', 'bike', 'cycle'],
            'bottle': ['bottle', 'water bottle'],
            'cup': ['cup', 'mug', 'glass'],
            'book': ['book', 'reading'],
            'laptop': ['laptop', 'computer', 'pc'],
        }
        
        for obj_name, keywords_list in keywords.items():
            for keyword in keywords_list:
                if keyword in command_lower:
                    return obj_name
        
        return None
    
    def process_command(self, msg):
        """Parse text command with LLM and extract target object"""

        user_command = msg.data
        self.get_logger().info(f'Received command: "{user_command}"')

        # Create prompt for object extraction
        prompt = f"Extract object name: {user_command}\nObject:"

        # Try LLM first
        extracted_object, inference_time = self.call_ollama(prompt)
        
        if extracted_object:
            extracted_object = extracted_object.strip().lower().split('\n')[0].strip()
            self.get_logger().info(f'LLM extracted: "{extracted_object}" ({inference_time:.2f}s)')
        else:
            # Fallback to keyword-based parser
            self.get_logger().warn('LLM failed, using keyword-based fallback parser')
            extracted_object = self.keyword_fallback_parser(user_command)
            if extracted_object:
                self.get_logger().info(f'Fallback extracted: "{extracted_object}"')

        # Find best match from coco classes
        if extracted_object:
            matched_object = self.find_best_match(extracted_object)

            if matched_object:
                # Publish target object to YOLO detector
                target_msg = String()
                target_msg.data = matched_object
                self.publisher_.publish(target_msg)
                self.get_logger().info(f'✓ Target: "{matched_object}" (class {self.coco_classes[matched_object]}) → YOLO')
            else:
                available = ", ".join(list(self.coco_classes.keys())[:5])
                self.get_logger().warn(f'"{extracted_object}" not in COCO. Available: {available}...')
        else:
            self.get_logger().error('Failed to extract object from command')

    def find_best_match(self, extracted_text):
        """Find best matching COCO class from extracted text"""

        extracted_lower = extracted_text.lower().strip()

        # Direct match
        if extracted_lower in self.coco_classes:
            return extracted_lower
        
        # Partial match - check if any class is contained in extracted text
        for class_name in self.coco_classes.keys():
            if class_name in extracted_lower and len(class_name) > 2:
                return class_name
            
        # Check if extracted text contains any coco class 
        for class_name in self.coco_classes.keys():
            if extracted_lower in class_name and len(class_name) > 2:
                return class_name
            
        return None

    
def main(args=None):
    rclpy.init(args=args)

    llm_command_parser = LLMCommandParser()

    rclpy.spin(llm_command_parser)

    llm_command_parser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()