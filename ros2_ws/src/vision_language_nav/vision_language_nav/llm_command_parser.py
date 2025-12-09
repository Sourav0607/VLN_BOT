#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import time


class LLMCommandParser(Node):
    def __init__(self):
        super().__init__('llm_command_parser')

        # Ollama API endpoint (runs locally on jetson)
        self.ollama_url = "http://localhost:11434/api/generate"
        self.model_name = "neural-chat:7b-v3.1-q4_K_M"

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
        self.get_logger().info(f'Ollama API: {self.ollama_url}')

        # Test the ollama connection
        self.test_ollama_connection()

    def test_ollama_connection(self):
        """Test if ollama service is running"""

        try:
            response = requests.get("http://localhost:11434/api/tags",
                                    timeout=5)
            if response.status_code == 200:
                models = response.json().get("models", [])
                self.get_logger().info(f'Connected to Ollama. Available models: {len(models)}')
            else:
                self.get_logger().error(f'Failed to connect to Ollama API')
        except Exception as e:
            self.get_logger().error(f'Error connecting to Ollama API: {e}')
            self.get_logger().error('Make sure Ollama is running: ollama serve')

    
    def process_command(self, msg):
        """Parse text command with LLM and extract target object"""

        user_command = msg.data
        self.get_logger().info(f'Received command: "{user_command}"')

        # Create optimized prompt for object extraction
        prompt = f"""You are a helpful assistant that extracts target objects from navigation commands. Extract only one target object from the command below. Reply with only the target object name, nothing else.
Command: "{user_command}"
Object: """

        try:
            start_time = time.time()

            # Call Ollama API with optimized parameters for Jetson
            response = requests.post(
                self.ollama_url,
                json={
                    "model": self.model_name,
                    "prompt": prompt,
                    "stream": False,
                    "temperature": 0.1,
                    "top_p": 0.9,
                    "top_k": 40,
                    "num_predict": 10, # Short response
                },
                timeout=30
            )

            inference_time = time.time() - start_time

            if response.status_code == 200:
                result = response.json()
                extracted_object = result['response'].strip().lower()

                # Clean up the response
                extracted_object = extracted_object.split('\n')[0].strip()

                self.get_logger().info(f'LLM extracted: "{extracted_object}" (inference time: {inference_time:.2f}s)')

                # Find best match from coco classes
                matched_object = self.find_best_match(extracted_object)

                if matched_object:
                    # Publish target object to YOLO detector
                    target_msg = String()
                    target_msg.data = matched_object
                    self.publisher_.publish(target_msg)
                    self.get_logger().info(f'✓ Published target: "{matched_object}" (class ID: {self.coco_classes[matched_object]}) to YOLO')
                else:
                    available_classes = ", ".join(list(self.coco_classes.keys())[:10])
                    self.get_logger().warn(
                        f'Object "{extracted_object}" not found in COCO classes. '
                        f'Available: {available_classes}...'
                    )
            else:
                self.get_logger().error(f'Error from Ollama API: Status {response.status_code}')

        except requests.exceptions.Timeout:
            self.get_logger().error('Ollama API request timed out (>30s). Model might be too large.')
        except requests.exceptions.ConnectionError:
            self.get_logger().error('Connection error while connecting to Ollama API.')
        except Exception as e:
            self.get_logger().error(f'An error occurred: {e}')

    def find_best_match(self, extracted_text):
        """Find best matching COCO class from extracted text"""

        extracted_lower = extracted_text.lower()

        # Direct match
        if extracted_lower in self.coco_classes:
            return extracted_lower
        
        # Partial match - check if any class is contained in extracted text
        for class_name in self.coco_classes.keys():
            if class_name in extracted_lower:
                return class_name
            
        # Check if extracted text contains any coco class 
        for class_name in self.coco_classes.keys():
            if extracted_lower in class_name:
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