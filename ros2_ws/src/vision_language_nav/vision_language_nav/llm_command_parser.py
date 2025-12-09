#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import time
import json


class LLMCommandParser(Node):
    def __init__(self):
        super().__init__('llm_command_parser')

        # Model name
        self.model_name = "tinyllama"
        self.use_llm = True  # Flag to enable/disable LLM

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

        # Enhanced keyword map (fallback)
        self.keyword_map = {
            'person': ['person', 'human', 'guy', 'girl', 'man', 'woman', 'people', 'someone'],
            'dog': ['dog', 'puppy', 'canine'],
            'cat': ['cat', 'kitten', 'feline'],
            'chair': ['chair', 'seat', 'stool'],
            'car': ['car', 'vehicle', 'automobile'],
            'bicycle': ['bicycle', 'bike', 'cycle'],
            'bottle': ['bottle'],
            'cup': ['cup', 'mug', 'glass'],
            'book': ['book'],
            'laptop': ['laptop', 'computer', 'pc'],
        }

        # Subscriber to receive LLM commands
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.process_command,
            10)     
        
        # Publisher to send parsed commands
        self.publisher_ = self.create_publisher(String, '/target_object', 10)

        self.get_logger().info("LLM Command Parser started!")
        self.get_logger().info(f"  Model: {self.model_name}")
        self.get_logger().info("  Checking Ollama service...")
        
        # Test Ollama connection
        if not self.test_ollama_connection():
            self.get_logger().warn("Ollama not available - will use keyword fallback")
            self.use_llm = False
        else:
            self.get_logger().info("✓ Ollama service detected")
            # Try to load model
            if self.preload_model():
                self.get_logger().info(f"Model '{self.model_name}' ready")
            else:
                self.get_logger().warn("Model failed to load - using fallback")
                self.use_llm = False

    def test_ollama_connection(self):
        """Test if Ollama service is running"""
        try:
            result = subprocess.run(
                ['curl', '-s', 'http://localhost:11434/api/tags'],
                capture_output=True,
                text=True,
                timeout=2
            )
            return result.returncode == 0
        except:
            return False

    def preload_model(self):
        """Try to load the model to check if it works"""
        try:
            self.get_logger().info(f"  Loading {self.model_name} model...")
            
            # Use echo to pipe a simple test
            test_cmd = f'echo "test" | CUDA_VISIBLE_DEVICES="" ollama run {self.model_name} --verbose 2>&1 | head -n 1'
            
            result = subprocess.run(
                test_cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=30,
                env={**os.environ, 'CUDA_VISIBLE_DEVICES': '', 'OLLAMA_NUM_GPU': '0'}
            )
            
            # Check if it returned without CUDA error
            if 'CUDA' in result.stderr or 'unable to allocate' in result.stderr:
                self.get_logger().error("  GPU allocation error detected")
                return False
            
            return True
            
        except subprocess.TimeoutExpired:
            self.get_logger().warn("  Model load timeout")
            return False
        except Exception as e:
            self.get_logger().error(f"  Model load error: {e}")
            return False

    def call_ollama_llm(self, prompt):
        """Call Ollama LLM with CPU-only mode"""
        try:
            start_time = time.time()
            
            # Create a temporary file with the prompt
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.txt') as f:
                f.write(prompt)
                prompt_file = f.name
            
            try:
                # Use cat to pipe prompt to ollama
                cmd = f'cat {prompt_file} | CUDA_VISIBLE_DEVICES="" OLLAMA_NUM_GPU=0 ollama run {self.model_name}'
                
                result = subprocess.run(
                    cmd,
                    shell=True,
                    capture_output=True,
                    text=True,
                    timeout=60,
                    env={**os.environ, 'CUDA_VISIBLE_DEVICES': '', 'OLLAMA_NUM_GPU': '0'}
                )
                
                inference_time = time.time() - start_time
                
                if result.returncode == 0 and 'CUDA' not in result.stderr:
                    return result.stdout.strip(), inference_time
                else:
                    self.get_logger().error(f"LLM error: {result.stderr[:200]}")
                    return None, inference_time
                    
            finally:
                # Clean up temp file
                os.unlink(prompt_file)
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('LLM timeout (>60s)')
            return None, 60.0
        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
            return None, 0.0
    
    def keyword_fallback_parser(self, command):
        """Fallback keyword-based parser"""
        command_lower = command.lower()
        
        for obj_name, keywords in self.keyword_map.items():
            for keyword in keywords:
                if keyword in command_lower:
                    return obj_name
        
        return None
    
    def process_command(self, msg):
        """Parse text command with LLM or fallback"""

        user_command = msg.data
        self.get_logger().info(f'Command: "{user_command}"')

        extracted_object = None
        used_method = "none"

        # Try LLM first if enabled
        if self.use_llm:
            prompt = f"Question: What object should the robot go to?\nCommand: {user_command}\nAnswer with only one word - the object name:"
            
            llm_response, inference_time = self.call_ollama_llm(prompt)
            
            if llm_response and 'CUDA' not in llm_response:
                extracted_object = llm_response.strip().lower().split('\n')[0].strip()
                used_method = f"LLM ({inference_time:.2f}s)"
                self.get_logger().info(f'LLM extracted: "{extracted_object}" in {inference_time:.2f}s')
        
        # Fallback to keyword parser
        if not extracted_object:
            extracted_object = self.keyword_fallback_parser(user_command)
            if extracted_object:
                used_method = "Keyword fallback"
                self.get_logger().info(f'Keyword extracted: "{extracted_object}"')

        # Find best match from COCO classes
        if extracted_object:
            matched_object = self.find_best_match(extracted_object)

            if matched_object:
                target_msg = String()
                target_msg.data = matched_object
                self.publisher_.publish(target_msg)
                self.get_logger().info(
                    f'Target: "{matched_object}" (class {self.coco_classes[matched_object]}) '
                    f'via {used_method} -> YOLO'
                )
            else:
                self.get_logger().warn(f'"{extracted_object}" not in COCO classes')
        else:
            self.get_logger().error('Failed to extract object')

    def find_best_match(self, extracted_text):
        """Find best matching COCO class"""
        extracted_lower = extracted_text.lower().strip()

        if extracted_lower in self.coco_classes:
            return extracted_lower
        
        for class_name in self.coco_classes.keys():
            if class_name in extracted_lower and len(class_name) > 2:
                return class_name
            
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