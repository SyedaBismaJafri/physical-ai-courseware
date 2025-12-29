---
sidebar_position: 2
title: "VLA Implementation"
---

# VLA Implementation

## Building a Vision-Language-Action System

In this section, we'll implement a complete Vision-Language-Action system that can interpret natural language commands and execute appropriate robot actions.

### Complete VLA Architecture

```python
# vla_robot_system.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import numpy as np
from transformers import CLIPProcessor, CLIPModel
import whisper
import openai

class VLARobotSystem(Node):
    def __init__(self):
        super().__init__('vla_robot_system')

        # Initialize components
        self.cv_bridge = CvBridge()

        # Initialize CLIP for vision-language understanding
        self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

        # Initialize Whisper for speech-to-text
        self.whisper_model = whisper.load_model("base")

        # Publishers and subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.command_subscriber = self.create_subscription(
            String,
            '/robot_commands',
            self.command_callback,
            10
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Internal state
        self.current_image = None
        self.command_queue = []

        self.get_logger().info('VLA Robot System initialized')

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            self.get_logger().info('Received image from camera')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def command_callback(self, msg):
        """Process incoming command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Process command and execute action
        self.execute_command(command)

    def execute_command(self, command):
        """Execute a natural language command"""
        if not self.current_image:
            self.get_logger().warn('No image available for VLA processing')
            return

        try:
            # Process vision-language understanding
            action = self.vla_inference(self.current_image, command)

            # Execute the action
            self.publish_action(action)

        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')

    def vla_inference(self, image, text):
        """Perform VLA inference to generate action"""
        # Preprocess image and text
        inputs = self.clip_processor(text=[text], images=image, return_tensors="pt", padding=True)

        # Get similarity scores
        outputs = self.clip_model(**inputs)
        logits_per_image = outputs.logits_per_image
        probs = logits_per_image.softmax(dim=1)

        # For this example, we'll map the command to simple actions
        # In a real system, you'd have a more sophisticated action head
        if "forward" in text.lower() or "move" in text.lower():
            linear_x = 0.5
            angular_z = 0.0
        elif "left" in text.lower() or "turn left" in text.lower():
            linear_x = 0.0
            angular_z = 0.5
        elif "right" in text.lower() or "turn right" in text.lower():
            linear_x = 0.0
            angular_z = -0.5
        elif "stop" in text.lower():
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = 0.1  # Small forward movement for unknown commands
            angular_z = 0.0

        # Create Twist message
        action = Twist()
        action.linear.x = linear_x
        action.angular.z = angular_z

        return action

    def publish_action(self, action):
        """Publish action to robot"""
        self.velocity_publisher.publish(action)
        self.get_logger().info(f'Published action: linear.x={action.linear.x}, angular.z={action.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    vla_robot_system = VLARobotSystem()

    try:
        rclpy.spin(vla_robot_system)
    except KeyboardInterrupt:
        pass
    finally:
        vla_robot_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Whisper Integration for Voice Commands

```python
# voice_command_processor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
import whisper
import tempfile
import numpy as np
import sounddevice as sd
from scipy.io import wavfile

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Initialize Whisper model
        self.whisper_model = whisper.load_model("base")

        # Publishers and subscribers
        self.audio_subscriber = self.create_subscription(
            UInt8MultiArray,
            '/audio_input',
            self.audio_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            String,
            '/robot_commands',
            10
        )

        self.get_logger().info('Voice Command Processor initialized')

    def audio_callback(self, msg):
        """Process incoming audio data"""
        try:
            # Convert UInt8MultiArray to audio data
            audio_data = np.array(msg.data, dtype=np.uint8)

            # Save to temporary file and transcribe
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                # This is a simplified example - in practice, you'd need proper audio encoding
                # For now, we'll simulate the transcription process
                transcription = self.transcribe_audio(audio_data)

                if transcription:
                    # Publish the transcribed command
                    command_msg = String()
                    command_msg.data = transcription
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info(f'Published transcribed command: {transcription}')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def transcribe_audio(self, audio_data):
        """Transcribe audio using Whisper"""
        # In a real implementation, you'd convert the audio_data to proper format
        # and use Whisper to transcribe it
        # This is a placeholder implementation
        try:
            # Placeholder: return a simulated transcription
            # In real implementation, you'd use whisper.transcribe() with proper audio file
            return "move forward slowly"
        except Exception as e:
            self.get_logger().error(f'Whisper transcription error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    voice_processor = VoiceCommandProcessor()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with LLM for Advanced Reasoning

```python
# llm_reasoning_module.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import openai
import json

class LLMReasoningModule(Node):
    def __init__(self):
        super().__init__('llm_reasoning_module')

        # Initialize OpenAI client (you'll need to set your API key)
        # openai.api_key = os.getenv("OPENAI_API_KEY")

        # Publishers and subscribers
        self.command_subscriber = self.create_subscription(
            String,
            '/high_level_commands',
            self.command_callback,
            10
        )

        self.action_publisher = self.create_publisher(
            String,
            '/robot_action_plan',
            10
        )

        self.get_logger().info('LLM Reasoning Module initialized')

    def command_callback(self, msg):
        """Process high-level command using LLM reasoning"""
        command = msg.data
        self.get_logger().info(f'Received high-level command: {command}')

        # Use LLM to generate an action plan
        action_plan = self.generate_action_plan(command)

        if action_plan:
            # Publish the action plan
            plan_msg = String()
            plan_msg.data = json.dumps(action_plan)
            self.action_publisher.publish(plan_msg)
            self.get_logger().info(f'Published action plan: {action_plan}')

    def generate_action_plan(self, command):
        """Generate an action plan using LLM"""
        # In a real implementation, you'd call an LLM API
        # This is a simplified example
        prompt = f"""
        Given the robot command "{command}", generate a step-by-step action plan.
        Return the plan as a list of simple actions that the robot can execute.
        Consider the robot's capabilities: movement (forward, backward, turn),
        navigation, and basic manipulation if equipped.

        Return the response as a JSON list of actions.
        """

        # Placeholder implementation - in real system, call actual LLM
        if "pick up the red block" in command.lower():
            return ["approach_object", "identify_red_block", "plan_grasp", "execute_grasp", "lift_object"]
        elif "go to the kitchen" in command.lower():
            return ["localize_current_position", "plan_path_to_kitchen", "navigate_to_kitchen", "stop_at_destination"]
        else:
            return ["interpret_command", "plan_basic_action", "execute_action"]

        # Real implementation would be:
        # response = openai.ChatCompletion.create(
        #     model="gpt-3.5-turbo",
        #     messages=[{"role": "user", "content": prompt}]
        # )
        # return json.loads(response.choices[0].message.content)

def main(args=None):
    rclpy.init(args=args)
    llm_module = LLMReasoningModule()

    try:
        rclpy.spin(llm_module)
    except KeyboardInterrupt:
        pass
    finally:
        llm_module.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Validation Layer

```python
# safety_validator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import json

class SafetyValidator(Node):
    def __init__(self):
        super().__init__('safety_validator')

        # Publishers and subscribers
        self.action_subscriber = self.create_subscription(
            String,
            '/robot_action_plan',
            self.action_callback,
            10
        )

        self.safety_action_publisher = self.create_publisher(
            Twist,
            '/safe_cmd_vel',
            10
        )

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Internal state
        self.laser_data = None
        self.safety_enabled = True

        self.get_logger().info('Safety Validator initialized')

    def laser_callback(self, msg):
        """Update laser scan data"""
        self.laser_data = msg

    def action_callback(self, msg):
        """Process action with safety validation"""
        try:
            action_plan = json.loads(msg.data)
            self.get_logger().info(f'Received action plan: {action_plan}')

            # Validate and execute safe actions
            self.validate_and_execute(action_plan)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in action plan')
        except Exception as e:
            self.get_logger().error(f'Error processing action: {e}')

    def validate_and_execute(self, action_plan):
        """Validate actions for safety before execution"""
        for action in action_plan:
            if self.is_safe_to_execute(action):
                self.execute_safe_action(action)
            else:
                self.get_logger().warn(f'Safety check failed for action: {action}')
                self.emergency_stop()

    def is_safe_to_execute(self, action):
        """Check if an action is safe to execute"""
        if not self.safety_enabled:
            return True

        # Check laser data for obstacles
        if self.laser_data:
            # Check if forward movement would hit an obstacle
            if "forward" in action or "move" in action:
                min_distance = min(self.laser_data.ranges)
                if min_distance < 0.5:  # 50cm safety threshold
                    return False

        # Add more safety checks as needed
        return True

    def execute_safe_action(self, action):
        """Execute a validated action"""
        # Convert action to Twist command
        twist_cmd = self.action_to_twist(action)
        if twist_cmd:
            self.safety_action_publisher.publish(twist_cmd)
            self.get_logger().info(f'Executing safe action: {action}')

    def action_to_twist(self, action):
        """Convert action string to Twist message"""
        twist = Twist()

        if "forward" in action:
            twist.linear.x = 0.3
        elif "backward" in action:
            twist.linear.x = -0.3
        elif "turn left" in action:
            twist.angular.z = 0.5
        elif "turn right" in action:
            twist.angular.z = -0.5
        elif "stop" in action:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # Default small movement for unknown actions
            twist.linear.x = 0.1

        return twist

    def emergency_stop(self):
        """Emergency stop the robot"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.safety_action_publisher.publish(stop_cmd)
        self.get_logger().warn('Emergency stop activated!')

def main(args=None):
    rclpy.init(args=args)
    safety_validator = SafetyValidator()

    try:
        rclpy.spin(safety_validator)
    except KeyboardInterrupt:
        pass
    finally:
        safety_validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete System Integration

Now let's create a launch file that brings all components together:

```python
# launch/vla_robot_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # VLA Robot System
        Node(
            package='my_robot_vla',
            executable='vla_robot_system',
            name='vla_robot_system',
            output='screen'
        ),

        # Voice Command Processor
        Node(
            package='my_robot_vla',
            executable='voice_command_processor',
            name='voice_command_processor',
            output='screen'
        ),

        # LLM Reasoning Module
        Node(
            package='my_robot_vla',
            executable='llm_reasoning_module',
            name='llm_reasoning_module',
            output='screen'
        ),

        # Safety Validator
        Node(
            package='my_robot_vla',
            executable='safety_validator',
            name='safety_validator',
            output='screen'
        )
    ])
```

## Best Practices for VLA Systems

- Implement multiple safety layers to prevent harmful actions
- Use simulation extensively for training and validation
- Continuously monitor and log system behavior
- Provide human override capabilities
- Validate LLM outputs before execution
- Consider privacy and security implications of voice processing

## Complete Whisper-to-Action Implementation

Here's a complete implementation of a Whisper-to-Action service that demonstrates the VLA concepts covered in this module:

```python title="backend/vla_service/whisper_to_action.py"
#!/usr/bin/env python3

"""
Whisper-to-Action Service

This script demonstrates converting speech to text using Whisper
and then mapping that text to robot actions using VLA concepts.
It serves as a starter script for Module 4 concepts in the Physical AI courseware.
"""

import os
import sys
import time
import threading
import queue
import numpy as np
import sounddevice as sd
import whisper
import torch
from transformers import CLIPProcessor, CLIPModel
from dataclasses import dataclass
from typing import Optional, Dict, Any


@dataclass
class VoiceCommand:
    """Data class for voice commands"""
    text: str
    confidence: float
    timestamp: float


class WhisperToActionService:
    """
    A service that converts voice commands to robot actions using Whisper and VLA concepts.
    """

    def __init__(self, model_size="base"):
        """
        Initialize the Whisper-to-Action service

        Args:
            model_size: Size of the Whisper model ('tiny', 'base', 'small', 'medium', 'large')
        """
        self.model_size = model_size
        self.whisper_model = None
        self.clip_model = None
        self.clip_processor = None

        # Audio parameters
        self.sample_rate = 16000
        self.chunk_duration = 1.0  # seconds
        self.chunk_size = int(self.sample_rate * self.chunk_duration)

        # Audio buffers and queues
        self.audio_queue = queue.Queue()
        self.command_queue = queue.Queue()

        # State management
        self.is_listening = False
        self.is_running = False
        self.audio_thread = None
        self.processing_thread = None

        # Initialize models
        self._initialize_models()

        print(f"Whisper-to-Action service initialized with {model_size} model")

    def _initialize_models(self):
        """Initialize Whisper and CLIP models"""
        try:
            print("Loading Whisper model...")
            self.whisper_model = whisper.load_model(self.model_size)
            print("Whisper model loaded successfully")

            print("Loading CLIP model...")
            self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
            self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
            print("CLIP model loaded successfully")

        except Exception as e:
            print(f"Error initializing models: {e}")
            raise

    def audio_callback(self, indata, frames, time, status):
        """
        Callback function for audio input
        """
        if status:
            print(f"Audio status: {status}")

        # Add audio data to queue for processing
        audio_data = indata.copy()
        self.audio_queue.put(audio_data)

    def start_listening(self):
        """
        Start listening for voice commands
        """
        if self.is_running:
            print("Service is already running")
            return

        self.is_running = True
        self.is_listening = True

        # Start audio input thread
        self.audio_thread = threading.Thread(target=self._audio_input_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        print("Started listening for voice commands...")

    def stop_listening(self):
        """
        Stop listening for voice commands
        """
        self.is_running = False
        self.is_listening = False

        if self.audio_thread:
            self.audio_thread.join(timeout=2.0)

        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)

        print("Stopped listening for voice commands")

    def _audio_input_loop(self):
        """
        Audio input loop using sounddevice
        """
        try:
            with sd.InputStream(
                callback=self.audio_callback,
                channels=1,
                samplerate=self.sample_rate,
                dtype='float32',
                blocksize=self.chunk_size
            ):
                while self.is_running:
                    time.sleep(0.1)
        except Exception as e:
            print(f"Error in audio input loop: {e}")

    def _processing_loop(self):
        """
        Processing loop for audio chunks
        """
        audio_buffer = np.array([], dtype=np.float32)

        while self.is_running:
            try:
                # Get audio data from queue
                if not self.audio_queue.empty():
                    audio_chunk = self.audio_queue.get_nowait()
                    audio_buffer = np.concatenate([audio_buffer, audio_chunk.flatten()])

                    # Process when we have enough audio (2 seconds worth)
                    if len(audio_buffer) >= self.sample_rate * 2:
                        self._process_audio_chunk(audio_buffer)
                        audio_buffer = np.array([], dtype=np.float32)  # Reset buffer
                else:
                    time.sleep(0.1)  # Small delay to prevent busy waiting
            except queue.Empty:
                time.sleep(0.1)
            except Exception as e:
                print(f"Error in processing loop: {e}")

    def _process_audio_chunk(self, audio_data):
        """
        Process an audio chunk with Whisper
        """
        try:
            # Transcribe the audio
            result = self.whisper_model.transcribe(
                audio_data,
                language="en",
                task="transcribe"
            )

            if result["text"].strip():  # Only process non-empty results
                # Calculate confidence (simplified - based on log probability)
                avg_logprob = result.get("avg_logprob", -1.0)
                confidence = max(0.0, min(1.0, (avg_logprob + 2) / 2))  # Normalize to [0, 1]

                command = VoiceCommand(
                    text=result["text"].strip(),
                    confidence=confidence,
                    timestamp=time.time()
                )

                # Add to command queue for further processing
                self.command_queue.put(command)

                print(f"Recognized: '{command.text}' (confidence: {command.confidence:.2f})")

                # Process the command to generate actions
                self.process_voice_command(command)

        except Exception as e:
            print(f"Error processing audio chunk: {e}")

    def process_voice_command(self, command: VoiceCommand):
        """
        Process a recognized voice command and generate appropriate action
        """
        if command.confidence < 0.5:  # Low confidence, ignore
            print(f"Ignoring low confidence command: {command.text} (confidence: {command.confidence:.2f})")
            return

        # Simple command mapping - in a real system, this would use more sophisticated NLP
        action = self.map_command_to_action(command.text)

        if action:
            print(f"Command '{command.text}' mapped to action: {action}")
            # Here you would publish the action to a robot control system
            self.execute_action(action)

    def map_command_to_action(self, text: str) -> Optional[Dict[str, Any]]:
        """
        Map voice command text to robot action
        """
        text_lower = text.lower()

        # Define action mappings
        if any(word in text_lower for word in ["forward", "go forward", "move forward", "straight"]):
            return {"type": "move", "direction": "forward", "speed": 0.3}
        elif any(word in text_lower for word in ["backward", "go backward", "move backward", "back"]):
            return {"type": "move", "direction": "backward", "speed": 0.3}
        elif any(word in text_lower for word in ["left", "turn left", "go left"]):
            return {"type": "turn", "direction": "left", "angle": 90}
        elif any(word in text_lower for word in ["right", "turn right", "go right"]):
            return {"type": "turn", "direction": "right", "angle": 90}
        elif any(word in text_lower for word in ["stop", "halt", "freeze"]):
            return {"type": "stop"}
        elif any(word in text_lower for word in ["dance", "show", "perform"]):
            return {"type": "dance", "pattern": "simple"}
        elif any(word in text_lower for word in ["hello", "hi", "greet"]):
            return {"type": "greet"}
        else:
            # For complex commands, you might use an LLM to interpret intent
            print(f"Unknown command: {text}. Using default action.")
            return {"type": "default", "command": text}

    def execute_action(self, action: Dict[str, Any]):
        """
        Execute the mapped action (placeholder implementation)
        """
        action_type = action.get("type")

        if action_type == "move":
            direction = action.get("direction")
            speed = action.get("speed", 0.3)
            print(f"Moving {direction} at speed {speed}")
            # In a real system, this would send commands to the robot
        elif action_type == "turn":
            direction = action.get("direction")
            angle = action.get("angle", 90)
            print(f"Turning {direction} by {angle} degrees")
            # In a real system, this would send turn commands to the robot
        elif action_type == "stop":
            print("Stopping robot")
            # In a real system, this would send stop command to the robot
        elif action_type == "dance":
            pattern = action.get("pattern", "simple")
            print(f"Performing {pattern} dance")
            # In a real system, this would execute a dance routine
        elif action_type == "greet":
            print("Robot greeting gesture")
            # In a real system, this might control robot's lights, sounds, or movements
        else:
            print(f"Executing action: {action}")

    def get_latest_command(self) -> Optional[VoiceCommand]:
        """
        Get the most recent voice command
        """
        latest_command = None
        while not self.command_queue.empty():
            try:
                command = self.command_queue.get_nowait()
                latest_command = command
            except queue.Empty:
                break
        return latest_command

    def demo_mode(self):
        """
        Run a demonstration of the service
        """
        print("\nStarting demo mode...")
        print("Available commands:")
        print("  - 'go forward' - Move robot forward")
        print("  - 'turn left' - Turn robot left")
        print("  - 'turn right' - Turn robot right")
        print("  - 'stop' - Stop the robot")
        print("  - 'hello' - Greet command")
        print("\nSpeak into your microphone...")

        self.start_listening()

        try:
            while self.is_running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nDemo interrupted by user")
        finally:
            self.stop_listening()


def main():
    """
    Main function to run the Whisper-to-Action service
    """
    if not torch.cuda.is_available():
        print("Warning: CUDA not available. Using CPU for processing (will be slower).")

    try:
        # Initialize the service
        service = WhisperToActionService(model_size="base")

        # Run in demo mode
        service.demo_mode()

    except Exception as e:
        print(f"Error running Whisper-to-Action service: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
```