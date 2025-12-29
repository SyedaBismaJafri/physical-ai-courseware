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