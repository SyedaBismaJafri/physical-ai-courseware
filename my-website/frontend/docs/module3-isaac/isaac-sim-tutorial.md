---
sidebar_position: 2
title: "Isaac Sim Tutorial"
---

# Isaac Sim Tutorial

## Overview

This tutorial will guide you through setting up and using NVIDIA Isaac Sim for robotics simulation. Isaac Sim provides high-fidelity physics simulation and photorealistic rendering capabilities.

## Setting Up Isaac Sim

### Prerequisites

- NVIDIA GPU with RTX capabilities (recommended)
- CUDA 11.8 or later
- Isaac Sim 2023.1 or later
- Omniverse Launcher

### Basic Isaac Sim Script

```python
# basic_isaac_sim.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.robots import Robot
import numpy as np

def setup_environment():
    """Set up the Isaac Sim environment"""
    # Create the world
    world = World(stage_units_in_meters=1.0)

    # Get assets root path
    assets_root_path = get_assets_root_path()

    if assets_root_path is None:
        print("Could not find Isaac Sim assets")
        return None

    # Add a simple ground plane
    create_prim(
        prim_path="/World/GroundPlane",
        prim_type="Plane",
        position=np.array([0, 0, 0]),
        scale=np.array([10, 10, 1])
    )

    # Add a simple robot (Franka Panda in this example)
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka.usd",
        prim_path="/World/Robot"
    )

    return world

def main():
    # Setup the environment
    world = setup_environment()

    if world is None:
        print("Failed to set up environment")
        return

    # Reset the world to start simulation
    world.reset()

    # Run simulation steps
    for i in range(1000):  # Run for 1000 steps
        # Perform any custom logic here
        world.step(render=True)

        # Example: Print simulation time every 100 steps
        if i % 100 == 0:
            print(f"Simulation step: {i}, Time: {world.current_time}")

    # Cleanup
    world.clear()

if __name__ == "__main__":
    main()
```

## ROS 2 Bridge Integration

Isaac Sim includes a ROS 2 bridge that allows communication between Isaac Sim and ROS 2 nodes.

### Installing the ROS 2 Bridge

```bash
# Install the ROS 2 bridge extension
omni.isaac.ros2_bridge
```

### Example ROS 2 Bridge Script

```python
# ros2_bridge_example.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
import numpy as np

# Import ROS 2 bridge components
from omni.isaac.ros2_bridge import ROS2Bridge

def setup_ros2_robot():
    """Set up a robot with ROS 2 bridge"""
    world = World(stage_units_in_meters=1.0)

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print("Could not find Isaac Sim assets")
        return None

    # Add robot with ROS 2 bridge
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Carter/carter_navi.usd",
        prim_path="/World/Carter"
    )

    # Initialize ROS 2 bridge
    ros2_bridge = ROS2Bridge()

    return world, ros2_bridge

def ros2_control_example():
    """Example of controlling robot via ROS 2"""
    import rclpy
    from geometry_msgs.msg import Twist

    # Initialize ROS 2
    rclpy.init()

    # Create publisher for robot velocity commands
    node = rclpy.create_node('carter_controller')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # Send a velocity command
    msg = Twist()
    msg.linear.x = 0.5  # Move forward at 0.5 m/s
    msg.angular.z = 0.2  # Turn at 0.2 rad/s

    publisher.publish(msg)
    node.get_logger().info('Published velocity command')

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

def main():
    # Setup Isaac Sim with ROS 2 bridge
    world, ros2_bridge = setup_ros2_robot()

    if world is None:
        print("Failed to set up environment")
        return

    # Reset the world
    world.reset()

    # Example of ROS 2 control
    ros2_control_example()

    # Run simulation
    for i in range(500):
        world.step(render=True)

    # Cleanup
    world.clear()

if __name__ == "__main__":
    main()
```

## Perception in Isaac Sim

Isaac Sim provides realistic sensor simulation for developing perception algorithms.

### Camera Sensor Example

```python
# camera_sensor_example.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera
import numpy as np

def setup_camera_sensor():
    """Set up a camera sensor in Isaac Sim"""
    world = World(stage_units_in_meters=1.0)

    # Add a robot or scene
    assets_root_path = get_assets_root_path()
    if assets_root_path:
        add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Props/KIT/milk.usd",
            prim_path="/World/Milk"
        )

    # Create a camera sensor
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([1.0, 1.0, 1.0]),
        orientation=np.array([0.5, -0.5, -0.5, 0.5])  # Quaternion (x, y, z, w)
    )

    # Add camera to world
    world.scene.add(camera)

    return world, camera

def capture_images():
    """Capture and process images from the camera"""
    world, camera = setup_camera_sensor()

    # Reset the world
    world.reset()

    # Render and capture image
    camera.get_rgb()

    # Get camera properties
    intrinsic_matrix = camera.get_intrinsic_matrix()
    print(f"Camera intrinsic matrix:\n{intrinsic_matrix}")

    # Run simulation to capture more images
    for i in range(10):
        world.step(render=True)

        # Capture RGB image
        rgb_image = camera.get_rgb()
        print(f"Captured image {i+1} with shape: {rgb_image.shape}")

    # Cleanup
    world.clear()

if __name__ == "__main__":
    capture_images()
```

## Isaac ROS Extensions

Isaac ROS provides GPU-accelerated perception and navigation packages.

### Installing Isaac ROS Extensions

```bash
# Install Isaac ROS extensions via Omniverse
# Extensions -> Isaac ROS -> Enable required extensions
```

### Using Isaac ROS Image Pipelines

```python
# isaac_ros_image_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class IsaacROSImageProcessor(Node):
    def __init__(self):
        super().__init__('isaac_ros_image_processor')

        # Create subscriber for camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            '/camera/rgb/image_processed',
            10
        )

        self.cv_bridge = CvBridge()
        self.get_logger().info('Isaac ROS Image Processor initialized')

    def image_callback(self, msg):
        """Process incoming image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Apply some processing (example: edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Convert back to ROS Image message
            processed_msg = self.cv_bridge.cv2_to_imgmsg(edges, encoding='mono8')
            processed_msg.header = msg.header  # Preserve header

            # Publish processed image
            self.publisher.publish(processed_msg)

            self.get_logger().info('Processed and published image')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = IsaacROSImageProcessor()

    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        image_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

- Use GPU acceleration for complex perception tasks
- Validate simulation results with real-world data
- Optimize simulation parameters for performance
- Use synthetic data generation for training AI models
- Implement proper error handling and logging
- Document simulation assumptions and limitations

## Isaac Sim ROS Bridge Implementation

Here's a complete implementation of an Isaac Sim ROS bridge that demonstrates the concepts covered in this module:

```python title="backend/isaac_sim_scripts/isaac_ros_bridge.py"
#!/usr/bin/env python3

"""
Isaac Sim ROS Bridge

This script demonstrates how to connect Isaac Sim with ROS 2.
It serves as a starter script for Module 3 concepts in the Physical AI courseware.
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.range_sensor import _range_sensor
import numpy as np
import carb

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PointStamped
    from sensor_msgs.msg import LaserScan, Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    print("ROS 2 not available. Running in simulation-only mode.")
    ROS_AVAILABLE = False


class IsaacSimROSBridge:
    """
    A bridge between Isaac Sim and ROS 2 for robot simulation and control.
    """

    def __init__(self):
        self.world = None
        self.robot = None
        self.ros_node = None
        self.cv_bridge = None

        # Initialize Isaac Sim world
        self._setup_isaac_world()

        if ROS_AVAILABLE:
            # Initialize ROS 2
            rclpy.init()
            self.ros_node = ROS2BridgeNode()
            self.cv_bridge = CvBridge()

            # Create ROS publishers and subscribers
            self._setup_ros_comms()

    def _setup_isaac_world(self):
        """
        Set up the Isaac Sim environment
        """
        # Create the world
        self.world = World(stage_units_in_meters=1.0)

        # Get assets root path
        assets_root_path = get_assets_root_path()

        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets. Ensure Isaac Sim is properly installed.")
            return

        # Add a ground plane
        create_prim(
            prim_path="/World/GroundPlane",
            prim_type="Plane",
            position=np.array([0, 0, 0]),
            scale=np.array([10, 10, 1])
        )

        # Add a simple robot (using Carter for navigation example)
        add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/Carter/carter_navi.usd",
            prim_path="/World/Carter"
        )

        # Reset the world to apply changes
        self.world.reset()

        print("Isaac Sim world initialized")

    def _setup_ros_comms(self):
        """
        Set up ROS 2 publishers and subscribers
        """
        if not self.ros_node:
            return

        # Create subscribers
        self.ros_node.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.ros_node.create_subscription(String, '/robot_command', self._command_callback, 10)

        # Create publishers
        self.ros_node.scan_publisher = self.ros_node.create_publisher(LaserScan, '/scan', 10)
        self.ros_node.image_publisher = self.ros_node.create_publisher(Image, '/camera/rgb/image_raw', 10)

        print("ROS 2 communication established")

    def _cmd_vel_callback(self, msg):
        """
        Handle velocity commands from ROS 2
        """
        if not self.world or not self.world.is_playing():
            return

        # Get the robot articulation
        robot = self.world.scene.get_object("Carter")
        if robot:
            # This is a simplified example - actual implementation would depend on the robot model
            # For Carter, we would typically send wheel velocity commands
            print(f"Received velocity command: linear.x={msg.linear.x}, angular.z={msg.angular.z}")

            # In a real implementation, you would interface with the robot's drive system
            # This is where you'd send commands to the robot in Isaac Sim

    def _command_callback(self, msg):
        """
        Handle high-level commands from ROS 2
        """
        command = msg.data
        print(f"Received high-level command: {command}")

        # Process the command and potentially update Isaac Sim state
        if command == "reset_simulation":
            self.world.reset()
        elif command.startswith("move_to:"):
            # Parse coordinates and move robot (simplified)
            try:
                coords = command.split(":")[1].split(",")
                x, y = float(coords[0]), float(coords[1])
                print(f"Moving to coordinates: ({x}, {y})")
                # In a real implementation, you'd update the robot's position/target
            except:
                print("Invalid move_to command format. Use: move_to:x,y")

    def run_simulation(self):
        """
        Run the simulation loop
        """
        print("Starting simulation loop...")

        step_count = 0
        try:
            while True:
                # Step the Isaac Sim world
                self.world.step(render=True)

                # Process ROS 2 callbacks
                if ROS_AVAILABLE and self.ros_node:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.01)

                # Publish sensor data periodically
                if step_count % 10 == 0:  # Publish every 10 steps
                    self._publish_sensor_data()

                step_count += 1

                # Add exit condition if needed
                if step_count > 10000:  # Example exit condition
                    break

        except KeyboardInterrupt:
            print("Simulation interrupted by user")
        finally:
            self.cleanup()

    def _publish_sensor_data(self):
        """
        Publish sensor data from Isaac Sim to ROS 2
        """
        if not ROS_AVAILABLE or not self.ros_node:
            return

        # Publish a simulated laser scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = -np.pi / 2
        scan_msg.angle_max = np.pi / 2
        scan_msg.angle_increment = np.pi / 180  # 1 degree increments
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = [5.0] * 181  # Simulated ranges (181 points for 180 degrees)

        self.ros_node.scan_publisher.publish(scan_msg)

        # Publish a simulated camera image (empty for now)
        # In a real implementation, you'd capture from Isaac Sim camera
        try:
            # Create a dummy image
            dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
            image_msg = self.cv_bridge.cv2_to_imgmsg(dummy_image, encoding="bgr8")
            image_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"

            self.ros_node.image_publisher.publish(image_msg)
        except Exception as e:
            print(f"Error publishing image: {e}")

    def cleanup(self):
        """
        Clean up resources
        """
        print("Cleaning up...")

        if self.world:
            self.world.clear()

        if ROS_AVAILABLE and rclpy.ok():
            rclpy.shutdown()


class ROS2BridgeNode(Node):
    """
    ROS 2 node for the Isaac Sim bridge
    """

    def __init__(self):
        super().__init__('isaac_sim_ros_bridge')
        self.get_logger().info('Isaac Sim ROS Bridge node initialized')


def main():
    """
    Main function to run the Isaac Sim ROS bridge
    """
    bridge = IsaacSimROSBridge()

    try:
        bridge.run_simulation()
    except Exception as e:
        print(f"Error running simulation: {e}")
    finally:
        bridge.cleanup()


if __name__ == "__main__":
    main()
```