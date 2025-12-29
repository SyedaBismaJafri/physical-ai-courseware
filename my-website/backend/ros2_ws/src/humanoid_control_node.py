#!/usr/bin/env python3

"""
Humanoid Control Node

This is a sample ROS 2 node that demonstrates basic humanoid robot control.
It serves as a reference for Module 1 concepts in the Physical AI courseware.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class HumanoidControlNode(Node):
    """
    A sample node for controlling a humanoid robot.
    This demonstrates basic ROS 2 concepts including:
    - Node creation and lifecycle
    - Publisher and subscriber patterns
    - Message types and data handling
    """

    def __init__(self):
        super().__init__('humanoid_control_node')

        # Create publishers
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Create subscribers
        self.command_subscriber = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        # Timer for periodic joint state publishing
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Robot joint positions (simplified humanoid model)
        self.joint_positions = {
            'left_hip': 0.0,
            'right_hip': 0.0,
            'left_knee': 0.0,
            'right_knee': 0.0,
            'left_ankle': 0.0,
            'right_ankle': 0.0,
            'left_shoulder': 0.0,
            'right_shoulder': 0.0,
            'left_elbow': 0.0,
            'right_elbow': 0.0,
        }

        self.get_logger().info('Humanoid Control Node initialized')

    def command_callback(self, msg):
        """
        Callback function to handle incoming commands
        """
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Process command and update joint positions
        if command == 'walk_forward':
            self.execute_walk_forward()
        elif command == 'wave':
            self.execute_wave()
        elif command == 'stand':
            self.execute_stand()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def execute_walk_forward(self):
        """
        Execute walking forward motion
        """
        self.get_logger().info('Executing walk forward')
        # Simplified walking gait - in reality this would be much more complex
        for joint in self.joint_positions:
            if 'hip' in joint or 'knee' in joint:
                self.joint_positions[joint] = math.sin(self.get_clock().now().nanoseconds * 1e-9) * 0.2

    def execute_wave(self):
        """
        Execute waving motion with right arm
        """
        self.get_logger().info('Executing wave')
        # Wave motion for right arm
        self.joint_positions['right_shoulder'] = math.sin(self.get_clock().now().nanoseconds * 1e-9) * 0.5
        self.joint_positions['right_elbow'] = math.cos(self.get_clock().now().nanoseconds * 1e-9) * 0.3

    def execute_stand(self):
        """
        Return to standing position
        """
        self.get_logger().info('Executing stand')
        # Reset all joints to neutral position
        for joint in self.joint_positions:
            self.joint_positions[joint] = 0.0

    def publish_joint_states(self):
        """
        Publish current joint states
        """
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_state_publisher.publish(msg)


def main(args=None):
    """
    Main function to run the humanoid control node
    """
    rclpy.init(args=args)

    humanoid_control_node = HumanoidControlNode()

    try:
        rclpy.spin(humanoid_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()