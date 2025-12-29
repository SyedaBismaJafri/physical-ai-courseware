#!/usr/bin/env python3

"""
Navigation Controller Node

This ROS 2 node handles robot navigation and path planning.
It serves as a starter script for Module 1 concepts in the Physical AI courseware.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class NavigationController(Node):
    """
    A navigation controller node for robot path planning and obstacle avoidance.
    """

    def __init__(self):
        super().__init__('navigation_controller')

        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Create subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Timer for navigation control
        self.timer = self.create_timer(0.1, self.navigation_control)

        # Navigation state
        self.current_pose = None
        self.laser_data = None
        self.target_pose = None  # Will be set later
        self.is_moving = False

        self.get_logger().info('Navigation Controller Node initialized')

    def laser_callback(self, msg):
        """
        Callback function to handle incoming laser scan data
        """
        self.laser_data = msg
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} ranges')

    def odom_callback(self, msg):
        """
        Callback function to handle incoming odometry data
        """
        self.current_pose = msg.pose.pose
        if self.target_pose:
            distance = self.calculate_distance_to_target()
            self.get_logger().debug(f'Distance to target: {distance:.2f}m')

    def calculate_distance_to_target(self):
        """
        Calculate Euclidean distance to target pose
        """
        if not self.current_pose or not self.target_pose:
            return float('inf')

        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def navigation_control(self):
        """
        Main navigation control loop
        """
        if not self.laser_data or not self.current_pose:
            return

        # Simple obstacle avoidance and target following
        cmd_vel = Twist()

        # Check for obstacles
        if self.has_obstacle_ahead():
            self.get_logger().info('Obstacle detected ahead, stopping')
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right to avoid
        else:
            # Move toward target if set
            if self.target_pose:
                cmd_vel = self.move_towards_target()
            else:
                # Default behavior: move forward slowly
                cmd_vel.linear.x = 0.2
                cmd_vel.angular.z = 0.0

        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)

    def has_obstacle_ahead(self):
        """
        Check if there's an obstacle in front of the robot
        """
        if not self.laser_data:
            return False

        # Check the front 30 degrees for obstacles within 1 meter
        front_ranges = self.laser_data.ranges[:15] + self.laser_data.ranges[-15:]
        min_distance = min([r for r in front_ranges if not math.isinf(r) and r > 0], default=float('inf'))

        return min_distance < 1.0  # 1 meter threshold

    def move_towards_target(self):
        """
        Generate velocity command to move towards target
        """
        cmd_vel = Twist()

        # Calculate direction to target
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Set target reached threshold
        if distance < 0.5:  # 50cm threshold
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            if self.is_moving:
                self.get_logger().info('Target reached!')
                self.is_moving = False
        else:
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)

            # Get current orientation (simplified - assumes robot faces along x-axis)
            current_angle = 2 * math.atan2(
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )

            # Simple proportional controller for angular velocity
            angle_diff = target_angle - current_angle
            # Normalize angle difference to [-π, π]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            cmd_vel.angular.z = max(-1.0, min(1.0, 2.0 * angle_diff))

            # Move forward if roughly aligned with target
            if abs(angle_diff) < 0.3:  # 0.3 rad ≈ 17 degrees
                cmd_vel.linear.x = min(0.5, distance)  # Slow down as we approach

            self.is_moving = True

        return cmd_vel

    def set_target(self, x, y):
        """
        Set a new navigation target
        """
        from geometry_msgs.msg import Pose
        self.target_pose = Pose()
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.is_moving = True
        self.get_logger().info(f'New target set: ({x}, {y})')


def main(args=None):
    """
    Main function to run the navigation controller node
    """
    rclpy.init(args=args)

    navigation_controller = NavigationController()

    # Example: Set a target after 5 seconds
    def set_example_target():
        navigation_controller.set_target(2.0, 2.0)

    # Schedule target setting (this is just an example)
    # In practice, targets would come from higher-level commands
    timer = navigation_controller.create_timer(5.0, set_example_target)

    try:
        rclpy.spin(navigation_controller)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()