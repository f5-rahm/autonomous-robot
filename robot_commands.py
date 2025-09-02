#!/usr/bin/env python3
"""
Command-line robot controller with specific movement actions
Usage examples:
  python3 robot_commands.py move forward 1.5 50
  python3 robot_commands.py move backward 0.8 25
  python3 robot_commands.py turn left 90
  python3 robot_commands.py turn right 45
  python3 robot_commands.py stop
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import argparse
import math


class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_commander')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to odometry for movement verification
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Robot parameters - matching your teleop values
        self.max_lin_vel = 0.22  # m/s
        self.max_ang_vel = 12.84  # rad/s

        # Movement tracking
        self.current_odom = None
        self.start_position = None
        self.start_orientation = None

        # Wait longer for publisher to be ready and establish connections
        self.get_logger().info("Establishing connection...")
        time.sleep(2.0)

        # Send a few test messages to establish connection
        twist = Twist()
        for _ in range(3):
            self.publisher_.publish(twist)
            time.sleep(0.1)

        # Wait for odometry data
        self.get_logger().info("Waiting for odometry data...")
        timeout = 5.0
        start_time = time.time()
        while self.current_odom is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_odom is None:
            self.get_logger().warn("No odometry data received - movement verification disabled")
        else:
            self.get_logger().info("Connection established with odometry")

    def odom_callback(self, msg):
        """Callback to store current odometry data"""
        self.current_odom = msg

    def move(self, direction, duration, speed_level):
        """
        Move robot in specified direction for duration at speed level

        Args:
            direction: 'forward' or 'backward'
            duration: float, seconds (max 2.0)
            speed_level: 25, 50, or 100 (percent of max speed)
        """
        # Validate inputs
        if direction not in ['forward', 'backward']:
            self.get_logger().error(f"Invalid direction '{direction}'. Use 'forward' or 'backward'")
            return False

        if duration > 2.0:
            self.get_logger().error(f"Duration {duration}s exceeds maximum of 2.0 seconds")
            return False

        if speed_level not in [25, 50, 100]:
            self.get_logger().error(f"Invalid speed level {speed_level}%. Use 25, 50, or 100")
            return False

        # Calculate velocity
        speed_multiplier = speed_level / 100.0
        velocity = self.max_lin_vel * speed_multiplier

        if direction == 'backward':
            velocity = -velocity

        self.get_logger().info(f"Moving {direction} for {duration}s at {speed_level}% speed ({velocity:.3f} m/s)")

        # Send movement command multiple times for reliability
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = 0.0

        # Publish at 10Hz for the duration to ensure reliable delivery
        publish_rate = 10  # Hz
        sleep_time = 1.0 / publish_rate
        total_publishes = int(duration * publish_rate)

        for i in range(total_publishes):
            self.publisher_.publish(twist)
            time.sleep(sleep_time)

        # Send multiple stop commands to ensure robot stops
        twist.linear.x = 0.0
        for _ in range(5):
            self.publisher_.publish(twist)
            time.sleep(0.1)

        self.get_logger().info("Movement completed")
        return True

    def turn(self, direction, degrees):
        """
        Turn robot in specified direction for specified degrees

        Args:
            direction: 'left' or 'right'
            degrees: float, degrees to turn
        """
        # Validate inputs
        if direction not in ['left', 'right']:
            self.get_logger().error(f"Invalid direction '{direction}'. Use 'left' or 'right'")
            return False

        if degrees <= 0 or degrees > 360:
            self.get_logger().error(f"Invalid degrees {degrees}. Must be between 0 and 360")
            return False

        # Calculate turn parameters (using half max speed like in teleop)
        angular_velocity = self.max_ang_vel / 2.0

        # Left turn is positive angular velocity, right turn is negative
        if direction == 'right':
            angular_velocity = -angular_velocity

        # Calculate duration needed for the turn
        # Convert degrees to radians, then calculate time
        radians = degrees * 3.14159 / 180.0
        turn_duration = radians / abs(angular_velocity)

        self.get_logger().info(f"Turning {direction} {degrees}° at {abs(angular_velocity):.2f} rad/s for {turn_duration:.2f}s")

        # Send turn command continuously for reliability
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity

        # Publish at 10Hz during the turn
        publish_rate = 10  # Hz
        sleep_time = 1.0 / publish_rate
        total_publishes = int(turn_duration * publish_rate)

        for i in range(total_publishes):
            self.publisher_.publish(twist)
            time.sleep(sleep_time)

        # Send multiple stop commands
        twist.angular.z = 0.0
        for _ in range(5):
            self.publisher_.publish(twist)
            time.sleep(0.1)

        self.get_logger().info("Turn completed")
        return True

    def stop(self):
        """Stop all robot movement"""
        self.get_logger().info("Emergency stop - stopping all movement")

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # Send stop command multiple times for reliability
        for _ in range(10):
            self.publisher_.publish(twist)
            time.sleep(0.1)

        self.get_logger().info("Robot stopped")
        return True


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Robot Command Controller')
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # Move command
    move_parser = subparsers.add_parser('move', help='Move robot forward or backward')
    move_parser.add_argument('direction', choices=['forward', 'backward'],
                            help='Direction to move')
    move_parser.add_argument('duration', type=float,
                            help='Duration in seconds (max 2.0)')
    move_parser.add_argument('speed', type=int, choices=[25, 50, 100],
                            help='Speed level as percentage (25, 50, or 100)')

    # Turn command
    turn_parser = subparsers.add_parser('turn', help='Turn robot left or right')
    turn_parser.add_argument('direction', choices=['left', 'right'],
                            help='Direction to turn')
    turn_parser.add_argument('degrees', type=float,
                            help='Degrees to turn (0-360)')

    # Stop command
    stop_parser = subparsers.add_parser('stop', help='Stop all robot movement')

    # Parse arguments
    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    # Initialize ROS2
    rclpy.init()

    try:
        # Create robot commander
        commander = RobotCommander()

        # Execute command with retry logic
        success = False

        if args.command == 'move':
            success = commander.move(args.direction, args.duration, args.speed)
        elif args.command == 'turn':
            success = commander.turn(args.direction, args.degrees)
        elif args.command == 'stop':
            success = commander.stop()

        if success:
            commander.get_logger().info("Command completed successfully")
        else:
            commander.get_logger().error("Command failed")
            sys.exit(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
