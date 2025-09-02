#!/usr/bin/env python3
#
# Copyright 2023-2024 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import select
import sys
import rclpy
import re
# GAMEPAD ADDITION - Add these imports
import threading
import struct
import time
import fcntl
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import Twist
from kaiaai import config


if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


class TeleopKeyboardNode(Node):
    def __init__(self, start_parameter_services=False):
        super().__init__(
            'teleop_keyboard_node',
            start_parameter_services=start_parameter_services
        )
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_lin_vel', 0.22),
                ('max_ang_vel', 12.84),
                ('lin_vel_step', 0.05),
                ('ang_vel_step', 0.1),
                ('lin_vel_step_large', 0.25),
                ('ang_vel_step_large', 0.5),
            ])
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.lin_vel_step = self.get_parameter('lin_vel_step').value
        self.ang_vel_step = self.get_parameter('ang_vel_step').value
        self.lin_vel_step_large = self.get_parameter('lin_vel_step_large').value
        self.ang_vel_step_large = self.get_parameter('ang_vel_step_large').value

        print('Max linear velocity {:.3f}\t Max angular velocity {:.3f}'.format(
            round(self.max_lin_vel, 3),
            round(self.max_ang_vel, 3))
        )

        self.tty_attr = None if os.name == 'nt' else termios.tcgetattr(sys.stdin)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # GAMEPAD ADDITION - Initialize gamepad
        self.gamepad_device = None
        self.gamepad_thread = None
        self.gamepad_running = False
        self.last_button_time = {}
        self.turn_in_progress = False  # Prevent overlapping turns

        # Dashboard state
        self.last_action = "Ready"
        self.control_mode = "Keyboard"
        self.dashboard_shown = False

        self._init_gamepad()

        # Show simple startup message
        print('\n' + '='*60)
        print('              ROBOT TELEOP READY')
        print('='*60)
        print('Use keyboard (w/a/s/d/x) or gamepad controls')
        print('CTRL-C to quit')
        print('='*60 + '\n')

    # GAMEPAD ADDITION - Simple status display
    def _show_status(self, action_msg):
        """Show simple status line with action and velocities"""
        print(f"\r{action_msg} | Linear: {self.linear_velocity:+.3f} m/s | Angular: {self.angular_velocity:+.3f} rad/s    ", end='', flush=True)

    # GAMEPAD ADDITION - Add gamepad initialization
    def _init_gamepad(self):
        """Initialize F710 gamepad if available"""
        try:
            if os.path.exists('/dev/input/js0'):
                self.gamepad_device = open('/dev/input/js0', 'rb')
                self.gamepad_running = True
                self.gamepad_thread = threading.Thread(target=self._gamepad_loop)
                self.gamepad_thread.daemon = True
                self.gamepad_thread.start()
                # Dashboard will be initialized after main init
        except:
            pass

    # GAMEPAD ADDITION - Add gamepad input loop
    def _gamepad_loop(self):
        """Read F710 gamepad input - D-pad and analog sticks"""
        if not self.gamepad_device:
            return
        try:
            # Set non-blocking mode
            fcntl.fcntl(self.gamepad_device.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
            # Dashboard will be shown after main initialization

            while self.gamepad_running:
                try:
                    data = self.gamepad_device.read(8)
                    if not data or len(data) != 8:
                        time.sleep(0.01)
                        continue

                    _, value, event_type, number = struct.unpack('IhBB', data)

                    # Handle axis events (D-pad and analog sticks)
                    if event_type & 0x02:  # Axis event

                        # Handle D-pad as axes (discrete movements)
                        if number == 4:  # D-pad X axis
                            if value < -16000:  # D-pad left
                                self._gamepad_key_action('a')
                            elif value > 16000:  # D-pad right
                                self._gamepad_key_action('d')
                        elif number == 5:  # D-pad Y axis
                            if value < -16000:  # D-pad up
                                self._gamepad_key_action('w')
                            elif value > 16000:  # D-pad down
                                self._gamepad_key_action('x')

                        # Handle left analog stick (axes 0 and 1) - use simple line updates
                        elif number == 0:  # Left stick X axis (turn)
                            # Apply deadzone
                            if abs(value) > 3000:
                                # Normalize to -1.0 to 1.0, invert X axis (negative=right turn)
                                norm_value = -value / 32767.0
                                self.angular_velocity = self.check_angular_limit_velocity(norm_value * self.max_ang_vel)
                                self._show_status("Analog Turn")
                            else:
                                self.angular_velocity = 0.0
                            # Always publish for smooth control
                            self._publish_twist()

                        elif number == 1:  # Left stick Y axis (forward/backward)
                            # Apply deadzone
                            if abs(value) > 3000:
                                # Normalize to -1.0 to 1.0, invert Y axis (negative=forward)
                                norm_value = -value / 32767.0
                                self.linear_velocity = self.check_linear_limit_velocity(norm_value * self.max_lin_vel)
                                self._show_status("Analog Move")
                            else:
                                self.linear_velocity = 0.0
                            # Always publish for smooth control
                            self._publish_twist()

                    # Handle button presses (no debug output)
                    if event_type & 0x01 and value:  # Button press only
                        current_time = time.time()

                        # Debounce buttons (prevent rapid repeats)
                        if number in self.last_button_time:
                            if current_time - self.last_button_time[number] < 0.3:
                                continue

                        self.last_button_time[number] = current_time

                        # Button mappings - using your preferred layout
                        if number == 0:    # X button -> 'a' (turn left)
                            self._gamepad_key_action('a')
                        elif number == 1:  # A button -> 'x' (backward)
                            self._gamepad_key_action('x')
                        elif number == 2:  # B button -> 'd' (turn right)
                            self._gamepad_key_action('d')
                        elif number == 3:  # Y button -> 'w' (forward)
                            self._gamepad_key_action('w')
                        elif number == 5:   # Right bumper -> stop
                            self._gamepad_key_action(' ')
                        elif number == 6:   # Left trigger -> 90 degree right turn
                            if not self.turn_in_progress:
                                self._show_status("Left Trigger (90° right turn)")
                                self._execute_turn(90)
                            else:
                                self._show_status("Left Trigger ignored (turn in progress)")
                        elif number == 7:   # Right trigger -> 90 degree left turn
                            if not self.turn_in_progress:
                                self._show_status("Right Trigger (90° left turn)")
                                self._execute_turn(-90)
                            else:
                                self._show_status("Right Trigger ignored (turn in progress)")
                        # Keep the old D-pad numbers just in case
                        elif number == 11:    # D-pad Up -> 'w'
                            self._gamepad_key_action('w')
                        elif number == 12:  # D-pad Down -> 'x'
                            self._gamepad_key_action('x')
                        elif number == 13:  # D-pad Left -> 'a'
                            self._gamepad_key_action('a')
                        elif number == 14:  # D-pad Right -> 'd'
                            self._gamepad_key_action('d')

                    # Handle button releases (no action needed for now)
                    elif event_type & 0x01 and not value:  # Button release
                        pass  # Just acknowledge releases

                except (BlockingIOError, OSError):
                    # No data available, sleep briefly
                    time.sleep(0.01)
                    continue

        except Exception as e:
            # Silent error handling - don't print to avoid dashboard interference
            pass

    # GAMEPAD ADDITION - Execute turn functions
    def _execute_turn(self, degrees):
        """Execute a turn at half speed for specified degrees"""
        if self.turn_in_progress:
            print("\nTurn already in progress - ignoring new turn request")
            return

        # Set turn flag to prevent overlapping turns
        self.turn_in_progress = True

        # Calculate turn duration based on degrees and half max angular velocity
        half_angular_vel = self.max_ang_vel / 2.0
        turn_duration = abs(degrees) / (half_angular_vel * 57.2958)  # Convert rad/s to deg/s

        direction = 1 if degrees > 0 else -1

        self.last_action = f"Executing {degrees}° turn..."
        self._show_status(f"Executing {degrees}° turn...")

        def turn_sequence():
            # Save current velocities
            saved_linear = self.linear_velocity
            saved_angular = self.angular_velocity

            # Execute turn
            self.linear_velocity = 0.0
            self.angular_velocity = direction * half_angular_vel
            self._publish_twist()

            # Turn for calculated duration
            time.sleep(turn_duration)

            # Stop turning and restore previous velocities
            self.linear_velocity = saved_linear
            self.angular_velocity = saved_angular
            self._publish_twist()

            # Clear turn flag
            self.turn_in_progress = False
            self.last_action = f"{degrees}° turn completed"
            self._show_status(f"{degrees}° turn completed")

        # Execute turn in separate thread so it doesn't block
        turn_thread = threading.Thread(target=turn_sequence)
        turn_thread.daemon = True
        turn_thread.start()

    # GAMEPAD ADDITION - Helper method to publish twist messages
    def _publish_twist(self):
        """Publish current velocity values immediately"""
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_velocity
        self.publisher_.publish(twist)

    # GAMEPAD ADDITION - Handle gamepad button as keyboard key
    def _gamepad_key_action(self, key_equiv):
        """Process gamepad input as if it were a keyboard key"""
        if key_equiv == 'w':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity + self.lin_vel_step)
            self._show_status("Forward step")
        elif key_equiv == 'x':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity - self.lin_vel_step)
            self._show_status("Backward step")
        elif key_equiv == 'a':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity + self.ang_vel_step)
            self._show_status("Left turn step")
        elif key_equiv == 'd':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity - self.ang_vel_step)
            self._show_status("Right turn step")
        elif key_equiv == ' ':  # Stop
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self._show_status("Emergency stop")

        # IMPORTANT: Publish the twist message immediately
        self._publish_twist()

    def get_key(self):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [])

        key = sys.stdin.read(1) if rlist else ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.tty_attr)
        return key

    def print_vels(self):
        print('\nLinear velocity {:.3f}\tAngular velocity {:.3f}'.format(
            round(self.linear_velocity, 3),
            round(self.angular_velocity, 3))
        )

    @staticmethod
    def constrain(input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

    def check_linear_limit_velocity(self, velocity):
        return self.constrain(velocity, -self.max_lin_vel, self.max_lin_vel)

    def check_angular_limit_velocity(self, velocity):
        return self.constrain(velocity, -self.max_ang_vel, self.max_ang_vel)

    def perform(self):
        key = self.get_key()
        should_publish = False

        if key == 'w':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity + self.lin_vel_step)
            self._show_status("w key (forward)")
            should_publish = True
        elif key == 'W':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity + self.lin_vel_step_large)
            self.print_vels()
            should_publish = True
        elif key == 'x':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity - self.lin_vel_step)
            self.print_vels()
            should_publish = True
        elif key == 'X':
            self.linear_velocity = \
                self.check_linear_limit_velocity(self.linear_velocity - self.lin_vel_step_large)
            self.print_vels()
            should_publish = True
        elif key == 'a':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity + self.ang_vel_step)
            self.print_vels()
            should_publish = True
        elif key == 'A':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity + self.ang_vel_step_large)
            self.print_vels()
            should_publish = True
        elif key == 'd':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity - self.ang_vel_step)
            self.print_vels()
            should_publish = True
        elif key == 'D':
            self.angular_velocity = \
                self.check_angular_limit_velocity(self.angular_velocity - self.ang_vel_step_large)
            self.print_vels()
            should_publish = True
        elif key == 's' or key == 'S':
            self.angular_velocity = 0.0
            self.print_vels()
            should_publish = True
        elif key == ' ':
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.print_vels()
            should_publish = True
        elif (key == '\x03'):
            # Clean exit with final dashboard update
            self.last_action = "CTRL-C pressed - shutting down"
            self.control_mode = "Shutdown"
            self._update_dashboard()

            # GAMEPAD ADDITION - Stop gamepad thread
            self.gamepad_running = False

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.publisher_.publish(twist)

            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.tty_attr)

            # GAMEPAD ADDITION - Cleanup gamepad
            if self.gamepad_device:
                self.gamepad_device.close()

            return False

        # Always publish twist message when there's any key input or periodically
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_velocity
        self.publisher_.publish(twist)

        return True


def main(args=None):
    if (len(sys.argv) == 4 and sys.argv[1] == '--ros-args'):
        yaml_path_name = sys.argv[3]
    else:
        # Hack around ros2 launch failure
        if (len(sys.argv) == 2 and sys.argv[1].startswith('robot_model:=')):
            description = sys.argv[1][13:]
        else:
            description = config.get_var('robot.model')

        yaml_path_name = os.path.join(
            get_package_share_path(description),
            'config',
            'teleop_keyboard.yaml'
            )

        args = [
            '--ros-args',
            '--params-file',
            yaml_path_name
        ]

    print('YAML file name : {}'.format(yaml_path_name))

    rclpy.init(args=args)
    node = TeleopKeyboardNode(start_parameter_services=False)

    while(node.perform()):
        rclpy.spin_once(node, timeout_sec=0.001)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
