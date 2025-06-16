#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import sys
import termios
import tty
import select
import time
import numpy as np
import threading

class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        self.publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        self.arming = False
        self.offboard_mode = False
        
        # Current position and yaw
        self.position = np.array([0.0, 0.0, 2.0])  # x, y, z
        self.yaw = 0.0
        
        # Movement step sizes
        self.position_step = 0.5  # meters
        self.yaw_step = np.pi/4   # 45 degrees

        # Create timer for continuous publishing
        self.timer = self.create_timer(0.1, self.publish_continuous)  # 10Hz
        
        self.get_logger().info('Drone Teleop Node Started')
        self.get_logger().info('Press "a" to arm/disarm')
        self.get_logger().info('Press "o" to enable/disable offboard mode')
        self.get_logger().info('Press "t" to takeoff')
        self.get_logger().info('Press "l" to land')
        self.get_logger().info('Movement controls:')
        self.get_logger().info('  w/s: move forward/backward')
        self.get_logger().info('  a/d: move left/right')
        self.get_logger().info('  q/e: rotate left/right')
        self.get_logger().info('  r/f: move up/down')
        self.get_logger().info('Press "x" to quit')

    def publish_continuous(self):
        if self.offboard_mode:
            self.send_offboard_mode(position=True)
            self.send_trajectory_setpoint()

    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command} with params: {param1}, {param2}, {param3}, {param4}, {param5}, {param6}, {param7}')

    def send_offboard_mode(self, position=True, velocity=False, acceleration=False, attitude=False, body_rate=False):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.attitude = attitude
        msg.body_rate = body_rate
        self.offboard_publisher.publish(msg)

    def send_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = self.position.tolist()
        msg.yaw = self.yaw
        self.trajectory_publisher.publish(msg)

    def arm_disarm(self, arm):
        # First disarm if we're arming
        if arm:
            self.send_command(400, 0.0)  # Disarm first
            time.sleep(0.1)
        
        # Then send the actual command
        self.send_command(400, 1.0 if arm else 0.0)
        self.arming = arm
        self.get_logger().info('Arming' if arm else 'Disarming')
        
        # If arming, wait a bit and then enable offboard mode
        if arm:
            time.sleep(1.0)
            self.enable_offboard(True)

    def enable_offboard(self, enable):
        self.offboard_mode = enable
        self.send_command(176, 1.0 if enable else 0.0)
        self.get_logger().info('Offboard mode ' + ('enabled' if enable else 'disabled'))

    def move_forward(self):
        self.position[0] += self.position_step * np.cos(self.yaw)
        self.position[1] += self.position_step * np.sin(self.yaw)
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Moving forward. Position: {self.position}')

    def move_backward(self):
        self.position[0] -= self.position_step * np.cos(self.yaw)
        self.position[1] -= self.position_step * np.sin(self.yaw)
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Moving backward. Position: {self.position}')

    def move_left(self):
        self.position[0] += self.position_step * np.sin(self.yaw)
        self.position[1] -= self.position_step * np.cos(self.yaw)
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Moving left. Position: {self.position}')

    def move_right(self):
        self.position[0] -= self.position_step * np.sin(self.yaw)
        self.position[1] += self.position_step * np.cos(self.yaw)
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Moving right. Position: {self.position}')

    def rotate_left(self):
        self.yaw += self.yaw_step
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Rotating left. Yaw: {np.degrees(self.yaw)}°')

    def rotate_right(self):
        self.yaw -= self.yaw_step
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Rotating right. Yaw: {np.degrees(self.yaw)}°')

    def move_up(self):
        self.position[2] += self.position_step
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Moving up. Position: {self.position}')

    def move_down(self):
        self.position[2] -= self.position_step
        self.send_trajectory_setpoint()
        self.get_logger().info(f'Moving down. Position: {self.position}')

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key

def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()
    
    try:
        while True:
            key = get_key()
            if key == 'a':  # Arm/Disarm
                node.arm_disarm(not node.arming)
            elif key == 'o':  # Toggle Offboard Mode
                node.enable_offboard(not node.offboard_mode)
            elif key == 't':  # Takeoff
                if not node.arming:
                    node.get_logger().info('Please arm the drone first!')
                    continue
                if not node.offboard_mode:
                    node.get_logger().info('Please enable offboard mode first!')
                    continue
                node.send_command(22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0)
                node.get_logger().info('Takeoff command sent')
                time.sleep(0.1)
            elif key == 'l':  # Land
                node.send_command(21)
                node.get_logger().info('Land command sent')
                time.sleep(0.1)
            elif key == 'w':  # Move forward
                node.move_forward()
            elif key == 's':  # Move backward
                node.move_backward()
            elif key == 'a':  # Move left
                node.move_left()
            elif key == 'd':  # Move right
                node.move_right()
            elif key == 'q':  # Rotate left
                node.rotate_left()
            elif key == 'e':  # Rotate right
                node.rotate_right()
            elif key == 'r':  # Move up
                node.move_up()
            elif key == 'f':  # Move down
                node.move_down()
            elif key == 'x':  # Quit
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 