#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class SimpleSquareMotion(Node):
    def __init__(self):
        super().__init__('simple_square_motion')
        
        # Publisher for velocity commands
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF2 setup
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters for square motion
        self.side_length = 1.0  # meters
        self.linear_speed = 0.1  # meters/second
        self.angular_speed = 0.4  # radians/second
        
        # Current position and orientation of the robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Wait for Gazebo to load completely
        self.get_logger().info('Waiting for Gazebo to fully load (10 seconds)...')
        time.sleep(10)  # Wait for 10 seconds
        
        # Start time
        self.start_time = self.get_clock().now()
        
        # Timer to publish velocity commands and TF
        self.timer = self.create_timer(0.1, self.motion_callback)
        
        self.get_logger().info('Simple square motion node started')
        
    def motion_callback(self):
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed_seconds = (current_time - self.start_time).nanoseconds / 1e9
        
        # Time calculations
        move_time = self.side_length / self.linear_speed  # Time to complete one side
        turn_time = (math.pi / 2) / self.angular_speed    # Time to turn 90 degrees
        cycle_time = move_time + turn_time                # Time for one move and turn
        square_time = 4 * cycle_time                      # Time to complete the square
        
        # Calculate current position in the cycle
        current_cycle = elapsed_seconds % square_time
        phase = int(current_cycle / cycle_time)           # Which side (0-3)
        phase_time = current_cycle % cycle_time           # Time within the current side+turn
        
        cmd_vel = Twist()
        
        # Determine if we're moving forward or turning
        if phase_time < move_time:
            # Moving forward
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = 0.0
            
            # Update position for TF2
            progress = phase_time / move_time  # How far along this side (0-1)
            
            # Calculate current position based on which side we're on
            if phase == 0:  # First side (moving along positive X)
                self.x = progress * self.side_length
                self.y = 0.0
                self.theta = 0.0
            elif phase == 1:  # Second side (moving along positive Y)
                self.x = self.side_length
                self.y = progress * self.side_length
                self.theta = math.pi / 2
            elif phase == 2:  # Third side (moving along negative X)
                self.x = self.side_length - progress * self.side_length
                self.y = self.side_length
                self.theta = math.pi
            elif phase == 3:  # Fourth side (moving along negative Y)
                self.x = 0.0
                self.y = self.side_length - progress * self.side_length
                self.theta = 3 * math.pi / 2
        else:
            # Turning
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.angular_speed
            
            # Calculate rotation progress
            turn_progress = (phase_time - move_time) / turn_time
            turn_angle = turn_progress * (math.pi / 2)
            
            # Update orientation for TF2 based on which turn we're making
            if phase == 0:
                self.theta = turn_angle
                self.x = self.side_length
                self.y = 0.0
            elif phase == 1:
                self.theta = (math.pi / 2) + turn_angle
                self.x = self.side_length
                self.y = self.side_length
            elif phase == 2:
                self.theta = math.pi + turn_angle
                self.x = 0.0
                self.y = self.side_length
            elif phase == 3:
                self.theta = (3 * math.pi / 2) + turn_angle
                self.x = 0.0
                self.y = 0.0
        
        # Publish velocity command
        self.vel_publisher.publish(cmd_vel)
        
        # Publish TF2 transformation
        self.publish_tf()
        
    def publish_tf(self):
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        
        # Child frame name
        t.child_frame_id = 'base_link'
        
        # Position
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Orientation (conversion from Euler angle to quaternion - simplified for Z-axis rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        # Publish transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSquareMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()