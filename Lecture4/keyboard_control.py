#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import threading

msg = """
YOLO robot keyboard control
---------------------------
Movement keys:
   w    
a  s  d

w/s : increase/decrease linear speed
a/d : increase/decrease angular speed

space : stop
q : exit
"""

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, '/yolobot/cmd_vel', 10)
        self.linear_speed = 1  # m/s
        self.angular_speed = 1  # rad/s
        self.get_logger().info(msg)
        
    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
        self.get_logger().info(f'Publishing velocity - linear: {linear}, angular: {angular}')

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = KeyboardController()
    
    # Create a thread for the ROS spinner
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
    spin_thread.daemon = True
    spin_thread.start()
    
    linear_vel = 0.0
    angular_vel = 0.0
    
    try:
        while True:
            key = get_key(settings)
            
            if key == 'w':
                linear_vel = node.linear_speed
                angular_vel = 0.0
            elif key == 's':
                linear_vel = -node.linear_speed
                angular_vel = 0.0
            elif key == 'a':
                angular_vel = node.angular_speed
                linear_vel = 0.0
            elif key == 'd':
                angular_vel = -node.angular_speed
                linear_vel = 0.0
            elif key == ' ':
                linear_vel = 0.0
                angular_vel = 0.0
            elif key == 'q':
                linear_vel = 0.0
                angular_vel = 0.0
                node.publish_velocity(linear_vel, angular_vel)
                break
            
            node.publish_velocity(linear_vel, angular_vel)
            
    except Exception as e:
        print(e)
    
    finally:
        # Ensure the robot stops when exiting
        twist = Twist()
        node.publisher.publish(twist)
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
