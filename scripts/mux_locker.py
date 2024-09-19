#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import ast

class TwistMuxController(Node):
    def __init__(self):
        super().__init__('twist_mux_controller')
        self.subscription = self.create_subscription(
            String,
            'message_py',
            self.listener_callback,
            10)
        self.lock_all_publisher = self.create_publisher(Bool, 'lock_all', 10)
        self.lock_navigation_publisher = self.create_publisher(Bool, 'lock_navigation', 10)
        self.cmd_vel_block_all_publisher = self.create_publisher(Twist, 'cmd_vel_block_all', 10)
        self.cmd_vel_block_navigation_publisher = self.create_publisher(Twist, 'cmd_vel_block_navigation', 10)
        
        self.previous_lock_all = False
        self.previous_lock_navigation = False
        

    def listener_callback(self, msg):
        # Parse the string to get the array of numbers
        data = ast.literal_eval(msg.data)
        
        if len(data) != 2:
            self.get_logger().warn('Received data does not contain exactly 2 numbers')
            return

        first_number, second_number = data

        # Check conditions and publish accordingly
        current_lock_all = first_number > 300
        current_lock_navigation = not (100 <= second_number <= 2000)

        # Solo publica si el estado de lock_all ha cambiado
        if current_lock_all != self.previous_lock_all:
            self.publish_lock_all(current_lock_all)
            self.previous_lock_all = current_lock_all

            if current_lock_all:
                self.publish_zero_velocity('all')

        # Solo publica si el estado de lock_navigation ha cambiado
        if current_lock_navigation != self.previous_lock_navigation:
            self.publish_lock_navigation(current_lock_navigation)
            self.previous_lock_navigation = current_lock_navigation

            if current_lock_navigation:
                self.publish_zero_velocity('navigation')

    def publish_lock_all(self, lock):
        msg = Bool()
        msg.data = lock
        self.lock_all_publisher.publish(msg)
        self.get_logger().info(f'Published lock_all: {lock}')

    def publish_lock_navigation(self, lock):
        msg = Bool()
        msg.data = lock
        self.lock_navigation_publisher.publish(msg)
        self.get_logger().info(f'Published lock_navigation: {lock}')

    def publish_zero_velocity(self, lock_type):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.005
        self.publish_enabled = False 
        if lock_type == 'all':
            self.cmd_vel_block_all_publisher.publish(twist)
        elif lock_type == 'navigation':
            self.cmd_vel_block_navigation_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    twist_mux_controller = TwistMuxController()
    rclpy.spin(twist_mux_controller)
    twist_mux_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
