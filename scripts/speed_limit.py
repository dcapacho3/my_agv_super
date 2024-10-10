#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistLimiter(Node):
    def __init__(self):
        super().__init__('twist_limiter')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_out', 10)

    def listener_callback(self, msg):
        limited_msg = Twist()
        
        # Limit linear velocity
        limited_msg.linear.x = max(min(msg.linear.x, 0.4), -0.4)
        limited_msg.linear.y = max(min(msg.linear.y, 0.4), -0.4)
        limited_msg.linear.z = max(min(msg.linear.z, 0.4), -0.4)
        
        # Limit angular velocity
        limited_msg.angular.x = max(min(msg.angular.x, 0.7), -0.7)
        limited_msg.angular.y = max(min(msg.angular.y, 0.7), -0.7)
        limited_msg.angular.z = max(min(msg.angular.z, 0.7), -0.7)
        
        self.publisher.publish(limited_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_limiter = TwistLimiter()
    rclpy.spin(twist_limiter)
    twist_limiter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()