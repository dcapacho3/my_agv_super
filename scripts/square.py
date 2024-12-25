#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import time

class SquareMotionController(Node):
    def __init__(self):
        super().__init__('square_motion_controller')
        
        # Create publisher
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_key', qos)
        
        # Movement parameters (can be modified)
        self.linear_speed = 0.2   # Linear speed in m/s
        self.angular_speed = 0.2   # Angular speed in rad/s
        self.side_duration = 3   # Time to move for each side in seconds
        self.total_laps = 5    # Number of times to repeat the square pattern
        self.direction_pause = 0.5  # Pause duration between direction changes in seconds
        
        # Initialize twist message
        self.twist = Twist()
        
        # Start the square motion
        self.get_logger().info('Starting square motion pattern...')
        self.get_logger().info(f'Will execute {self.total_laps} laps')
        self.execute_square_motion()

    def stop_robot(self):
        """Stop all robot movement"""
        self.twist = Twist()
        self.publisher.publish(self.twist)
        time.sleep(self.direction_pause)  # Pause between movements to protect motors

    def move_forward(self):
        """Move robot forward"""
        self.stop_robot()  # Ensure complete stop before changing direction
        self.twist = Twist()
        self.twist.linear.x = self.linear_speed
        self.publisher.publish(self.twist)
        time.sleep(self.side_duration)

    def move_right(self):
        """Move robot to the right"""
        self.stop_robot()  # Ensure complete stop before changing direction
        self.twist = Twist()
        self.twist.linear.y = -self.linear_speed
        self.publisher.publish(self.twist)
        time.sleep(self.side_duration)

    def move_backward(self):
        """Move robot backward"""
        self.stop_robot()  # Ensure complete stop before changing direction
        self.twist = Twist()
        self.twist.linear.x = -self.linear_speed
        self.publisher.publish(self.twist)
        time.sleep(self.side_duration)

    def move_left(self):
        """Move robot to the left"""
        self.stop_robot()  # Ensure complete stop before changing direction
        self.twist = Twist()
        self.twist.linear.y = self.linear_speed
        self.publisher.publish(self.twist)
        time.sleep(self.side_duration)

    def execute_square_motion(self):
        """Execute the square motion pattern for the specified number of laps"""
        try:
            for lap in range(self.total_laps):
                self.get_logger().info(f'Starting lap {lap + 1} of {self.total_laps}')
                
                # Execute one complete square
                self.get_logger().info('Moving forward...')
                self.move_forward()

                self.get_logger().info('Moving right...')
                self.move_right()

                self.get_logger().info('Moving backward...')
                self.move_backward()

                self.get_logger().info('Moving left...')
                self.move_left()

                self.get_logger().info(f'Lap {lap + 1} completed!')
                self.stop_robot()  # Ensure complete stop at the end of each lap

            self.get_logger().info('All laps completed successfully!')

        except KeyboardInterrupt:
            self.get_logger().info('Motion interrupted by user')
        finally:
            self.stop_robot()
            self.get_logger().info('Motion stopped')

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the node
    square_motion_controller = SquareMotionController()
    
    try:
        rclpy.spin(square_motion_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        square_motion_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
