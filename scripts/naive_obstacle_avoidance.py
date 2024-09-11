#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import sleep

class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscription to laser for obstacle detection
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        # Subscription to cmd_vel_out to monitor movement commands
        self.cmd_vel_out_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_out_callback,
            10)
        
        # Publisher for avoidance commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_avoidance',
            10)

        self.last_cmd_vel_out_msg = Twist()
        self.last_cmd_vel_avoidance_msg = Twist()

        # Variable to control the publishing state
        self.publishing_enabled = False
        self.last_publishing_enabled = False

        # Variable to check if avoidance should be activated
        self.should_activate_avoidance = False

        # Maximum and minimum distances to consider obstacles (in meters)
        self.max_obstacle_distance = 0.25
        self.min_obstacle_distance = 0.15

        # Maximum allowed lateral speed
        self.max_lateral_speed = 0.2

        # Angular speed for final turn
        self.angular_speed = 0.2

        # Angle to rotate at the end in radians (15 degrees)
        self.angle_to_rotate = 15 * 3.14159 / 180

        # Store the last lateral displacement direction
        self.last_lateral_direction = 0.0

    def cmd_vel_out_callback(self, msg):
        # Check if the current message is different from the last avoidance message
        if (msg.linear.x != self.last_cmd_vel_avoidance_msg.linear.x or
            msg.linear.y != self.last_cmd_vel_avoidance_msg.linear.y or
            msg.angular.z != self.last_cmd_vel_avoidance_msg.angular.z):
            
            # Check if there's movement (velocity is not zero)
            if (msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.angular.z != 0.0):
                self.should_activate_avoidance = True
            else:
                self.should_activate_avoidance = False
        else:
            self.should_activate_avoidance = False

        # Update the last stored message from cmd_vel_out
        self.last_cmd_vel_out_msg = msg

    def laser_callback(self, msg):
        if not self.should_activate_avoidance:
            self.publishing_enabled = False
            return

        # Logic to detect obstacles and determine evasion direction
        closest_obstacle_distance = float('inf')
        obstacle_angle = 0.0

        for i, range in enumerate(msg.ranges):
            if range < msg.range_max and self.min_obstacle_distance < range < self.max_obstacle_distance:
                if range < closest_obstacle_distance:
                    closest_obstacle_distance = range
                    obstacle_angle = msg.angle_min + i * msg.angle_increment

        if closest_obstacle_distance < self.max_obstacle_distance:
            self.publishing_enabled = True
            self.get_logger().info(f'Obstacle detected at {closest_obstacle_distance} meters.')

            # Calculate lateral speed based on obstacle distance
            lateral_speed = min(self.max_lateral_speed, (self.max_obstacle_distance - closest_obstacle_distance) / self.max_obstacle_distance * self.max_lateral_speed + 0.3)

            # Adjust lateral displacement direction based on obstacle angle
            if obstacle_angle < 0:
                lateral_speed = -lateral_speed  # Move to the right
            else:
                lateral_speed = lateral_speed  # Move to the left

            # Save the last lateral displacement direction
            self.last_lateral_direction = lateral_speed

            # Publish velocity command
            self.publish_cmd_vel(0.0, lateral_speed)
        else:
            self.publishing_enabled = False
            self.get_logger().info('No obstacles within the maximum distance.')

            # Stop publishing if no obstacles are detected
            if self.last_publishing_enabled:
                self.publish_cmd_vel(0.0, 0.0)

                # Rotate on its axis 15 degrees in the opposite direction to the last lateral movement
                if self.last_lateral_direction > 0:
                    self.rotate_in_place(self.angular_speed)
                else:
                    self.rotate_in_place(-self.angular_speed)

        self.last_publishing_enabled = self.publishing_enabled

    def publish_cmd_vel(self, linear_speed, lateral_speed):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.linear.y = lateral_speed  # Lateral movement
        cmd_vel_msg.angular.z = 0.0  # No rotation
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.last_cmd_vel_avoidance_msg = cmd_vel_msg
        self.get_logger().info(f'Published cmd_vel: linear.x={linear_speed}, linear.y={lateral_speed}')

    def rotate_in_place(self, angular_speed):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = angular_speed
        duration = abs(self.angle_to_rotate / angular_speed)
        self.get_logger().info(f'Rotating in place: angular.z={angular_speed} for duration={duration}')
        
        # Publish rotation command
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        while self.get_clock().now() < end_time:
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            sleep(0.1)

        # Stop rotation
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.last_cmd_vel_avoidance_msg = cmd_vel_msg
        self.get_logger().info('Rotation complete.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
