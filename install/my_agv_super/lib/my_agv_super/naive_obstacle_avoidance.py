#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf_transformations
import numpy as np

class GoToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal')

        print('funcionito')
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            QoSProfile(depth=10))

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSProfile(depth=10))

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10))

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.scan_ranges = []
        self.leng = 0

        self.odom_received = False
        self.moving_to_goal = False
        self.obstacle_detected = False
        self.stop_robot()

        self.timer = self.create_timer(0.1, self.move)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(orientation_q)
        self.odom_received = True

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.obstacle_detected = False
        self.leng = len(self.scan_ranges)
        for i in range(self.leng):
            if 0 < self.scan_ranges[i] < 0.4:
                self.obstacle_detected = True
                
                break

    def move(self):
        if not self.odom_received:
            self.stop_robot()
            self.get_logger().info('Waiting for Odom data...')
            return

        if not self.scan_ranges:
            self.stop_robot()
            self.get_logger().info('Waiting for scan...')
            return

        if self.obstacle_detected:
            self.move_angle()
        else:
            self.stop_robot()
            self.moving_to_goal = False

    def move_angle(self):
        obstacle_detected = False
        xi_left = 0.0
        yi_left = 0.0
        xi_right = 0.0
        yi_right = 0.0
        angular_velocity = 0.0
        
        for i in range(int(self.leng/4)):
            if 0 < self.scan_ranges[i] < 0.4:
                obstacle_detected = True
                xi_left += self.scan_ranges[i] * np.cos((2*np.pi*i)/self.leng)
                yi_left += self.scan_ranges[i] * np.sin((2*np.pi*i)/self.leng)

        theta_left = np.arctan2(yi_left,xi_left)

        for i in range(int(self.leng*3/4), self.leng):
            if 0 < self.scan_ranges[i] < 0.4:
                obstacle_detected = True
                xi_right += self.scan_ranges[i] * np.cos((2*np.pi*i)/self.leng)
                yi_right += self.scan_ranges[i] * np.sin((2*np.pi*i)/self.leng)            

        theta_right = np.arctan2(yi_right,xi_right)

        if obstacle_detected:
            theta = (theta_left + theta_right)/2
            if theta > 0:
                target_angle = theta - np.pi / 3
            elif theta < 0:    
                target_angle = theta + np.pi / 3  # Girar lejos del obstáculo
            else:
                target_angle = np.pi/2  # Valor predeterminado si no se detectan obstáculos

            angle_error = target_angle - self.current_orientation
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            if abs(angle_error) > 0.08:
                angular_velocity = 0.4 if angle_error > 0 else -0.2
            elif abs(angle_error) > 0.05:
                angular_velocity = 0.16 if angle_error > 0 else -0.08
            else:
                angular_velocity = 0.0
                self.moving_to_goal = False

        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def quaternion_to_euler(self, orientation_q):
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

