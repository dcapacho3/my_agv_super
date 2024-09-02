#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SectorDetectionNode(Node):

    def __init__(self):
        super().__init__('sector_detection_node')

        # Suscripción al LIDAR
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)

        # Distancias de consideración para obstáculos
        self.max_obstacle_distance = 0.45  # Distancia máxima para considerar un obstáculo (en metros)
        self.min_obstacle_distance = 0.15  # Distancia mínima para considerar un obstáculo (en metros)

        # Configuración de sectores
        self.center_range = (-0.8, 0.8)  # Rango de ángulo para considerar el sector central (ajustable)

    def laser_callback(self, msg):
        center_obstacle_detected = False
        left_obstacle_detected = False
        right_obstacle_detected = False

        for i, range_value in enumerate(msg.ranges):
            if self.min_obstacle_distance < range_value < self.max_obstacle_distance:
                angle = msg.angle_min + i * msg.angle_increment

                if self.center_range[0] <= angle <= self.center_range[1]:
                    center_obstacle_detected = True
                elif angle < self.center_range[0]:
                    left_obstacle_detected = True
                elif angle > self.center_range[1]:
                    right_obstacle_detected = True

        # Imprimir en qué sector se encuentra el obstáculo
        if center_obstacle_detected:
            self.get_logger().info('Obstacle detected in the center sector.')
        elif left_obstacle_detected:
            self.get_logger().info('Obstacle detected in the left sector.')
        elif right_obstacle_detected:
            self.get_logger().info('Obstacle detected in the right sector.')
        else:
            self.get_logger().info('No obstacle detected within the specified range.')

def main(args=None):
    rclpy.init(args=args)
    node = SectorDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

