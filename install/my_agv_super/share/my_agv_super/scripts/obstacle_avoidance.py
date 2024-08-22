#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Suscripción al láser para detectar obstáculos
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)

        # Publicación de comandos de velocidad
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_avoidance',
            10)

        # Distancias de consideración para obstáculos
        self.max_obstacle_distance = 0.45  # Distancia máxima para considerar un obstáculo (en metros)
        self.min_obstacle_distance = 0.15  # Distancia mínima para considerar un obstáculo (en metros)
        self.consideration_distance = 0.45  # Distancia máxima para tomar en cuenta en el histograma (en metros)

        # Configuración del histograma de sectores
        self.num_sectors = 16  # Número total de sectores (ajustable)
        self.front_fraction = 0.8  # Porcentaje de la mitad frontal a considerar (80%)

        self.sector_angle = 2 * np.pi / self.num_sectors

        # Definir la zona frontal basada en el porcentaje de la mitad del círculo
        half_num_sectors = self.num_sectors // 2
        front_sector_count = int(self.front_fraction * half_num_sectors)
        self.front_sector_start = half_num_sectors - front_sector_count
        self.front_sector_end = half_num_sectors + front_sector_count

        # Velocidades máximas
        self.max_angular_speed = 0.9  # Ajusta según tus necesidades
        self.max_linear_speed = 0.2  # Ajusta según tus necesidades

        # Estado de publicación
        self.publishing_enabled = False
        self.last_publishing_enabled = False

        # Flag para activar/desactivar el desplazamiento lateral
        self.lateral_movement_enabled = False  # Desactivar desplazamiento lateral

    def laser_callback(self, msg):
        # Crear el histograma de sectores
        histogram = np.zeros(self.num_sectors)

        # Indexación de sectores laterales
        half_num_sectors = self.num_sectors // 2
        left_sector_indices = list(range(self.front_sector_end + 1, self.num_sectors))  # Sectores para la izquierda
        right_sector_indices = list(range(0, self.front_sector_start))  # Sectores para la derecha

        for i, range_value in enumerate(msg.ranges):
            if self.min_obstacle_distance < range_value < self.max_obstacle_distance:
                angle = msg.angle_min + i * msg.angle_increment
                sector = int((angle + np.pi) / self.sector_angle)
                if 0 <= sector < self.num_sectors:
                    if range_value <= self.consideration_distance:
                        histogram[sector] += (1.0 - (range_value / self.consideration_distance))

        # Evasión de obstáculos frontales
        front_density = sum(histogram[self.front_sector_start:self.front_sector_end + 1])
        left_density = sum(histogram[left_sector_indices])
        right_density = sum(histogram[right_sector_indices])

        if front_density > 0:
            self.publishing_enabled = True
            # Girar hacia el lado con menor densidad de obstáculos
            if left_density < right_density:
                angular_speed = self.max_angular_speed
            else:
                angular_speed = -self.max_angular_speed

            # Ajustar para asegurar que el giro sea suficientemente amplio
            angular_speed = min(self.max_angular_speed, abs(angular_speed))

            self.publish_cmd_vel(0.0, angular_speed)

        else:
            self.publishing_enabled = False

        if not self.publishing_enabled and self.last_publishing_enabled:
            # Detener la publicación si no hay obstáculos detectados
            self.publish_cmd_vel(0.0, 0.0)

        self.last_publishing_enabled = self.publishing_enabled

    def publish_cmd_vel(self, linear_speed, angular_speed):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: linear={linear_speed}, angular={angular_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

