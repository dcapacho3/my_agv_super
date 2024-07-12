#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Suscripción al láser para detectar obstáculos
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.laser_subscription  # Evitar que se destruya prematuramente

        # Publicación de comandos de velocidad
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_avoidance',
            10)

        # Variable para controlar el estado de publicación
        self.publishing_enabled = False
        self.last_publishing_enabled = False

        # Distancia máxima a la que se consideran obstáculos (en metros)
        self.max_obstacle_distance = 0.6  # Ajusta según tus necesidades

    def laser_callback(self, msg):
        # Lógica para detectar obstáculos y determinar dirección de evasión
        closest_obstacle_distance = float('inf')
        obstacle_angle = 0.0
        for i, range in enumerate(msg.ranges):
            if range < msg.range_max and range < self.max_obstacle_distance:
                if range < closest_obstacle_distance:
                    closest_obstacle_distance = range
                    obstacle_angle = msg.angle_min + i * msg.angle_increment

        if closest_obstacle_distance < self.max_obstacle_distance:
            self.publishing_enabled = True
            self.get_logger().info(f'Obstacle detected at {closest_obstacle_distance} meters.')

            # Calcular dirección de giro basado en el ángulo del obstáculo
            if obstacle_angle < 0:
                angular_speed = 0.5   # Girar a la derecha
            else:
                angular_speed = -0.5  # Girar a la izquierda

            # Publicar comando de velocidad
            self.publish_cmd_vel(0.2, angular_speed)

        else:
            self.publishing_enabled = False
            self.get_logger().info('No obstacles within the maximum distance.')

            # Detener la publicación si no hay obstáculos detectados
            if self.last_publishing_enabled:
                self.publish_cmd_vel(0.0, 0.0)

        self.last_publishing_enabled = self.publishing_enabled

    def publish_cmd_vel(self, linear_speed, angular_speed):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
