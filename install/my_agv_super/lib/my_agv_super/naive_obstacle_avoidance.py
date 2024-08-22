#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import sleep

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
        self.cmd_vel_out_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_out_callback,
            10)
        self.cmd_vel_out_subscription  # Evitar que se destruya prematuramente

        self.last_cmd_vel_out_msg = Twist()
        self.last_cmd_vel_avoidance_msg = Twist()

        # Variable para controlar el estado de publicación
        self.publishing_enabled = False
        self.last_publishing_enabled = False

        # Distancia máxima y mínima a la que se consideran obstáculos (en metros)
        self.max_obstacle_distance = 0.4  # Ajusta según tus necesidades
        self.min_obstacle_distance = 0.15  # Ajusta según tus necesidades

        # Velocidad lateral máxima permitida
        self.max_lateral_speed = 0.5  # Ajusta según tus necesidades

        # Velocidad angular para el giro final
        self.angular_speed = 0.5  # Ajusta según tus necesidades

        # Ángulo a girar al final en radianes (15 grados)
        self.angle_to_rotate = 45 * 3.14159 / 180  # 15 degrees to radians

        # Almacenar la última dirección de desplazamiento lateral
        self.last_lateral_direction = 0.0
        
                # Inicializar la variable de evasión de obstáculos
        self.should_activate_avoidance = False
        
    def cmd_vel_out_callback(self, msg):
        # Comparar el mensaje actual con el último mensaje almacenado en cmd_vel_avoidance
        if (msg.linear.x != self.last_cmd_vel_avoidance_msg.linear.x or
            msg.linear.y != self.last_cmd_vel_avoidance_msg.linear.y or
            msg.angular.z != self.last_cmd_vel_avoidance_msg.angular.z):
            
            # Verificar que no esté frenado (velocidad no debe ser cero)
            if (msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.angular.z != 0.0):
                self.should_activate_avoidance = True
            else:
                self.should_activate_avoidance = False
        else:
            self.should_activate_avoidance = False

        # Actualizar el último mensaje almacenado en cmd_vel_out
        self.last_cmd_vel_out_msg = msg
        
       	

    def laser_callback(self, msg):
        # Lógica para detectar obstáculos y determinar dirección de evasión
        closest_obstacle_distance = float('inf')
        obstacle_angle = 0.0

        self.get_logger().info(f'Received LIDAR data: {msg.ranges[:10]}')  # Log the first 10 LIDAR ranges

        for i, range in enumerate(msg.ranges):
            if range < msg.range_max and self.min_obstacle_distance < range < self.max_obstacle_distance:
                if range < closest_obstacle_distance:
                    closest_obstacle_distance = range
                    obstacle_angle = msg.angle_min + i * msg.angle_increment

        self.get_logger().info(f'Closest obstacle distance: {closest_obstacle_distance}')  # Log closest obstacle distance
        self.get_logger().info(f'Obstacle angle: {obstacle_angle}')  # Log obstacle angle

        if closest_obstacle_distance < self.max_obstacle_distance:
            self.publishing_enabled = True
            self.get_logger().info(f'Obstacle detected at {closest_obstacle_distance} meters.')

            # Calcular velocidad lateral basada en la distancia al obstáculo
            lateral_speed = min(self.max_lateral_speed, (self.max_obstacle_distance - closest_obstacle_distance) / self.max_obstacle_distance * self.max_lateral_speed +0.3)

            # Ajustar la dirección del desplazamiento lateral basado en el ángulo del obstáculo
            if obstacle_angle < 0:
                lateral_speed = -lateral_speed  # Desplazarse a la derecha
            else:
                lateral_speed = lateral_speed  # Desplazarse a la izquierda

            # Guardar la última dirección de desplazamiento lateral
            self.last_lateral_direction = lateral_speed

            # Publicar comando de velocidad
            self.publish_cmd_vel(0.0, lateral_speed)
        else:
            self.publishing_enabled = False
            self.get_logger().info('No obstacles within the maximum distance.')
            print( self.should_activate_avoidance)

            # Detener la publicación si no hay obstáculos detectados
            if self.last_publishing_enabled:
                self.publish_cmd_vel(0.0, 0.0)

                # Girar en el propio eje 15 grados en la dirección opuesta al último movimiento lateral
                if self.last_lateral_direction > 0:
                    self.rotate_in_place(self.angular_speed)
                else:
                    self.rotate_in_place(-self.angular_speed)

        self.last_publishing_enabled = self.publishing_enabled

    def publish_cmd_vel(self, linear_speed, lateral_speed):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.linear.y = lateral_speed  # Movimiento lateral
        cmd_vel_msg.angular.z = 0.0  # Sin rotación
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={linear_speed}, linear.y={lateral_speed}')  # Log published cmd_vel
        print( self.should_activate_avoidance)

    def rotate_in_place(self, angular_speed):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = angular_speed
        duration = abs(self.angle_to_rotate / angular_speed)
        self.get_logger().info(f'Rotating in place: angular.z={angular_speed} for duration={duration}')
        print( self.should_activate_avoidance)
        
        # Publicar comando de rotación
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        while self.get_clock().now() < end_time:
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            sleep(0.1)

        # Detener rotación
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info('Rotation complete.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

