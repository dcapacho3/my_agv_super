#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys

class TeleopGamepad(Node):
    def __init__(self):
        super().__init__('teleop_gamepad')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_gamepad', 10)
        pygame.init()

        # Inicializar el joystick
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            self.get_logger().error('No se detectó ningún joystick.')
            sys.exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.speed = 0.5
        self.turn = 1.0
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def publish_cmd_vel(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        twist = Twist()

        # Control de movimiento omnidireccional
        twist.linear.x = self.joystick.get_axis(1) * -self.speed  # Eje vertical del stick izquierdo (adelante/atrás)
        twist.linear.y = self.joystick.get_axis(0) * self.speed   # Eje horizontal del stick izquierdo (izquierda/derecha)

        # Control de rotación
        twist.angular.z = (self.joystick.get_axis(3) - self.joystick.get_axis(2)) * self.turn  # Triggers izquierdo y derecho para rotar

        # Frenar con el botón A (generalmente botón 0 en muchos gamepads)
        if self.joystick.get_button(0):
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0

        # Publicar el comando de velocidad
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_gamepad = TeleopGamepad()

    try:
        rclpy.spin(teleop_gamepad)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_gamepad.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

