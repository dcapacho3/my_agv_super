#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_gamepad', 10)
        self.speed = 0.2
        self.turn_speed = 0.5  # Velocidad de giro
        self.timeout_duration = 0.1  # 0.1 segundos
        self.debounce_duration = 0.2  # 0.2 segundos para debounce
        self.last_received_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.button_b_pressed = False
        self.counter = 0
        self.last_button_b_state = 0
        self.last_button_b_change_time = self.get_clock().now()
        self.mode_normal = True  # Flag para modo normal
        self.mode_special = False  # Flag para modo especial

        # Parámetro para habilitar/deshabilitar publicación
        self.publish_enabled = True

    def joy_callback(self, msg):
        current_time = self.get_clock().now()
        
        current_button_b_state = msg.buttons[1]  # Suponiendo que el botón B es el segundo en el array de botones
        current_button_4_state = msg.buttons[4]  # Botón 4
        current_button_5_state = msg.buttons[5]  # Botón 5
        time_since_last_change = (current_time - self.last_button_b_change_time).nanoseconds / 1e9
        
        # Verificar si el estado del botón B ha cambiado y si ha pasado suficiente tiempo
        if current_button_b_state != self.last_button_b_state and time_since_last_change > self.debounce_duration:
            self.last_button_b_change_time = current_time
            self.last_button_b_state = current_button_b_state

            if current_button_b_state == 1:  # Si el botón B está presionado
                if not self.button_b_pressed:
                    self.get_logger().info('Button B pressed')
                    self.button_b_pressed = True
                    self.counter += 1
                    # Alternar entre modo normal y modo especial basado en la paridad del contador
                    if self.counter % 2 == 0:
                        self.mode_normal = True
                        self.mode_special = False
                        self.publish_enabled = True  # Habilitar publicación en modo normal
                    else:
  

                        self.mode_normal = False
                        self.mode_special = True
                         # Deshabilitar publicación en modo especial
            else:
                if self.button_b_pressed:
                    self.get_logger().info('Button B released')
                    
                    self.button_b_pressed = False

        # Crear el mensaje Twist
        twist = Twist()
        
        if self.mode_normal:
            twist.linear.x = self.speed * msg.axes[7]  # Adelante y atrás
            twist.linear.y = self.speed * msg.axes[6]  # Izquierda y derecha
            
            # Controlar el giro con los botones 4 y 5
            if current_button_4_state == 1:
                twist.angular.z = self.turn_speed  # Giro en una dirección (por ejemplo, derecha)
            elif current_button_5_state == 1:
                twist.angular.z = -self.turn_speed  # Giro en la dirección opuesta (por ejemplo, izquierda)
            else:
                twist.angular.z = 0.0  # Sin giro si ninguno de los botones está presionado

        elif self.mode_special:
            # En modo especial, publicar Twist vacío para detener el movimiento
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publish_enabled = False 

        # Publicar el mensaje Twist solo si la publicación está habilitada
        if self.publish_enabled:
            self.publisher.publish(twist)

        self.last_received_time = self.get_clock().now()

    def check_timeout(self):
        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_received_time).nanoseconds / 1e9

        if time_since_last_msg > self.timeout_duration:
            # Si ha pasado más de timeout_duration segundos, publicar Twist con ceros solo si estamos en modo normal
            if self.mode_normal:
                twist = Twist()  # Mensaje vacío, detiene el movimiento
                if self.publish_enabled:
                    self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

