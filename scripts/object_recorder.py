#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sqlite3

class ObjectRecorder(Node):
    def __init__(self):
        super().__init__('object_recorder')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_position = None

        # Conectar a la base de datos existente
        self.database_connection = sqlite3.connect('database/products.db')
        self.cursor = self.database_connection.cursor()

        self.get_logger().info('Object Recorder Node Initialized. ')

        self.keep_running = True  # Variable de control para mantener el bucle principal

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def save_object_position(self):
        if self.current_position:
            object_name = input("Ingrese nombre del objeto: ")
            x = self.current_position.x
            y = self.current_position.y
            self.cursor.execute("INSERT INTO products (name, x, y) VALUES (?, ?, ?)", (object_name, x, y))
            self.database_connection.commit()
            self.get_logger().info(f'Object "{object_name}" recorded at position ({x}, {y}).')
        else:
            self.get_logger().warning('No position data available.')

    def quit_node(self):
        self.get_logger().info('Quitting Node.')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecorder()

    try:
        while rclpy.ok() and node.keep_running:
            rclpy.spin_once(node, timeout_sec=0.1)
            action = input("Presione 's' para guardar la posicion del objeto, o 'n' to salir: ").lower()
            if action == 's':
                node.save_object_position()
                while True:
                     choice = input("¿Desea agregar otro objeto? (y/n): ").lower()
                     if choice == 'y': 
                        break  # Salir del bucle y esperar a que se presione 's' nuevamente
                     elif choice == 'n':
                        node.quit_node()  # Salir del programa
                        node.keep_running = False
                        break
                     else:
                        print("Opción no válida. Por favor, ingrese 'y' o 'n'.")

    except KeyboardInterrupt:
        pass

    node.database_connection.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

