#!/usr/bin/env python3

import os
import sqlite3
import customtkinter as ctk
import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image
import numpy as np
import yaml
import threading
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult

from ament_index_python.packages import get_package_share_directory
from guiwaypoint import AutonomousNavigator
from std_msgs.msg import String

class NavigationWindow(ctk.CTk):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.title("Navigation Interface")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))
        self.resizable(width=1, height=1)

        # Inicializar la lista de productos seleccionados
        self.selected_products = []

        # Inicializar el nodo y el navegador de ROS
        rclpy.init(args=None)
        self.node = rclpy.create_node('navigate_node')
        self.navigator = BasicNavigator()
        self.odom_subscriber = self.node.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.current_pose = None
        self.continue_nav_publisher = self.node.create_publisher(String, '/continue_nav', 10)

        # Variable para controlar el estado del botón
        self.navigation_started = False
        self.continue_nav_published = False  # Nueva variable para controlar la publicación

        # Frame superior
        self.top_frame = ctk.CTkFrame(self, height=50)
        self.top_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=10)

        # Ejemplo de etiqueta en el frame superior
        self.label_superior = ctk.CTkLabel(self.top_frame, text="Información Adicional en la parte superior")
        self.label_superior.pack(pady=5)

        # Frame para la información de fecha, hora, etc.
        self.info_frame = ctk.CTkFrame(self, width=200)
        self.info_frame.pack(side=ctk.LEFT, fill=ctk.Y, padx=10, pady=10)

        # Etiquetas de reloj y fecha
        self.label_reloj = ctk.CTkLabel(self.info_frame, font=('ARIAL', 18, 'bold'))
        self.label_reloj.pack(side=ctk.TOP, padx=10, pady=10)
        self.label_fecha = ctk.CTkLabel(self.info_frame, font=('ARIAL', 18, 'bold'))
        self.label_fecha.pack(side=ctk.TOP, padx=10, pady=70)
        shop_vision_label = ctk.CTkLabel(self.info_frame, text="Shop Vision", font=('Helvetica', 20, 'bold'))
        shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)

        # Iniciar la actualización del reloj y la fecha
        self.actualizar_reloj_y_fecha()

        # Frame para el mapa
        self.map_frame = ctk.CTkFrame(self, width=800, height=600)
        self.map_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame para el gráfico y la lista de productos
        self.right_frame = ctk.CTkFrame(self, width=300)
        self.right_frame.pack(side=ctk.RIGHT, fill=ctk.Y, padx=10, pady=10, expand=False)

        # Frame para la lista de productos seleccionados
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=150)
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Frame para el botón
        self.button_frame = ctk.CTkFrame(self.right_frame, width=120)
        self.button_frame.pack(side=ctk.BOTTOM, fill=ctk.X, padx=10, pady=10)

        # Crear un marco interno para el botón con más altura
        self.button_inner_frame = ctk.CTkFrame(self.button_frame, height=100, width=100)
        self.button_inner_frame.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Botón "Iniciar Navegación" con mayor altura
        self.start_navigation_button = ctk.CTkButton(
            self.button_inner_frame,
            text="Iniciar Navegación",
            command=self.start_navigation,
            height=80,
        )
        self.start_navigation_button.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True)

        # Crear y mostrar el gráfico
        self.create_plot(self.map_frame)

        # Actualizar la lista de productos seleccionados
        self.view_selected_products()
        self.after(1000, self.view_selected_products)
        
      
    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

    def show_info(self, message, title="Info"):
        info_window = ctk.CTkToplevel()
        info_window.title(title)
        info_window.geometry("300x150")
        label = ctk.CTkLabel(info_window, text=message, padx=20, pady=20)
        label.pack(expand=True)
        ok_button = ctk.CTkButton(info_window, text="OK", command=info_window.destroy)
        ok_button.pack(pady=10)

    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after(1000, self.actualizar_reloj_y_fecha)

    def create_plot(self, frame):
        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=ctk.BOTH, expand=True)

        self.update_map_plot()

    def update_map_plot(self):
        self.ax.clear()
        # Cargar el mapa y actualizar el gráfico
        self.map_array, self.resolution, self.origin = self.load_map()
        self.ax.imshow(self.map_array, cmap='gray', origin='lower')

        # Obtener y plotear ubicaciones de productos
        self.plot_product_locations()
        
    def load_map(self):
        bringup_dir = get_package_share_directory('my_agv_super')
        map_yaml_path = os.path.join(bringup_dir, 'maps/cafe_world_map.yaml')
        with open(map_yaml_path, 'r') as f:
            yaml_content = yaml.safe_load(f)

        resolution = yaml_content['resolution']
        origin = yaml_content['origin']
        map_image_path = yaml_content['image']

        if not os.path.isabs(map_image_path):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        map_image = Image.open(map_image_path)
        map_array = np.array(map_image)

        return map_array, resolution, origin

    def plot_product_locations(self):
        # Obtener ubicaciones de la base de datos
        locations = self.get_product_locations()

        for loc in locations:
            # Convertir las coordenadas de mapa a píxeles
            pixel_x = int((loc['x'] - self.origin[0]) / self.resolution)
            pixel_y = int((loc['y'] - self.origin[1]) / self.resolution)
            # Plottear las ubicaciones en el mapa
            self.ax.plot(pixel_x, pixel_y, 'ro')  # 'ro' indica puntos rojos

        self.canvas.draw()

    def get_product_locations(self):
        db_dir = os.path.join('src/my_agv_super/database/products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        conn.close()
        return locations

    def start_navigation(self):
        if not self.navigation_started:
            print("Iniciando navegación...")
            # Crear y ejecutar un hilo para la navegación
            navigation_thread = threading.Thread(target=self.run_navigation)
            navigation_thread.start()
            self.navigation_started = True  # Marcar que la navegación ha comenzado
        else:
            print("La navegación ya ha comenzado. Ejecutando otra acción...")
            self.perform_alternate_action()

    def run_navigation(self):
        # Esta función será ejecutada en un hilo separado
        navigator = AutonomousNavigator()
        waypoints_reached = navigator.navigate()
        print("Navegación completa")
        # Publicar el mensaje al finalizar la navegación
        self.publish_continue_nav()

    def perform_alternate_action(self):
            self.publish_continue_nav()
            self.continue_nav_published = True  # Marcar que el mensaje ha sido publicado
            self.publish_stop()

    def publish_continue_nav(self):
        msg = String()
        msg.data = "continue"
        self.continue_nav_publisher.publish(msg)
        
    def publish_stop(self):
        msg = String()
        msg.data = "stop"
        self.continue_nav_publisher.publish(msg)
    

    def view_selected_products(self):
        self.selected_frame.destroy()  # Destruir el marco anterior
        self.selected_frame = ctk.CTkFrame(self.right_frame, width=150)
        self.selected_frame.pack(side=ctk.TOP, fill=ctk.BOTH, expand=True, padx=10, pady=10)

        # Re-crear la lista de productos seleccionados
        conn = sqlite3.connect('src/my_agv_super/database/products.db')
        cursor = conn.cursor()
        cursor.execute('SELECT name FROM selected_products')
        rows = cursor.fetchall()
        conn.close()

        for row in rows:
            product_name = row[0]
            label = ctk.CTkLabel(self.selected_frame, text=product_name)
            label.pack(pady=5)

if __name__ == "__main__":
    app = NavigationWindow()
    app.mainloop()

