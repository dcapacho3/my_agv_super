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
import signal
import subprocess
import time
import math
import matplotlib.patches as patches


class NavigationWindow(ctk.CTk):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        
         # Configuración de la interfaz
         
       
        self.title("Navigation Interface")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))
        self.resizable(width=1, height=1)

        # Inicializar la lista de productos seleccionados
        self.selected_products = []
        
        self.node = None
        self.executor = None
        self.navigator = None
        self.odom_subscriber = None
        self.continue_nav_publisher = None
        self.current_pose = None
        
        self.launch_processes = []
        self.launch_thread = None
        
        self.ros_thread = threading.Thread(target=self.init_ros)
        self.ros_thread.daemon = True  # Daemon para que termine cuando la GUI se cierre
        self.ros_thread.start()
        
        
        self.robot_patch = None
        self.robot_width = 0.35  # in meters
        self.robot_length = 0.15  # in meters
        
        
        
        
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

        
        self.start_calibration_button = ctk.CTkButton(
            self.button_inner_frame,
            text="Iniciar Navegacion",
            command=self.start_calibration,  # Nueva función que realizarás
            height=80,
        )
        self.start_calibration_button.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True, padx=5)


        # Botón "Iniciar Navegación" con mayor altura
        self.start_navigation_button = ctk.CTkButton(
            self.button_inner_frame,
            text="Siguiente producto",
            command=self.start_navigation,
            height=80,
        )
        self.start_navigation_button.pack(side=ctk.LEFT, fill=ctk.BOTH, expand=True)
        

        # Crear y mostrar el gráfico
        self.create_plot(self.map_frame)
        
        self.update_robot_position()

        # Actualizar la lista de productos seleccionados
        self.view_selected_products()
        self.after(1000, self.view_selected_products)
        
        
        
    def init_ros(self):
       rclpy.init(args=None)
       self.node = rclpy.create_node('navigate_node')
       self.executor = rclpy.executors.SingleThreadedExecutor()
       self.executor.add_node(self.node)
       self.navigator = BasicNavigator()
       self.odom_subscriber = self.node.create_subscription(Odometry, 'odom', self.odom_callback, 10)
       self.continue_nav_publisher = self.node.create_publisher(String, '/continue_nav', 10)
       self.status_subscriber = self.node.create_subscription(String, '/navigation_status', self.status_callback, 10)
    
    # Spin the executor in a loop
       while rclpy.ok():
          self.executor.spin_once(timeout_sec=0.1)


        
      
    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        }
        
    def status_callback(self, msg):
        status, waypoint_name, completion_percentage, visited_waypoints = msg.data.split('|')
        visited_waypoints = set(visited_waypoints.split(','))
  
        # Actualizar el waypoint actual en función del mensaje
        self.update_waypoint_status(status, waypoint_name, visited_waypoints)

       
    def update_waypoint_status(self, status, waypoint_name, visited_waypoints):
    # Obtener la ubicación de los waypoints desde la base de datos
        locations = self.get_product_locations()

        for loc in locations:
            if loc['name'] == waypoint_name:
                pixel_x = int((loc['x'] - self.origin[0]) / self.resolution)
                pixel_y = int((loc['y'] - self.origin[1]) / self.resolution)

                if status == "REACHED":
                    # El waypoint ya ha sido visitado, cambiar a verde
                    self.update_marker_color(pixel_x, pixel_y, 'green')
                    self.update_text_color(waypoint_name, 'green')
                elif status == "NAVIGATING":
                    # El waypoint está en navegación, cambiar a amarillo
                    self.update_marker_color(pixel_x, pixel_y, 'yellow')
                    self.update_text_color(waypoint_name, 'yellow')


    def update_marker_color(self, x, y, color):
    # Actualizar el color del marcador en el gráfico
        for marker in self.ax.lines:
            if marker.get_xdata() == x and marker.get_ydata() == y:
                marker.set_color(color)
        self.canvas.draw()

    def update_text_color(self, name, color):
        # Actualizar el color del texto en el panel de productos
        for widget in self.selected_frame.winfo_children():
            if isinstance(widget, ctk.CTkLabel) and widget.cget("text") == name:
                widget.configure(text_color=color)

      
    def get_yaw_from_quaternion(self, quaternion):
        # Convert quaternion to Euler angles
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw
      
    def spin_ros_node(self):
        rclpy.spin(self.node)




    def show_info(self, message, title="Info"):
        info_window = ctk.CTkToplevel()
        info_window.title(title)
        info_window.geometry("300x150")
        label = ctk.CTkLabel(info_window, text=message, padx=20, pady=20)
        label.pack(expand=True)
        ok_button = ctk.CTkButton(info_window, text="OK", command=info_window.destroy)
        ok_button.pack(pady=10)
        
    def update_robot_position(self):
        if self.current_pose:
            # Convert map coordinates to pixel coordinates
            pixel_x = int((self.current_pose['x'] - self.origin[0]) / self.resolution)
            pixel_y = int((self.current_pose['y'] - self.origin[1]) / self.resolution)
           # print(f"Robot position: pixel_x={pixel_x}, pixel_y={pixel_y}")  # Debug print

            orientation = self.current_pose.get('orientation', 0)  # in radians

            # Update the robot's position and orientation
            robot_width_pixels = self.robot_width / self.resolution
            robot_length_pixels = self.robot_length / self.resolution
            
            # Calculate the position for the rectangle's bottom-left corner
            corner_x = pixel_x - robot_width_pixels / 2
            corner_y = pixel_y - robot_length_pixels / 2

            # Update the robot patch
            self.robot_patch.set_xy((corner_x, corner_y))
            self.robot_patch.set_angle(math.degrees(orientation))

            self.canvas.draw()
        else:
            print("No current pose available")
        
        self.after(100, self.update_robot_position)


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
        
        #self.robot_position, = self.ax.plot([], [], 'bo', markersize=10, label='Robot')
        
        # Create the robot patch
        robot_width_pixels = self.robot_width / self.resolution
        robot_length_pixels = self.robot_length / self.resolution
        self.robot_patch = patches.Rectangle((0, 0), robot_width_pixels, robot_length_pixels, 
                                             fill=True, facecolor='blue', edgecolor='black')
        self.ax.add_patch(self.robot_patch)

        self.ax.legend()

    def update_map_plot(self):
        self.ax.clear()
        # Cargar el mapa y actualizar el gráfico
        self.map_array, self.resolution, self.origin = self.load_map()
        self.ax.imshow(np.flipud(self.map_array), cmap='gray', origin='lower')

        # Obtener y plotear ubicaciones de productos
        self.plot_product_locations()
        
        self.robot_position, = self.ax.plot([], [], 'bo', markersize=10, label='Robot')
        self.ax.legend()
        
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
            print("Iniciando lanzamiento de archivos y navegación...")
            
            # Launch the ROS 2 launch files
            # Crear y ejecutar un hilo para la navegación
            navigation_thread = threading.Thread(target=self.run_navigation)
            navigation_thread.start()
            self.navigation_started = True  # Marcar que la navegación ha comenzado
        else:
            print("La navegación ya ha comenzado. Ejecutando otra acción...")
            self.perform_alternate_action()
            
            
    def launch_ros2_files(self):
        launch_commands = [
            "ros2 launch my_agv_super mux.launch.py",
            "ros2 launch my_agv_super navagv.launch.py"
        ]
        
        devnull = open(os.devnull, 'w')
        for cmd in launch_commands:
            process = subprocess.Popen(
                cmd, 
                shell=True, 
                stdout=devnull, 
                stderr=subprocess.STDOUT
            )
            self.launch_processes.append(process)
        
        print("Launch files started successfully (output suppressed).")
        
        

    def run_navigation(self):
        # Esta función será ejecutada en un hilo separado
        navigator = AutonomousNavigator()
        waypoints_reached = navigator.navigate()
        print("Navegación completa")
        # Publicar el mensaje al finalizar la navegación
        self.publish_continue_nav()
        
        
    def __del__(self):
        # Terminate launch file processes when the window is closed
        for process in self.launch_processes:
            process.terminate()
        
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()

    def perform_alternate_action(self):
            self.publish_continue_nav()
            self.continue_nav_published = True  # Marcastartr que el mensaje ha sido publicado
            self.publish_stop()

    def publish_continue_nav(self):
        msg = String()
        msg.data = "continue"
        self.continue_nav_publisher.publish(msg)
        
    def publish_stop(self):
        msg = String()
        msg.data = "stop"
        self.continue_nav_publisher.publish(msg)
        
        
# Nueva función que manejará la acción del nuevo botón
    def start_calibration(self):
        if not self.launch_thread or not self.launch_thread.is_alive():
            self.launch_thread = threading.Thread(target=self.launch_ros2_files)
            self.launch_thread.start()
            self.start_calibration_button.configure(state="disabled")
            print("Launching ROS 2 files in background...")
        else:
            print("ROS 2 launch files are already running.")


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
