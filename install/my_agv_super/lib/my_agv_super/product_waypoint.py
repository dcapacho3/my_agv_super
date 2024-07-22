#!/usr/bin/env python3
import sqlite3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
import math
import numpy as np
import yaml

# Ruta al archivo YAML del mapa global
map_yaml_path = 'maps/cafe_world_map.yaml'

# Función para cargar el mapa global
def load_map(map_yaml_path):
    with open(map_yaml_path, 'r') as f:
        yaml_content = yaml.safe_load(f)
    
    # Extraer información del YAML
    resolution = yaml_content['resolution']
    origin = yaml_content['origin']
    map_image = yaml_content['image']
    
    # Cargar matriz de ocupación desde el archivo de imagen
    map_array = np.array([[0] * 100 for _ in range(100)])  # Crea una matriz vacía como ejemplo
    return map_array, resolution, origin

# Función para verificar si un camino entre dos puntos está libre de obstáculos
def is_path_clear(start, end, map_array, resolution, origin):
    def coords_to_indices(x, y):
        map_x = int((x - origin[0]) / resolution)
        map_y = int((y - origin[1]) / resolution)
        return map_x, map_y
    
    if not ('x' in start and 'y' in start) or not ('x' in end and 'y' in end):
        raise ValueError("Start and end must be dictionaries with 'x' and 'y' keys")
    
    start_idx = coords_to_indices(start['x'], start['y'])
    end_idx = coords_to_indices(end['x'], end['y'])
    
    def line_pixels(x0, y0, x1, y1):
        pixels = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            pixels.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return pixels

    line_pixels_list = line_pixels(start_idx[0], start_idx[1], end_idx[0], end_idx[1])
    
    for (x, y) in line_pixels_list:
        if x < 0 or x >= map_array.shape[1] or y < 0 or y >= map_array.shape[0]:
            continue
        if map_array[y, x] == 0:  # Considera 0 como ocupado
            return False
    
    return True

# Función para obtener ubicaciones de productos desde la base de datos
def get_product_locations():
    conn = sqlite3.connect('database/products.db')
    cursor = conn.cursor()
    cursor.execute('SELECT name, x, y FROM selected_products')
    locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
    conn.close()
    return locations

# Función para calcular la distancia euclidiana entre dos puntos
def calculate_distance(p1, p2):
    return math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

# Función para ordenar los waypoints por distancia mínima desde un punto inicial
def sort_waypoints_by_distance(start_point, locations, map_array, resolution, origin):
    locations_with_weights = []

    for loc in locations:
        # Verificar si el camino entre el punto actual y el waypoint está libre de obstáculos
        if is_path_clear(start_point, loc, map_array, resolution, origin):
            weight = 1000000  # Peso 0 si el waypoint está libre de obstáculos
        else:
            weight = 0  # Peso alto si el waypoint tiene obstáculos
        
        # Calcular la distancia entre el punto actual y el waypoint
        distance = calculate_distance(start_point, loc)

        # Penalizar los waypoints con obstáculos al incrementar la distancia ponderada
        weighted_distance = distance + weight

        # Agregar el waypoint con su distancia ponderada a la lista
        locations_with_weights.append((loc, weighted_distance))
    
    # Ordenar los waypoints por la distancia ponderada
    locations_with_weights.sort(key=lambda loc: loc[1])
    
    # Devolver la lista ordenada de waypoints
    sorted_locations = [loc for loc, _ in locations_with_weights]
    
    return sorted_locations

# Clase para obtener la posición actual del robot desde la odometría
class OdomSubscriber:
    def __init__(self, node):
        self.node = node
        self.subscription = node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_pose = None

    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

def calculate_completion_percentage(start_pose, current_pose, goal_pose):
    total_distance = calculate_distance(start_pose, goal_pose)
    remaining_distance = calculate_distance(current_pose, goal_pose)
    completion_percentage = ((total_distance - remaining_distance) / total_distance) * 100
    return max(0, min(completion_percentage, 100))

def main():
    rclpy.init()
    node = rclpy.create_node('navigator_node')
    odom_subscriber = OdomSubscriber(node)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Cargar el mapa
    map_array, resolution, origin = load_map(map_yaml_path)

    # Obtener la posición actual del robot
    while odom_subscriber.current_pose is None:
        rclpy.spin_once(node)

    start_pose = odom_subscriber.current_pose

    locations = get_product_locations()

    if not locations:
        print("No selected products found.")
        return

    # Ordenar los waypoints por distancia desde la posición inicial del robot, evitando obstáculos
    sorted_locations = sort_waypoints_by_distance(start_pose, locations, map_array, resolution, origin)

    # Imprimir el orden de los waypoints
    print("Orden de waypoints:")
    for index, loc in enumerate(sorted_locations):
        print(f"{index + 1}. {loc['name']} (x: {loc['x']}, y: {loc['y']})")

    goal_poses = []
    pose_names = {}

    for loc in sorted_locations:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = loc['x']
        goal_pose.pose.position.y = loc['y']
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_poses.append(goal_pose)
        pose_names[len(goal_poses) - 1] = loc['name']

    current_waypoint = 0
    total_waypoints = len(goal_poses)

    while current_waypoint < total_waypoints:
        goal_pose = goal_poses[current_waypoint]
        navigator.goToPose(goal_pose)

        # Esperar a que la navegación se complete
        while not navigator.isNavComplete():
            rclpy.spin_once(node, timeout_sec=1.0)
            feedback = navigator.getFeedback()
            if feedback:
                current_pose = odom_subscriber.current_pose
                if current_pose:
                    completion_percentage = calculate_completion_percentage(start_pose, current_pose, {'x': goal_pose.pose.position.x, 'y': goal_pose.pose.position.y})
                    print(f'Navegando hacia "{pose_names[current_waypoint]}" {current_waypoint + 1}/{total_waypoints} ({completion_percentage:.1f}% completado)')

        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print(f'Waypoint "{pose_names[current_waypoint]}" alcanzado exitosamente.')
            while True:
                user_input = input("Presione 'c' para continuar al siguiente waypoint...")
                if user_input.lower() == 'c':
                    break
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Navigation failed!')
        
        current_waypoint += 1

    print("Navegación completada.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

