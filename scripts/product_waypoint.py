#!/usr/bin/env python3
import sqlite3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
import math
import numpy as np
import yaml
from PIL import Image
import os
from shapely.geometry import Point, Polygon, LineString, MultiPoint
from shapely.ops import unary_union
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as pltPolygon
from sklearn.cluster import DBSCAN
from scipy.spatial import distance_matrix
from scipy.optimize import linear_sum_assignment
from itertools import permutations
from ament_index_python.packages import get_package_share_directory




# Ruta al archivo YAML del mapa global
bringup_dir = get_package_share_directory('my_agv_super')
#map_yaml_path = os.path.join(bringup_dir, 'maps/labrobsuper_map.yaml')
map_yaml_path = os.path.join(bringup_dir, 'maps/cafe_world_map.yaml')


#map_yaml_path = 'maps/cafe_world_map.yaml'
#map_yaml_path = 'maps/labrobsuper_map.yaml'

# Función para cargar el mapa global
def load_map(map_yaml_path):
    with open(map_yaml_path, 'r') as f:
        yaml_content = yaml.safe_load(f)
    
    resolution = yaml_content['resolution']
    origin = yaml_content['origin']
    map_image_path = yaml_content['image']
    
    if not os.path.isabs(map_image_path):
        map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)
    
    print(f'Loading map image from: {map_image_path}')
    map_image = Image.open(map_image_path)
    map_array = np.array(map_image)
    
    return map_array, resolution, origin

# Función para verificar si un camino entre dos puntos está libre de obstáculos
def is_path_clear(start, end, obstacle_areas):
    line = LineString([(start['x'], start['y']), (end['x'], end['y'])])
    for area in obstacle_areas:
        poly = Polygon(area)
        if line.intersects(poly):
            return False
    return True
   

# Función para obtener ubicaciones de productos desde la base de datos
def get_product_locations():

    db_dir = os.path.join('src/my_agv_super/database/products.db')
    conn = sqlite3.connect(db_dir)
    cursor = conn.cursor()
    cursor.execute('SELECT name, x, y FROM selected_products')
    locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
    conn.close()
    return locations


def calculate_distance(p1, p2):
    return math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

def calculate_distance_matrix(locations):
    locs = [(loc['x'], loc['y']) for loc in locations]
    return distance_matrix(locs, locs)
    
def tsp_bruteforce(distance_matrix):
    n = len(distance_matrix)
    min_path = None
    min_distance = float('inf')
    for perm in permutations(range(n)):
        dist = sum(distance_matrix[perm[i], perm[i + 1]] for i in range(n - 1))
        #dist += distance_matrix[perm[-1], perm[0]]  # Return to start
        if dist < min_distance:
            min_distance = dist
            min_path = perm
    return min_path

# Función para ordenar los waypoints por distancia mínima desde un punto inicial
def sort_waypoints_by_tsp(locations, obstacle_areas):
    distance_matrix = calculate_distance_matrix(locations)
    tsp_order = tsp_bruteforce(distance_matrix)
    
    # Adjust distances with obstacle weights
    locations_with_weights = []
    for i, loc in enumerate(locations):
        loc_with_weight = loc.copy()
        loc_with_weight['weight'] = 0
        for j in range(len(locations)):
            if not is_path_clear(loc, locations[j], obstacle_areas):
                loc_with_weight['weight'] += 1000000  # Peso alto para evitar esta ruta
        locations_with_weights.append(loc_with_weight)
    
    # Reorder locations according to TSP result
    ordered_locations = [locations_with_weights[i] for i in tsp_order]
    return ordered_locations
    
# Función para encontrar los puntos de obstáculos en el mapa
def find_obstacle_coords(map_array, resolution, origin):
    obstacle_coords = []
    
    for y in range(map_array.shape[0]):
        for x in range(map_array.shape[1]):
            if map_array[y, x] == 0:  # Considera 0 como ocupado
                map_x = origin[0] + x * resolution
                map_y = origin[1] + (map_array.shape[0] - y - 1) * resolution  # Invertir el eje y
                obstacle_coords.append((map_x, map_y))
    
    return obstacle_coords

# Función para agrupar puntos de obstáculos en áreas
def cluster_obstacles(obstacle_coords, eps=0.5, min_samples=5):
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(obstacle_coords)
    labels = clustering.labels_
    
    unique_labels = set(labels)
    clusters = []
    for label in unique_labels:
        if label == -1:
            continue
        cluster = [obstacle_coords[i] for i in range(len(labels)) if labels[i] == label]
        clusters.append(cluster)
    
    return clusters

# Función para generar áreas de obstáculos a partir de clusters y filtrar por tamaño
def generate_obstacle_areas(clusters, size_threshold=1.0):
    obstacle_areas = []
    for cluster in clusters:
        multipoint = MultiPoint(cluster)
        convex_hull = multipoint.convex_hull
        if convex_hull.area <= size_threshold:  # Filtrar áreas grandes
            # Aquí se pueden ajustar las formas complejas si se requiere
            # Para simplificar, usamos el envolvente convexa como el área
            obstacle_areas.append(convex_hull.exterior.coords)
    
    return obstacle_areas
def visualize_obstacles(map_array, resolution, origin, clusters, obstacle_areas, locations, tsp_path, start_pose):
    plt.ion()
    plt.figure(figsize=(10, 10))
    plt.imshow(map_array, cmap='gray', origin='lower')

    # Dibujar los clusters de obstáculos
    cluster_colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    for i, cluster in enumerate(clusters):
        cluster_color = cluster_colors[i % len(cluster_colors)]
        for point in cluster:
            x_index = int((point[0] - origin[0]) / resolution)
            y_index = int((point[1] - origin[1]) / resolution)
            y_index = map_array.shape[0] - y_index - 1
            plt.scatter(x_index, y_index, color=cluster_color, s=10)

    # Dibujar las áreas de obstáculos
    for area in obstacle_areas:
        area_coords = [(int((point[0] - origin[0]) / resolution), map_array.shape[0] - int((point[1] - origin[1]) / resolution) - 1) for point in area]
        if len(area_coords) > 2:
            plt_polygon = pltPolygon(area_coords, fill=None, edgecolor='r')
            plt.gca().add_patch(plt_polygon)

    # Dibujar las ubicaciones de los productos
    for loc in locations:
        x_index = int((loc['x'] - origin[0]) / resolution)
        y_index = int((loc['y'] - origin[1]) / resolution)
        y_index = map_array.shape[0] - y_index - 1
        plt.scatter(x_index, y_index, color='blue', marker='o', s=50, label=loc['name'])
    
    # Incluir la posición inicial en la trayectoria TSP
    tsp_coords = [(start_pose['x'], start_pose['y'])] + [(locations[i]['x'], locations[i]['y']) for i in tsp_path]
    
    # Convertir a índices de coordenadas en el mapa
    tsp_coords_index = [(int((x - origin[0]) / resolution), map_array.shape[0] - int((y - origin[1]) / resolution) - 1) for x, y in tsp_coords]
    
    # Traza la trayectoria
    plt.plot(*zip(*tsp_coords_index), color='magenta', marker='o', markersize=5, linestyle='-', linewidth=2, label='TSP Path')

    plt.legend(loc='upper right', bbox_to_anchor=(1.2, 1))
    plt.title('Obstáculos Agrupados y Áreas de Obstáculos con Ubicaciones de Productos y Trayectoria TSP')
    plt.xlabel('X')
    plt.ylabel('Y')
    #plt.show()


    
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
    plt.show()

    # Cargar el mapa
    print("Loading map...")
    map_array, resolution, origin = load_map(map_yaml_path)
    print("Map loaded successfully.")

    # Obtener la posición actual del robot
    print("Waiting for initial position from odometry...")
    while odom_subscriber.current_pose is None:
        rclpy.spin_once(node)
    print(f"Initial position: {odom_subscriber.current_pose}")

    start_pose = odom_subscriber.current_pose

    locations = get_product_locations()
  

    if not locations:
        print("No selected products found.")
        return
        
    obstacle_coords = find_obstacle_coords(map_array, resolution, origin)
    clusters = cluster_obstacles(obstacle_coords)
    obstacle_areas = generate_obstacle_areas(clusters)

    # Ordenar los waypoints por distancia desde la posición inicial del robot, evitando obstáculos
    print("Sorting waypoints by distance...")
    sorted_locations = sort_waypoints_by_tsp(locations, obstacle_areas)
   
    print("Waypoints sorted successfully.")

    # Imprimir el orden de los waypoints
    print("Orden de waypoints:")
    
    tsp_path = [sorted_locations.index(loc) for loc in sorted_locations]
    visualize_obstacles(map_array, resolution, origin, clusters, obstacle_areas, sorted_locations, tsp_path, start_pose)
    print("Starting navigation...")

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
            plt.pause(0.001)

        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print(f'Waypoint "{pose_names[current_waypoint]}" alcanzado exitosamente.')
            while True:
                user_input = input("Presione 'c' para continuar al siguiente waypoint...")
                if user_input.lower() == 'c':
                    break
        elif result == NavigationResult.CANCELED:
            print(f'Navegación cancelada hacia "{pose_names[current_waypoint]}".')
            break
        elif result == NavigationResult.FAILED:
            print(f'Fallo al alcanzar el waypoint "{pose_names[current_waypoint]}".')
            break

        current_waypoint += 1

    print("Navegación completa.")
    plt.ioff()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
