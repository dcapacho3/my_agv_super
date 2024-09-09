#!/usr/bin/env python3
import sqlite3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # Importa el mensaje de tipo String para el control
from robot_navigator import BasicNavigator, NavigationResult
import math
import numpy as np
import yaml
from PIL import Image
import os
from itertools import permutations
from ament_index_python.packages import get_package_share_directory

class AutonomousNavigator:
    def __init__(self):
        self.node = rclpy.create_node('navigator_node')
        self.odom_subscriber = OdomSubscriber(self.node)
        self.continue_subscriber = ContinueSubscriber(self.node)  # Suscriptor al tópico continue_nav
        self.status_publisher = self.node.create_publisher(String, '/navigation_status', 10)
        self.visited_waypoints = set()
        
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()


    def publish_status(self, status, waypoint_name=None, completion_percentage=None):
        msg = String()
        msg.data = f"{status}|{waypoint_name or ''}|{completion_percentage or ''}|{','.join(self.visited_waypoints)}"
        self.status_publisher.publish(msg)


    def get_product_locations(self):
        db_dir = os.path.join('src/my_agv_super/database/products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        conn.close()
        return locations

    def calculate_distance(self, p1, p2):
        return math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

    def calculate_distance_matrix(self, locations):
        locs = [(loc['x'], loc['y']) for loc in locations]
        return np.array([[self.calculate_distance({'x': loc1[0], 'y': loc1[1]}, {'x': loc2[0], 'y': loc2[1]}) for loc2 in locs] for loc1 in locs])

    def tsp_bruteforce(self, distance_matrix):
        n = len(distance_matrix)
        min_path = None
        min_distance = float('inf')
        for perm in permutations(range(n)):
            dist = sum(distance_matrix[perm[i], perm[i + 1]] for i in range(n - 1))
            if dist < min_distance:
                min_distance = dist
                min_path = perm
        return min_path

    def sort_waypoints_by_tsp(self, locations):
        distance_matrix = self.calculate_distance_matrix(locations)
        tsp_order = self.tsp_bruteforce(distance_matrix)
        return [locations[i] for i in tsp_order]

    def calculate_completion_percentage(self, start_pose, current_pose, goal_pose):
        total_distance = self.calculate_distance(start_pose, goal_pose)
        remaining_distance = self.calculate_distance(current_pose, goal_pose)
        completion_percentage = ((total_distance - remaining_distance) / total_distance) * 100
        return max(0, min(completion_percentage, 100))

    def navigate(self):
        print("Waiting for initial position from odometry...")
        while self.odom_subscriber.current_pose is None:
            rclpy.spin_once(self.node)
        start_pose = self.odom_subscriber.current_pose
        print(f"Initial position: {start_pose}")

        locations = self.get_product_locations()
        if not locations:
            print("No selected products found.")
            return

        sorted_locations = self.sort_waypoints_by_tsp(locations)
        tsp_path = [sorted_locations.index(loc) for loc in sorted_locations]

        # Solicitar confirmación del usuario antes de iniciar la navegación
        self.publish_status("READY")
        while not self.continue_subscriber.should_continue:
            rclpy.spin_once(self.node)
        
        goal_poses = []
        pose_names = {}

        for loc in sorted_locations:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = loc['x']
            goal_pose.pose.position.y = loc['y']
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)
            pose_names[len(goal_poses) - 1] = loc['name']

        current_waypoint = 0
        total_waypoints = len(goal_poses)

        while current_waypoint < total_waypoints:
            goal_pose = goal_poses[current_waypoint]
            self.navigator.goToPose(goal_pose)
            self.publish_status("NAVIGATING", pose_names[current_waypoint])


            while not self.navigator.isNavComplete():
                rclpy.spin_once(self.node, timeout_sec=1.0)
                feedback = self.navigator.getFeedback()
                if feedback:
                    current_pose = self.odom_subscriber.current_pose
                    if current_pose:
                        completion_percentage = self.calculate_completion_percentage(start_pose, current_pose, {'x': goal_pose.pose.position.x, 'y': goal_pose.pose.position.y})
                        self.publish_status("NAVIGATING", pose_names[current_waypoint], completion_percentage)
                    

            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                self.publish_status("REACHED", pose_names[current_waypoint])
                self.visited_waypoints.add(pose_names[current_waypoint])
                
            elif result == NavigationResult.CANCELED:
                self.publish_status("CANCELED", pose_names[current_waypoint])
                break
            elif result == NavigationResult.FAILED:
                self.publish_status("FAILED", pose_names[current_waypoint])
                break
            self.continue_subscriber.should_continue = False
            

            # Esperar confirmación para continuar al siguiente waypoint
 
            self.publish_status("WAITING", pose_names[current_waypoint])
            while not self.continue_subscriber.should_continue:
                rclpy.spin_once(self.node)
            
            current_waypoint += 1

        self.publish_status("COMPLETED")

        return current_waypoint

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

class ContinueSubscriber:
    def __init__(self, node):
        self.node = node
        self.subscription = node.create_subscription(
            String,
            '/continue_nav',
            self.continue_callback,
            10
        )
        self.should_continue = False

    def continue_callback(self, msg):
        if msg.data == "continue":
            self.should_continue = True

def main(args=None):
    rclpy.init(args=args)
    navigator = AutonomousNavigator()
    navigator.navigate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
