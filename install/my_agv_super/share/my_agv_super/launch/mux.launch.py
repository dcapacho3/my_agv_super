from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel',
            description='cmd vel output topic'),


    use_sim_time = LaunchConfiguration('use_sim_time')

    mux_params = os.path.join(get_package_share_directory('my_agv_super'),'config','mux.yaml')

    mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[mux_params, {'use_sim_time': use_sim_time}],
            remappings=[
            ('/cmd_vel_out', '/cmd_vel_out') ]
         )

 

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        mux_node,

        # twist_stamper       
    ])
