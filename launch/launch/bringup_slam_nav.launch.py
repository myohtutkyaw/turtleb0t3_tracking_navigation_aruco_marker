#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params   = LaunchConfiguration('params_file')
    use_sim  = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/jimmy/maps/stadium.yaml'),
        DeclareLaunchArgument('params_file', default_value='/home/jimmy/nav2_tb3.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Map server + AMCL + full Nav2 stack
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim},
                        {'yaml_filename': map_yaml},
                        params]
        ),
    ])
