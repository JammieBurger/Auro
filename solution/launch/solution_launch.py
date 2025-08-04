#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace

def generate_launch_description():
    # Launch configurations
    num_robots = LaunchConfiguration('num_robots', default='1')
    random_seed = LaunchConfiguration('random_seed', default='42')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    obstacles = LaunchConfiguration('obstacles', default='true')
    sensor_noise = LaunchConfiguration('sensor_noise', default='false')
    headless = LaunchConfiguration('headless', default='false')
    
    # Zone configuration
    zone_top_left = LaunchConfiguration('zone_top_left', default='true')
    zone_top_right = LaunchConfiguration('zone_top_right', default='true')
    zone_bottom_left = LaunchConfiguration('zone_bottom_left', default='true')
    zone_bottom_right = LaunchConfiguration('zone_bottom_right', default='true')
    
    # Paths
    rviz_config = PathJoinSubstitution([
        FindPackageShare('assessment'), 'rviz', 'namespaced.rviz'
    ])
    
    # Launch assessment world
    assessment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment_launch.py'
            ])
        ),
        launch_arguments={
            'num_robots': num_robots,
            'random_seed': random_seed,
            'use_rviz': use_rviz,
            'rviz_config': rviz_config,
            'obstacles': obstacles,
            'sensor_noise': sensor_noise,
            'headless': headless,
            'zone_top_left': zone_top_left,
            'zone_top_right': zone_top_right,
            'zone_bottom_left': zone_bottom_left,
            'zone_bottom_right': zone_bottom_right,
            'item_manager': 'true',
            'wait_for_items': 'false',
        }.items()
    )
    
    # Robot controller node
    robot_controller = GroupAction([
        PushRosNamespace('robot1'),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        
        Node(
            package='solution',
            executable='robot_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])
    
    # Create launch description
    ld = LaunchDescription()
    
    # Set global parameter
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    
    # Add actions
    ld.add_action(assessment_launch)
    ld.add_action(robot_controller)
    
    return ld
