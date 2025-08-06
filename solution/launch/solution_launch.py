#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace
from launch import LaunchContext
import yaml

def robot_controller_actions(context: LaunchContext):
    """Create robot controller nodes based on number of robots."""
    num_robots = int(context.launch_configurations['num_robots'])
    
    # Load initial poses configuration
    yaml_path = os.path.join(get_package_share_directory('assessment'), 'config', 'initial_poses.yaml')
    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)
    
    initial_poses = configuration[num_robots]
    
    actions = []
    
    for robot_name in initial_poses.keys():
        group = GroupAction([
            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),
            
            Node(
                package='solution',
                executable='robot_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_name': robot_name
                }]
            )
        ])
        actions.append(group)
    
    return actions

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
    
    # Declare arguments
    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='42',
        description='Random number seed for item manager')
    
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
    
    # Robot controllers (dynamic based on num_robots)
    robot_controllers = OpaqueFunction(function=robot_controller_actions)
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_random_seed_cmd)
    
    # Set global parameter
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    
    # Add actions
    ld.add_action(assessment_launch)
    ld.add_action(robot_controllers)
    
    return ld