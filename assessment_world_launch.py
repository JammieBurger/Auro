#!/usr/bin/env python3

"""
Launch file for AURO Assessment World
This file sets up the complete simulation environment for the assessment task.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate the launch description for the assessment world"""
    
    # Package directories
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_assessment = FindPackageShare('auro_assessment')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='assessment_world.world')
    robot_count = LaunchConfiguration('robot_count', default='1')
    robot_name_1 = LaunchConfiguration('robot_name_1', default='tb3_0')
    robot_name_2 = LaunchConfiguration('robot_name_2', default='tb3_1')
    robot_name_3 = LaunchConfiguration('robot_name_3', default='tb3_2')
    x_pose_1 = LaunchConfiguration('x_pose_1', default='0.0')
    y_pose_1 = LaunchConfiguration('y_pose_1', default='0.0')
    x_pose_2 = LaunchConfiguration('x_pose_2', default='2.0')
    y_pose_2 = LaunchConfiguration('y_pose_2', default='0.0')
    x_pose_3 = LaunchConfiguration('x_pose_3', default='-2.0')
    y_pose_3 = LaunchConfiguration('y_pose_3', default='0.0')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='assessment_world.world',
        description='World file name'
    )
    
    declare_robot_count = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots to spawn (1-3)'
    )
    
    declare_robot_name_1 = DeclareLaunchArgument(
        'robot_name_1',
        default_value='tb3_0',
        description='Name of first robot'
    )
    
    declare_robot_name_2 = DeclareLaunchArgument(
        'robot_name_2',
        default_value='tb3_1',
        description='Name of second robot'
    )
    
    declare_robot_name_3 = DeclareLaunchArgument(
        'robot_name_3',
        default_value='tb3_2',
        description='Name of third robot'
    )
    
    declare_x_pose_1 = DeclareLaunchArgument(
        'x_pose_1',
        default_value='0.0',
        description='X position of first robot'
    )
    
    declare_y_pose_1 = DeclareLaunchArgument(
        'y_pose_1',
        default_value='0.0',
        description='Y position of first robot'
    )
    
    declare_x_pose_2 = DeclareLaunchArgument(
        'x_pose_2',
        default_value='2.0',
        description='X position of second robot'
    )
    
    declare_y_pose_2 = DeclareLaunchArgument(
        'y_pose_2',
        default_value='0.0',
        description='Y position of second robot'
    )
    
    declare_x_pose_3 = DeclareLaunchArgument(
        'x_pose_3',
        default_value='-2.0',
        description='X position of third robot'
    )
    
    declare_y_pose_3 = DeclareLaunchArgument(
        'y_pose_3',
        default_value='0.0',
        description='Y position of third robot'
    )
    
    # World file path
    world_file = PathJoinSubstitution([
        pkg_assessment,
        'worlds',
        world_name
    ])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_gazebo_ros,
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'use_sim_time': use_sim_time,
            'verbose': 'true'
        }.items()
    )
    
    # Robot spawning function
    def spawn_robot(robot_name, x_pose, y_pose):
        """Create nodes for spawning and controlling a single robot"""
        
        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher_{robot_name}',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open(os.path.join(
                    get_package_share_directory('turtlebot3_description'),
                    'urdf', 'turtlebot3_waffle_pi.urdf'
                )).read()
            }],
            remappings=[
                ('/tf', f'/{robot_name}/tf'),
                ('/tf_static', f'/{robot_name}/tf_static')
            ]
        )
        
        # Spawn robot in Gazebo
        spawn_robot_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot_name}',
            arguments=[
                '-entity', robot_name,
                '-topic', f'/{robot_name}/robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
                '-robot_namespace', robot_name
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )
        
        # Item detector for this robot
        item_detector = Node(
            package='auro_assessment',
            executable='item_detector',
            name=f'item_detector_{robot_name}',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_topic': f'/{robot_name}/camera/image_raw',
                'camera_info_topic': f'/{robot_name}/camera/camera_info',
                'depth_topic': f'/{robot_name}/camera/depth/image_raw',
                'detection_topic': f'/{robot_name}/item_detections',
                'robot_frame': f'{robot_name}/base_link'
            }]
        )
        
        return [robot_state_publisher, spawn_robot_cmd, item_detector]
    
    # Spawn robots based on robot_count
    robot_nodes_1 = spawn_robot('tb3_0', x_pose_1, y_pose_1)
    robot_nodes_2 = spawn_robot('tb3_1', x_pose_2, y_pose_2)
    robot_nodes_3 = spawn_robot('tb3_2', x_pose_3, y_pose_3)
    
    # Assessment task manager
    task_manager = Node(
        package='auro_assessment',
        executable='assessment_task_manager',
        name='assessment_task_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world_config_file': PathJoinSubstitution([
                pkg_assessment,
                'config',
                'assessment_world.yaml'
            ])
        }]
    )
    
    # RViz visualization
    rviz_config = PathJoinSubstitution([
        pkg_assessment,
        'rviz',
        'assessment_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_name)
    ld.add_action(declare_robot_count)
    ld.add_action(declare_robot_name_1)
    ld.add_action(declare_robot_name_2)
    ld.add_action(declare_robot_name_3)
    ld.add_action(declare_x_pose_1)
    ld.add_action(declare_y_pose_1)
    ld.add_action(declare_x_pose_2)
    ld.add_action(declare_y_pose_2)
    ld.add_action(declare_x_pose_3)
    ld.add_action(declare_y_pose_3)
    
    # Add Gazebo
    ld.add_action(gazebo_launch)
    
    # Add task manager
    ld.add_action(task_manager)
    
    # Add robots conditionally
    for node in robot_nodes_1:
        ld.add_action(node)
    
    # Add second robot if robot_count >= 2
    for node in robot_nodes_2:
        ld.add_action(Node(
            package=node.package,
            executable=node.executable,
            name=node.name,
            namespace=node.namespace,
            parameters=node.parameters,
            remappings=node.remappings,
            arguments=node.arguments,
            condition=IfCondition(
                "$(shell test $(arg robot_count) -ge 2; echo $?)"
            )
        ))
    
    # Add third robot if robot_count >= 3
    for node in robot_nodes_3:
        ld.add_action(Node(
            package=node.package,
            executable=node.executable,
            name=node.name,
            namespace=node.namespace,
            parameters=node.parameters,
            remappings=node.remappings,
            arguments=node.arguments,
            condition=IfCondition(
                "$(shell test $(arg robot_count) -ge 3; echo $?)"
            )
        ))
    
    # Add RViz
    ld.add_action(rviz_node)
    
    return ld
