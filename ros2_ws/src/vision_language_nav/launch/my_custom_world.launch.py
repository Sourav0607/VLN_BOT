#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_vision_nav = get_package_share_directory('vision_language_nav')
    
    # Your custom world path
    world_path = os.path.join(pkg_vision_nav, 'worlds', 'my_world.world')
    person_detector_node = Node(
        package='vision_language_nav',
        executable='person_detector',
        name='person_detector',
        output='screen'
    )
    
    # Launch Gazebo with your saved world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'false'
        }.items()
    )
    
    return LaunchDescription([
        gazebo,
        person_detector_node,
    ])  