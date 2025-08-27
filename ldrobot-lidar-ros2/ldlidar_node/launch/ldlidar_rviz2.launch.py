# Copyright 2024 Walter Lucetti
# Licensed under the Apache License, Version 2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch args ---
    node_name = LaunchConfiguration('node_name')
    params_file = LaunchConfiguration('params_file')

    # default params file (del paquete)
    default_params = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ldlidar.yaml'
    )

    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name of the node'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the LdLidar parameters YAML'
    )

    # --- RViz2 settings ---
    rviz2_config = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'config',
        'ldlidar.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[['-d'], [rviz2_config]]
    )

    # --- Include LDLidar + lifecycle manager, pasando params_file ---
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_with_mgr.launch.py'
        ]),
        launch_arguments={
            'node_name': node_name,
            'params_file': params_file
        }.items()
    )

    # --- LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(declare_node_name_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(rviz2_node)
    ld.add_action(ldlidar_launch)
    return ld

