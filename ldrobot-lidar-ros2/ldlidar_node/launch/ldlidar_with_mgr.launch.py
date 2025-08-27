# Copyright 2022 Walter Lucetti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

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

    # default params file (del paquete)
    default_params = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ldlidar.yaml'
    )
    params_file = LaunchConfiguration('params_file')

    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name of the LdLidar node'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the LdLidar parameters YAML'
    )

    # --- Lifecycle manager config ---
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr.yaml'
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config_path]
    )

    # --- Include LdLidar bringup (pasando node_name y params_file) ---
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': node_name,
            'params_file': params_file
        }.items()
    )

    # --- LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(declare_node_name)
    ld.add_action(declare_params_file)
    ld.add_action(lc_mgr_node)
    ld.add_action(ldlidar_launch)
    return ld

