#!/usr/bin/env python3
#
# Copyright 2025 Mark Tomaszewski
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_joint_state_publisher_gui = LaunchConfiguration('use_joint_state_publisher_gui', default='true')

    # TODO: Add parameterization for robot configuration
    urdf_file = os.path.join(
        get_package_share_directory('roamr_description'),
        'urdf',
        'roamr_robot.urdf.xacro'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]),
                    value_type=str,
                ),
            }
        ],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_state_publisher_gui),
    )

    rviz_file = os.path.join(
        get_package_share_directory('roamr_description'),
        'rviz',
        'roamr_robot.rviz',
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Use RViz GUI if true'),
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='true',
            description='Use joint state_publisher GUI if true'),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])