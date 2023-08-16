#!/usr/bin/env python3
#
# Copyright 2023 REMAKE.AI
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
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def make_nodes(context: LaunchContext, description, use_sim_time, x_pose, y_pose):
    description_str = context.perform_substitution(description)
    use_sim_time_str = context.perform_substitution(use_sim_time)
    x_pose_str = context.perform_substitution(x_pose)
    y_pose_str = context.perform_substitution(y_pose)

    urdf_path_name = os.path.join(
      get_package_share_path(description_str),
      'gazebo',
      'urdf',
      'robot.urdf')
    print('URDF file name : {}'.format(urdf_path_name))

    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()
    robot_description = ParameterValue(Command(['xacro ', urdf_path_name]), value_type=str)

    sdf_path_name = os.path.join(
        get_package_share_path(description_str),
        'gazebo',
        'robot',
        'model.sdf'
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_str,
                'robot_description': robot_description
            }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', description_str,
                '-file', sdf_path_name,
                '-timeout', '180',
                '-x', x_pose_str,
                '-y', y_pose_str,
                '-z', '0.01'
            ],
            output='screen'
        )
    ]

def generate_launch_description():
    default_description = os.getenv('KAIA_ROBOT_DESCRIPTION', default='kaia_snoopy_description')
    pkg_gazebo_ros = get_package_share_path('gazebo_ros')

    world_name = LaunchConfiguration('world')
    world = os.path.join( get_package_share_path('kaia_gazebo'), 'worlds', world_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='description',
            default_value=default_description,
            description='Robot description package name, overrides KAIA_ROBOT_DESCRIPTION'
        ),
        DeclareLaunchArgument(
            name='x_pose',
            default_value='-2.0',
            description='Robot starting x position'
        ),
        DeclareLaunchArgument(
            name='y_pose',
            default_value='-0.5',
            description='Robot starting y position'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value='kaia_world',
            description='World name'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),
        spawn_kaia_bot_cmd,
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('description'),
            LaunchConfiguration('use_sim_time'),
            LaunchConfiguration('x_pose'),
            LaunchConfiguration('y_pose')
        ])
    ])
