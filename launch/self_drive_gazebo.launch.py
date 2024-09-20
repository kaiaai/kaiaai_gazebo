#!/usr/bin/env python3
#
# Copyright 2023-2024, KAIA.AI REMAKE.AI
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
import re
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from kaiaai import config


def make_nodes(context: LaunchContext, robot_model):
    robot_model_str = context.perform_substitution(robot_model)

    if len(robot_model_str) == 0:
      robot_model_str = config.get_var('robot.model')

    param_path_name = os.path.join(
        get_package_share_path(robot_model_str),
        'config',
        'self_drive_gazebo.yaml'
        )
    print('Config : {}'.format(param_path_name))

    return [
        Node(
            package="kaiaai_gazebo",
            executable="self_drive_gazebo",
            output="screen",
            parameters = [param_path_name]
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_model',
            default_value='',
            description='Robot description package name'
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('robot_model'),
        ]),
    ])
