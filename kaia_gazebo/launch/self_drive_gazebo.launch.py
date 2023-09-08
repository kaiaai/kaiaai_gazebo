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
import re
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context: LaunchContext, description):
    description_str = context.perform_substitution(description)

    param_path_name = os.path.join(
        get_package_share_path(description_str),
        'config',
        'self_drive_gazebo.yaml'
        )
    print('Config : {}'.format(param_path_name))

    return [
        Node(
            package="kaia_gazebo",
            executable="self_drive_gazebo",
            output="screen",
            parameters = [param_path_name]
        )
    ]


def generate_launch_description():
    default_description_name = os.getenv('KAIAAI_ROBOT', default='kaiaai_snoopy')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='description',
            default_value=default_description_name,
            description='Robot description package name, overrides KAIAAI_ROBOT'
        ),
        OpaqueFunction(function=make_nodes, args=[
            LaunchConfiguration('description'),
        ]),
    ])
