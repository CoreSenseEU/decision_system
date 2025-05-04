#!/usr/bin/env python3

# Copyright 2025 KAS-Lab
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch.substitutions import PythonExpression

from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch_ros.actions import SetRemap


def generate_launch_description():
    kb_arg = DeclareLaunchArgument(
        'kb',
        default_value='krr_world.pl'
    )

    task_arg = DeclareLaunchArgument(
        'task',
        default_value='task1'
    )

    no_sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true'
    )

    prolog_server = Node(
        package='prolog_kb',
        executable='prolog_server', 
        parameters=[{
            'knowledge_base': LaunchConfiguration('kb'),
        }],
        emulate_tty=True
    )

    prolog_pose_factory = Node(
        package='prolog_kb',
        executable='prolog_pose_factory'
    )

    get_drop_locations_prolog_adapter = Node(
        package='prolog_kb',
        executable='get_drop_locations_prolog_adapter'
    )

    get_object_info_prolog_adapter = Node(
        package='prolog_kb',
        executable='get_object_info_prolog_adapter'
    )

    get_objects_in_room_prolog_adapter = Node(
        package='prolog_kb',
        executable='get_objects_in_room_prolog_adapter'
    )

    pick_object_prolog_adapter = Node(
        package='prolog_kb',
        executable='pick_object_prolog_adapter'
    )

    place_object_prolog_adapter = Node(
        package='prolog_kb',
        executable='place_object_prolog_adapter'
    )

    krr_simulation_launch = GroupAction(
        condition=IfCondition(
            LaunchConfiguration('sim')
        ),
        actions=[
            SetRemap(src='/get_drop_locations', dst='/shadow/get_drop_locations'),
            SetRemap(src='/get_object_info', dst='/shadow/get_object_info'),
            SetRemap(src='/get_objects_in_room', dst='/shadow/get_objects_in_room'),
            SetRemap(src='/pick_object', dst='/shadow/pick_object'),
            SetRemap(src='/place_object', dst='/shadow/place_object'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('krr_agent'),
                        'launch',
                        PythonExpression(["'", LaunchConfiguration('task'), "' + '.launch.py'"])
                    ])
                ]),
            ),
        ]
    )

    return LaunchDescription([
        kb_arg,
        task_arg,
        no_sim_arg,
        prolog_server,
        prolog_pose_factory,
        get_drop_locations_prolog_adapter,
        get_object_info_prolog_adapter,
        get_objects_in_room_prolog_adapter,
        pick_object_prolog_adapter,
        place_object_prolog_adapter,
        krr_simulation_launch,
    ])
