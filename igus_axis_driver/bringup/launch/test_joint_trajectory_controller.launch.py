# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="axis_",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("igus_axis_driver"),
            "config",
            "rrbot_singleAxis_joint_trajectory_publisher.yaml",
        ]
    )
    prefix = LaunchConfiguration("prefix")

    position_goals=                launch_ros.parameter_descriptions.ParameterFile(
                    param_file=os.path.join(get_package_share_directory('igus_axis_driver'), 'config', 'rrbot_singleAxis_joint_trajectory_publisher.yaml')
,
                    allow_substs=True)

    return LaunchDescription(declared_arguments +         [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_joint_trajectory_controller",
                parameters=[position_goals],
                output="both",
            )
        ])
