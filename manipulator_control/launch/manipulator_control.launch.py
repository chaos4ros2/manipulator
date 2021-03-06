# Copyright 2020 RT Corporation
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

# 参考 : https://github.com/rt-net/crane_plus/blob/use_new_ros2_control/crane_plus_control/launch/crane_plus_control.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('manipulator_description'),
        'urdf',
        'manipulator.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    manipulator_controllers = os.path.join(
        get_package_share_directory('manipulator_control'),
        'config',
        'manipulator_controllers.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, manipulator_controllers],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    load_controllers = []
    for controller in ['joint_state_controller', 'manipulator_arm_controller']:
        load_controllers.append(
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_start_controller', controller],
                shell=True,
                output='screen',
            )
        )

    return LaunchDescription([
      controller_manager,
    ] + load_controllers)
