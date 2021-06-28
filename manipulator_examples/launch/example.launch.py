# 参考：https://github.com/rt-net/crane_plus/blob/master/crane_plus_examples/launch/example.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    xacro_file = os.path.join(get_package_share_directory('manipulator_description'),
                              'urdf', 'manipulator.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_config}

    # robot_description_semantic_config = load_file(
    #     'crane_plus_moveit_config', 'config/crane_plus.srdf')
    # robot_description_semantic = {
    #     'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('manipulator_moveit_config', 'config/kinematics.yaml')

    declare_example_name = DeclareLaunchArgument(
        'example', default_value='joint_values',
        description=('Set an example executable name: '
                     '[joint_values]')
    )

    example_node = Node(name=[LaunchConfiguration('example'), '_node'],
                        package='manipulator_examples',
                        executable=LaunchConfiguration('example'),
                        output='screen',
                        parameters=[robot_description,
                                    kinematics_yaml])

    return LaunchDescription([declare_example_name, example_node])