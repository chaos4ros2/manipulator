# 参考：https://github.com/rt-net/crane_plus/blob/master/crane_plus_examples/launch/demo.launch.py

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('manipulator_moveit_config'),
                '/launch/run_move_group.launch.py']),
        )

    control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('manipulator_control'),
                '/launch/manipulator_control.launch.py']),
        )

    return LaunchDescription([move_group,
                              control_node
                              ])