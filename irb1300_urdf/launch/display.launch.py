import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    irb1300_urdf_path = get_package_share_directory('irb1300_urdf')
    urdf_file = os.path.join(irb1300_urdf_path, 'urdf', '1300_urdf.urdf')
    rviz_config_file = os.path.join(irb1300_urdf_path, 'config', 'display.rviz')

    if not os.path.isfile(rviz_config_file):
        raise RuntimeError(
            f'RViz config not found: {rviz_config_file}\n'
            'Run: cd ~/ros2_ws && colcon build --packages-select irb1300_urdf && source install/setup.bash'
        )

    # Force RViz to use our config even if ~/.rviz2/default.rviz is stale.
    rviz2_dir = os.path.expanduser('~/.rviz2')
    os.makedirs(rviz2_dir, exist_ok=True)
    shutil.copy2(rviz_config_file, os.path.join(rviz2_dir, 'default.rviz'))

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    return LaunchDescription([
        LogInfo(msg=f'Loading RViz config: {rviz_config_file}'),
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file,
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file, '-f', 'base_link'],
        ),
    ])
