import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    irb1300_urdf_path = get_package_share_directory('irb1300_urdf')
    urdf_file = os.path.join(irb1300_urdf_path, 'urdf', '1300_urdf.urdf')
    rviz_config_file = os.path.join(irb1300_urdf_path, 'config', 'display.rviz')

    default_trajectory = (
        "/data/IRMV_IMITATION_SEMANTIC_origin/project/irb1300_kin_dyn/results/forward_kinematics.npz"
    )
    if not os.path.isfile(default_trajectory):
        default_trajectory = os.path.expanduser(
            "~/irb1300_kin_dyn/results/forward_kinematics.npz"
        )

    rviz2_dir = os.path.expanduser('~/.rviz2')
    os.makedirs(rviz2_dir, exist_ok=True)
    if os.path.isfile(rviz_config_file):
        shutil.copy2(rviz_config_file, os.path.join(rviz2_dir, 'default.rviz'))

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    return LaunchDescription([
        LogInfo(msg='Task 1 trajectory playback: RViz + /joint_states from forward_kinematics.npz'),
        DeclareLaunchArgument(
            'trajectory',
            default_value=default_trajectory,
            description='Path to forward_kinematics.npz from Task 1',
        ),
        DeclareLaunchArgument('rate', default_value='20.0', description='Playback rate [Hz]'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='irb1300_urdf',
            executable='play_task1_trajectory.py',
            name='task1_trajectory_player',
            output='screen',
            arguments=[
                '--trajectory', LaunchConfiguration('trajectory'),
                '--rate', LaunchConfiguration('rate'),
                '--loop',
            ],
        ),
        Node(
            package='irb1300_urdf',
            executable='publish_ee_trajectory_marker.py',
            name='ee_trajectory_marker_publisher',
            output='screen',
            arguments=[
                '--trajectory', LaunchConfiguration('trajectory'),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file, '-f', 'base_link'],
        ),
    ])
