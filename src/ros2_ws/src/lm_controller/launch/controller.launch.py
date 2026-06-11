import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('lm_controller')

    urdf_path = os.path.join(pkg_share, 'lite6_urdf', 'lite6.urdf')

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch in simulation mode'
    )

    sim = LaunchConfiguration('sim')

    # ── Gazebo simulation ──────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        condition=IfCondition(sim)
    )

    # ── LM Null Space Controller node ──────────────────────────────────────────
    controller_node = Node(
        package='lm_controller',
        executable='lm_controller_node',
        name='lm_null_space_controller',
        output='screen',
        parameters=[{
            'urdf_path':    urdf_path,
            'control_freq': 250,
            'dof':          6,
            'lambda_damp':  0.04,
            'alpha':        0.1,
            'sim_mode':      True,
        }],
    )

    return LaunchDescription([
        sim_arg,
        gazebo_launch,
        controller_node,
    ])