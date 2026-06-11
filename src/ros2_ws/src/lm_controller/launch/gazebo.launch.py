import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('lm_controller')

    urdf_path = os.path.join(pkg_share, 'lite6_urdf', 'lite6.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    visual_path = os.path.join(pkg_share, 'lite6_urdf', 'visual')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Replace relative mesh paths with absolute paths
    robot_description = robot_description.replace(
        'filename="visual/',
        f'filename="{visual_path}/'
    )

    # Inject gazebo_ros2_control plugin
    gazebo_plugin = f"""
    <gazebo>
      <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
        <parameters>{controllers_yaml}</parameters>
      </plugin>
    </gazebo>

    <ros2_control name="lite6" type="system">
      <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
      </hardware>
      <joint name="joint1">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint2">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint3">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint4">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint5">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint6">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>
    """

    robot_description = robot_description.replace('</robot>', gazebo_plugin + '</robot>')

    # ── Gazebo ────────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # ── Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ── Spawn robot in Gazebo ─────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'lite6'],
        output='screen'
    )

    # ── Joint State Broadcaster ───────────────────────────────────────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ── Joint Trajectory Controller ───────────────────────────────────────
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        joint_trajectory_controller,
    ])