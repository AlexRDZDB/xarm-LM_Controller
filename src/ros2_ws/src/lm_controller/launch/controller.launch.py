from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # ── Launch arguments ───────────────────────────────────────────────────────
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Run in simulation mode (true) or real robot (false)'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.xxx',
        description='IP address of the real xArm robot'
    )

    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='7',
        description='Degrees of freedom of the xArm (5, 6, or 7)'
    )

    control_freq_arg = DeclareLaunchArgument(
        'control_freq',
        default_value='250',
        description='Control loop frequency in Hz'
    )

    sim = LaunchConfiguration('sim')

    # ── Sim node ───────────────────────────────────────────────────────────────
    sim_node = Node(
        package='lm_controller',
        executable='xarm_interface_node',
        name='xarm_interface_node',
        output='screen',
        condition=IfCondition(sim),
        parameters=[{
            'dof':          LaunchConfiguration('dof'),
            'control_freq': LaunchConfiguration('control_freq'),
        }],
        arguments=['--sim']
    )

    # ── Real robot node ────────────────────────────────────────────────────────
    real_node = Node(
        package='lm_controller',
        executable='xarm_interface_node',
        name='xarm_interface_node',
        output='screen',
        condition=UnlessCondition(sim),
        parameters=[{
            'robot_ip':     LaunchConfiguration('robot_ip'),
            'dof':          LaunchConfiguration('dof'),
            'control_freq': LaunchConfiguration('control_freq'),
        }]
    )

    return LaunchDescription([
        sim_arg,
        robot_ip_arg,
        dof_arg,
        control_freq_arg,
        sim_node,
        real_node,
    ])