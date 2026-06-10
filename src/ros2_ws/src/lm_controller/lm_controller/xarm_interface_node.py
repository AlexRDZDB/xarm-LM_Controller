import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class XArmInterfaceNode(Node):
    def __init__(self, sim_mode: bool):
        super().__init__('xarm_interface_node')

        self.sim_mode = sim_mode
        self.get_logger().info(f'Starting in {"SIMULATION" if sim_mode else "REAL ROBOT"} mode')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_ip', '192.168.1.xxx')
        self.declare_parameter('dof', 7)
        self.declare_parameter('control_freq', 250)

        self.robot_ip    = self.get_parameter('robot_ip').value
        self.dof         = self.get_parameter('dof').value
        self.control_freq = self.get_parameter('control_freq').value

        # ── Joint state tracking ───────────────────────────────────────────────
        self.current_joint_positions = np.zeros(self.dof)
        self.robot_ready = False

        # ── Simulation Interface ───────────────────────────────────────────────────
        if self.sim_mode
            # Publisher
            self.joint_cmd_pub = self.create_publisher(
                JointState,
                '/xarm_sim/joint_commands',
                10
            )

            self.joint_state_sub = self.create_subscription(
                JointState,
                '/xarm_sim/joint_states',
                self.joint_state_callback,
                10
            )

        # ── Real Robot Interface ───────────────────────────────────────────────────
        if not self.sim_mode:
            # Publisher — sends joint position commands to Isaac Sim
            self.joint_cmd_pub = self.create_publisher(
                JointState,
                '/xarm/joint_commands',
                10
            )

            # Subscriber — receives joint states from Isaac Sim
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/xarm/joint_states',
                self.joint_state_callback,
                10
            )

        # ── xArm SDK (real robot only) ─────────────────────────────────────────
        if not self.sim_mode:
            self._init_real_robot()

        # ── Control timer ──────────────────────────────────────────────────────
        period = 1.0 / self.control_freq
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(f'XArm interface ready at {self.control_freq} Hz')

    # ── Real robot initialization ──────────────────────────────────────────────
    def _init_real_robot(self):
        try:
            from xarm.wrapper import XArmAPI
            self.arm = XArmAPI(self.robot_ip)
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(1)    # servo mode — direct joint streaming
            self.arm.set_state(0)   # sport state
            self.robot_ready = True
            self.get_logger().info(f'Connected to real xArm at {self.robot_ip}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to real xArm: {e}')
            self.robot_ready = False

    # ── Joint state callback ───────────────────────────────────────────────────
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= self.dof:
            self.current_joint_positions = np.array(msg.position[:self.dof])

    # ── Publish joint command to Isaac Sim ─────────────────────────────────────
    def publish_joint_command(self, q: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = [f'joint{i+1}' for i in range(self.dof)]
        msg.position = q.tolist()
        self.joint_cmd_pub.publish(msg)

    # ── Send to real robot ─────────────────────────────────────────────────────
    def send_to_real_robot(self, q: np.ndarray):
        if not self.robot_ready:
            return
        # SDK expects degrees, convert from radians
        q_deg = np.degrees(q).tolist()
        self.arm.set_servo_angle_j(angles=q_deg, is_radian=False)

    # ── Main control loop ──────────────────────────────────────────────────────
    def control_loop(self):
        # Placeholder — LM null-space controller output goes here
        # For now just echo current state back as command (no-op)
        q_cmd = self.current_joint_positions.copy()

        # Sim: publish to Isaac Sim via ROS 2
        self.publish_joint_command(q_cmd)

        # Real: send via xArm SDK
        if not self.sim_mode:
            self.send_to_real_robot(q_cmd)


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    # sim_mode is passed via launch argument, read from ROS args
    import sys
    sim_mode = '--sim' in sys.argv or any('sim:=true' in a for a in sys.argv)

    node = XArmInterfaceNode(sim_mode=sim_mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not sim_mode and hasattr(node, 'arm'):
            node.arm.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()