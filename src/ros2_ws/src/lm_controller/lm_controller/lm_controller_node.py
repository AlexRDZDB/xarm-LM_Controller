import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import numpy as np
import pinocchio as pin


class LMNullSpaceController(Node):
    def __init__(self, sim_mode: bool):
        super().__init__('lm_null_space_controller')

        self.sim_mode = sim_mode
        self.get_logger().info(f'Starting in {"SIMULATION" if sim_mode else "REAL ROBOT"} mode')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_ip', '192.168.1.179')
        self.declare_parameter('dof', 6)
        self.declare_parameter('control_freq', 250)
        self.declare_parameter('urdf_path', os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'lite6_urdf',
            'lite6.urdf'
        ))
        self.declare_parameter('lambda_damp', 0.04)
        self.declare_parameter('alpha', 10.0)

        self.robot_ip     = self.get_parameter('robot_ip').value
        self.dof          = self.get_parameter('dof').value
        self.control_freq = self.get_parameter('control_freq').value
        self.urdf_path    = self.get_parameter('urdf_path').value
        self.lam          = self.get_parameter('lambda_damp').value
        self.alpha        = self.get_parameter('alpha').value

        # ── State ──────────────────────────────────────────────────────────────
        self.q_current  = np.zeros(self.dof)
        self.robot_ready = False
        self.x_desired   = None

        # ── Pinocchio model ────────────────────────────────────────────────────
        self.model        = pin.buildModelFromUrdf(self.urdf_path)
        self.data         = self.model.createData()
        self.eef_frame_id = self.model.getFrameId('link6')
        self.q_min        = self.model.lowerPositionLimit
        self.q_max        = self.model.upperPositionLimit
        self.q_mid        = (self.q_min + self.q_max) / 2.0
        self.get_logger().info(f'Pinocchio model loaded from {self.urdf_path}')

        # ── Topics ─────────────────────────────────────────────────────────────
        if self.sim_mode:
            joint_states_topic = '/joint_states'
            joint_cmd_topic    = '/joint_trajectory_controller/joint_trajectory'
        else:
            joint_states_topic = '/xarm/joint_states'
            joint_cmd_topic    = '/xarm/joint_commands'

        # ── Subscriber ─────────────────────────────────────────────────────────
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            10
        )

        self.target_sub = self.create_subscription(
            Point,
            '/lm_controller/target_position',
            self.target_callback,
            10
        )

        # ── Publisher ──────────────────────────────────────────────────────────
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory,
            joint_cmd_topic,
            10
        )

        # ── Real robot SDK ─────────────────────────────────────────────────────
        if not self.sim_mode:
            self._init_real_robot()

        # ── Control timer ──────────────────────────────────────────────────────
        period = 1.0 / self.control_freq
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(f'Controller ready at {self.control_freq} Hz')

    # ── Real robot init ────────────────────────────────────────────────────────
    def _init_real_robot(self):
        try:
            from xarm.wrapper import XArmAPI
            self.arm = XArmAPI(self.robot_ip)
            self.arm.motion_enable(enable=True)
            self.arm.clean_error()
            self.arm.clean_warn()
            self.arm.set_mode(1)
            self.arm.set_state(0)
            self.robot_ready = True
            self.get_logger().info(f'Connected to xArm at {self.robot_ip}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')

    def target_callback(self, msg: Point):
        self.x_desired = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'New target: {self.x_desired}')

    # ── Joint state callback ───────────────────────────────────────────────────
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= self.dof:
            self.q_current = np.array(msg.position[:self.dof])

    # ── Publish joint command ──────────────────────────────────────────────────
    def publish_joint_command(self, q_cmd: np.ndarray):
        msg = JointTrajectory()
        msg.joint_names = [f'joint{i+1}' for i in range(self.dof)]

        point = JointTrajectoryPoint()
        point.positions = q_cmd.tolist()
        point.time_from_start = Duration(sec=0, nanosec=4000000)

        msg.points = [point]
        self.joint_cmd_pub.publish(msg)

    # ── Compute Jacobian ───────────────────────────────────────────────────────
    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        pin.computeJointJacobians(self.model, self.data, q)
        J = pin.getFrameJacobian(
            self.model, self.data,
            self.eef_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return J

    # ── LM Null Space Controller ───────────────────────────────────────────────
    def compute_lm_null_space(self, q: np.ndarray) -> np.ndarray:
        I = np.eye(self.dof)

        # Jacobian
        J = self.compute_jacobian(q)

        # J⁺_LM = Jᵀ * (J * Jᵀ + λI)⁻¹
        J_lm = J.T @ np.linalg.inv(J @ J.T + self.lam * I)

        # Forward kinematics for x_err
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        x_current = self.data.oMf[self.eef_frame_id].translation

        # Pad x_err to 6D (position only, zero orientation error)
        x_err = np.zeros(6)
        x_err[:3] = self.x_desired - x_current

        # Joint centering
        q_null = self.alpha * (self.q_mid - q)

        # Full LM null space command
        q_dot = J_lm @ x_err + (I - J_lm @ J) @ q_null

        # Integrate velocity to position
        q_cmd = q + q_dot / self.control_freq

        self.get_logger().info(f'q_dot: {q_dot}')
        self.get_logger().info(f'q_cmd: {q_cmd}')
        return q_cmd

    # ── Control loop ──────────────────────────────────────────────────────────
    def control_loop(self):
        if self.x_desired is None:
            self.get_logger().info('Waiting for x_desired...')
            return
        
        self.get_logger().info(f'x_err: {self.x_desired - self.data.oMf[self.eef_frame_id].translation}')
        q_cmd = self.compute_lm_null_space(self.q_current)

        self.publish_joint_command(q_cmd)

        if not self.sim_mode:
            q_deg = np.degrees(q_cmd).tolist()
            self.arm.set_servo_angle_j(angles=q_deg, is_radian=False)


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    # Temporary node to read sim_mode parameter
    temp_node = rclpy.create_node('temp')
    temp_node.declare_parameter('sim_mode', False)
    sim_mode = temp_node.get_parameter('sim_mode').value
    temp_node.destroy_node()

    node = LMNullSpaceController(sim_mode=sim_mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not sim_mode and hasattr(node, 'arm'):
            node.arm.disconnect()
        node.destroy_node()
        rclpy.shutdown()