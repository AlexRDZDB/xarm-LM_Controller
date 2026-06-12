# lm_controller

## Overview

This package provides an LM (Levenberg-Marquardt) null-space controller for the uFactory xArm robot. The main node is `lm_controller/lm_controller_node.py`, which listens for a target end-effector position, computes a joint-space command, and publishes joint trajectories to either Gazebo or the real robot.

## What the lm_controller_node does

The node implemented in `lm_controller/lm_controller_node.py`:

- loads the xArm URDF from `lite6_urdf/lite6.urdf`
- subscribes to `/joint_states` in simulation mode (or `/xarm/joint_states` on the real robot)
- subscribes to `/lm_controller/target_position` for the desired end-effector pose
- computes a joint command using a damped least-squares Jacobian-based controller
- publishes a `trajectory_msgs/JointTrajectory` to the appropriate joint controller topic

In simulation mode, it uses the Gazebo joint trajectory controller interface. On the real robot, it can also drive the xArm hardware through the XArm SDK.

## Brief note on the Levenberg-Marquardt controller

The controller used here is a damped least-squares / Levenberg-Marquardt style inverse-kinematics controller.

The key idea is to stabilize the usual Jacobian-based solution by adding a small regularization term:

$$
J_{LM} = J^T (J J^T + \lambda I)^{-1}
$$

where:
- $J$ is the manipulator Jacobian
- $\lambda$ is the damping term (from the `lambda_damp` parameter)
- $I$ is the identity matrix

This makes the controller more numerically stable, especially when the robot is near singular configurations. The implementation also adds a null-space term to keep the joints near the middle of their range and avoid awkward postures.

In practice, this gives a smooth way to move the end effector toward a target position while keeping the robot configuration sensible.

## Launching the controller in simulation mode

The package includes a launch file at `launch/controller.launch.py`. It starts Gazebo and the LM controller node together.

### 1. Source your ROS 2 environment

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
source /workspace/src/ros2_ws/install/setup.bash
```

### 2. Launch the controller in sim mode

```bash
ros2 launch lm_controller controller.launch.py sim:=true
```

This starts:
- Gazebo
- the robot state publisher
- the joint trajectory controller
- the LM controller node

### 3. Send a target position

Publish a `geometry_msgs/Point` to `/lm_controller/target_position` with the desired end-effector coordinates, for example:

```bash
ros2 topic pub /lm_controller/target_position geometry_msgs/Point "{x: 0.2, y: 0.0, z: 0.3}" 
```

The controller will then compute and publish a joint command to track that position.

## Useful parameters

The node exposes these parameters in `lm_controller_node.py`:

- `robot_ip`: xArm IP address (used only in real-robot mode)
- `dof`: number of joints (default: 6)
- `control_freq`: control loop frequency (default: 250 Hz)
- `lambda_damp`: LM damping factor
- `alpha`: null-space centering gain

## Notes

- The simulation launch path is intended for Gazebo-based testing.
- For real hardware, set `sim:=false` when using the hardware-specific launch path.
- The package depends on the ROS 2 Python tooling and on the `pinocchio` and xArm libraries for the controller logic.
