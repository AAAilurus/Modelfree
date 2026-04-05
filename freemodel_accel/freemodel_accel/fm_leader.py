#!/usr/bin/env python3
import os
import csv
import numpy as np
import scipy.linalg
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FMLeader(Node):
    def __init__(self):
        super().__init__('fm_leader')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('q_des', [0.5, -0.6])
        self.declare_parameter('noise_std', 0.0)
        self.declare_parameter('num_traj', 50)
        self.declare_parameter('T_each', 300)
        self.declare_parameter('sigma_q', 0.4)
        self.declare_parameter('joint_names', ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('save_dir', '/root/Modelfree/freemodel_out')
        self.declare_parameter('joint_states_topic', '/so100/joint_states')
        self.declare_parameter('cmd_topic', '/so100/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('u_topic', '/so100/lqr_u')

        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.q_des = np.array(self.get_parameter('q_des').value, dtype=float)
        self.noise_std = float(self.get_parameter('noise_std').value)
        self.num_traj = int(self.get_parameter('num_traj').value)
        self.T_each = int(self.get_parameter('T_each').value)
        self.sigma_q = float(self.get_parameter('sigma_q').value)
        self.joints = list(self.get_parameter('joint_names').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.js_topic = str(self.get_parameter('joint_states_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.u_topic = str(self.get_parameter('u_topic').value)

        sample_time = self.dt
        self.Ad = np.array([
            [1.0, 0.0, sample_time, 0.0],
            [0.0, 1.0, 0.0, sample_time],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)
        self.Bd = np.array([
            [0.5 * sample_time**2, 0.0],
            [0.0, 0.5 * sample_time**2],
            [sample_time, 0.0],
            [0.0, sample_time]
        ], dtype=float)

        self.Q_star = np.diag([100.0, 100.0, 10.0, 10.0])
        self.R_star = np.diag([0.5, 0.5])

        riccati_solution = scipy.linalg.solve_discrete_are(
            self.Ad, self.Bd, self.Q_star, self.R_star
        )
        self.K_star = np.linalg.solve(
            self.R_star + self.Bd.T @ riccati_solution @ self.Bd,
            self.Bd.T @ riccati_solution @ self.Ad
        )

        self.have_joint_state = False
        self.current_joint_position = np.zeros(2, dtype=float)
        self.current_joint_velocity = np.zeros(2, dtype=float)

        self.trajectory_index = 0
        self.step_index = 0
        self.internal_error_state = None

        self.logged_Ek = []
        self.logged_Uk = []
        self.logged_Ek1 = []
        self.logged_Uk1 = []
        self.logged_q_command = []
        self.logged_dq_command = []

        np.random.seed(42)

        self.create_subscription(JointState, self.js_topic, self.joint_state_callback, 20)
        self.command_publisher = self.create_publisher(JointTrajectory, self.cmd_topic, 10)
        self.control_input_publisher = self.create_publisher(Float64MultiArray, self.u_topic, 10)

        self.control_timer = self.create_timer(self.dt, self.run_step)

        self.get_logger().info(
            f"[fm_leader] rate_hz={1.0/self.dt:.1f} dt={self.dt:.4f} "
            f"noise_std={self.noise_std} total_steps={self.num_traj * self.T_each}"
        )

    def joint_state_callback(self, joint_state_message):
        try:
            selected_joint_indices = [joint_state_message.name.index(joint_name) for joint_name in self.joints]
        except ValueError:
            return

        self.current_joint_position = np.array(
            [joint_state_message.position[index] for index in selected_joint_indices],
            dtype=float
        )
        self.current_joint_velocity = np.array(
            [
                joint_state_message.velocity[index] if len(joint_state_message.velocity) > index else 0.0
                for index in selected_joint_indices
            ],
            dtype=float
        )
        self.have_joint_state = True

    def publish_control_input(self, control_input):
        control_message = Float64MultiArray()
        control_message.data = [float(value) for value in control_input]
        self.control_input_publisher.publish(control_message)

    def publish_trajectory_command(self, commanded_joint_position, commanded_joint_velocity, commanded_joint_acceleration):
        trajectory_message = JointTrajectory()
        trajectory_message.header.stamp = self.get_clock().now().to_msg()
        trajectory_message.joint_names = list(self.joints)

        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [float(value) for value in commanded_joint_position]
        trajectory_point.velocities = [float(value) for value in commanded_joint_velocity]
        trajectory_point.accelerations = [float(value) for value in commanded_joint_acceleration]
        trajectory_point.time_from_start.sec = 0
        trajectory_point.time_from_start.nanosec = int(self.dt * 1e9)

        trajectory_message.points = [trajectory_point]
        self.command_publisher.publish(trajectory_message)

    def save_csv(self, file_path, rows, header):
        with open(file_path, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(header)
            csv_writer.writerows(rows)

    def save_all(self):
        os.makedirs(self.save_dir, exist_ok=True)

        self.save_csv(os.path.join(self.save_dir, 'Ek.csv'), self.logged_Ek, ['e1', 'e2', 'e3', 'e4'])
        self.save_csv(os.path.join(self.save_dir, 'Uk.csv'), self.logged_Uk, ['u1', 'u2'])
        self.save_csv(os.path.join(self.save_dir, 'Ek1.csv'), self.logged_Ek1, ['e1', 'e2', 'e3', 'e4'])
        self.save_csv(os.path.join(self.save_dir, 'Uk1.csv'), self.logged_Uk1, ['u1', 'u2'])
        self.save_csv(os.path.join(self.save_dir, 'Qcmd.csv'), self.logged_q_command, ['qcmd1', 'qcmd2'])
        self.save_csv(os.path.join(self.save_dir, 'DQcmd.csv'), self.logged_dq_command, ['dqcmd1', 'dqcmd2'])

        with open(os.path.join(self.save_dir, 'K_star.csv'), 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['k1', 'k2', 'k3', 'k4'])
            for row in self.K_star:
                csv_writer.writerow([float(value) for value in row])

        self.get_logger().info(f"[fm_leader] saved {len(self.logged_Ek)} samples -> {self.save_dir}")

    def start_new_trajectory(self):
        self.internal_error_state = np.array([
            self.sigma_q * np.random.randn(),
            self.sigma_q * np.random.randn(),
            0.0,
            0.0
        ], dtype=float)

        initial_joint_position = self.q_des + self.internal_error_state[:2]
        initial_joint_velocity = self.internal_error_state[2:]
        initial_joint_acceleration = np.zeros(2, dtype=float)

        self.step_index = 0
        self.publish_trajectory_command(
            initial_joint_position,
            initial_joint_velocity,
            initial_joint_acceleration
        )
        self.publish_control_input(initial_joint_acceleration)

    def run_step(self):
        if not self.have_joint_state:
            return

        if self.trajectory_index >= self.num_traj:
            self.save_all()
            self.get_logger().info("[fm_leader] done")
            raise SystemExit

        if self.step_index == 0:
            self.start_new_trajectory()
            self.step_index = 1
            return

        measured_error_state = np.hstack([
            self.current_joint_position - self.q_des,
            self.current_joint_velocity
        ])

        command_noise = self.noise_std * np.random.randn(2)
        commanded_acceleration = -self.K_star @ measured_error_state + command_noise

        commanded_joint_velocity = self.current_joint_velocity + commanded_acceleration * self.dt
        commanded_joint_position = (
            self.current_joint_position
            + self.current_joint_velocity * self.dt
            + 0.5 * commanded_acceleration * self.dt**2
        )

        self.publish_trajectory_command(
            commanded_joint_position,
            commanded_joint_velocity,
            commanded_acceleration
        )
        self.publish_control_input(commanded_acceleration)

        self.logged_q_command.append(commanded_joint_position.copy())
        self.logged_dq_command.append(commanded_joint_velocity.copy())

        data_noise = self.noise_std * np.random.randn(2)
        current_control_for_data = -self.K_star @ self.internal_error_state + data_noise
        next_error_state = self.Ad @ self.internal_error_state + self.Bd @ current_control_for_data
        next_noise = self.noise_std * np.random.randn(2)
        next_control_for_data = -self.K_star @ next_error_state + next_noise

        self.logged_Ek.append(self.internal_error_state.copy())
        self.logged_Uk.append(current_control_for_data.copy())
        self.logged_Ek1.append(next_error_state.copy())
        self.logged_Uk1.append(next_control_for_data.copy())

        self.internal_error_state = next_error_state
        self.step_index += 1

        if self.step_index > self.T_each:
            self.trajectory_index += 1
            self.step_index = 0


def main():
    rclpy.init()
    node = FMLeader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
