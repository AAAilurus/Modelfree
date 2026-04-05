#!/usr/bin/env python3
import numpy as np
import scipy.linalg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FMFollower(Node):
    def __init__(self):
        super().__init__('fm_follower')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('q_des', [0.5, -0.6])
        self.declare_parameter('q_learned_path', '/root/Modelfree/freemodel_out/Q_learned.csv')

        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.q_des = np.array(self.get_parameter('q_des').value, dtype=float)

        learned_q_data = np.genfromtxt(
            self.get_parameter('q_learned_path').value,
            delimiter=',',
            skip_header=1
        )
        learned_q_data = np.atleast_2d(learned_q_data)
        learned_Q = np.diag(learned_q_data[:, 1])
        fixed_R = np.diag([0.5, 0.5])

        sample_time = self.dt
        Ad = np.array([
            [1.0, 0.0, sample_time, 0.0],
            [0.0, 1.0, 0.0, sample_time],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)
        Bd = np.array([
            [0.5 * sample_time**2, 0.0],
            [0.0, 0.5 * sample_time**2],
            [sample_time, 0.0],
            [0.0, sample_time]
        ], dtype=float)

        riccati_solution = scipy.linalg.solve_discrete_are(Ad, Bd, learned_Q, fixed_R)
        self.K = np.linalg.solve(fixed_R + Bd.T @ riccati_solution @ Bd, Bd.T @ riccati_solution @ Ad)

        self.current_joint_position = np.zeros(2, dtype=float)
        self.current_joint_velocity = np.zeros(2, dtype=float)

        self.command_publisher = self.create_publisher(
            JointTrajectory,
            '/so101/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.create_subscription(JointState, '/so101/joint_states', self.joint_state_callback, 10)
        self.create_timer(self.dt, self.run_step)

    def joint_state_callback(self, joint_state_message):
        self.current_joint_position = np.array(joint_state_message.position[:2], dtype=float)
        self.current_joint_velocity = np.array(joint_state_message.velocity[:2], dtype=float)

    def run_step(self):
        state_error = np.hstack([
            self.current_joint_position - self.q_des,
            self.current_joint_velocity
        ])

        commanded_acceleration = -self.K @ state_error
        commanded_joint_velocity = self.current_joint_velocity + commanded_acceleration * self.dt
        commanded_joint_position = (
            self.current_joint_position
            + self.current_joint_velocity * self.dt
            + 0.5 * commanded_acceleration * self.dt**2
        )

        trajectory_message = JointTrajectory()
        trajectory_message.joint_names = ['Shoulder_Pitch', 'Elbow']

        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = commanded_joint_position.tolist()
        trajectory_point.velocities = commanded_joint_velocity.tolist()
        trajectory_point.accelerations = commanded_acceleration.tolist()
        trajectory_point.time_from_start.sec = 0
        trajectory_point.time_from_start.nanosec = int(self.dt * 1e9)

        trajectory_message.points = [trajectory_point]
        self.command_publisher.publish(trajectory_message)


def main():
    rclpy.init()
    node = FMFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
