#!/usr/bin/env python3
import os
import csv
import numpy as np
import rclpy
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_matrix_csv(file_path, shape=None):
    data = np.genfromtxt(file_path, delimiter=",", dtype=float, skip_header=1)
    data = np.atleast_2d(data)
    data = data[np.all(np.isfinite(data), axis=1)]
    if shape is not None:
        data = data.reshape(shape)
    return data


class FreeModelDemo(Node):
    def __init__(self):
        super().__init__("fm_demo")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.declare_parameter("joint_names", ["Shoulder_Pitch", "Elbow"])
        self.declare_parameter("initial_joint_position", [0.0, 0.0])
        self.declare_parameter(
            "target_joint_positions_flat",
            [0.5, -0.6, 0.3, 0.4, -0.4, 0.5, -0.5, -0.2]
        )

        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("trajectory_command_duration_seconds", 0.18)
        self.declare_parameter("move_time_per_waypoint_seconds", 3.0)
        self.declare_parameter("hold_time_at_target_seconds", 3.0)
        self.declare_parameter("hold_time_at_initial_seconds", 1.0)
        self.declare_parameter("output_directory", "/root/Modelfree/freemodel_out")

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.num_joints = len(self.joint_names)

        self.initial_joint_position = np.array(
            self.get_parameter("initial_joint_position").value, dtype=float
        )

        flat = list(self.get_parameter("target_joint_positions_flat").value)
        if len(flat) % self.num_joints != 0:
            raise RuntimeError("target_joint_positions_flat length must be multiple of num_joints")

        self.targets = [
            np.array(flat[i:i+self.num_joints], dtype=float)
            for i in range(0, len(flat), self.num_joints)
        ]
        self.sequence = [self.initial_joint_position.copy()] + [q.copy() for q in self.targets] + [self.initial_joint_position.copy()]

        self.dt = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.command_duration = float(self.get_parameter("trajectory_command_duration_seconds").value)
        self.move_time_per_waypoint_seconds = float(self.get_parameter("move_time_per_waypoint_seconds").value)
        self.hold_time_at_target_seconds = float(self.get_parameter("hold_time_at_target_seconds").value)
        self.hold_time_at_initial_seconds = float(self.get_parameter("hold_time_at_initial_seconds").value)
        self.output_directory = str(self.get_parameter("output_directory").value)

        self.move_steps_per_waypoint = max(1, int(round(self.move_time_per_waypoint_seconds / self.dt)))
        self.hold_steps_target = max(1, int(round(self.hold_time_at_target_seconds / self.dt)))
        self.hold_steps_initial = max(1, int(round(self.hold_time_at_initial_seconds / self.dt)))

        self.true_controller_gain = load_matrix_csv(
            os.path.join(self.output_directory, "K_star.csv"),
            shape=(self.num_joints, 2 * self.num_joints),
        )
        self.learned_controller_gain = load_matrix_csv(
            os.path.join(self.output_directory, "K_learned.csv"),
            shape=(self.num_joints, 2 * self.num_joints),
        )

        self.current_leader_q = np.zeros(self.num_joints)
        self.current_follower_q = np.zeros(self.num_joints)
        self.current_leader_dq = np.zeros(self.num_joints)
        self.current_follower_dq = np.zeros(self.num_joints)

        self.leader_cmd_q = None
        self.follower_cmd_q = None
        self.leader_cmd_dq = None
        self.follower_cmd_dq = None

        self.last_u_leader = np.zeros(self.num_joints)
        self.last_u_follower = np.zeros(self.num_joints)

        self.got_leader = False
        self.got_follower = False
        self.states_initialized = False

        self.current_index = 0
        self.current_waypoint = self.sequence[self.current_index].copy()
        self.phase = "move"
        self.phase_step_counter = 0
        self.demo_finished = False

        self.plot_directory = os.path.join(self.output_directory, "plots")
        self.log_directory = os.path.join(self.output_directory, "logs")
        os.makedirs(self.plot_directory, exist_ok=True)
        os.makedirs(self.log_directory, exist_ok=True)

        self.logged_time = []
        self.logged_desired_q = []
        self.logged_leader_q = []
        self.logged_follower_q = []
        self.logged_leader_dq = []
        self.logged_follower_dq = []
        self.logged_leader_u = []
        self.logged_follower_u = []
        self.logged_phase = []
        self.logged_waypoint = []

        self.create_subscription(JointState, "/so100/joint_states", self.leader_cb, 10)
        self.create_subscription(JointState, "/so101/joint_states", self.follower_cb, 10)

        self.pub_leader = self.create_publisher(
            JointTrajectory, "/so100/joint_trajectory_controller/joint_trajectory", 10
        )
        self.pub_follower = self.create_publisher(
            JointTrajectory, "/so101/joint_trajectory_controller/joint_trajectory", 10
        )

        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info("[fm_demo] PURE u-based double-integrator, q_cmd only")
        self.get_logger().info("[fm_demo] leader uses K_star, follower uses K_learned")
        self.get_logger().info(f"[fm_demo] sequence={[q.tolist() for q in self.sequence]}")

    def extract_state(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.joint_names]
        except ValueError:
            return None, None
        q = np.array([msg.position[i] for i in idx], dtype=float)
        dq = np.array([msg.velocity[i] if i < len(msg.velocity) else 0.0 for i in idx], dtype=float)
        return q, dq

    def leader_cb(self, msg):
        q, dq = self.extract_state(msg)
        if q is None:
            return
        self.current_leader_q = q
        self.current_leader_dq = dq
        self.got_leader = True

    def follower_cb(self, msg):
        q, dq = self.extract_state(msg)
        if q is None:
            return
        self.current_follower_q = q
        self.current_follower_dq = dq
        self.got_follower = True

    def initialize_if_needed(self):
        if self.states_initialized:
            return
        if not (self.got_leader and self.got_follower):
            return

        self.leader_cmd_q = self.current_leader_q.copy()
        self.follower_cmd_q = self.current_follower_q.copy()
        self.leader_cmd_dq = np.zeros(self.num_joints)
        self.follower_cmd_dq = np.zeros(self.num_joints)

        self.states_initialized = True

    def publish_position_command(self, pub, q_cmd):
        msg = JointTrajectory()
        msg.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = q_cmd.tolist()
        pt.time_from_start.sec = int(self.command_duration)
        pt.time_from_start.nanosec = int((self.command_duration - int(self.command_duration)) * 1e9)
        msg.points = [pt]
        pub.publish(msg)

    def required_hold_steps(self):
        if self.current_index == 0 or self.current_index == len(self.sequence) - 1:
            return self.hold_steps_initial
        return self.hold_steps_target

    def step_controller(self, cmd_q, cmd_dq, q, dq, q_des, K):
        e = np.hstack([q - q_des, dq])
        u = -K @ e
        cmd_dq_new = cmd_dq + u * self.dt
        cmd_q_new = cmd_q + cmd_dq * self.dt + 0.5 * u * (self.dt ** 2)
        return cmd_q_new, cmd_dq_new, u

    def freeze_hold_states(self):
        self.leader_cmd_q = self.current_waypoint.copy()
        self.follower_cmd_q = self.current_waypoint.copy()
        self.leader_cmd_dq[:] = 0.0
        self.follower_cmd_dq[:] = 0.0
        self.last_u_leader[:] = 0.0
        self.last_u_follower[:] = 0.0

    def advance_waypoint(self):
        self.current_index += 1
        self.phase = "move"
        self.phase_step_counter = 0

        if self.current_index >= len(self.sequence):
            self.demo_finished = True
            self.timer.cancel()
            self.save_outputs()
            self.get_logger().info("[fm_demo] demo completed")
            return

        self.current_waypoint = self.sequence[self.current_index].copy()
        self.leader_cmd_q = self.current_leader_q.copy()
        self.follower_cmd_q = self.current_follower_q.copy()
        self.leader_cmd_dq = np.zeros(self.num_joints)
        self.follower_cmd_dq = np.zeros(self.num_joints)
        self.last_u_leader[:] = 0.0
        self.last_u_follower[:] = 0.0

        self.get_logger().info(
            f"[fm_demo] next waypoint {self.current_index + 1}/{len(self.sequence)} {self.current_waypoint.tolist()}"
        )

    def log_step(self, t):
        self.logged_time.append(t)
        self.logged_desired_q.append(self.current_waypoint.copy())
        self.logged_leader_q.append(self.current_leader_q.copy())
        self.logged_follower_q.append(self.current_follower_q.copy())

        if self.phase == "hold":
            self.logged_leader_dq.append(np.zeros(self.num_joints))
            self.logged_follower_dq.append(np.zeros(self.num_joints))
            self.logged_leader_u.append(np.zeros(self.num_joints))
            self.logged_follower_u.append(np.zeros(self.num_joints))
        else:
            self.logged_leader_dq.append(self.current_leader_dq.copy())
            self.logged_follower_dq.append(self.current_follower_dq.copy())
            self.logged_leader_u.append(self.last_u_leader.copy())
            self.logged_follower_u.append(self.last_u_follower.copy())

        self.logged_phase.append(self.phase)
        self.logged_waypoint.append(self.current_index)

    def save_outputs(self):
        if not self.logged_time:
            return

        t = np.array(self.logged_time)
        qd = np.array(self.logged_desired_q)
        qL = np.array(self.logged_leader_q)
        qF = np.array(self.logged_follower_q)
        dqL = np.array(self.logged_leader_dq)
        dqF = np.array(self.logged_follower_dq)
        uL = np.array(self.logged_leader_u)
        uF = np.array(self.logged_follower_u)

        csv_path = os.path.join(self.log_directory, "four_state_demo_log.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            header = ["time"]
            for j in range(self.num_joints):
                header.append(f"desired_q{j+1}")
            for j in range(self.num_joints):
                header.append(f"leader_q{j+1}")
            for j in range(self.num_joints):
                header.append(f"leader_dq{j+1}")
            for j in range(self.num_joints):
                header.append(f"leader_u{j+1}")
            for j in range(self.num_joints):
                header.append(f"follower_q{j+1}")
            for j in range(self.num_joints):
                header.append(f"follower_dq{j+1}")
            for j in range(self.num_joints):
                header.append(f"follower_u{j+1}")
            header.extend(["phase", "waypoint_index"])
            w.writerow(header)

            for i in range(len(t)):
                row = [t[i]]
                row.extend(qd[i].tolist())
                row.extend(qL[i].tolist())
                row.extend(dqL[i].tolist())
                row.extend(uL[i].tolist())
                row.extend(qF[i].tolist())
                row.extend(dqF[i].tolist())
                row.extend(uF[i].tolist())
                row.extend([self.logged_phase[i], self.logged_waypoint[i]])
                w.writerow(row)

        fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        fig.suptitle("Leader vs Follower Four-State Graph", fontsize=15, fontweight="bold")

        ax[0, 0].plot(t, qL[:, 0], label="Leader q1")
        ax[0, 0].plot(t, qF[:, 0], "--", label="Follower q1")
        ax[0, 0].plot(t, qd[:, 0], ":", label="Desired q1")
        ax[0, 0].grid(True, alpha=0.3)
        ax[0, 0].legend()

        ax[0, 1].plot(t, qL[:, 1], label="Leader q2")
        ax[0, 1].plot(t, qF[:, 1], "--", label="Follower q2")
        ax[0, 1].plot(t, qd[:, 1], ":", label="Desired q2")
        ax[0, 1].grid(True, alpha=0.3)
        ax[0, 1].legend()

        ax[1, 0].plot(t, dqL[:, 0], label="Leader dq1")
        ax[1, 0].plot(t, dqF[:, 0], "--", label="Follower dq1")
        ax[1, 0].grid(True, alpha=0.3)
        ax[1, 0].legend()

        ax[1, 1].plot(t, dqL[:, 1], label="Leader dq2")
        ax[1, 1].plot(t, dqF[:, 1], "--", label="Follower dq2")
        ax[1, 1].grid(True, alpha=0.3)
        ax[1, 1].legend()

        plt.tight_layout()
        plot_path = os.path.join(self.plot_directory, "leader_follower_four_state_graph.png")
        plt.savefig(plot_path, dpi=200)
        plt.close(fig)

        self.get_logger().info(f"[fm_demo] saved log -> {csv_path}")
        self.get_logger().info(f"[fm_demo] saved plot -> {plot_path}")

    def step(self):
        if not (self.got_leader and self.got_follower):
            return
        if self.demo_finished:
            return

        self.initialize_if_needed()
        if not self.states_initialized:
            return

        t = len(self.logged_time) * self.dt
        self.phase_step_counter += 1

        if self.phase == "move":
            self.leader_cmd_q, self.leader_cmd_dq, self.last_u_leader = self.step_controller(
                self.leader_cmd_q,
                self.leader_cmd_dq,
                self.current_leader_q,
                self.current_leader_dq,
                self.current_waypoint,
                self.true_controller_gain,
            )
            self.follower_cmd_q, self.follower_cmd_dq, self.last_u_follower = self.step_controller(
                self.follower_cmd_q,
                self.follower_cmd_dq,
                self.current_follower_q,
                self.current_follower_dq,
                self.current_waypoint,
                self.learned_controller_gain,
            )

            self.publish_position_command(self.pub_leader, self.leader_cmd_q)
            self.publish_position_command(self.pub_follower, self.follower_cmd_q)

            if self.phase_step_counter >= self.move_steps_per_waypoint:
                self.phase = "hold"
                self.phase_step_counter = 0
                self.freeze_hold_states()

        elif self.phase == "hold":
            self.freeze_hold_states()
            self.publish_position_command(self.pub_leader, self.current_waypoint)
            self.publish_position_command(self.pub_follower, self.current_waypoint)

            if self.phase_step_counter >= self.required_hold_steps():
                self.advance_waypoint()
                self.log_step(t)
                return

        self.log_step(t)


def main():
    rclpy.init()
    node = FreeModelDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not node.demo_finished:
                node.save_outputs()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
