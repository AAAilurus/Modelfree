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
    matrix_data = np.genfromtxt(file_path, delimiter=",", dtype=float, skip_header=1)
    matrix_data = np.atleast_2d(matrix_data)
    matrix_data = matrix_data[np.all(np.isfinite(matrix_data), axis=1)]
    if shape is not None:
        matrix_data = matrix_data.reshape(shape)
    return matrix_data


class FreeModelDemo(Node):
    def __init__(self):
        super().__init__("fm_demo")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("initial_joint_position", [0.0, 0.0])
        self.declare_parameter(
            "target_joint_positions_flat",
            [0.5, -0.6, 0.3, 0.4, -0.4, 0.5, -0.5, -0.2]
        )
        self.declare_parameter("initial_settle_time_seconds", 2.0)
        self.declare_parameter("time_per_target_seconds", 4.0)
        self.declare_parameter("hold_time_at_target_seconds", 1.0)
        self.declare_parameter("return_to_initial_time_seconds", 3.0)
        self.declare_parameter("hold_time_at_initial_seconds", 1.0)
        self.declare_parameter("transition_time_seconds", 1.0)
        self.declare_parameter("position_step_scale", 0.005)
        self.declare_parameter("trajectory_command_duration_seconds", 0.1)
        self.declare_parameter("output_directory", "/root/Modelfree/freemodel_out")
        self.declare_parameter("joint_names", ["Shoulder_Pitch", "Elbow"])

        self.control_period_seconds = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.initial_joint_position = np.array(self.get_parameter("initial_joint_position").value, dtype=float)
        self.initial_settle_time_seconds = float(self.get_parameter("initial_settle_time_seconds").value)
        self.time_per_target_seconds = float(self.get_parameter("time_per_target_seconds").value)
        self.hold_time_at_target_seconds = float(self.get_parameter("hold_time_at_target_seconds").value)
        self.return_to_initial_time_seconds = float(self.get_parameter("return_to_initial_time_seconds").value)
        self.hold_time_at_initial_seconds = float(self.get_parameter("hold_time_at_initial_seconds").value)
        self.transition_time_seconds = float(self.get_parameter("transition_time_seconds").value)
        self.position_step_scale = float(self.get_parameter("position_step_scale").value)
        self.trajectory_command_duration_seconds = float(
            self.get_parameter("trajectory_command_duration_seconds").value
        )
        self.output_directory = str(self.get_parameter("output_directory").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        target_joint_positions_flat = list(self.get_parameter("target_joint_positions_flat").value)
        if len(target_joint_positions_flat) % 2 != 0:
            raise RuntimeError("target_joint_positions_flat must contain an even number of values")

        self.target_joint_positions_list = [
            np.array(target_joint_positions_flat[index:index + 2], dtype=float)
            for index in range(0, len(target_joint_positions_flat), 2)
        ]

        self.initial_settle_steps = int(round(self.initial_settle_time_seconds / self.control_period_seconds))
        self.time_per_target_steps = int(round(self.time_per_target_seconds / self.control_period_seconds))
        self.hold_time_at_target_steps = int(round(self.hold_time_at_target_seconds / self.control_period_seconds))
        self.return_to_initial_steps = int(round(self.return_to_initial_time_seconds / self.control_period_seconds))
        self.hold_time_at_initial_steps = int(round(self.hold_time_at_initial_seconds / self.control_period_seconds))
        self.transition_steps = max(1, int(round(self.transition_time_seconds / self.control_period_seconds)))

        if self.hold_time_at_target_steps >= self.time_per_target_steps:
            raise RuntimeError("hold_time_at_target_seconds must be smaller than time_per_target_seconds")
        if self.hold_time_at_initial_steps >= self.return_to_initial_steps:
            raise RuntimeError("hold_time_at_initial_seconds must be smaller than return_to_initial_time_seconds")

        self.true_controller_gain = load_matrix_csv(
            os.path.join(self.output_directory, "K_star.csv"),
            shape=(2, 4)
        )
        self.learned_controller_gain = load_matrix_csv(
            os.path.join(self.output_directory, "K_learned.csv"),
            shape=(2, 4)
        )

        self.current_leader_joint_position = np.zeros(2, dtype=float)
        self.current_leader_joint_velocity = np.zeros(2, dtype=float)
        self.current_follower_joint_position = np.zeros(2, dtype=float)
        self.current_follower_joint_velocity = np.zeros(2, dtype=float)

        self.received_leader_joint_state = False
        self.received_follower_joint_state = False

        self.demo_phase = "move_to_initial_position"
        self.current_target_index = 0
        self.phase_step_counter = 0
        self.demo_finished = False
        self.transition_start_joint_position = None
        self.transition_end_joint_position = None

        self.logged_time = []
        self.logged_desired_joint_position = []
        self.logged_leader_joint_position = []
        self.logged_leader_joint_velocity = []
        self.logged_follower_joint_position = []
        self.logged_follower_joint_velocity = []
        self.logged_phase = []

        self.plot_directory = os.path.join(self.output_directory, "plots")
        self.log_directory = os.path.join(self.output_directory, "logs")
        os.makedirs(self.plot_directory, exist_ok=True)
        os.makedirs(self.log_directory, exist_ok=True)

        self.create_subscription(
            JointState,
            "/so100/joint_states",
            self.leader_joint_state_callback,
            10
        )
        self.create_subscription(
            JointState,
            "/so101/joint_states",
            self.follower_joint_state_callback,
            10
        )

        self.leader_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/so100/joint_trajectory_controller/joint_trajectory",
            10
        )
        self.follower_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/so101/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.control_timer = self.create_timer(self.control_period_seconds, self.run_demo_step)

        self.get_logger().info(
            f"[fm_demo] initial_joint_position={self.initial_joint_position.tolist()} "
            f"target_joint_positions_list={[target.tolist() for target in self.target_joint_positions_list]}"
        )

    def extract_selected_joint_state(self, joint_state_message):
        try:
            selected_joint_indices = [joint_state_message.name.index(joint_name) for joint_name in self.joint_names]
        except ValueError:
            return None, None

        selected_joint_position = np.array(
            [joint_state_message.position[index] for index in selected_joint_indices],
            dtype=float
        )
        selected_joint_velocity = np.array(
            [
                joint_state_message.velocity[index] if index < len(joint_state_message.velocity) else 0.0
                for index in selected_joint_indices
            ],
            dtype=float
        )
        return selected_joint_position, selected_joint_velocity

    def leader_joint_state_callback(self, joint_state_message):
        selected_joint_position, selected_joint_velocity = self.extract_selected_joint_state(joint_state_message)
        if selected_joint_position is not None:
            self.current_leader_joint_position = selected_joint_position
            self.current_leader_joint_velocity = selected_joint_velocity
            self.received_leader_joint_state = True

    def follower_joint_state_callback(self, joint_state_message):
        selected_joint_position, selected_joint_velocity = self.extract_selected_joint_state(joint_state_message)
        if selected_joint_position is not None:
            self.current_follower_joint_position = selected_joint_position
            self.current_follower_joint_velocity = selected_joint_velocity
            self.received_follower_joint_state = True

    def publish_joint_position_command(self, trajectory_publisher, commanded_joint_position):
        trajectory_message = JointTrajectory()
        trajectory_message.joint_names = list(self.joint_names)

        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = commanded_joint_position.tolist()
        trajectory_point.velocities = [0.0, 0.0]
        trajectory_point.time_from_start.sec = int(self.trajectory_command_duration_seconds)
        trajectory_point.time_from_start.nanosec = int(
            (self.trajectory_command_duration_seconds - int(self.trajectory_command_duration_seconds)) * 1e9
        )

        trajectory_message.points = [trajectory_point]
        trajectory_publisher.publish(trajectory_message)

    def is_hold_subphase_for_target(self):
        return self.phase_step_counter >= (self.time_per_target_steps - self.hold_time_at_target_steps)

    def is_hold_subphase_for_return(self):
        return self.phase_step_counter >= (self.return_to_initial_steps - self.hold_time_at_initial_steps)

    def get_interpolated_target_joint_position(self):
        alpha = min(1.0, self.phase_step_counter / max(1, self.transition_steps))
        return (1.0 - alpha) * self.transition_start_joint_position + alpha * self.transition_end_joint_position

    def save_logged_data_and_plot(self):
        if len(self.logged_time) == 0:
            return

        time_array = np.array(self.logged_time)
        desired_joint_position_array = np.array(self.logged_desired_joint_position)
        leader_joint_position_array = np.array(self.logged_leader_joint_position)
        leader_joint_velocity_array = np.array(self.logged_leader_joint_velocity)
        follower_joint_position_array = np.array(self.logged_follower_joint_position)
        follower_joint_velocity_array = np.array(self.logged_follower_joint_velocity)

        csv_path = os.path.join(self.log_directory, "four_state_demo_log.csv")
        with open(csv_path, "w", newline="") as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                "time",
                "desired_q1", "desired_q2",
                "leader_q1", "leader_q2",
                "leader_dq1", "leader_dq2",
                "follower_q1", "follower_q2",
                "follower_dq1", "follower_dq2",
                "phase"
            ])
            for index in range(len(time_array)):
                csv_writer.writerow([
                    time_array[index],
                    desired_joint_position_array[index, 0], desired_joint_position_array[index, 1],
                    leader_joint_position_array[index, 0], leader_joint_position_array[index, 1],
                    leader_joint_velocity_array[index, 0], leader_joint_velocity_array[index, 1],
                    follower_joint_position_array[index, 0], follower_joint_position_array[index, 1],
                    follower_joint_velocity_array[index, 0], follower_joint_velocity_array[index, 1],
                    self.logged_phase[index]
                ])

        figure, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        figure.suptitle("Leader vs Follower Four-State Graph", fontsize=15, fontweight="bold")

        axes[0, 0].plot(time_array, leader_joint_position_array[:, 0], label="Leader q1")
        axes[0, 0].plot(time_array, follower_joint_position_array[:, 0], "--", label="Follower q1")
        axes[0, 0].plot(time_array, desired_joint_position_array[:, 0], ":", label="Desired q1")
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()

        axes[0, 1].plot(time_array, leader_joint_position_array[:, 1], label="Leader q2")
        axes[0, 1].plot(time_array, follower_joint_position_array[:, 1], "--", label="Follower q2")
        axes[0, 1].plot(time_array, desired_joint_position_array[:, 1], ":", label="Desired q2")
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()

        axes[1, 0].plot(time_array, leader_joint_velocity_array[:, 0], label="Leader dq1")
        axes[1, 0].plot(time_array, follower_joint_velocity_array[:, 0], "--", label="Follower dq1")
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend()

        axes[1, 1].plot(time_array, leader_joint_velocity_array[:, 1], label="Leader dq2")
        axes[1, 1].plot(time_array, follower_joint_velocity_array[:, 1], "--", label="Follower dq2")
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend()

        plt.tight_layout()
        plot_path = os.path.join(self.plot_directory, "leader_follower_four_state_graph.png")
        plt.savefig(plot_path, dpi=200)
        plt.close(figure)

        self.get_logger().info(f"[fm_demo] saved log -> {csv_path}")
        self.get_logger().info(f"[fm_demo] saved plot -> {plot_path}")

    def run_demo_step(self):
        if not (self.received_leader_joint_state and self.received_follower_joint_state):
            return

        if self.demo_finished:
            return

        current_time_seconds = len(self.logged_time) * self.control_period_seconds

        if self.demo_phase == "move_to_initial_position":
            self.publish_joint_position_command(
                self.leader_trajectory_publisher,
                self.initial_joint_position
            )
            self.publish_joint_position_command(
                self.follower_trajectory_publisher,
                self.initial_joint_position
            )

            self.logged_time.append(current_time_seconds)
            self.logged_desired_joint_position.append(self.initial_joint_position.copy())
            self.logged_leader_joint_position.append(self.current_leader_joint_position.copy())
            self.logged_leader_joint_velocity.append(self.current_leader_joint_velocity.copy())
            self.logged_follower_joint_position.append(self.current_follower_joint_position.copy())
            self.logged_follower_joint_velocity.append(self.current_follower_joint_velocity.copy())
            self.logged_phase.append("initial_settle")

            self.phase_step_counter += 1
            if self.phase_step_counter >= self.initial_settle_steps:
                self.demo_phase = "transition_to_target"
                self.phase_step_counter = 0
                self.current_target_index = 0
                self.transition_start_joint_position = self.initial_joint_position.copy()
                self.transition_end_joint_position = self.target_joint_positions_list[0].copy()
                self.get_logger().info(
                    f"[fm_demo] transitioning to target 1/{len(self.target_joint_positions_list)} "
                    f"{self.target_joint_positions_list[0].tolist()}"
                )
            return

        if self.demo_phase == "transition_to_target":
            current_target_joint_position = self.get_interpolated_target_joint_position()

            leader_state_error = np.hstack([
                self.current_leader_joint_position - current_target_joint_position,
                self.current_leader_joint_velocity
            ])
            follower_state_error = np.hstack([
                self.current_follower_joint_position - current_target_joint_position,
                self.current_follower_joint_velocity
            ])

            leader_control_input = -self.true_controller_gain @ leader_state_error
            # limit acceleration for smooth motion
            max_acc = 0.15
            leader_control_input = np.clip(leader_control_input, -max_acc, max_acc)
            follower_control_input = -self.learned_controller_gain @ follower_state_error
            follower_control_input = np.clip(follower_control_input, -max_acc, max_acc)

            leader_commanded_joint_position = (
                self.current_leader_joint_position
                + self.position_step_scale * leader_control_input
            )
            follower_commanded_joint_position = (
                self.current_follower_joint_position
                + self.position_step_scale * follower_control_input
            )

            self.publish_joint_position_command(
                self.leader_trajectory_publisher,
                leader_commanded_joint_position
            )
            self.publish_joint_position_command(
                self.follower_trajectory_publisher,
                follower_commanded_joint_position
            )

            self.logged_time.append(current_time_seconds)
            self.logged_desired_joint_position.append(current_target_joint_position.copy())
            self.logged_leader_joint_position.append(self.current_leader_joint_position.copy())
            self.logged_leader_joint_velocity.append(self.current_leader_joint_velocity.copy())
            self.logged_follower_joint_position.append(self.current_follower_joint_position.copy())
            self.logged_follower_joint_velocity.append(self.current_follower_joint_velocity.copy())
            self.logged_phase.append("target_transition")

            self.phase_step_counter += 1
            if self.phase_step_counter >= self.transition_steps:
                self.demo_phase = "move_through_targets"
                self.phase_step_counter = 0
                self.get_logger().info(
                    f"[fm_demo] starting hold/move on target {self.current_target_index + 1}/{len(self.target_joint_positions_list)} "
                    f"{self.target_joint_positions_list[self.current_target_index].tolist()}"
                )
            return

        if self.demo_phase == "move_through_targets":
            current_target_joint_position = self.target_joint_positions_list[self.current_target_index]

            if self.is_hold_subphase_for_target():
                leader_commanded_joint_position = current_target_joint_position.copy()
                follower_commanded_joint_position = current_target_joint_position.copy()
                phase_name = "target_hold"
            else:
                leader_state_error = np.hstack([
                    self.current_leader_joint_position - current_target_joint_position,
                    self.current_leader_joint_velocity
                ])
                follower_state_error = np.hstack([
                    self.current_follower_joint_position - current_target_joint_position,
                    self.current_follower_joint_velocity
                ])

                leader_control_input = -self.true_controller_gain @ leader_state_error
                follower_control_input = -self.learned_controller_gain @ follower_state_error

                leader_commanded_joint_position = (
                    self.current_leader_joint_position
                    + self.position_step_scale * leader_control_input
                )
                follower_commanded_joint_position = (
                    self.current_follower_joint_position
                    + self.position_step_scale * follower_control_input
                )
                phase_name = "target_move"

            self.publish_joint_position_command(
                self.leader_trajectory_publisher,
                leader_commanded_joint_position
            )
            self.publish_joint_position_command(
                self.follower_trajectory_publisher,
                follower_commanded_joint_position
            )

            if self.phase_step_counter % max(1, int(1.0 / self.control_period_seconds)) == 0:
                subphase_name = "hold" if self.is_hold_subphase_for_target() else "move"
                self.get_logger().info(
                    f"[fm_demo] target {self.current_target_index + 1}/{len(self.target_joint_positions_list)} "
                    f"subphase={subphase_name} "
                    f"target_joint_position={np.round(current_target_joint_position, 3)} "
                    f"leader_joint_position={np.round(self.current_leader_joint_position, 3)} "
                    f"follower_joint_position={np.round(self.current_follower_joint_position, 3)}"
                )

            self.logged_time.append(current_time_seconds)
            self.logged_desired_joint_position.append(current_target_joint_position.copy())
            self.logged_leader_joint_position.append(self.current_leader_joint_position.copy())
            self.logged_leader_joint_velocity.append(self.current_leader_joint_velocity.copy())
            self.logged_follower_joint_position.append(self.current_follower_joint_position.copy())
            self.logged_follower_joint_velocity.append(self.current_follower_joint_velocity.copy())
            self.logged_phase.append(phase_name)

            self.phase_step_counter += 1
            if self.phase_step_counter >= self.time_per_target_steps:
                self.current_target_index += 1
                self.phase_step_counter = 0

                if self.current_target_index >= len(self.target_joint_positions_list):
                    self.demo_phase = "return_to_initial_position"
                    self.get_logger().info("[fm_demo] all targets completed -> returning to initial_joint_position")
                else:
                    self.demo_phase = "transition_to_target"
                    self.transition_start_joint_position = self.target_joint_positions_list[self.current_target_index - 1].copy()
                    self.transition_end_joint_position = self.target_joint_positions_list[self.current_target_index].copy()
                    self.get_logger().info(
                        f"[fm_demo] transitioning to target {self.current_target_index + 1}/{len(self.target_joint_positions_list)} "
                        f"{self.target_joint_positions_list[self.current_target_index].tolist()}"
                    )
            return

        if self.demo_phase == "return_to_initial_position":
            if self.is_hold_subphase_for_return():
                leader_commanded_joint_position = self.initial_joint_position.copy()
                follower_commanded_joint_position = self.initial_joint_position.copy()
                phase_name = "return_hold"
            else:
                leader_state_error = np.hstack([
                    self.current_leader_joint_position - self.initial_joint_position,
                    self.current_leader_joint_velocity
                ])
                follower_state_error = np.hstack([
                    self.current_follower_joint_position - self.initial_joint_position,
                    self.current_follower_joint_velocity
                ])

                leader_control_input = -self.true_controller_gain @ leader_state_error
                follower_control_input = -self.learned_controller_gain @ follower_state_error

                leader_commanded_joint_position = (
                    self.current_leader_joint_position
                    + self.position_step_scale * leader_control_input
                )
                follower_commanded_joint_position = (
                    self.current_follower_joint_position
                    + self.position_step_scale * follower_control_input
                )
                phase_name = "return_move"

            self.publish_joint_position_command(
                self.leader_trajectory_publisher,
                leader_commanded_joint_position
            )
            self.publish_joint_position_command(
                self.follower_trajectory_publisher,
                follower_commanded_joint_position
            )

            if self.phase_step_counter % max(1, int(1.0 / self.control_period_seconds)) == 0:
                subphase_name = "hold" if self.is_hold_subphase_for_return() else "move"
                self.get_logger().info(
                    f"[fm_demo] returning_to_initial_position "
                    f"subphase={subphase_name} "
                    f"leader_joint_position={np.round(self.current_leader_joint_position, 3)} "
                    f"follower_joint_position={np.round(self.current_follower_joint_position, 3)}"
                )

            self.logged_time.append(current_time_seconds)
            self.logged_desired_joint_position.append(self.initial_joint_position.copy())
            self.logged_leader_joint_position.append(self.current_leader_joint_position.copy())
            self.logged_leader_joint_velocity.append(self.current_leader_joint_velocity.copy())
            self.logged_follower_joint_position.append(self.current_follower_joint_position.copy())
            self.logged_follower_joint_velocity.append(self.current_follower_joint_velocity.copy())
            self.logged_phase.append(phase_name)

            self.phase_step_counter += 1
            if self.phase_step_counter >= self.return_to_initial_steps:
                self.demo_phase = "done"
                self.demo_finished = True
                self.control_timer.cancel()
                self.save_logged_data_and_plot()
                self.get_logger().info("[fm_demo] demo completed")
            return


def main():
    rclpy.init()
    demo_node = FreeModelDemo()
    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not demo_node.demo_finished:
                demo_node.save_logged_data_and_plot()
        except Exception:
            pass
        try:
            demo_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()