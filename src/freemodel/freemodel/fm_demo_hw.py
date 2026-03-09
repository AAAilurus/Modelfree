#!/usr/bin/env python3
"""
fm_demo_hw.py  -  Waypoint Tracking + Leader/Follower Comparison — Hardware Version

Hardware version of fm_demo.py.  Same algorithm, same style.

Leader:   SO100 uses K_star
Follower: SO101 uses K_learned

Key differences from simulation fm_demo.py:
  - Rate: 50 Hz (matches hardware nodes)
  - Default out_dir points to ~/Modelfree/freemodel_hw_out
  - Subscribes to /so100/joint_states and /so101/joint_states from hardware nodes
  - Publishes to /so100/arm_position_controller/commands (leader_hw_node)
    and /so101/arm_position_controller/commands (follower_hw_node)

Waypoints:
  waypoints_flat: [q1_1,q2_1,q1_2,q2_2,...]   (double_array)
  num_waypoints:  M                             (int)
  hold_secs:      hold duration per waypoint    (double)
  rate_hz:        control rate (double)
  settle_time_s:  send q_init repeatedly first to sync both arms (double)

State used in plots:
  state1 = q1 (joint1 position)
  state2 = q2 (joint2 position)
  state3 = dq1 (joint1 velocity)
  state4 = dq2 (joint2 velocity)

Control law (NO constraints, NO clamping):
  e = [q - q_des, dq]   (dq ignored if use_dq=False)
  u = -K @ e
  q_cmd = q + control_scale * u   (incremental position command)
"""
import os
import csv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_matrix_csv(path, delimiter=",", shape=None):
    """Header-safe numeric CSV loader."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    try:
        arr = np.loadtxt(path, delimiter=delimiter, dtype=float)
    except Exception:
        arr = np.genfromtxt(path, delimiter=delimiter, dtype=float, skip_header=1)

    arr = np.atleast_2d(arr)
    arr = arr[np.all(np.isfinite(arr), axis=1)]

    arr = np.array(arr, dtype=float)
    if shape is not None:
        arr = arr.reshape(shape)
    return arr


class FMDemoHW(Node):
    def __init__(self):
        super().__init__("fm_demo_hw")

        # --- Params
        self.ns_leader   = self.declare_parameter("ns_leader",   "/so100").value
        self.ns_follower = self.declare_parameter("ns_follower", "/so101").value

        self.joints = self.declare_parameter("joint_names", ["Shoulder_Pitch", "Elbow"]).value

        self.rate_hz       = float(self.declare_parameter("rate_hz",        50.0).value)
        self.control_scale = float(self.declare_parameter("control_scale",  0.005).value)
        self.use_dq        = bool(self.declare_parameter("use_dq",          False).value)

        self.num_waypoints  = int(self.declare_parameter("num_waypoints",  2).value)
        self.waypoints_flat = self.declare_parameter("waypoints_flat",     [0.0, 0.0, 0.5, -0.6]).value
        self.hold_secs      = float(self.declare_parameter("hold_secs",    3.0).value)
        self.settle_time_s  = float(self.declare_parameter("settle_time_s", 1.0).value)

        self.out_dir   = os.path.expanduser(
            self.declare_parameter("out_dir", "~/Modelfree/freemodel_hw_out").value)
        self.plots_dir = os.path.join(self.out_dir, "plots")
        self.logs_dir  = os.path.join(self.out_dir, "logs")
        os.makedirs(self.plots_dir, exist_ok=True)
        os.makedirs(self.logs_dir,  exist_ok=True)

        # --- Build waypoints list
        wp_flat = list(self.waypoints_flat)
        if len(wp_flat) < 2 * self.num_waypoints:
            raise RuntimeError(
                f"waypoints_flat length {len(wp_flat)} < 2*num_waypoints={2*self.num_waypoints}")

        self.waypoints = []
        for i in range(self.num_waypoints):
            self.waypoints.append([float(wp_flat[2*i]), float(wp_flat[2*i+1])])

        self.hold_steps   = int(round(self.hold_secs    * self.rate_hz))
        self.settle_steps = int(round(self.settle_time_s * self.rate_hz))

        Ts = 1.0 / self.rate_hz
        self.get_logger().info(
            f"[fm_demo_hw] rate_hz={self.rate_hz:.1f}  Ts={Ts:.3f}  "
            f"control_scale={self.control_scale}  use_dq={self.use_dq}"
        )
        self.get_logger().info(f"[fm_demo_hw] settle_time_s={self.settle_time_s}")
        self.get_logger().info(f"[fm_demo_hw] waypoints={self.waypoints}")
        self.get_logger().info(
            f"[fm_demo_hw] hold_steps={ [self.hold_steps]*len(self.waypoints) } (hold_secs each)")

        # --- Load K matrices (header-safe)
        K_star_path  = os.path.join(self.out_dir, "K_star.csv")
        K_learn_path = os.path.join(self.out_dir, "K_learned.csv")

        self.K_star  = load_matrix_csv(K_star_path,  shape=(2, 4))
        self.K_learn = load_matrix_csv(K_learn_path, shape=(2, 4))

        self.get_logger().info(
            f"[fm_demo_hw] ||K* - K_learned||_F = "
            f"{np.linalg.norm(self.K_star - self.K_learn):.6f}")

        # --- Topics
        self.sub_l = self.create_subscription(
            JointState, f"{self.ns_leader}/joint_states",   self.cb_leader,   10)
        self.sub_f = self.create_subscription(
            JointState, f"{self.ns_follower}/joint_states", self.cb_follower, 10)

        self.pub_l = self.create_publisher(
            Float64MultiArray, f"{self.ns_leader}/arm_position_controller/commands",   10)
        self.pub_f = self.create_publisher(
            Float64MultiArray, f"{self.ns_follower}/arm_position_controller/commands", 10)

        # --- State buffers
        self.have_l = False
        self.have_f = False
        self.q_l    = np.zeros(2)
        self.q_f    = np.zeros(2)
        self.dq_l   = np.zeros(2)
        self.dq_f   = np.zeros(2)

        # --- Logging
        self.log_t   = []
        self.log_wp  = []
        self.log_des = []
        self.log_lq  = []
        self.log_ldq = []
        self.log_lu  = []
        self.log_fq  = []
        self.log_fdq = []
        self.log_fu  = []

        # --- Waypoint scheduler
        self.wp_idx   = 0
        self.wp_count = 0
        self.q_des    = np.array(self.waypoints[self.wp_idx], dtype=float)

        self.tick = 0
        self.Ts   = Ts

        # Timer
        self.timer = self.create_timer(self.Ts, self.step)

        self.get_logger().info(
            f"[fm_demo_hw] Sending q_init={np.array(self.waypoints[0])} during settle...")

    def cb_leader(self, msg: JointState):
        self._extract_joint_state(msg, is_leader=True)

    def cb_follower(self, msg: JointState):
        self._extract_joint_state(msg, is_leader=False)

    def _extract_joint_state(self, msg: JointState, is_leader: bool):
        # Map joints by name
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q  = np.zeros(2)
        dq = np.zeros(2)

        ok = True
        for j, jname in enumerate(self.joints):
            if jname not in name_to_idx:
                ok = False
                break
            idx  = name_to_idx[jname]
            q[j]  = float(msg.position[idx]) if idx < len(msg.position) else 0.0
            dq[j] = float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0

        if not ok:
            return

        if is_leader:
            self.q_l,  self.dq_l  = q, dq
            self.have_l = True
        else:
            self.q_f,  self.dq_f  = q, dq
            self.have_f = True

    def _publish_cmds(self, qcmd_l, qcmd_f):
        ml = Float64MultiArray()
        mf = Float64MultiArray()
        ml.data = [float(qcmd_l[0]), float(qcmd_l[1])]
        mf.data = [float(qcmd_f[0]), float(qcmd_f[1])]
        self.pub_l.publish(ml)
        self.pub_f.publish(mf)

    def step(self):
        # Need both states first
        if not (self.have_l and self.have_f):
            return

        # --- settle phase: force both to first waypoint for a bit
        if self.tick < self.settle_steps:
            q_init = np.array(self.waypoints[0], dtype=float)
            self._publish_cmds(q_init, q_init)
            self.tick += 1
            return

        # --- waypoint switching
        if self.wp_count >= self.hold_steps:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.get_logger().info("[fm_demo_hw] Done - saving logs and plots.")
                self.save_and_plot()
                self.timer.cancel()
                return
            self.q_des    = np.array(self.waypoints[self.wp_idx], dtype=float)
            self.wp_count = 0
            self.get_logger().info(
                f"[fm_demo_hw] -> waypoint {self.wp_idx+1}/{len(self.waypoints)}  "
                f"q_des={self.q_des}")

        # --- Control
        if self.use_dq:
            e_l = np.hstack([self.q_l - self.q_des, self.dq_l])
            e_f = np.hstack([self.q_f - self.q_des, self.dq_f])
        else:
            e_l = np.hstack([self.q_l - self.q_des, np.zeros(2)])
            e_f = np.hstack([self.q_f - self.q_des, np.zeros(2)])

        u_l = -self.K_star  @ e_l
        u_f = -self.K_learn @ e_f

        qcmd_l = self.q_l + self.control_scale * u_l
        qcmd_f = self.q_f + self.control_scale * u_f

        self._publish_cmds(qcmd_l, qcmd_f)

        # --- Log
        t = (self.tick - self.settle_steps) * self.Ts
        self.log_t.append(float(t))
        self.log_wp.append(int(self.wp_idx))
        self.log_des.append(self.q_des.copy())

        self.log_lq.append(self.q_l.copy())
        self.log_ldq.append(self.dq_l.copy())
        self.log_lu.append(u_l.copy())

        self.log_fq.append(self.q_f.copy())
        self.log_fdq.append(self.dq_f.copy())
        self.log_fu.append(u_f.copy())

        self.tick     += 1
        self.wp_count += 1

    def _plot_4state_grid(self, t, leader_q, leader_dq, foll_q, foll_dq, out_png):
        fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        fig.suptitle("Leader vs Follower \nLeader=blue dotted, Follower=orange solid",
                     fontsize=15, fontweight="bold")

        # follower orange solid (slightly transparent)
        follower_style = dict(color="tab:orange", lw=3.0, ls="-", alpha=0.90, zorder=5)
        # leader blue dotted ON TOP
        leader_style   = dict(color="tab:blue",   lw=3.2, ls=":", alpha=1.00, zorder=10)

        # q1
        ax[0,0].plot(t, foll_q[:,0],  label="Follower (K_learned)", **follower_style)
        ax[0,0].plot(t, leader_q[:,0], label="Leader (K*)",          **leader_style)
        ax[0,0].set_ylabel("q1 (rad)")
        ax[0,0].grid(True, alpha=0.3)

        # q2
        ax[0,1].plot(t, foll_q[:,1],  label="Follower (K_learned)", **follower_style)
        ax[0,1].plot(t, leader_q[:,1], label="Leader (K*)",          **leader_style)
        ax[0,1].set_ylabel("q2 (rad)")
        ax[0,1].grid(True, alpha=0.3)

        # dq1
        ax[1,0].plot(t, foll_dq[:,0],  label="Follower (K_learned)", **follower_style)
        ax[1,0].plot(t, leader_dq[:,0], label="Leader (K*)",           **leader_style)
        ax[1,0].set_ylabel("dq1 (rad/s)")
        ax[1,0].set_xlabel("time (s)")
        ax[1,0].grid(True, alpha=0.3)

        # dq2
        ax[1,1].plot(t, foll_dq[:,1],  label="Follower (K_learned)", **follower_style)
        ax[1,1].plot(t, leader_dq[:,1], label="Leader (K*)",           **leader_style)
        ax[1,1].set_ylabel("dq2 (rad/s)")
        ax[1,1].set_xlabel("time (s)")
        ax[1,1].grid(True, alpha=0.3)

        # one legend for whole fig
        handles, labels = ax[0,0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper right", frameon=True)

        plt.tight_layout()
        plt.savefig(out_png, dpi=200)
        plt.close(fig)

    def save_and_plot(self):
        leader_q  = np.array(self.log_lq,  dtype=float)
        leader_dq = np.array(self.log_ldq, dtype=float)
        foll_q    = np.array(self.log_fq,  dtype=float)
        foll_dq   = np.array(self.log_fdq, dtype=float)
        t         = np.array(self.log_t,   dtype=float)

        # CSV
        csv_path = os.path.join(self.logs_dir, "follower_waypoints_trajectory.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "t", "wp_idx", "des_q1", "des_q2",
                "leader_q1", "leader_q2", "leader_dq1", "leader_dq2",
                "foll_q1",   "foll_q2",   "foll_dq1",   "foll_dq2",
            ])
            for k in range(len(t)):
                w.writerow([
                    float(t[k]),             int(self.log_wp[k]),
                    float(self.log_des[k][0]), float(self.log_des[k][1]),
                    float(leader_q[k,0]),    float(leader_q[k,1]),
                    float(leader_dq[k,0]),   float(leader_dq[k,1]),
                    float(foll_q[k,0]),      float(foll_q[k,1]),
                    float(foll_dq[k,0]),     float(foll_dq[k,1]),
                ])

        # Plot
        out_png = os.path.join(self.plots_dir, "leader_vs_follower_4state_waypoints.png")
        self._plot_4state_grid(t, leader_q, leader_dq, foll_q, foll_dq, out_png)

        self.get_logger().info(f"[fm_demo_hw] CSV   -> {csv_path}")
        self.get_logger().info(f"[fm_demo_hw] Plot  -> {out_png}")


def main():
    rclpy.init()
    node = None
    try:
        node = FMDemoHW()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
