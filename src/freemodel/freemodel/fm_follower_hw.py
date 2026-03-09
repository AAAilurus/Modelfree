#!/usr/bin/env python3
"""
fm_follower_hw.py  -  Phase 3: Follower — Hardware Version (SO101)

Hardware version of fm_follower.py.  Same algorithm, same style.

Key differences from simulation fm_follower.py:
  - Rate: 50 Hz (matches hardware nodes)
  - Loads K_learned.csv directly (not via Q.npy + DARE re-solve)
  - Default paths point to ~/Modelfree/freemodel_hw_out
  - Publishes to /so101/arm_position_controller/commands (consumed by follower_hw_node)

Does NOT know q_des — estimates goal by watching leader settle (same as simulation).
"""
import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _load_csv_matrix(path):
    """Header-safe numeric CSV loader (same helper used in fm_demo.py)."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    try:
        arr = np.loadtxt(path, delimiter=',', dtype=float)
    except Exception:
        arr = np.genfromtxt(path, delimiter=',', dtype=float, skip_header=1)
    arr = np.atleast_2d(arr)
    arr = arr[np.all(np.isfinite(arr), axis=1)]
    return np.array(arr, dtype=float)


class FMFollowerHW(Node):
    """
    Phase 3 Hardware: Follower uses learned K (from K_learned.csv) to replicate leader behavior.
    Does NOT know q_des — estimates goal by watching leader settle.
    """
    def __init__(self):
        super().__init__('fm_follower_hw')

        self.declare_parameter('rate_hz',          50.0)
        self.declare_parameter('k_learned_path',   '~/Modelfree/freemodel_hw_out/K_learned.csv')
        self.declare_parameter('joints',           ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('u_max',            5.0)
        self.declare_parameter('dq_cmd_max',       2.0)
        self.declare_parameter('log_dir',          '~/Modelfree/freemodel_hw_out/follower_logs')
        self.declare_parameter('stop_tol_q',       0.01)
        self.declare_parameter('stop_tol_dq',      0.02)
        self.declare_parameter('stop_hold_cycles', 50)
        self.declare_parameter('leader_vel_tol',   0.02)
        self.declare_parameter('goal_est_cycles',  30)

        self.dt              = 1.0 / float(self.get_parameter('rate_hz').value)
        self.k_path          = os.path.expanduser(
                                   str(self.get_parameter('k_learned_path').value))
        self.joints          = list(self.get_parameter('joints').value)
        self.u_max           = float(self.get_parameter('u_max').value)
        self.dq_cmd_max      = float(self.get_parameter('dq_cmd_max').value)
        self.log_dir         = os.path.expanduser(
                                   str(self.get_parameter('log_dir').value))
        self.stop_tol_q      = float(self.get_parameter('stop_tol_q').value)
        self.stop_tol_dq     = float(self.get_parameter('stop_tol_dq').value)
        self.stop_hold       = int(self.get_parameter('stop_hold_cycles').value)
        self.leader_vel_tol  = float(self.get_parameter('leader_vel_tol').value)
        self.goal_est_cycles = int(self.get_parameter('goal_est_cycles').value)

        # Load K_learned directly from CSV (fm_offline_spsa saves K_learned.csv)
        if not os.path.exists(self.k_path):
            raise RuntimeError(
                f"[fm_follower_hw] K_learned not found: {self.k_path}  "
                f"Run fm_offline_spsa.py first.")
        self.K = _load_csv_matrix(self.k_path).reshape(2, 4)

        self.get_logger().info(f"[fm_follower_hw] Loaded K:\n{np.round(self.K, 4)}")

        self.have_follower      = False
        self.q                  = np.zeros(2)
        self.dq                 = np.zeros(2)
        self.dq_cmd             = np.zeros(2)
        self._at_goal           = 0
        self.have_leader        = False
        self.leader_q           = np.zeros(2)
        self.leader_dq          = np.zeros(2)
        self.leader_still_count = 0
        self.q_des              = None
        self.goal_confirmed     = False
        self.log_q = []; self.log_e = []; self.log_u = []

        self.sub_leader   = self.create_subscription(
            JointState, '/so100/joint_states', self.cb_leader, 10)
        self.sub_follower = self.create_subscription(
            JointState, '/so101/joint_states', self.cb_follower, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/so101/arm_position_controller/commands', 10)
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info("[fm_follower_hw] Phase 3 ready — waiting for leader to settle...")

    def cb_leader(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        self.leader_q  = np.array([msg.position[i] for i in idx], dtype=float)
        self.leader_dq = np.array(
            [msg.velocity[i] if len(msg.velocity) > i else 0.0 for i in idx],
            dtype=float)
        self.have_leader = True

    def cb_follower(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        self.q  = np.array([msg.position[i] for i in idx], dtype=float)
        self.dq = np.array(
            [msg.velocity[i] if len(msg.velocity) > i else 0.0 for i in idx],
            dtype=float)
        self.have_follower = True

    def estimate_goal(self):
        if not self.have_leader or self.goal_confirmed:
            return
        vel = np.linalg.norm(self.leader_dq)
        if vel < self.leader_vel_tol:
            self.leader_still_count += 1
        else:
            self.leader_still_count = 0
        if self.leader_still_count >= self.goal_est_cycles:
            self.q_des          = self.leader_q.copy()
            self.goal_confirmed = True
            self.get_logger().info(
                f"[fm_follower_hw] ✓ Goal estimated from leader: {self.q_des.round(4)}")
            self.get_logger().info(
                "[fm_follower_hw] Starting to move SO101...")

    def step(self):
        self.estimate_goal()
        if not self.goal_confirmed or not self.have_follower:
            return

        e = np.hstack([self.q - self.q_des, self.dq])
        u = np.clip(-self.K @ e, -self.u_max, self.u_max)
        self.dq_cmd = np.clip(
            self.dq_cmd + u * self.dt, -self.dq_cmd_max, self.dq_cmd_max)
        q_cmd = self.q + self.dq_cmd * self.dt

        msg = Float64MultiArray()
        msg.data = q_cmd.tolist()
        self.pub.publish(msg)

        self.log_q.append(self.q.copy())
        self.log_e.append(e.copy())
        self.log_u.append(u.copy())

        pos_err = np.linalg.norm(self.q - self.q_des)
        vel_mag = np.linalg.norm(self.dq)
        if pos_err < self.stop_tol_q and vel_mag < self.stop_tol_dq:
            self._at_goal += 1
        else:
            self._at_goal = 0

        if self._at_goal >= self.stop_hold:
            self.get_logger().info(
                f"[fm_follower_hw] ✓ REACHED GOAL  pos_err={pos_err:.5f}  "
                f"vel={vel_mag:.5f}  estimated_goal={self.q_des.round(4)}")
            self.save_logs()
            raise SystemExit

    def save_logs(self):
        os.makedirs(self.log_dir, exist_ok=True)
        path = os.path.join(self.log_dir, 'follower_run.npz')
        np.savez(path,
                 q=np.array(self.log_q),
                 e=np.array(self.log_e),
                 u=np.array(self.log_u),
                 q_des=self.q_des if self.q_des is not None else np.zeros(2),
                 dt=self.dt)
        self.get_logger().info(f"[fm_follower_hw] Logs saved -> {path}")


def main():
    rclpy.init()
    node = FMFollowerHW()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try:
            node.save_logs()
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


if __name__ == '__main__':
    main()
