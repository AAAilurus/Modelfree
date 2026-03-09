#!/usr/bin/env python3
"""
fm_leader_hw.py  -  Phase 1: Data Collection — Hardware Version (SO100 only)

Hardware version of fm_leader.py.  Works with real Feetech servos through
leader_hw_node (which must be started with enable_commands:=true so the arm
physically moves to the commanded positions).

rate_hz=50 -> Ts=0.02 -> same Ad, Bd, K_star as the simulation version.
Dataset is built from a simulated linear model rollout (Ad @ e + Bd @ u),
NOT from real sensor feedback — identical algorithm to fm_leader.py.

settle_steps: at startup the node reads the current servo position and
sends it repeatedly for `settle_steps` ticks so the arm can reach q_des
before data collection begins.
"""
import os
import csv
import numpy as np
import scipy.linalg
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class FMLeaderHW(Node):
    def __init__(self):
        super().__init__('fm_leader_hw')

        self.declare_parameter('rate_hz',      50.0)
        self.declare_parameter('q_des',        [0.5, -0.6])
        self.declare_parameter('noise_std',    0.05)
        self.declare_parameter('num_traj',     50)
        self.declare_parameter('T_each',       300)
        self.declare_parameter('sigma_q',      0.4)
        self.declare_parameter('joint_names',  ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('save_dir',     '~/Modelfree/freemodel_hw_out')
        self.declare_parameter('settle_steps', 50)

        self.dt           = 1.0 / float(self.get_parameter('rate_hz').value)
        self.q_des        = np.array(self.get_parameter('q_des').value,   dtype=float)
        self.noise_std    = float(self.get_parameter('noise_std').value)
        self.num_traj     = int(self.get_parameter('num_traj').value)
        self.T_each       = int(self.get_parameter('T_each').value)
        self.sigma_q      = float(self.get_parameter('sigma_q').value)
        self.joints       = list(self.get_parameter('joint_names').value)
        self.save_dir     = os.path.expanduser(str(self.get_parameter('save_dir').value))
        self.settle_steps = int(self.get_parameter('settle_steps').value)

        Ts = self.dt  # 0.02 - matches lqr_with_Q and simulation version
        self.Ad = np.array([[1, 0, Ts, 0 ],
                            [0, 1, 0,  Ts],
                            [0, 0, 1,  0 ],
                            [0, 0, 0,  1 ]], dtype=float)
        self.Bd = np.array([[0.5*Ts**2, 0        ],
                            [0,         0.5*Ts**2],
                            [Ts,        0        ],
                            [0,         Ts       ]], dtype=float)

        Q_star = np.diag([100., 100., 10., 10.])
        R_star = np.diag([0.5, 0.5])
        P = scipy.linalg.solve_discrete_are(self.Ad, self.Bd, Q_star, R_star)
        self.K_star = np.linalg.solve(
            R_star + self.Bd.T @ P @ self.Bd,
            self.Bd.T @ P @ self.Ad)

        self.have_js      = False
        self.q            = np.zeros(2)
        self.dq           = np.zeros(2)
        self.e            = None
        self.traj_idx     = 0
        self.step_idx     = 0
        self._settle_tick = 0   # counts ticks during settle phase

        self.Ek  = []
        self.Uk  = []
        self.Ek1 = []
        self.Uk1 = []

        np.random.seed(42)

        self.sub = self.create_subscription(
            JointState,
            '/so100/joint_states',
            self.cb_js, 10)
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/so100/arm_position_controller/commands', 10)
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f"[fm_leader_hw] Ts={Ts}  num_traj={self.num_traj}  "
            f"T_each={self.T_each}  total_samples={self.num_traj * self.T_each}  "
            f"settle_steps={self.settle_steps}")
        self.get_logger().info(
            f"[fm_leader_hw] K_star:\n{np.round(self.K_star, 4)}")
        self.get_logger().info(
            "[fm_leader_hw] Waiting for /so100/joint_states from leader_hw_node ...")

    def cb_js(self, msg):
        # Sensor read only used as liveness gate and for settle phase.
        # Dataset is built from simulated rollout, NOT sensor.
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        self.q  = np.array([msg.position[i] for i in idx], dtype=float)
        self.dq = np.array(
            [msg.velocity[i] if len(msg.velocity) > i else 0.0 for i in idx],
            dtype=float)
        self.have_js = True

    def _send(self, q_cmd):
        m = Float64MultiArray()
        m.data = list(q_cmd)
        self.pub.publish(m)

    def _start_new_traj(self):
        # Random position error, ZERO initial velocity
        self.e = np.array([
            self.sigma_q * np.random.randn(),
            self.sigma_q * np.random.randn(),
            0.0,
            0.0,
        ], dtype=float)
        self.step_idx = 0
        q0 = self.q_des + self.e[0:2]
        self._send(q0)
        self.get_logger().info(
            f"[fm_leader_hw] traj {self.traj_idx + 1}/{self.num_traj}  "
            f"e0={np.round(self.e, 3)}")

    def step(self):
        if not self.have_js:
            return

        # Settle phase: send q_des repeatedly until the arm reaches it
        if self._settle_tick < self.settle_steps:
            self._send(self.q_des)
            self._settle_tick += 1
            if self._settle_tick == self.settle_steps:
                self.get_logger().info(
                    f"[fm_leader_hw] Settle complete — starting data collection "
                    f"q_des={self.q_des}")
            return

        if self.traj_idx >= self.num_traj:
            self._save_all()
            self.get_logger().info(
                "[fm_leader_hw] DONE - run fm_offline_spsa.py next")
            raise SystemExit

        if self.e is None:
            self._start_new_traj()
            return

        eta = self.noise_std * np.random.randn(2)
        u   = -self.K_star @ self.e + eta

        e2   = self.Ad @ self.e + self.Bd @ u

        eta2 = self.noise_std * np.random.randn(2)
        u2   = -self.K_star @ e2 + eta2

        self.Ek.append(self.e.copy())
        self.Uk.append(u.copy())
        self.Ek1.append(e2.copy())
        self.Uk1.append(u2.copy())

        self._send(self.q_des + e2[0:2])

        self.e = e2
        self.step_idx += 1

        if self.step_idx % 100 == 0:
            self.get_logger().info(
                f"[fm_leader_hw] traj {self.traj_idx + 1}  "
                f"step {self.step_idx}/{self.T_each}  "
                f"|e|={np.linalg.norm(self.e):.4f}")

        if self.step_idx >= self.T_each:
            self.traj_idx += 1
            self.e = None

    def _save_all(self):
        os.makedirs(self.save_dir, exist_ok=True)

        def _csv(path, rows, header):
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(rows)

        _csv(os.path.join(self.save_dir, 'Ek.csv'),
             self.Ek,  ['e1', 'e2', 'e3', 'e4'])
        _csv(os.path.join(self.save_dir, 'Uk.csv'),
             self.Uk,  ['u1', 'u2'])
        _csv(os.path.join(self.save_dir, 'Ek1.csv'),
             self.Ek1, ['e1', 'e2', 'e3', 'e4'])
        _csv(os.path.join(self.save_dir, 'Uk1.csv'),
             self.Uk1, ['u1', 'u2'])

        with open(os.path.join(self.save_dir, 'K_star.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['k1', 'k2', 'k3', 'k4'])
            for row in self.K_star:
                w.writerow([float(x) for x in row])

        self.get_logger().info(
            f"[fm_leader_hw] Saved {len(self.Ek)} samples -> {self.save_dir}")


def main():
    rclpy.init()
    node = FMLeaderHW()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try:
            node._save_all()
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
