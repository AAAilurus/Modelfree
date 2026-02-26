#!/usr/bin/env python3
import os, json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class FMPipeline(Node):
    """
    Pure record-only logger.
    Watches leader (SO100) and saves (Ek, Uk, Ek1, Uk1) pairs to disk.
    Not needed for learning (SPSA uses simulation data).
    Useful for analysis and verification.
    """
    def __init__(self):
        super().__init__('fm_pipeline')
        self.declare_parameter('rate_hz',         100.0)
        self.declare_parameter('q_des',           [0.5, -0.6])
        self.declare_parameter('joints',          ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('n_samples',       60000)
        self.declare_parameter('save_dir',        '/root/so100_ws/freemodel_out')
        self.declare_parameter('start_threshold', 1e-2)

        self.dt              = 1.0 / float(self.get_parameter('rate_hz').value)
        self.q_des           = np.array(self.get_parameter('q_des').value, dtype=float)
        self.joints          = list(self.get_parameter('joints').value)
        self.N               = int(self.get_parameter('n_samples').value)
        self.save_dir        = str(self.get_parameter('save_dir').value)
        self.start_threshold = float(self.get_parameter('start_threshold').value)

        self.Ek  = np.zeros((self.N, 4))
        self.Uk  = np.zeros((self.N, 2))
        self.Ek1 = np.zeros((self.N, 4))
        self.Uk1 = np.zeros((self.N, 2))
        self.phase      = "WAIT"
        self.count      = 0
        self.leader_q   = np.zeros(2)
        self.leader_dq  = np.zeros(2)
        self.leader_u   = None
        self.have_state = False
        self.prev_e     = None
        self.prev_u     = None

        self.sub_js = self.create_subscription(JointState, '/so100/joint_states', self.cb_js, 10)
        self.sub_u  = self.create_subscription(Float64MultiArray, '/so100/lqr_u', self.cb_u, 10)
        self.timer  = self.create_timer(self.dt, self.step)
        self.get_logger().info(f"[fm_pipeline] recording {self.N} samples -> {self.save_dir}")

    def cb_js(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        self.leader_q  = np.array([msg.position[i] for i in idx], dtype=float)
        self.leader_dq = np.array([msg.velocity[i] for i in idx], dtype=float)
        self.have_state = True

    def cb_u(self, msg):
        if len(msg.data) >= 2:
            self.leader_u = np.array(msg.data[:2], dtype=float)

    def step(self):
        if self.count >= self.N or not self.have_state or self.leader_u is None:
            return
        e = np.hstack([self.leader_q - self.q_des, self.leader_dq])
        u = self.leader_u.copy()

        if self.phase == "WAIT":
            if np.linalg.norm(e) > self.start_threshold:
                self.phase  = "RECORD"
                self.prev_e = e.copy()
                self.prev_u = u.copy()
                self.get_logger().info("[fm_pipeline] -> RECORDING")
            return

        if self.prev_e is None:
            self.prev_e = e.copy(); self.prev_u = u.copy(); return

        k = self.count
        self.Ek[k]  = self.prev_e
        self.Uk[k]  = self.prev_u
        self.Ek1[k] = e
        self.Uk1[k] = u
        self.count += 1
        self.prev_e = e.copy()
        self.prev_u = u.copy()

        if self.count % 5000 == 0:
            self.get_logger().info(f"[fm_pipeline] {self.count}/{self.N}")

        if self.count >= self.N:
            self.save_and_exit()

    def save_and_exit(self):
        os.makedirs(self.save_dir, exist_ok=True)
        np.save(os.path.join(self.save_dir, 'Ek.npy'),  self.Ek)
        np.save(os.path.join(self.save_dir, 'Uk.npy'),  self.Uk)
        np.save(os.path.join(self.save_dir, 'Ek1.npy'), self.Ek1)
        np.save(os.path.join(self.save_dir, 'Uk1.npy'), self.Uk1)
        meta = {'q_des': self.q_des.tolist(), 'joints': self.joints,
                'dt': self.dt, 'N_pairs': self.N}
        with open(os.path.join(self.save_dir, 'record_meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f"[fm_pipeline] Saved -> {self.save_dir}")
        raise SystemExit

def main():
    rclpy.init()
    node = None
    try:
        node = FMPipeline()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        if node: node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()
