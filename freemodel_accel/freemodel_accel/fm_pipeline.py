#!/usr/bin/env python3
import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool


class FMPipeline(Node):
    def __init__(self):
        super().__init__('fm_pipeline')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('q_des', [0.5, -0.6])
        self.declare_parameter('joints', ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('save_dir', '/root/Modelfree/freemodel_out')
        self.declare_parameter('joint_states_topic', '/so100/joint_states')
        self.declare_parameter('u_topic', '/so100/lqr_u')
        self.declare_parameter('traj_reset_topic', '/so100/traj_reset')

        self.q_des = np.array(self.get_parameter('q_des').value, dtype=float)
        self.joints = list(self.get_parameter('joints').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.js_topic = str(self.get_parameter('joint_states_topic').value)
        self.u_topic = str(self.get_parameter('u_topic').value)
        self.traj_reset_topic = str(self.get_parameter('traj_reset_topic').value)

        self.q = np.zeros(2, dtype=float)
        self.dq = np.zeros(2, dtype=float)
        self.have_state = False

        self.prev_e = None
        self.prev_u = None
        self.just_reset = True

        self.Ek = []
        self.Uk = []
        self.Ek1 = []
        self.Uk1 = []

        self.create_subscription(JointState, self.js_topic, self.cb_js, 50)
        self.create_subscription(Float64MultiArray, self.u_topic, self.cb_u, 50)
        self.create_subscription(Bool, self.traj_reset_topic, self.cb_reset, 10)

        self.get_logger().info(f"[fm_pipeline] js_topic={self.js_topic}")
        self.get_logger().info(f"[fm_pipeline] u_topic={self.u_topic}")
        self.get_logger().info(f"[fm_pipeline] traj_reset_topic={self.traj_reset_topic}")
        self.get_logger().info(f"[fm_pipeline] save_dir={self.save_dir}")

    def cb_reset(self, msg):
        if msg.data:
            self.prev_e = None
            self.prev_u = None
            self.just_reset = True
            self.get_logger().info("[fm_pipeline] trajectory reset")

    def cb_js(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        self.q = np.array([msg.position[i] for i in idx], dtype=float)
        self.dq = np.array([msg.velocity[i] if len(msg.velocity) > i else 0.0 for i in idx], dtype=float)
        self.have_state = True

    def cb_u(self, msg):
        if len(msg.data) < 2 or not self.have_state:
            return

        u = np.array(msg.data[:2], dtype=float)
        e = np.hstack([self.q - self.q_des, self.dq])

        if self.just_reset:
            self.prev_e = e.copy()
            self.prev_u = u.copy()
            self.just_reset = False
            return

        if self.prev_e is not None and self.prev_u is not None:
            self.Ek.append(self.prev_e.copy())
            self.Uk.append(self.prev_u.copy())
            self.Ek1.append(e.copy())
            self.Uk1.append(u.copy())

            if len(self.Ek) % 2000 == 0:
                self.get_logger().info(f"[fm_pipeline] samples={len(self.Ek)}")

        self.prev_e = e.copy()
        self.prev_u = u.copy()

    def save_csv(self, path, arr, header):
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            for row in arr:
                w.writerow([float(x) for x in row])

    def save(self):
        os.makedirs(self.save_dir, exist_ok=True)

        Ek = np.array(self.Ek, dtype=float)
        Uk = np.array(self.Uk, dtype=float)
        Ek1 = np.array(self.Ek1, dtype=float)
        Uk1 = np.array(self.Uk1, dtype=float)

        if len(Ek) > 0:
            self.save_csv(os.path.join(self.save_dir, 'Ek.csv'), Ek, ['e1','e2','e3','e4'])
            self.save_csv(os.path.join(self.save_dir, 'Uk.csv'), Uk, ['u1','u2'])
            self.save_csv(os.path.join(self.save_dir, 'Ek1.csv'), Ek1, ['e1','e2','e3','e4'])
            self.save_csv(os.path.join(self.save_dir, 'Uk1.csv'), Uk1, ['u1','u2'])

        self.get_logger().info(f"[fm_pipeline] saved {len(Ek)} samples -> {self.save_dir}")


def main():
    rclpy.init()
    node = FMPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
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
