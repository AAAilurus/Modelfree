#!/usr/bin/env python3
import numpy as np
import scipy.linalg
import os, json, csv
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class FMLeader(Node):
    def __init__(self):
        super().__init__('fm_leader')
        self.declare_parameter('rate_hz',        100.0)
        self.declare_parameter('q_des',          [0.7, -1.2])
        self.declare_parameter('noise_std',      0.05)
        self.declare_parameter('u_max',          5.0)
        self.declare_parameter('dq_max',         2.0)
        self.declare_parameter('q_min',         -1.5)
        self.declare_parameter('q_max',          1.5)
        self.declare_parameter('Q_star_diag',    [100.0, 100.0, 10.0, 10.0])
        self.declare_parameter('R_star_diag',    [0.5, 0.5])
        self.declare_parameter('joint_names',    ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('n_traj',         50)
        self.declare_parameter('steps_per_traj', 300)
        self.declare_parameter('save_dir',       '/root/so100_ws/freemodel_out')

        self.dt          = 1.0 / float(self.get_parameter('rate_hz').value)
        self.q_des_final = np.array(self.get_parameter('q_des').value, dtype=float)
        self.noise_std   = float(self.get_parameter('noise_std').value)
        self.u_max       = float(self.get_parameter('u_max').value)
        self.dq_max      = float(self.get_parameter('dq_max').value)
        self.q_min       = float(self.get_parameter('q_min').value)
        self.q_max_val   = float(self.get_parameter('q_max').value)
        self.joints      = list(self.get_parameter('joint_names').value)
        self.n_traj      = int(self.get_parameter('n_traj').value)
        self.steps       = int(self.get_parameter('steps_per_traj').value)
        self.save_dir    = str(self.get_parameter('save_dir').value)

        self.N  = self.n_traj * self.steps
        Ts      = self.dt
        Q_star  = np.diag(np.array(self.get_parameter('Q_star_diag').value, dtype=float))
        R_star  = np.diag(np.array(self.get_parameter('R_star_diag').value, dtype=float))
        self.Ad = np.array([[1,0,Ts,0],[0,1,0,Ts],[0,0,1,0],[0,0,0,1]], dtype=float)
        self.Bd = np.array([[0.5*Ts**2,0],[0,0.5*Ts**2],[Ts,0],[0,Ts]], dtype=float)
        P = scipy.linalg.solve_discrete_are(self.Ad, self.Bd, Q_star, R_star)
        self.K_star = np.linalg.solve(R_star + self.Bd.T@P@self.Bd, self.Bd.T@P@self.Ad)

        self.q      = np.zeros(2)
        self.dq     = np.zeros(2)
        self.dq_cmd = np.zeros(2)

        # data storage
        self.Ek    = []
        self.Uk    = []
        self.Ek1   = []
        self.Uk1   = []
        self.count = 0
        self.prev_e = None
        self.prev_u = None
        self.phase  = "DEMO"

        self.sub     = self.create_subscription(
            JointState, '/so100/joint_states', self.cb_js, 10)
        self.pub_u   = self.create_publisher(
            Float64MultiArray, '/so100/lqr_u', 10)
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            '/so100/arm_position_controller/commands', 10)
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f"[fm_leader] DEMO: {self.n_traj} traj x {self.steps} steps "
            f"= {self.N} samples  goal={self.q_des_final}")
        self.get_logger().info(f"[fm_leader] K_star=\n{self.K_star}")

    def cb_js(self, msg):
        n2i = {n:i for i,n in enumerate(msg.name)}
        if any(j not in n2i for j in self.joints):
            return
        for k, j in enumerate(self.joints):
            i = n2i[j]
            self.q[k]  = msg.position[i]
            self.dq[k] = msg.velocity[i] if len(msg.velocity) > i else 0.0

    def step(self):
        if self.phase == "DEMO":
            self._demo_step()
        else:
            self._hold_step()

    def _demo_step(self):
        # always go straight to final goal — no cycling
        e   = np.hstack([self.q - self.q_des_final, self.dq])
        eta = self.noise_std * np.random.randn(2)
        u   = np.clip(-(self.K_star @ e) + eta, -self.u_max, self.u_max)

        mu = Float64MultiArray()
        mu.data = u.tolist()
        self.pub_u.publish(mu)

        self.dq_cmd = np.clip(
            self.dq_cmd + u*self.dt, -self.dq_max, self.dq_max)
        q_cmd = np.clip(
            self.q + self.dq_cmd*self.dt, self.q_min, self.q_max_val)
        mc = Float64MultiArray()
        mc.data = q_cmd.tolist()
        self.pub_cmd.publish(mc)

        if self.prev_e is not None:
            self.Ek.append(self.prev_e.copy())
            self.Uk.append(self.prev_u.copy())
            self.Ek1.append(e.copy())
            self.Uk1.append(u.copy())
            self.count += 1

            if self.count % 3000 == 0:
                self.get_logger().info(
                    f"[fm_leader] Demo: {self.count}/{self.N}")

            if self.count >= self.N:
                self._save_data()
                self.phase  = "HOLD"
                self.dq_cmd = np.zeros(2)
                self.get_logger().info(
                    f"[fm_leader] PHASE 2: Holding goal {self.q_des_final} "
                    f"— run SPSA now, then start follower")

        self.prev_e = e.copy()
        self.prev_u = u.copy()

    def _hold_step(self):
        e = np.hstack([self.q - self.q_des_final, self.dq])
        u = np.clip(-(self.K_star @ e), -self.u_max, self.u_max)
        self.dq_cmd = np.clip(
            self.dq_cmd + u*self.dt, -self.dq_max, self.dq_max)
        q_cmd = np.clip(
            self.q + self.dq_cmd*self.dt, self.q_min, self.q_max_val)
        mc = Float64MultiArray()
        mc.data = q_cmd.tolist()
        self.pub_cmd.publish(mc)

    def _save_data(self):
        os.makedirs(self.save_dir, exist_ok=True)

        # save all as CSV
        def save_csv(path, data, header):
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(data)

        save_csv(os.path.join(self.save_dir, 'Ek.csv'),
                 self.Ek, ['e1','e2','e3','e4'])
        save_csv(os.path.join(self.save_dir, 'Uk.csv'),
                 self.Uk, ['u1','u2'])
        save_csv(os.path.join(self.save_dir, 'Ek1.csv'),
                 self.Ek1, ['e1','e2','e3','e4'])
        save_csv(os.path.join(self.save_dir, 'Uk1.csv'),
                 self.Uk1, ['u1','u2'])

        meta = {'q_des': self.q_des_final.tolist(),
                'joints': self.joints, 'dt': self.dt,
                'n_traj': self.n_traj, 'steps_per_traj': self.steps,
                'N_total': self.N}
        with open(os.path.join(self.save_dir, 'record_meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(
            f"[fm_leader] ✓ Data saved as CSV -> {self.save_dir}")

def main():
    rclpy.init()
    node = FMLeader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()
