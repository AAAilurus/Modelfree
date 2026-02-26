#!/usr/bin/env python3
import os, csv
import numpy as np
import scipy.linalg
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class FMFollower(Node):
    def __init__(self):
        super().__init__('fm_follower')
        self.declare_parameter('rate_hz',          100.0)
        self.declare_parameter('q_learned_path',
                               '/root/so100_ws/freemodel_out/Q_learned.csv')
        self.declare_parameter('R_diag',           [0.5, 0.5])
        self.declare_parameter('joints',           ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('u_max',            5.0)
        self.declare_parameter('dq_cmd_max',       2.0)
        self.declare_parameter('log_dir',
                               '/root/so100_ws/freemodel_follow_logs')
        self.declare_parameter('stop_tol_q',       0.01)
        self.declare_parameter('stop_tol_dq',      0.02)
        self.declare_parameter('stop_hold_cycles', 50)
        self.declare_parameter('leader_vel_tol',   0.02)
        self.declare_parameter('goal_est_cycles',  30)

        self.dt          = 1.0 / float(self.get_parameter('rate_hz').value)
        self.q_path      = str(self.get_parameter('q_learned_path').value)
        self.R           = np.diag(np.array(
            self.get_parameter('R_diag').value, dtype=float))
        self.joints      = list(self.get_parameter('joints').value)
        self.u_max       = float(self.get_parameter('u_max').value)
        self.dq_cmd_max  = float(self.get_parameter('dq_cmd_max').value)
        self.log_dir     = str(self.get_parameter('log_dir').value)
        self.stop_tol_q  = float(self.get_parameter('stop_tol_q').value)
        self.stop_tol_dq = float(self.get_parameter('stop_tol_dq').value)
        self.stop_hold   = int(self.get_parameter('stop_hold_cycles').value)
        self.leader_vel_tol  = float(self.get_parameter('leader_vel_tol').value)
        self.goal_est_cycles = int(self.get_parameter('goal_est_cycles').value)

        # load Q from CSV
        if not os.path.exists(self.q_path):
            raise RuntimeError(f"Q file not found: {self.q_path}")
        Q_diag = []
        with open(self.q_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                Q_diag.append(float(row['value']))
        self.Q = np.diag(Q_diag)

        Ts = self.dt
        Ad = np.array([[1,0,Ts,0],[0,1,0,Ts],[0,0,1,0],[0,0,0,1]], dtype=float)
        Bd = np.array([[0.5*Ts**2,0],[0,0.5*Ts**2],[Ts,0],[0,Ts]], dtype=float)
        P  = scipy.linalg.solve_discrete_are(Ad, Bd, self.Q, self.R)
        self.K = np.linalg.solve(self.R + Bd.T@P@Bd, Bd.T@P@Ad)

        self.get_logger().info(f"[fm_follower] Q diag: {np.diag(self.Q).round(4)}")
        self.get_logger().info(f"[fm_follower] K:\n{self.K}")

        # follower state
        self.q      = np.zeros(2)
        self.dq     = np.zeros(2)
        self.dq_cmd = np.zeros(2)
        self._at_goal = 0
        self.have_follower = False

        # leader state
        self.leader_q           = np.zeros(2)
        self.leader_dq          = np.zeros(2)
        self.leader_still_count = 0
        self.q_des              = None
        self.goal_confirmed     = False
        self.have_leader        = False

        # logs — all will become CSV
        self.log_time      = []
        self.log_leader_q  = []
        self.log_leader_dq = []
        self.log_follower_q  = []
        self.log_follower_dq = []
        self.log_u           = []
        self.t = 0.0

        self.sub_leader   = self.create_subscription(
            JointState, '/so100/joint_states', self.cb_leader, 10)
        self.sub_follower = self.create_subscription(
            JointState, '/so101/joint_states', self.cb_follower, 10)
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/so101/arm_position_controller/commands', 10)
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(
            "[fm_follower] Phase 3 ready — waiting for leader to settle...")

    def cb_leader(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        self.leader_q  = np.array([msg.position[i] for i in idx], dtype=float)
        self.leader_dq = np.array(
            [msg.velocity[i] if len(msg.velocity) > i else 0.0
             for i in idx], dtype=float)
        self.have_leader = True

    def cb_follower(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.joints]
        except ValueError:
            return
        self.q  = np.array([msg.position[i] for i in idx], dtype=float)
        self.dq = np.array(
            [msg.velocity[i] if len(msg.velocity) > i else 0.0
             for i in idx], dtype=float)
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
                f"[fm_follower] ✓ Goal estimated: {self.q_des.round(4)}")

    def step(self):
        self.estimate_goal()
        if not self.goal_confirmed or not self.have_follower:
            return

        e = np.hstack([self.q - self.q_des, self.dq])
        u = np.clip(-self.K @ e, -self.u_max, self.u_max)
        self.dq_cmd = np.clip(
            self.dq_cmd + u*self.dt, -self.dq_cmd_max, self.dq_cmd_max)
        q_cmd = self.q + self.dq_cmd * self.dt

        msg = Float64MultiArray()
        msg.data = q_cmd.tolist()
        self.pub.publish(msg)

        # log everything
        self.log_time.append(round(self.t, 4))
        self.log_leader_q.append(self.leader_q.copy())
        self.log_leader_dq.append(self.leader_dq.copy())
        self.log_follower_q.append(self.q.copy())
        self.log_follower_dq.append(self.dq.copy())
        self.log_u.append(u.copy())
        self.t += self.dt

        pos_err = np.linalg.norm(self.q - self.q_des)
        vel_mag = np.linalg.norm(self.dq)
        if pos_err < self.stop_tol_q and vel_mag < self.stop_tol_dq:
            self._at_goal += 1
        else:
            self._at_goal = 0

        if self._at_goal >= self.stop_hold:
            self.get_logger().info(
                f"[fm_follower] ✓ REACHED GOAL  "
                f"pos_err={pos_err:.5f}  vel={vel_mag:.5f}")
            self.save_logs()
            raise SystemExit

    def save_logs(self):
        os.makedirs(self.log_dir, exist_ok=True)

        # follower_trajectory.csv — main comparison file
        path = os.path.join(self.log_dir, 'follower_trajectory.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time',
                        'leader_q1','leader_q2',
                        'leader_dq1','leader_dq2',
                        'follower_q1','follower_q2',
                        'follower_dq1','follower_dq2',
                        'u1','u2'])
            for i in range(len(self.log_time)):
                w.writerow([
                    self.log_time[i],
                    self.log_leader_q[i][0],  self.log_leader_q[i][1],
                    self.log_leader_dq[i][0], self.log_leader_dq[i][1],
                    self.log_follower_q[i][0],  self.log_follower_q[i][1],
                    self.log_follower_dq[i][0], self.log_follower_dq[i][1],
                    self.log_u[i][0], self.log_u[i][1]
                ])

        # goal info
        meta_path = os.path.join(self.log_dir, 'goal_info.csv')
        with open(meta_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['q_des1','q_des2','dt','total_steps'])
            w.writerow([self.q_des[0], self.q_des[1],
                        self.dt, len(self.log_time)])

        self.get_logger().info(
            f"[fm_follower] ✓ Logs saved -> {self.log_dir}")

def main():
    rclpy.init()
    node = FMFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try: node.save_logs()
        except: pass
        try: node.destroy_node()
        except: pass
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()
