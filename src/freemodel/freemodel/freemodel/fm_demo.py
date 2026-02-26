#!/usr/bin/env python3
import os, csv, numpy as np, scipy.linalg, rclpy, matplotlib
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class FMDemo(Node):
    def __init__(self):
        super().__init__('fm_demo')
        self.declare_parameter('rate_hz',        100.0)
        self.declare_parameter('q_des',          [0.5, -0.8])
        self.declare_parameter('q_start',        [0.0,  0.0])
        self.declare_parameter('q_learned_path', '/root/so100_ws/freemodel_out/Q_learned.csv')
        self.declare_parameter('Q_star_diag',    [100.0, 100.0, 10.0, 10.0])
        self.declare_parameter('R_diag',         [0.5, 0.5])
        self.declare_parameter('joints',         ['Shoulder_Pitch', 'Elbow'])
        self.declare_parameter('u_max',          5.0)
        self.declare_parameter('dq_cmd_max',     2.0)
        self.declare_parameter('q_min',         -1.5)
        self.declare_parameter('q_max',          1.5)
        self.declare_parameter('stop_tol_q',     0.01)
        self.declare_parameter('stop_tol_dq',    0.02)
        self.declare_parameter('stop_hold',      50)
        self.declare_parameter('init_tol',       0.02)
        self.declare_parameter('init_hold',      50)
        self.declare_parameter('log_dir',        '/root/so100_ws/freemodel_follow_logs')
        self.declare_parameter('plot_dir',       '/root/so100_ws/freemodel_out/plots')

        self.dt          = 1.0/float(self.get_parameter('rate_hz').value)
        self.q_des       = np.array(self.get_parameter('q_des').value,   dtype=float)
        self.q_start     = np.array(self.get_parameter('q_start').value, dtype=float)
        self.joints      = list(self.get_parameter('joints').value)
        self.u_max       = float(self.get_parameter('u_max').value)
        self.dq_cmd_max  = float(self.get_parameter('dq_cmd_max').value)
        self.q_min       = float(self.get_parameter('q_min').value)
        self.q_max_val   = float(self.get_parameter('q_max').value)
        self.stop_tol_q  = float(self.get_parameter('stop_tol_q').value)
        self.stop_tol_dq = float(self.get_parameter('stop_tol_dq').value)
        self.stop_hold   = int(self.get_parameter('stop_hold').value)
        self.init_tol    = float(self.get_parameter('init_tol').value)
        self.init_hold   = int(self.get_parameter('init_hold').value)
        self.log_dir     = str(self.get_parameter('log_dir').value)
        self.plot_dir    = str(self.get_parameter('plot_dir').value)

        Ts = self.dt
        Q_star = np.diag(np.array(self.get_parameter('Q_star_diag').value, dtype=float))
        R      = np.diag(np.array(self.get_parameter('R_diag').value,      dtype=float))
        self.Ad = np.array([[1,0,Ts,0],[0,1,0,Ts],[0,0,1,0],[0,0,0,1]], dtype=float)
        self.Bd = np.array([[0.5*Ts**2,0],[0,0.5*Ts**2],[Ts,0],[0,Ts]],  dtype=float)

        P_star         = scipy.linalg.solve_discrete_are(self.Ad, self.Bd, Q_star, R)
        self.K_star    = np.linalg.solve(R+self.Bd.T@P_star@self.Bd, self.Bd.T@P_star@self.Ad)

        q_path = str(self.get_parameter('q_learned_path').value)
        if not os.path.exists(q_path):
            raise RuntimeError(f"Q_learned.csv not found: {q_path}")
        Q_diag = [float(row['value']) for row in csv.DictReader(open(q_path))]
        Q_learned      = np.diag(Q_diag)
        P_learned      = scipy.linalg.solve_discrete_are(self.Ad, self.Bd, Q_learned, R)
        self.K_learned = np.linalg.solve(R+self.Bd.T@P_learned@self.Bd, self.Bd.T@P_learned@self.Ad)

        self.get_logger().info(f"[fm_demo] q_start={self.q_start}  q_des={self.q_des}")
        self.get_logger().info(f"[fm_demo] K_star:\n{self.K_star.round(4)}")
        self.get_logger().info(f"[fm_demo] K_learned:\n{self.K_learned.round(4)}")
        self.get_logger().info(f"[fm_demo] ||K_star-K_learned||_F = {np.linalg.norm(self.K_star-self.K_learned,'fro'):.5f}")

        self.q_l=np.zeros(2); self.dq_l=np.zeros(2); self.dqcmd_l=np.zeros(2)
        self.q_f=np.zeros(2); self.dq_f=np.zeros(2); self.dqcmd_f=np.zeros(2)
        self.have_l=False; self.have_f=False

        self.phase='INIT'
        self.hold_init_l=0; self.hold_init_f=0
        self.hold_l=0;      self.hold_f=0
        self.hold_ret_l=0;  self.hold_ret_f=0
        self.cycle=0;       self.max_cycles=10
        self.saved=False

        self.log_t=[]; self.log_lq=[]; self.log_ldq=[]; self.log_lu=[]
        self.log_fq=[]; self.log_fdq=[]; self.log_fu=[]; self.t=0.0

        self.sub_l = self.create_subscription(JointState,'/so100/joint_states',self.cb_l,10)
        self.sub_f = self.create_subscription(JointState,'/so101/joint_states',self.cb_f,10)
        self.pub_l = self.create_publisher(Float64MultiArray,'/so100/arm_position_controller/commands',10)
        self.pub_f = self.create_publisher(Float64MultiArray,'/so101/arm_position_controller/commands',10)
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f"[fm_demo] INIT — moving both arms to {self.q_start} ...")

    def cb_l(self, msg):
        try: idx=[msg.name.index(j) for j in self.joints]
        except ValueError: return
        self.q_l  = np.array([msg.position[i] for i in idx], dtype=float)
        self.dq_l = np.array([msg.velocity[i] if len(msg.velocity)>i else 0.0 for i in idx], dtype=float)
        self.have_l=True

    def cb_f(self, msg):
        try: idx=[msg.name.index(j) for j in self.joints]
        except ValueError: return
        self.q_f  = np.array([msg.position[i] for i in idx], dtype=float)
        self.dq_f = np.array([msg.velocity[i] if len(msg.velocity)>i else 0.0 for i in idx], dtype=float)
        self.have_f=True

    def _send(self, pub, q, dqcmd, u):
        dqcmd[:] = np.clip(dqcmd+u*self.dt, -self.dq_cmd_max, self.dq_cmd_max)
        q_cmd    = np.clip(q+dqcmd*self.dt,  self.q_min,       self.q_max_val)
        msg=Float64MultiArray(); msg.data=q_cmd.tolist(); pub.publish(msg)

    def step(self):
        if not self.have_l or not self.have_f: return

        if self.phase=='INIT':
            u_l=np.clip(-15.0*(self.q_l-self.q_start),-self.u_max,self.u_max)
            u_f=np.clip(-15.0*(self.q_f-self.q_start),-self.u_max,self.u_max)
            self._send(self.pub_l,self.q_l,self.dqcmd_l,u_l)
            self._send(self.pub_f,self.q_f,self.dqcmd_f,u_f)
            if np.linalg.norm(self.q_l-self.q_start)<self.init_tol: self.hold_init_l+=1
            else: self.hold_init_l=0
            if np.linalg.norm(self.q_f-self.q_start)<self.init_tol: self.hold_init_f+=1
            else: self.hold_init_f=0
            if self.hold_init_l>=self.init_hold and self.hold_init_f>=self.init_hold:
                self.dqcmd_l[:]=0.0; self.dqcmd_f[:]=0.0
                self.phase='RUN'
                self.get_logger().info(f"[fm_demo] Both at {self.q_start} — cycle 1/{self.max_cycles} START")
            return

        if self.phase=='RUN':
            e_l=np.hstack([self.q_l-self.q_des, self.dq_l])
            u_l=np.clip(-self.K_star@e_l,   -self.u_max, self.u_max)
            self._send(self.pub_l,self.q_l,self.dqcmd_l,u_l)

            e_f=np.hstack([self.q_f-self.q_des, self.dq_f])
            u_f=np.clip(-self.K_learned@e_f, -self.u_max, self.u_max)
            self._send(self.pub_f,self.q_f,self.dqcmd_f,u_f)

            self.log_t.append(round(self.t,4))
            self.log_lq.append(self.q_l.copy());  self.log_ldq.append(self.dq_l.copy()); self.log_lu.append(u_l.copy())
            self.log_fq.append(self.q_f.copy());  self.log_fdq.append(self.dq_f.copy()); self.log_fu.append(u_f.copy())
            self.t+=self.dt

            if np.linalg.norm(self.q_l-self.q_des)<self.stop_tol_q and np.linalg.norm(self.dq_l)<self.stop_tol_dq: self.hold_l+=1
            else: self.hold_l=0
            if np.linalg.norm(self.q_f-self.q_des)<self.stop_tol_q and np.linalg.norm(self.dq_f)<self.stop_tol_dq: self.hold_f+=1
            else: self.hold_f=0

            if self.hold_l>=self.stop_hold and self.hold_f>=self.stop_hold:
                self.cycle+=1
                l_err=np.linalg.norm(self.q_l-self.q_des); f_err=np.linalg.norm(self.q_f-self.q_des)
                self.get_logger().info(f"[fm_demo] Cycle {self.cycle}/{self.max_cycles}  leader_err={l_err:.5f}  follower_err={f_err:.5f}")
                if self.cycle>=self.max_cycles:
                    self.get_logger().info("[fm_demo] 10 cycles done — saving and exiting")
                    self.phase='DONE'
                    self.save_and_plot()
                    raise SystemExit
                self.hold_l=0; self.hold_f=0; self.hold_ret_l=0; self.hold_ret_f=0
                self.dqcmd_l[:]=0.0; self.dqcmd_f[:]=0.0
                self.phase='RETURN'
            return

        if self.phase=='RETURN':
            u_l=np.clip(-15.0*(self.q_l-self.q_start),-self.u_max,self.u_max)
            u_f=np.clip(-15.0*(self.q_f-self.q_start),-self.u_max,self.u_max)
            self._send(self.pub_l,self.q_l,self.dqcmd_l,u_l)
            self._send(self.pub_f,self.q_f,self.dqcmd_f,u_f)
            if np.linalg.norm(self.q_l-self.q_start)<self.init_tol: self.hold_ret_l+=1
            else: self.hold_ret_l=0
            if np.linalg.norm(self.q_f-self.q_start)<self.init_tol: self.hold_ret_f+=1
            else: self.hold_ret_f=0
            if self.hold_ret_l>=self.init_hold and self.hold_ret_f>=self.init_hold:
                self.dqcmd_l[:]=0.0; self.dqcmd_f[:]=0.0
                self.hold_l=0; self.hold_f=0
                self.phase='RUN'
                self.get_logger().info(f"[fm_demo] Back at start — cycle {self.cycle+1}/{self.max_cycles} START")

    def save_and_plot(self):
        if self.saved: return
        self.saved=True
        if not self.log_t:
            self.get_logger().info("[fm_demo] No data to save"); return

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.plot_dir, exist_ok=True)

        csv_path=os.path.join(self.log_dir,'follower_trajectory.csv')
        with open(csv_path,'w',newline='') as f:
            w=csv.writer(f)
            w.writerow(['time','leader_q1','leader_q2','leader_dq1','leader_dq2','leader_u1','leader_u2',
                        'follower_q1','follower_q2','follower_dq1','follower_dq2','follower_u1','follower_u2'])
            for i in range(len(self.log_t)):
                w.writerow([self.log_t[i],
                    self.log_lq[i][0],self.log_lq[i][1],self.log_ldq[i][0],self.log_ldq[i][1],self.log_lu[i][0],self.log_lu[i][1],
                    self.log_fq[i][0],self.log_fq[i][1],self.log_fdq[i][0],self.log_fdq[i][1],self.log_fu[i][0],self.log_fu[i][1]])

        with open(os.path.join(self.log_dir,'run_info.csv'),'w',newline='') as f:
            w=csv.writer(f)
            w.writerow(['q_start1','q_start2','q_des1','q_des2','dt','steps','cycles_done'])
            w.writerow([self.q_start[0],self.q_start[1],self.q_des[0],self.q_des[1],self.dt,len(self.log_t),self.cycle])

        self.get_logger().info(f"[fm_demo] CSV saved -> {csv_path}")

        t=np.array(self.log_t); lq=np.array(self.log_lq); fq=np.array(self.log_fq)
        ldq=np.array(self.log_ldq); fdq=np.array(self.log_fdq)

        fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
        axes[0].plot(t,lq[:,0],'b-', lw=2,label='SO100 Leader q1  (LQR K_star)')
        axes[0].plot(t,fq[:,0],'r--',lw=2,label='SO101 Follower q1 (SPSA K_learned)')
        axes[0].axhline(y=self.q_des[0],color='k',ls=':',lw=1.2,label=f'Goal={self.q_des[0]}')
        axes[0].set_ylabel('Position (rad)',fontsize=12); axes[0].set_title('Joint 1 — Position',fontsize=12)
        axes[0].legend(fontsize=10); axes[0].grid(True,alpha=0.4)
        axes[1].plot(t,lq[:,1],'b-', lw=2,label='SO100 Leader q2  (LQR K_star)')
        axes[1].plot(t,fq[:,1],'r--',lw=2,label='SO101 Follower q2 (SPSA K_learned)')
        axes[1].axhline(y=self.q_des[1],color='k',ls=':',lw=1.2,label=f'Goal={self.q_des[1]}')
        axes[1].set_ylabel('Position (rad)',fontsize=12); axes[1].set_xlabel('Time (s)',fontsize=12)
        axes[1].set_title('Joint 2 — Position',fontsize=12)
        axes[1].legend(fontsize=10); axes[1].grid(True,alpha=0.4)
        fig.suptitle(f'Leader (LQR) vs Follower (SPSA) — Position\nGoal={self.q_des.tolist()}  Cycles={self.cycle}',fontsize=13,fontweight='bold')
        plt.tight_layout()
        p1=os.path.join(self.plot_dir,'plot1_position_comparison.png')
        plt.savefig(p1,dpi=150); plt.close()
        self.get_logger().info(f"[fm_demo] Saved {p1}")

        fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
        axes[0].plot(t,ldq[:,0],'b-', lw=2,label='SO100 Leader dq1  (LQR K_star)')
        axes[0].plot(t,fdq[:,0],'r--',lw=2,label='SO101 Follower dq1 (SPSA K_learned)')
        axes[0].axhline(y=0,color='k',ls=':',lw=0.8)
        axes[0].set_ylabel('Velocity (rad/s)',fontsize=12); axes[0].set_title('Joint 1 — Velocity',fontsize=12)
        axes[0].legend(fontsize=10); axes[0].grid(True,alpha=0.4)
        axes[1].plot(t,ldq[:,1],'b-', lw=2,label='SO100 Leader dq2  (LQR K_star)')
        axes[1].plot(t,fdq[:,1],'r--',lw=2,label='SO101 Follower dq2 (SPSA K_learned)')
        axes[1].axhline(y=0,color='k',ls=':',lw=0.8)
        axes[1].set_ylabel('Velocity (rad/s)',fontsize=12); axes[1].set_xlabel('Time (s)',fontsize=12)
        axes[1].set_title('Joint 2 — Velocity',fontsize=12)
        axes[1].legend(fontsize=10); axes[1].grid(True,alpha=0.4)
        fig.suptitle(f'Leader (LQR) vs Follower (SPSA) — Velocity\nGoal={self.q_des.tolist()}  Cycles={self.cycle}',fontsize=13,fontweight='bold')
        plt.tight_layout()
        p2=os.path.join(self.plot_dir,'plot2_velocity_comparison.png')
        plt.savefig(p2,dpi=150); plt.close()
        self.get_logger().info(f"[fm_demo] Saved {p2}")
        self.get_logger().info("[fm_demo] All done")

def main():
    rclpy.init()
    node=FMDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try: node.save_and_plot()
        except: pass
        try: node.destroy_node()
        except: pass
        try: rclpy.shutdown()
        except: pass

if __name__=='__main__':
    main()
