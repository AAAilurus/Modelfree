#!/usr/bin/env python3
import csv, os, argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load_csv_columns(path, cols):
    data = {c: [] for c in cols}
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            for c in cols:
                data[c].append(float(row[c]))
    return {c: np.array(v) for c, v in data.items()}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_dir', default='/root/so100_ws/freemodel_out')
    parser.add_argument('--log_dir',  default='/root/so100_ws/freemodel_follow_logs')
    parser.add_argument('--out_dir',  default='/root/so100_ws/freemodel_out/plots')
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    # ── load trajectory CSV ────────────────────────────────────────────────────
    traj = load_csv_columns(
        os.path.join(args.log_dir, 'follower_trajectory.csv'),
        ['time','leader_q1','leader_q2','leader_dq1','leader_dq2',
         'follower_q1','follower_q2','follower_dq1','follower_dq2'])
    t = traj['time']

    # ── load SPSA history ──────────────────────────────────────────────────────
    spsa = load_csv_columns(
        os.path.join(args.data_dir, 'spsa_history.csv'),
        ['it','K_err','q1','q2','q3','q4'])

    Q_star = [100., 100., 10., 10.]

    # ══════════════════════════════════════════════════════════════════════════
    # Plot 1 — Position comparison: Leader vs Follower
    # ══════════════════════════════════════════════════════════════════════════
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(t, traj['leader_q1'],   'b-',  linewidth=2.0,
                 label='SO100 Leader q1')
    axes[0].plot(t, traj['follower_q1'], 'r--', linewidth=2.0,
                 label='SO101 Follower q1')
    axes[0].set_ylabel('Position (rad)', fontsize=12)
    axes[0].set_title('Joint 1 — Position: Leader vs Follower', fontsize=12)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.4)

    axes[1].plot(t, traj['leader_q2'],   'b-',  linewidth=2.0,
                 label='SO100 Leader q2')
    axes[1].plot(t, traj['follower_q2'], 'r--', linewidth=2.0,
                 label='SO101 Follower q2')
    axes[1].set_ylabel('Position (rad)', fontsize=12)
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].set_title('Joint 2 — Position: Leader vs Follower', fontsize=12)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.4)

    fig.suptitle('Leader vs Follower — Joint Positions', fontsize=14,
                 fontweight='bold')
    plt.tight_layout()
    p1 = os.path.join(args.out_dir, 'plot1_position_comparison.png')
    plt.savefig(p1, dpi=150)
    plt.close()
    print(f"Saved {p1}")

    # ══════════════════════════════════════════════════════════════════════════
    # Plot 2 — Velocity comparison: Leader vs Follower
    # ══════════════════════════════════════════════════════════════════════════
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(t, traj['leader_dq1'],   'b-',  linewidth=2.0,
                 label='SO100 Leader dq1')
    axes[0].plot(t, traj['follower_dq1'], 'r--', linewidth=2.0,
                 label='SO101 Follower dq1')
    axes[0].set_ylabel('Velocity (rad/s)', fontsize=12)
    axes[0].set_title('Joint 1 — Velocity: Leader vs Follower', fontsize=12)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.4)

    axes[1].plot(t, traj['leader_dq2'],   'b-',  linewidth=2.0,
                 label='SO100 Leader dq2')
    axes[1].plot(t, traj['follower_dq2'], 'r--', linewidth=2.0,
                 label='SO101 Follower dq2')
    axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].set_title('Joint 2 — Velocity: Leader vs Follower', fontsize=12)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.4)

    fig.suptitle('Leader vs Follower — Joint Velocities', fontsize=14,
                 fontweight='bold')
    plt.tight_layout()
    p2 = os.path.join(args.out_dir, 'plot2_velocity_comparison.png')
    plt.savefig(p2, dpi=150)
    plt.close()
    print(f"Saved {p2}")

    # ══════════════════════════════════════════════════════════════════════════
    # Plot 3 — SPSA convergence
    # ══════════════════════════════════════════════════════════════════════════
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].semilogy(spsa['it'], spsa['K_err'], 'b-', linewidth=1.8)
    axes[0].axhline(y=1e-3, color='r', linestyle='--', linewidth=1.2,
                    label='Tolerance 1e-3')
    axes[0].set_xlabel('Iteration', fontsize=12)
    axes[0].set_ylabel(r'$\|K(Q)-K^*\|_F$', fontsize=12)
    axes[0].set_title('SPSA K Error Convergence', fontsize=12)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.4)

    colors = ['tab:blue','tab:orange','tab:green','tab:red']
    labels = ['Q11','Q22','Q33','Q44']
    qdata  = [spsa['q1'], spsa['q2'], spsa['q3'], spsa['q4']]
    for i in range(4):
        axes[1].plot(spsa['it'], qdata[i], color=colors[i],
                     linewidth=1.5, label=f'Learned {labels[i]}')
        axes[1].axhline(y=Q_star[i], color=colors[i],
                        linestyle='--', linewidth=0.8, alpha=0.6,
                        label=f'True {labels[i]}={Q_star[i]}')
    axes[1].set_xlabel('Iteration', fontsize=12)
    axes[1].set_ylabel('Q diagonal value', fontsize=12)
    axes[1].set_title('Q Diagonal Convergence', fontsize=12)
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.4)

    fig.suptitle('SPSA Learning Results', fontsize=14, fontweight='bold')
    plt.tight_layout()
    p3 = os.path.join(args.out_dir, 'plot3_spsa_convergence.png')
    plt.savefig(p3, dpi=150)
    plt.close()
    print(f"Saved {p3}")

    print(f"\n✓ All plots saved to {args.out_dir}")

if __name__ == '__main__':
    main()
