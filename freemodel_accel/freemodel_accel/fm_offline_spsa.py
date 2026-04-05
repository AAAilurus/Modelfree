#!/usr/bin/env python3
"""
fm_offline_spsa.py  -  Phase 2: Offline SPSA Q-Learning (no ROS)

Run after fm_leader finishes:
  python3 /root/so100_ws/src/freemodel/freemodel/fm_offline_spsa.py
"""
import os
import csv
import argparse
import numpy as np


def read_csv_matrix(path):
    with open(path, 'r') as f:
        r = csv.reader(f)
        next(r, None)
        data = [[float(x) for x in row] for row in r if row]
    return np.asarray(data, dtype=float)


def build_phi(Ek, Uk, Ek1, Uk1):
    N      = Ek.shape[0]
    n      = Ek.shape[1]
    m      = Uk.shape[1]
    dim    = n + m
    Phi    = np.zeros((N, dim * dim), dtype=float)
    for k in range(N):
        zk  = np.concatenate([Ek[k],  Uk[k]])
        zk1 = np.concatenate([Ek1[k], Uk1[k]])
        Phi[k, :] = np.kron(zk, zk) - np.kron(zk1, zk1)
    return Phi


def eval_K_from_Qdiag(q_diag, Phi, Ek, Uk, R, reg=1e-4):
    N, n = Ek.shape
    m    = Uk.shape[1]
    Q    = np.diag(q_diag)

    theta = np.array(
        [Ek[k] @ Q @ Ek[k] + Uk[k] @ R @ Uk[k] for k in range(N)],
        dtype=float)

    vecH, *_ = np.linalg.lstsq(Phi, theta, rcond=None)
    H = vecH.reshape(n + m, n + m)
    H = 0.5 * (H + H.T)

    Hux = H[n:, :n]
    Huu = 0.5 * (H[n:, n:] + H[n:, n:].T) + reg * np.eye(m)

    return np.linalg.solve(Huu, Hux)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out_dir',  default='/root/Modelfree/freemodel_out')
    ap.add_argument('--maxIter',  type=int,   default=1000)
    ap.add_argument('--tol_K',    type=float, default=1e-3)
    ap.add_argument('--alphaQ',   type=float, default=8e-1)
    ap.add_argument('--c_spsa',   type=float, default=1e-5)
    ap.add_argument('--pd_floor', type=float, default=1e-6)
    ap.add_argument('--reg_huu',  type=float, default=1e-4)
    ap.add_argument('--R_diag',   type=float, nargs='+', default=[0.5, 0.5])
    args = ap.parse_args()

    out = args.out_dir

    Ek     = read_csv_matrix(os.path.join(out, 'Ek.csv'))
    Uk     = read_csv_matrix(os.path.join(out, 'Uk.csv'))
    Ek1    = read_csv_matrix(os.path.join(out, 'Ek1.csv'))
    Uk1    = read_csv_matrix(os.path.join(out, 'Uk1.csv'))
    K_star = read_csv_matrix(os.path.join(out, 'K_star.csv'))

    N, n = Ek.shape
    m    = Uk.shape[1]
    R    = np.diag(np.asarray(args.R_diag, dtype=float))

    print(f"\n=== fm_offline_spsa: SPSA Q-learning ===")
    print(f"N={N}  n={n}  m={m}  maxIter={args.maxIter}")
    print("Building Phi ... ", end='', flush=True)
    Phi      = build_phi(Ek, Uk, Ek1, Uk1)
    rank_phi = int(np.linalg.matrix_rank(Phi))
    print(f"done   rank(Phi)={rank_phi}/{Phi.shape[1]}")

    q_diag    = np.ones(n) * 10.0
    hist      = []
    converged = False
    final_it  = args.maxIter - 1
    final_err = None

    for it in range(args.maxIter):

        Kj   = eval_K_from_Qdiag(q_diag, Phi, Ek, Uk, R, reg=args.reg_huu)
        kerr = float(np.linalg.norm(Kj - K_star, 'fro'))
        final_err = kerr
        hist.append([it, kerr, *q_diag.tolist()])

        if it % 50 == 0 or it < 10:
            print(f"iter {it:4d}:  ||K-K*||={kerr:.4e}  "
                  f"Q_diag={np.round(q_diag, 3).tolist()}")

        if kerr <= args.tol_K:
            converged = True
            final_it  = it
            print(f"\nCONVERGED at iter={it}  ||K-K*||={kerr:.8e}")
            break

        Delta = 2.0 * (np.random.rand(n) > 0.5).astype(float) - 1.0
        c     = args.c_spsa

        qp = np.maximum(q_diag + c * Delta, args.pd_floor)
        qm = np.maximum(q_diag - c * Delta, args.pd_floor)

        Kp = eval_K_from_Qdiag(qp, Phi, Ek, Uk, R, reg=args.reg_huu)
        Km = eval_K_from_Qdiag(qm, Phi, Ek, Uk, R, reg=args.reg_huu)

        Ep = float(np.linalg.norm(Kp - K_star, 'fro') ** 2)
        Em = float(np.linalg.norm(Km - K_star, 'fro') ** 2)

        g = (Ep - Em) / (2.0 * c * Delta)

        q_diag = np.maximum(q_diag - args.alphaQ * g, args.pd_floor)

    K_learned = eval_K_from_Qdiag(q_diag, Phi, Ek, Uk, R, reg=args.reg_huu)

    os.makedirs(out, exist_ok=True)

    with open(os.path.join(out, 'Q_learned.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['index', 'value'])
        for i, v in enumerate(q_diag):
            w.writerow([i, float(v)])

    with open(os.path.join(out, 'K_learned.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['k1', 'k2', 'k3', 'k4'])
        for row in K_learned:
            w.writerow([float(x) for x in row])

    with open(os.path.join(out, 'spsa_history.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['it', 'K_err', 'q1', 'q2', 'q3', 'q4'])
        w.writerows(hist)

    with open(os.path.join(out, 'result_summary.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['metric', 'value'])
        w.writerow(['rank_Phi',    rank_phi])
        w.writerow(['converged',   converged])
        w.writerow(['final_iter',  int(final_it)])
        w.writerow(['final_K_err', float(final_err)])
        for i in range(n):
            w.writerow([f'Q_learned_{i}', float(q_diag[i])])

    print(f"\nLearned Q_diag = {np.round(q_diag, 4).tolist()}")
    print(f"K_learned:\n{np.round(K_learned, 4)}")
    print(f"K_star:\n{np.round(K_star, 4)}")
    print(f"\n||K_learned - K*||_F = "
          f"{float(np.linalg.norm(K_learned - K_star, 'fro')):.6e}")
    print(f"\nAll outputs saved -> {out}")


if __name__ == '__main__':
    main()
