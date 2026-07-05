#!/usr/bin/env python3
"""Grafica el experimento EKF_NAV+OCCLUDE de Webots desde el log del supervisor.
Trayectoria real (POS) vs estimación a bordo (EKFPOSE), con el tramo ciego
marcado. Uso: plot_ekfnav.py <log> <out.png>"""
import sys
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

GOAL = (1800.0, 1000.0)
C_EKF, C_TRUTH, C_BLIND = "#4269d0", "#555555", "#efb118"

log, out = sys.argv[1], sys.argv[2]
truth, ekf, blind_truth = [], [], []
occluded = False
gt_started = False
for line in open(log, errors='replace'):
    line = line.strip()
    if 'GT iniciado' in line:
        gt_started = True
    if not gt_started:
        continue
    if 'cámara OCLUIDA' in line:
        occluded = True
    elif 'cámara RESTAURADA' in line:
        occluded = False
    elif line.startswith('POS|1|'):
        _, _, x, y, a = line.split('|')
        truth.append((float(x), float(y)))
        if occluded:
            blind_truth.append((float(x), float(y)))
    elif line.startswith('[1] EKFPOSE|1|') or line.startswith('EKFPOSE|1|'):
        p = line.split('EKFPOSE|1|')[1].split('|')
        ekf.append((float(p[0]), float(p[1]), occluded))
    if 'llegó' in line:
        break

fig, ax = plt.subplots(figsize=(9, 6.5))
tx, ty = zip(*truth)
ax.plot(tx, ty, '--', color=C_TRUTH, lw=2, label='trayectoria real (física Webots)')
if blind_truth:
    bx, by = zip(*blind_truth)
    ax.plot(bx, by, '-', color=C_BLIND, lw=4, alpha=0.9,
            label='tramo A CIEGAS (cámara ocluida)')
ex = [p[0] for p in ekf]; ey = [p[1] for p in ekf]
ax.plot(ex, ey, 'o-', color=C_EKF, lw=1.5, ms=4, label='pose EKF a bordo (1 Hz)')
ax.plot(*GOAL, 'x', color=C_TRUTH, ms=12, mew=3)
ax.annotate('goal', GOAL, textcoords='offset points', xytext=(10, 10), fontsize=11)
ax.plot(tx[0], ty[0], 's', color=C_TRUTH, ms=8)
ax.annotate('inicio', (tx[0], ty[0]), textcoords='offset points', xytext=(8, -14))

final_d = math.hypot(tx[-1] - GOAL[0], ty[-1] - GOAL[1])
ax.set_title(f'GT navegando con pose EKF + oclusión de cámara — '
             f'error final real: {final_d:.0f} mm')
ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)')
ax.invert_yaxis()
ax.set_aspect('equal')
ax.grid(True, alpha=0.25)
ax.legend(frameon=False, loc='upper left')
fig.tight_layout()
fig.savefig(out, dpi=130)
print(f'final real=({tx[-1]:.0f},{ty[-1]:.0f})  dist_goal={final_d:.0f}mm  '
      f'muestras: truth={len(truth)} ekf={len(ekf)} ciegas={len(blind_truth)}')
print('plot →', out)
