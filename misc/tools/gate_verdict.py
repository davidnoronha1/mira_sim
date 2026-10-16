"""Verify a diagnose_gate.py run against Gazebo truth (flow-only friendly).

Usage: python3 gate_verdict.py TELEMETRY_DIR
Flow-only has no absolute position, so the EKF track is compared as
displacement from the start. Gate plane y=0, aperture x 6.56..7.44 (hull
margin), z -2.05..-0.85 (as record_gate_slam.py).
"""
import json
import math
import sys

import numpy as np
from scipy.spatial.transform import Rotation

d = json.load(open(f'{sys.argv[1]}/telemetry.json'))
truth, ekf = np.array(d['truth']), np.array(d['ekf'])
yaw = lambda rows: Rotation.from_quat(rows[:, 4:8]).as_euler('ZYX')[:, 0]
t = ekf[:, 0]
truth_xy = np.column_stack([np.interp(t, truth[:, 0], truth[:, i]) for i in (1, 2)])
truth_yaw = np.interp(t, truth[:, 0], np.unwrap(yaw(truth)))
ekf_disp, truth_disp = ekf[:, 1:3] - ekf[0, 1:3], truth_xy - truth_xy[0]
err = np.linalg.norm(ekf_disp - truth_disp, axis=1)
yaw_err = np.degrees((yaw(ekf) - truth_yaw + np.pi) % (2 * np.pi) - np.pi)
travelled = np.sum(np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1))
print(f'duration {t[-1] - t[0]:.0f} s, truth path {travelled:.1f} m')
print(f'EKF displacement error: final {err[-1]:.2f} m, max {err.max():.2f} m ({100 * err.max() / max(travelled, 1e-6):.1f} % of path)')
print(f'EKF yaw error: rms {np.sqrt(np.mean(yaw_err ** 2)):.2f} deg, max {np.abs(yaw_err).max():.2f} deg')
for frac in (.25, .5, .75, 1.0):
    i = min(int(frac * len(t)), len(t) - 1)
    print(f'  t+{t[i] - t[0]:5.1f}s truth ({truth_xy[i, 0]:6.2f},{truth_xy[i, 1]:6.2f}) disp err {err[i]:.2f} m yaw err {yaw_err[i]:+.1f}')
crossings = []
for a, b in zip(truth[:-1], truth[1:]):
    if a[2] < 0 <= b[2]:
        f = -a[2] / (b[2] - a[2])
        crossings.append([a[i] + f * (b[i] - a[i]) for i in range(4)])
for c in crossings:
    ok = 6.56 < c[1] < 7.44 and -2.05 < c[3] < -0.85
    print(f'gate plane crossing at t={c[0]:.1f} x={c[1]:.3f} z={c[3]:.2f}: {"APERTURE PASS" if ok else "MISS"}')
if not crossings:
    print(f'no gate plane crossing; closest y {truth[:, 2].max():.2f}')
