"""Summarize EKF3 core 0 velocity and body-odometry fusion from a dataflash log.

Usage: /opt/mavenv/bin/python ekf_log.py LOG.BIN [SECONDS]
Prints 2 s bins: EKF NE speed, body-odometry innovations (XKFD) and test ratio,
the vision-delta messages received (VISD) and the yaw source state.
"""
import collections
import sys

import numpy as np
from pymavlink import mavutil

log = mavutil.mavlink_connection(sys.argv[1])
limit = float(sys.argv[2]) if len(sys.argv) > 2 else 150
rows = collections.defaultdict(list)
start = None
types = ['XKF1', 'XKFD', 'VISD', 'XKF4', 'XKY0', 'MSG']
while True:
    m = log.recv_match(type=types)
    if m is None:
        break
    t = m.TimeUS * 1e-6
    if m.get_type() == 'MSG':
        if 'EKF' in m.Message or 'yaw' in m.Message.lower():
            print(f'{t - (start or t):7.1f}s MSG {m.Message}')
        continue
    if getattr(m, 'C', 0) != 0:
        continue
    if start is None and m.get_type() == 'XKF1':
        start = t
    if start is None:
        continue
    if t - start > limit:
        break
    rows[m.get_type()].append((t - start, m))

for b in np.arange(0, limit, 2):
    def window(kind):
        return [m for t, m in rows[kind] if b <= t < b + 2]
    k1, kd, vd, k4 = window('XKF1'), window('XKFD'), window('VISD'), window('XKF4')
    speed = np.median([np.hypot(m.VN, m.VE) for m in k1]) if k1 else float('nan')
    innov = (np.median([np.hypot(m.IX, m.IY) for m in kd]) if kd else float('nan'))
    ratio = np.median([m.IVT for m in kd if hasattr(m, 'IVT')]) if kd and hasattr(kd[0], 'IVT') else float('nan')
    ss = k4[-1].SS if k4 else -1
    print(f'{b:5.0f}s EKF speed {speed:5.2f}  XKFD n={len(kd):3d} innov {innov:5.2f} ratio {ratio:5.2f}  VISD n={len(vd):3d}  SS={ss}')
