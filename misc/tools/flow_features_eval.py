"""Offline: feature detectors for LK flow, and floor-line heading, against truth.

Usage: python3 flow_features_eval.py /tmp/flowonly_ds.npz
Dataset (from an earlier recording): frames (N,240,320) half-scale gray, stamps,
truth rows (t, x, y, z, qx, qy, qz, qw, vx, vy, ...) in Gazebo ENU, K (full scale).
"""
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

d = np.load(sys.argv[1])
F, S, T = d['frames'], d['stamps'], d['truth']
K = d['K'].reshape(3, 3).copy()
K[:2] *= .5
Ki = np.linalg.inv(K)
FLOOR = 2.2
body_cam = R.from_euler('y', np.pi / 2) * R.from_euler('ZYX', [-np.pi / 2, 0, -np.pi / 2])
CLAHE = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))


def truth_at(t):
    return T[np.clip(np.searchsorted(T[:, 0], t), 1, len(T) - 1)]


def cam_state(t):
    x = truth_at(t)
    return R.from_quat(x[4:8]) * body_cam, x[3] - 0.05, x[8:10], R.from_quat(x[4:8])


def detect_current(g):
    return g, cv2.goodFeaturesToTrack(g, 200, 0.01, 8)


def detect_grid(g, corner_ratio=0.0, rows=4, cols=5, total=200):
    e = CLAHE.apply(g)
    h, w = e.shape
    per = total // (rows * cols)
    pts = []
    for r in range(rows):
        for c in range(cols):
            y0, y1, x0, x1 = r * h // rows, (r + 1) * h // rows, c * w // cols, (c + 1) * w // cols
            p = cv2.goodFeaturesToTrack(e[y0:y1, x0:x1], per * 3 if corner_ratio else per, 0.02, 6)
            if p is not None:
                pts.append(p.reshape(-1, 2) + (x0, y0))
    if not pts:
        return e, None
    p = np.concatenate(pts).astype(np.float32)
    if corner_ratio:
        ev = cv2.cornerEigenValsAndVecs(e, 7, 3)
        i = np.int32(np.round(p))
        l1, l2 = ev[i[:, 1], i[:, 0], 0], ev[i[:, 1], i[:, 0], 1]
        p = p[np.minimum(l1, l2) / np.maximum(np.maximum(l1, l2), 1e-12) > corner_ratio]
    return e, p.reshape(-1, 1, 2)


DETECTORS = {
    'current': detect_current,
    'clahe_grid': detect_grid,
    'clahe_grid_corner': lambda g: detect_grid(g, 0.15),
}


def lk(g0, g1, p0, rel, zoom, depth_c, down):
    p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None)
    pb, sb, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None)
    ok = (st.ravel() == 1) & (sb.ravel() == 1) & (np.linalg.norm((pb - p0).reshape(-1, 2), axis=1) < 1)
    if ok.sum() < 25:
        return None, ok.sum()
    h = lambda p: np.column_stack((p.reshape(-1, 2), np.ones(len(p.reshape(-1, 2)))))
    b = (h(p0[ok]) @ Ki.T) @ (np.diag([zoom, zoom, 1]) @ rel).T
    b = b[:, :2] / b[:, 2:]
    a = h(p1[ok]) @ Ki.T
    z = depth_c * down[2] / (a @ down)
    return np.median(-(a[:, :2] - b) * z[:, None], axis=0), ok.sum()


def coverage(p, shape):
    """Fraction of 4x5 cells with a feature, and fraction of features at corners (both directions)."""
    if p is None:
        return 0.0
    i = np.int32(p.reshape(-1, 2))
    cells = {(y * 4 // shape[0], x * 5 // shape[1]) for x, y in i}
    return len(cells) / 20


# --- flow -------------------------------------------------------------------
res = {n: [] for n in DETECTORS}
cov = {n: [] for n in DETECTORS}
step = 2
for i in range(step, len(F), step):
    t0, t1 = S[i - step], S[i]
    dt = t1 - t0
    if not 0 < dt < .25:
        continue
    r0, z0, _, _ = cam_state(t0)
    r1, z1, v, _ = cam_state(t1)
    h0, h1 = FLOOR + z0, FLOOR + z1
    down = r1.inv().apply([0, 0, -1])
    if h1 < .2 or down[2] < .5:
        continue
    rel = (r1.inv() * r0).as_matrix()
    for n, det in DETECTORS.items():
        e0, p0 = det(F[i - step])
        e1 = CLAHE.apply(F[i]) if n != 'current' else F[i]
        cov[n].append(coverage(p0, F[i].shape))
        if p0 is None or len(p0) < 25:
            continue
        tr, _ = lk(e0, e1, p0, rel, h0 / h1, h1 / down[2], down)
        if tr is None:
            continue
        vb = body_cam.apply([tr[0] / dt, tr[1] / dt, 0])[:2]
        res[n].append((t1, *vb, *v))
for n in DETECTORS:
    a = np.array(res[n])
    est, tru = a[:, 1:3], a[:, 3:5]
    m = np.linalg.norm(tru, axis=1) > .2
    e = est - tru
    slope = np.sum(est[m, 0] * tru[m, 0]) / np.sum(tru[m, 0] ** 2)
    print(f'{n:18s} n={len(a):4d} rms={np.sqrt((e ** 2).sum(1).mean()):.3f} rms_moving={np.sqrt((e[m] ** 2).sum(1).mean()):.3f} '
          f'fwd_slope={slope:.3f} p95={np.percentile(np.linalg.norm(e, axis=1), 95):.3f} '
          f'max={np.linalg.norm(e, axis=1).max():.3f} cell_coverage={np.mean(cov[n]):.2f}')


# --- floor-line heading -----------------------------------------------------
def line_axis(gray, cam_rot, height, symmetry=np.pi / 2, max_pixels=4000):
    """Dominant floor-line direction (rad, mod symmetry) in the frame cam_rot maps into."""
    g = cv2.GaussianBlur(gray, (5, 5), 0).astype(np.float32)
    gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)
    mag[:4], mag[-4:], mag[:, :4], mag[:, -4:] = 0, 0, 0, 0
    thr = max(np.percentile(mag, 90), 4.0)
    ys, xs = np.nonzero(mag > thr)
    if len(xs) < 200:
        return None
    if len(xs) > max_pixels:
        k = np.random.default_rng(0).choice(len(xs), max_pixels, replace=False)
        ys, xs = ys[k], xs[k]
    w = mag[ys, xs]
    tangent = np.column_stack((-gy[ys, xs], gx[ys, xs])) / w[:, None]
    p0 = np.column_stack((xs, ys, np.ones(len(xs))))
    p1 = p0 + np.column_stack((tangent * 2.0, np.zeros(len(xs))))
    m = cam_rot.as_matrix()

    def floor(p):
        ray = (p @ Ki.T) @ m.T
        s = height / -ray[:, 2]
        return ray[:, :2] * s[:, None], ray[:, 2]

    f0, c0 = floor(p0)
    f1, _ = floor(p1)
    ok = c0 < -0.3
    dxy = f1[ok] - f0[ok]
    angle = np.arctan2(dxy[:, 1], dxy[:, 0])
    k = 2 * np.pi / symmetry
    z = np.sum(w[ok] * np.exp(1j * k * angle))
    strength = abs(z) / np.sum(w[ok])
    return np.angle(z) / k, strength


errors, strengths = [], []
rng = np.random.default_rng(1)
for i in range(0, len(F), 3):
    rc, zc, _, rb = cam_state(S[i])
    height = FLOOR + zc
    if height < .3:
        continue
    yaw_err = rng.uniform(-0.6, 0.6)  # pretend the EKF heading is off by up to 34 deg
    wrong = R.from_euler('z', yaw_err) * rc
    out = line_axis(F[i], wrong, height)
    if out is None:
        continue
    axis, strength = out
    correction = (0.0 - axis + np.pi / 4) % (np.pi / 2) - np.pi / 4  # sim lanes along x: axis 0 mod 90
    errors.append(np.degrees((yaw_err + correction + np.pi) % (2 * np.pi) - np.pi))
    strengths.append(strength)
errors, strengths = np.array(errors), np.array(strengths)
good = strengths > 0.3
print(f'heading: n={len(errors)} strong={good.sum()} rms={np.sqrt(np.mean(errors[good] ** 2)):.2f} deg '
      f'p95={np.percentile(np.abs(errors[good]), 95):.2f} max={np.abs(errors[good]).max():.2f} '
      f'strength median={np.median(strengths):.2f} weak_rms={np.sqrt(np.mean(errors[~good] ** 2)) if (~good).any() else 0:.2f}')
