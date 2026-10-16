"""Overlay feature detections on saved bottom frames: current vs candidate detector."""
import glob
import sys

import cv2
import numpy as np


def current(gray):
    return cv2.goodFeaturesToTrack(gray, 200, 0.01, 8)


def candidate(gray, max_features=200, grid=(6, 8)):
    """CLAHE + per-cell Shi-Tomasi so features spread over the whole floor."""
    g = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
    h, w = g.shape
    rows, cols = grid
    per = max(1, max_features // (rows * cols))
    out = []
    for r in range(rows):
        for c in range(cols):
            y0, y1 = r * h // rows, (r + 1) * h // rows
            x0, x1 = c * w // cols, (c + 1) * w // cols
            p = cv2.goodFeaturesToTrack(g[y0:y1, x0:x1], per, 0.01, 5)
            if p is not None:
                out.append(p + np.array([x0, y0], np.float32))
    return np.concatenate(out) if out else None


for path in sorted(glob.glob(f'{sys.argv[1]}/f*.png')):
    img = cv2.imread(path)
    gray = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    panels = []
    for name, fn in (('current', current), ('clahe+grid', candidate)):
        vis = cv2.resize(img, (gray.shape[1] * 2, gray.shape[0] * 2))
        pts = fn(gray)
        for x, y in (pts.reshape(-1, 2) if pts is not None else []):
            cv2.circle(vis, (int(x * 2), int(y * 2)), 4, (0, 0, 255), 2)
        cv2.putText(vis, f'{name}: {0 if pts is None else len(pts)}', (10, 30), 0, 1, (0, 255, 255), 2)
        panels.append(vis)
    cv2.imwrite(path.replace('/f', '/overlay_f').replace('.png', '.jpg'), np.hstack(panels))
