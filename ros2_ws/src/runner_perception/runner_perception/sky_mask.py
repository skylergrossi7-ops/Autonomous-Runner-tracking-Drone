"""Geometry and appearance-based sky rejection for simulation cameras."""

import cv2
import numpy as np


def mask_above_horizon(depth, enabled=False, horizon_fraction=0.46, bgr=None):
    """Invalidate sky while preserving objects extending above the horizon.

    With an RGB frame, the candidate region is deliberately enlarged but only
    blue/bright sky pixels connected to the top border are removed. Without
    RGB, retain the deterministic legacy row mask used by unit tests.
    """
    cloud_depth = np.asarray(depth, dtype=np.float32).copy()
    if not enabled or cloud_depth.ndim != 2 or cloud_depth.size == 0:
        return cloud_depth
    fraction = min(1.0, max(0.0, float(horizon_fraction)))
    horizon_row = int(round(cloud_depth.shape[0] * fraction))
    if bgr is None or np.asarray(bgr).shape[:2] != cloud_depth.shape:
        cloud_depth[:horizon_row, :] = np.nan
        return cloud_depth

    image = np.asarray(bgr, dtype=np.uint8)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue = ((hsv[..., 0] >= 85) & (hsv[..., 0] <= 135)
            & (hsv[..., 1] >= 20) & (hsv[..., 2] >= 80))
    cloud = ((hsv[..., 1] < 45) & (hsv[..., 2] >= 145))
    candidate = np.zeros(cloud_depth.shape, dtype=np.uint8)
    candidate[:horizon_row] = np.uint8((blue | cloud)[:horizon_row])
    candidate = cv2.morphologyEx(
        candidate, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )
    count, labels, _, _ = cv2.connectedComponentsWithStats(candidate, 8)
    top_labels = set(int(value) for value in np.unique(labels[0]))
    sky = np.zeros_like(candidate, dtype=bool)
    for label in top_labels:
        if 0 < label < count:
            sky |= labels == label
    cloud_depth[sky] = np.nan
    return cloud_depth
