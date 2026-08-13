"""Geometry-based sky rejection for fixed-pitch simulation cameras."""

import numpy as np


def mask_above_horizon(depth, enabled=False, horizon_fraction=0.46):
    """Return a cloud-depth copy with rows above the horizon set invalid."""
    cloud_depth = np.asarray(depth, dtype=np.float32).copy()
    if not enabled or cloud_depth.ndim != 2 or cloud_depth.size == 0:
        return cloud_depth
    fraction = min(1.0, max(0.0, float(horizon_fraction)))
    horizon_row = int(round(cloud_depth.shape[0] * fraction))
    cloud_depth[:horizon_row, :] = np.nan
    return cloud_depth
