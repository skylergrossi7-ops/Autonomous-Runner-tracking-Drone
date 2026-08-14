import numpy as np

from runner_perception.sky_mask import mask_above_horizon


def test_enabled_mask_removes_only_rows_above_horizon():
    depth = np.ones((10, 4), dtype=np.float32)
    masked = mask_above_horizon(depth, enabled=True, horizon_fraction=0.4)

    assert np.isnan(masked[:4]).all()
    assert np.isfinite(masked[4:]).all()
    assert np.isfinite(depth).all()


def test_disabled_mask_keeps_cloud_depth_unchanged():
    depth = np.arange(12, dtype=np.float32).reshape(3, 4)

    assert np.array_equal(mask_above_horizon(depth), depth)


def test_horizon_fraction_is_clamped():
    depth = np.ones((3, 2), dtype=np.float32)

    assert np.isnan(mask_above_horizon(depth, True, 2.0)).all()
    assert np.isfinite(mask_above_horizon(depth, True, -1.0)).all()


def test_rgb_mask_preserves_non_sky_object_above_horizon():
    depth = np.ones((20, 20), dtype=np.float32)
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[:] = (220, 170, 120)  # blue Gazebo sky in BGR
    image[2:16, 8:12] = (20, 180, 230)  # tall yellow obstacle

    masked = mask_above_horizon(
        depth, enabled=True, horizon_fraction=0.8, bgr=image
    )

    assert np.isnan(masked[:16, :8]).all()
    assert np.isfinite(masked[2:16, 8:12]).all()
    assert np.isfinite(masked[16:]).all()
