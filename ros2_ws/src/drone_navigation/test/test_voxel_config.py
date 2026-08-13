from pathlib import Path

import yaml


CONFIG_PATH = (
    Path(__file__).parents[1] / "config" / "nav2_voxel_params.yaml"
)


def load_costmap_parameters():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    return config["local_costmap"]["local_costmap"]["ros__parameters"]


def test_local_costmap_uses_voxel_layer_not_obstacle_layer():
    params = load_costmap_parameters()

    assert params["plugins"] == ["voxel_layer", "inflation_layer"]
    assert "obstacle_layer" not in params
    assert params["voxel_layer"]["plugin"] == (
        "nav2_costmap_2d::VoxelLayer"
    )


def test_voxel_layer_uses_filtered_pointcloud_for_marking_and_clearing():
    params = load_costmap_parameters()
    voxel_layer = params["voxel_layer"]
    source = voxel_layer["filtered_points"]

    assert voxel_layer["observation_sources"] == "filtered_points"
    assert source["topic"] == "/camera/depth_ai/filtered_points"
    assert source["data_type"] == "PointCloud2"
    assert source["marking"] is True
    assert source["clearing"] is True


def test_voxel_height_count_respects_nav2_limit():
    voxel_layer = load_costmap_parameters()["voxel_layer"]

    assert 1 <= voxel_layer["z_voxels"] <= 16
