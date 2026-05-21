from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def load_yaml(relative_path):
    with (PACKAGE_ROOT / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_nav2_params_define_required_servers_and_costmaps():
    params = load_yaml("config/nav2_params.yaml")

    assert "bt_navigator" in params
    assert "controller_server" in params
    assert "planner_server" in params
    assert "behavior_server" in params
    assert "velocity_smoother" in params
    assert "local_costmap" in params
    assert "global_costmap" in params
    assert "map_server" in params
    assert "lifecycle_manager" in params


def test_nav2_defaults_are_non_holonomic_for_v1():
    params = load_yaml("config/nav2_params.yaml")
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["max_vel_y"] == 0.0
    assert follow_path["min_vel_y"] == 0.0
    assert follow_path["vy_samples"] == 1


def test_global_costmap_uses_static_map_layer():
    params = load_yaml("config/nav2_params.yaml")
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]

    assert "static_layer" in global_params["plugins"]
    assert global_params["static_layer"]["plugin"] == "nav2_costmap_2d::StaticLayer"
    assert params["map_server"]["ros__parameters"]["topic_name"] == "map"


def test_obstacle_2d_mode_uses_pointcloud_obstacle_layer():
    params = load_yaml("params/perception_2d_obstacle.yaml")
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    obstacle_layer = local_params["rgbd_obstacle_layer"]

    assert "rgbd_obstacle_layer" in local_params["plugins"]
    assert obstacle_layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle_layer["pointcloud"]["data_type"] == "PointCloud2"
    assert obstacle_layer["pointcloud"]["marking"] is True
    assert obstacle_layer["pointcloud"]["clearing"] is True


def test_voxel_3d_mode_uses_voxel_layer():
    params = load_yaml("params/perception_3d_voxel.yaml")
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    voxel_layer = local_params["rgbd_voxel_layer"]

    assert "rgbd_voxel_layer" in local_params["plugins"]
    assert voxel_layer["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert voxel_layer["pointcloud"]["data_type"] == "PointCloud2"
    assert voxel_layer["pointcloud"]["marking"] is True
    assert voxel_layer["pointcloud"]["clearing"] is True
