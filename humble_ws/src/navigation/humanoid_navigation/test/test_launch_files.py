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
