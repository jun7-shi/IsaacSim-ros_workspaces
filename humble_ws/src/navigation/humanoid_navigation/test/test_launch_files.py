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


def test_sim_ground_truth_localization_params_document_required_frames():
    params = load_yaml("params/localization_sim_ground_truth.yaml")

    assert params["localization_mode"] == "sim_ground_truth"
    assert params["required_tf"] == ["map", "odom", "base_link"]
    assert params["odom_topic"] == "/odom"
    assert "slam_localization" in params["reserved_modes"]
    assert "external_tf" in params["reserved_modes"]


def test_nav2_params_use_sim_ground_truth_frame_contract():
    params = load_yaml("config/nav2_params.yaml")
    bt_params = params["bt_navigator"]["ros__parameters"]
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]

    assert bt_params["global_frame"] == "map"
    assert bt_params["robot_base_frame"] == "base_link"
    assert bt_params["odom_topic"] == "/odom"
    assert local_params["global_frame"] == "odom"
    assert global_params["global_frame"] == "map"


def test_main_launch_file_declares_required_mode_arguments():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    for argument in (
        "robot_profile",
        "map",
        "params_file",
        "perception_mode",
        "localization_mode",
        "use_sim_time",
        "rviz",
    ):
        assert f'DeclareLaunchArgument("{argument}"' in launch_text


def test_main_launch_defaults_to_g1_obstacle_2d_and_sim_ground_truth():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'default_value="g1"' in launch_text
    assert 'default_value="obstacle_2d"' in launch_text
    assert 'default_value="sim_ground_truth"' in launch_text
    assert 'default_value="true"' in launch_text


def test_main_launch_starts_nav2_and_g1_adapter():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "bringup_launch.py" in launch_text
    assert "g1_cmd_vel_adapter" in launch_text
    assert "nav2_bringup" in launch_text


def test_rviz_config_uses_map_fixed_frame_and_navigation_displays():
    rviz_text = (PACKAGE_ROOT / "rviz" / "humanoid_navigation.rviz").read_text(
        encoding="utf-8"
    )

    assert "Fixed Frame: map" in rviz_text
    assert "/map" in rviz_text
    assert "/g1/head_rgbd/points" in rviz_text
    assert "/local_costmap/costmap" in rviz_text
    assert "/global_costmap/costmap" in rviz_text
    assert "/plan" in rviz_text


def test_readme_documents_static_map_launch_and_required_topics():
    readme_text = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")

    assert "robot_profile:=g1" in readme_text
    assert "localization_mode:=sim_ground_truth" in readme_text
    assert "perception_mode:=obstacle_2d" in readme_text
    assert "map:=/absolute/path/to/map.yaml" in readme_text
    assert "map -> odom -> base_link" in readme_text
    assert "/odom" in readme_text
    assert "/g1/head_rgbd/points" in readme_text
    assert "/cmd_vel_smoothed" in readme_text


def test_v1_acceptance_notes_document_sim_commands_and_blockers():
    acceptance_text = (PACKAGE_ROOT / "docs" / "v1_acceptance.md").read_text(
        encoding="utf-8"
    )

    assert "conda run -n unitree_sim_lab" in acceptance_text
    assert "Isaac-Move-Cylinder-G129-Dex1-Wholebody" in acceptance_text
    assert "ros2 launch humanoid_navigation humanoid_navigation.launch.py" in acceptance_text
    assert "map -> odom -> base_link" in acceptance_text
    assert "/odom" in acceptance_text
    assert "/g1/head_rgbd/points" in acceptance_text
    assert "rt/run_command/cmd" in acceptance_text
    assert "blocked" in acceptance_text.lower()
    assert "PointCloud2" in acceptance_text
