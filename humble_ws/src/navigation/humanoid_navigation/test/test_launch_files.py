from pathlib import Path
import importlib.util
import sys

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


def test_nav2_defaults_use_static_map_path_follower_for_v1():
    params = load_yaml("config/nav2_params.yaml")
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert (
        follow_path["plugin"]
        == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert follow_path["use_collision_detection"] is False
    assert follow_path["use_rotate_to_heading"] is False
    assert follow_path["allow_reversing"] is False


def test_g1_static_controller_counts_rotation_as_progress():
    params = load_yaml("config/nav2_params.yaml")
    controller_params = params["controller_server"]["ros__parameters"]
    progress_checker = controller_params["progress_checker"]
    follow_path = controller_params["FollowPath"]

    assert controller_params["progress_checker_plugin"] == "progress_checker"
    assert progress_checker["plugin"] == "nav2_controller::PoseProgressChecker"
    assert progress_checker["required_movement_radius"] <= 0.2
    assert progress_checker["required_movement_angle"] <= 0.2
    assert progress_checker["movement_time_allowance"] >= 20.0
    assert follow_path["desired_linear_vel"] >= 0.5


def test_controller_uses_humble_goal_checker_plugins_key():
    params = load_yaml("config/nav2_params.yaml")
    controller_params = params["controller_server"]["ros__parameters"]

    assert controller_params["goal_checker_plugins"] == ["stopped_goal_checker"]
    assert "goal_checker_plugin" not in controller_params


def test_g1_profile_keeps_policy_command_range_as_safety_clamp():
    profile = load_yaml("profiles/g1.yaml")
    policy_command = profile["policy_command"]

    assert policy_command["min_vel_x"] == -0.6
    assert policy_command["max_vel_x"] == 1.0
    assert policy_command["max_vel_theta"] == 1.57


def test_g1_profile_uses_keyboard_like_command_rate_for_sim_policy():
    profile = load_yaml("profiles/g1.yaml")

    assert profile["motion"]["max_vel_x"] >= 0.8
    assert profile["dds"]["publish_rate_hz"] >= 100.0
    assert profile["dds"]["cmd_timeout_sec"] >= 0.5


def test_profile_overrides_keep_pure_pursuit_controller_params():
    params = load_yaml("config/nav2_params.yaml")
    profile = load_yaml("profiles/g1.yaml")
    launch_path = PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py"
    spec = importlib.util.spec_from_file_location("humanoid_navigation_launch", launch_path)
    launch_module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(PACKAGE_ROOT))
    spec.loader.exec_module(launch_module)

    launch_module._apply_profile_overrides(params, profile)

    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert (
        follow_path["plugin"]
        == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert follow_path["desired_linear_vel"] <= profile["motion"]["max_vel_x"]
    assert follow_path["rotate_to_heading_angular_vel"] <= profile["motion"]["max_vel_theta"]
    assert follow_path["max_angular_accel"] == profile["motion"]["acc_lim_theta"]
    assert "vx_samples" not in follow_path
    assert "vy_samples" not in follow_path
    assert "vtheta_samples" not in follow_path


def test_global_costmap_uses_static_map_layer():
    params = load_yaml("config/nav2_params.yaml")
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]

    assert "static_layer" in global_params["plugins"]
    assert global_params["static_layer"]["plugin"] == "nav2_costmap_2d::StaticLayer"
    assert params["map_server"]["ros__parameters"]["topic_name"] == "map"


def test_lifecycle_manager_activates_map_server_for_map_topic():
    params = load_yaml("config/nav2_params.yaml")
    lifecycle_nodes = params["lifecycle_manager"]["ros__parameters"]["node_names"]

    assert "map_server" in lifecycle_nodes


def test_behavior_server_provides_default_nav2_recovery_actions():
    params = load_yaml("config/nav2_params.yaml")
    behavior_params = params["behavior_server"]["ros__parameters"]

    assert "backup" in behavior_params["behavior_plugins"]
    assert behavior_params["backup"]["plugin"] == "nav2_behaviors/BackUp"


def test_obstacle_2d_mode_uses_pointcloud_obstacle_layer():
    params = load_yaml("params/perception_2d_obstacle.yaml")
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    obstacle_layer = local_params["rgbd_obstacle_layer"]

    assert "rgbd_obstacle_layer" in local_params["plugins"]
    assert obstacle_layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle_layer["pointcloud"]["data_type"] == "PointCloud2"
    assert obstacle_layer["pointcloud"]["marking"] is True
    assert obstacle_layer["pointcloud"]["clearing"] is True


def test_static_only_mode_uses_no_sensor_obstacle_layer():
    params = load_yaml("params/perception_static_only.yaml")
    local_params = params["local_costmap"]["local_costmap"]["ros__parameters"]

    assert local_params["plugins"] == ["inflation_layer"]
    assert "rgbd_obstacle_layer" not in local_params
    assert "rgbd_voxel_layer" not in local_params


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


def test_main_launch_defaults_to_g1_static_only_and_sim_ground_truth():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'default_value="g1"' in launch_text
    assert 'default_value="static_only"' in launch_text
    assert 'default_value="sim_ground_truth"' in launch_text
    assert 'default_value="true"' in launch_text
    assert "--export_nav_static_map" in launch_text
    assert "map launch argument is required" in launch_text


def test_main_launch_starts_nav2_and_g1_adapter():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "navigation_launch.py" in launch_text
    assert "rviz_launch.py" not in launch_text
    assert "bringup_launch.py" not in launch_text
    assert "g1_cmd_vel_adapter" in launch_text
    assert "nav2_bringup" in launch_text


def test_main_launch_starts_rviz_without_shutdown_handler():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'package="rviz2"' in launch_text
    assert 'executable="rviz2"' in launch_text
    assert "OnProcessExit" not in launch_text
    assert "Shutdown" not in launch_text


def test_rviz_only_launch_reopens_rviz_without_nav2_nodes():
    launch_text = (
        PACKAGE_ROOT / "launch" / "humanoid_navigation_rviz.launch.py"
    ).read_text(encoding="utf-8")

    assert 'package="rviz2"' in launch_text
    assert 'executable="rviz2"' in launch_text
    assert "nav2_bringup" not in launch_text
    assert "lifecycle_manager" not in launch_text
    assert "map_server" not in launch_text


def test_sim_ground_truth_launch_does_not_start_amcl():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "nav2_map_server" in launch_text
    assert "lifecycle_manager_localization" in launch_text
    assert '"node_names": ["map_server"]' in launch_text
    assert "amcl" not in launch_text.lower()


def test_nav2_launch_disables_composition_for_sim_ground_truth_debuggability():
    launch_text = (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"use_composition": "False"' in launch_text


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


def test_rviz_config_exposes_nav2_goal_workflow():
    rviz_text = (PACKAGE_ROOT / "rviz" / "humanoid_navigation.rviz").read_text(
        encoding="utf-8"
    )

    assert "nav2_rviz_plugins/Navigation 2" in rviz_text
    assert "nav2_rviz_plugins/GoalTool" in rviz_text
    assert "/goal_pose" in rviz_text
    assert "rviz_common/Tool Properties" in rviz_text
    assert "rviz_default_plugins/TopDownOrtho" in rviz_text


def test_readme_documents_static_map_launch_and_required_topics():
    readme_text = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")

    assert "robot_profile:=g1" in readme_text
    assert "localization_mode:=sim_ground_truth" in readme_text
    assert "perception_mode:=static_only" in readme_text
    assert "--export_nav_static_map" in readme_text
    assert "kitchen_g1_nav_map.yaml" in readme_text
    assert "map -> odom -> base_link" in readme_text
    assert "/odom" in readme_text
    assert "humanoid_navigation_rviz.launch.py" in readme_text
    assert "/g1/head_rgbd/points" in readme_text
    assert "/cmd_vel" in readme_text


def test_v1_acceptance_notes_document_sim_commands_and_blockers():
    acceptance_text = (PACKAGE_ROOT / "docs" / "v1_acceptance.md").read_text(
        encoding="utf-8"
    )

    assert "conda run -n unitree_sim_lab" in acceptance_text
    assert "Isaac-Kitchen-G129-Dex1-Wholebody" in acceptance_text
    assert "KitchenRoom.usd" in acceptance_text
    assert "--enable_nav_ros_clock" in acceptance_text
    assert "--enable_nav_ros_tf_odom" in acceptance_text
    assert "--nav_minimal_dds" in acceptance_text
    assert "--disable_image_server" in acceptance_text
    assert "--enable_dex1_dds" not in acceptance_text
    assert "--enable_nav_ros_pointcloud" not in acceptance_text
    assert "--headless" in acceptance_text
    assert "  --no_render" in acceptance_text
    assert "ros2 launch humanoid_navigation humanoid_navigation.launch.py" in acceptance_text
    assert "perception_mode:=static_only" in acceptance_text
    assert "--export_nav_static_map" in acceptance_text
    assert "kitchen_g1_nav_map.yaml" in acceptance_text
    assert "isaacsim.asset.gen.omap" in acceptance_text
    assert "OnPlaybackTick -> IsaacReadSimulationTime -> ROS2PublishClock" in acceptance_text
    assert "map -> odom -> base_link" in acceptance_text
    assert "/clock" in acceptance_text
    assert "/odom" in acceptance_text
    assert "/g1/head_rgbd/points" not in acceptance_text
    assert "5a09d46" in acceptance_text
    assert "6daaefd" in acceptance_text
    assert "Latest Runtime Result" in acceptance_text
    assert "short `/navigate_to_pose` goal succeeded" in acceptance_text
    assert "Passed" in acceptance_text
    assert "no ROS `PointCloud2` publisher was found" not in acceptance_text
    assert "rt/run_command/cmd" in acceptance_text
    assert "blocked" not in acceptance_text.lower()
    assert "PointCloud2" not in acceptance_text


def test_default_static_map_is_packaged_for_humanoid_navigation():
    map_yaml = load_yaml("maps/g1_static_warehouse.yaml")
    map_image = PACKAGE_ROOT / "maps" / map_yaml["image"]

    assert map_yaml["image"] == "g1_static_warehouse.png"
    assert map_yaml["resolution"] == 0.05
    assert map_yaml["origin"] == [-11.975, -17.975, 0.0]
    assert map_yaml["occupied_thresh"] == 0.65
    assert map_image.exists()


def test_unitree_isaaclab_static_map_doc_explains_export_flow():
    doc_text = (
        PACKAGE_ROOT / "docs" / "unitree_isaaclab_static_map.md"
    ).read_text(encoding="utf-8")

    assert "Isaac-Kitchen-G129-Dex1-Wholebody" in doc_text
    assert "conda run -n unitree_sim_lab" in doc_text
    assert "--export_nav_static_map" in doc_text
    assert "kitchen_g1_nav_map.yaml" in doc_text
    assert "isaacsim.asset.gen.omap" in doc_text
    assert "UsdPhysics.CollisionAPI" in doc_text
    assert "/World/envs/env_0/Robot" in doc_text
    assert "/World/envs/env_0/Object" in doc_text


def test_g1_nav_usd_asset_layer_lives_in_package_assets():
    asset_path = (
        PACKAGE_ROOT
        / "assets"
        / "g1_nav"
        / "g1_29dof_with_dex1_nav_depth.usd"
    )
    asset_text = asset_path.read_text(encoding="utf-8")

    assert 'defaultPrim = "Robot"' in asset_text
    assert "g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd" in asset_text
    assert 'def Camera "head_d435_depth_camera"' in asset_text
    assert 'custom string rosTopic = "/g1/head_rgbd/points"' in asset_text
    assert 'custom string rosFrameId = "g1_head_d435_depth_optical_frame"' in asset_text
    assert "xformOp:orient" in asset_text
    assert "xformOp:scale" in asset_text
    assert "xformOp:rotateXYZ" not in asset_text


def test_g1_nav_asset_manifest_documents_carter_map_reuse():
    manifest = load_yaml("assets/g1_nav/manifest.yaml")

    assert manifest["asset"] == "g1_29dof_with_dex1_nav_depth.usd"
    assert manifest["mount_link"] == "d435_link"
    assert manifest["pointcloud_topic"] == "/g1/head_rgbd/points"
    assert manifest["default_map_package"] == "humanoid_navigation"
    assert manifest["default_map"] == "maps/g1_static_warehouse.yaml"


def test_v2_backlog_defines_slam_localization_path():
    backlog_text = (PACKAGE_ROOT / "docs" / "v2_backlog.md").read_text(
        encoding="utf-8"
    )

    assert "HUM-37" in backlog_text
    assert "slam_localization" in backlog_text
    assert "RGBD SLAM" in backlog_text
    assert "map -> odom" in backlog_text
    assert "static-map localization" in backlog_text
    assert "map save/update" in backlog_text
    assert "`map`, `odom`, `base_link`, `/odom`" in backlog_text


def test_v2_backlog_defines_real_g1_validation_path():
    backlog_text = (PACKAGE_ROOT / "docs" / "v2_backlog.md").read_text(
        encoding="utf-8"
    )

    assert "HUM-38" in backlog_text
    assert "real G1" in backlog_text
    assert "low-speed" in backlog_text
    assert "/g1/head_rgbd/points" in backlog_text
    assert "camera extrinsics" in backlog_text
    assert "emergency stop" in backlog_text
    assert "fall detection" in backlog_text
    assert "communication timeout" in backlog_text


def test_v2_backlog_defines_bumi_profile_requirements():
    backlog_text = (PACKAGE_ROOT / "docs" / "v2_backlog.md").read_text(
        encoding="utf-8"
    )

    assert "HUM-39" in backlog_text
    assert "Bumi" in backlog_text
    assert "profiles/bumi.yaml" in backlog_text
    assert "same schema as `profiles/g1.yaml`" in backlog_text
    assert "geometry_msgs/Twist" in backlog_text
    assert "dedicated adapter" in backlog_text
    assert "Nav2 `cmd_vel` contract" in backlog_text
    assert "missing Bumi data" in backlog_text
