import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from humanoid_navigation.profile_loader import load_profile


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="g1"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("perception_mode", default_value="static_only"),
            DeclareLaunchArgument("localization_mode", default_value="sim_ground_truth"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context):
    package_dir = get_package_share_directory("humanoid_navigation")
    nav2_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    profile_name = LaunchConfiguration("robot_profile").perform(context)
    perception_mode = LaunchConfiguration("perception_mode").perform(context)
    localization_mode = LaunchConfiguration("localization_mode").perform(context)

    if localization_mode != "sim_ground_truth":
        raise RuntimeError(f"unsupported localization_mode: {localization_mode}")

    profile_path = os.path.join(package_dir, "profiles", f"{profile_name}.yaml")
    profile = load_profile(profile_path)
    base_params = _default_params_file(package_dir, context)
    merged_params = _merged_nav2_params_file(
        base_params=base_params,
        perception_params=_perception_params_file(package_dir, perception_mode),
        profile=profile,
    )

    map_file = _map_file(package_dir, context)

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            merged_params,
            {
                "yaml_filename": map_file,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": merged_params,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=[
            "-d",
            os.path.join(package_dir, "rviz", "humanoid_navigation.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    g1_adapter = Node(
        package="humanoid_navigation",
        executable="g1_cmd_vel_adapter",
        name="g1_cmd_vel_adapter",
        output="screen",
        parameters=[_adapter_parameters(profile)],
    )

    return [map_server, map_lifecycle_manager, nav2_navigation, rviz_launch, g1_adapter]


def _default_params_file(package_dir, context):
    params_file = LaunchConfiguration("params_file").perform(context)
    if params_file:
        return params_file
    return os.path.join(package_dir, "config", "nav2_params.yaml")


def _map_file(package_dir, context):
    map_file = LaunchConfiguration("map").perform(context)
    if map_file:
        return map_file
    raise RuntimeError(
        "map launch argument is required. Generate a map from the Unitree "
        "IsaacLab env with sim_main.py --export_nav_static_map, then pass "
        "map:=/absolute/path/to/generated.yaml."
    )


def _perception_params_file(package_dir, perception_mode):
    perception_files = {
        "static_only": "perception_static_only.yaml",
        "obstacle_2d": "perception_2d_obstacle.yaml",
        "voxel_3d": "perception_3d_voxel.yaml",
    }
    if perception_mode not in perception_files:
        raise RuntimeError(f"unsupported perception_mode: {perception_mode}")
    return os.path.join(package_dir, "params", perception_files[perception_mode])


def _merged_nav2_params_file(base_params, perception_params, profile):
    merged = _read_yaml(base_params)
    _deep_update(merged, _read_yaml(perception_params))
    _apply_profile_overrides(merged, profile)

    temp_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="humanoid_navigation_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    with temp_file:
        yaml.safe_dump(merged, temp_file, sort_keys=False)
    return temp_file.name


def _read_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _deep_update(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_update(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def _apply_profile_overrides(params, profile):
    frames = profile["frames"]
    topics = profile["topics"]
    footprint = profile["footprint"]
    motion = profile["motion"]

    params["bt_navigator"]["ros__parameters"]["global_frame"] = frames["map"]
    params["bt_navigator"]["ros__parameters"]["robot_base_frame"] = frames["base"]
    params["bt_navigator"]["ros__parameters"]["odom_topic"] = topics["odom"]

    for costmap_name, global_frame in (
        ("local_costmap", frames["odom"]),
        ("global_costmap", frames["map"]),
    ):
        costmap_params = params[costmap_name][costmap_name]["ros__parameters"]
        costmap_params["global_frame"] = global_frame
        costmap_params["robot_base_frame"] = frames["base"]
        costmap_params["footprint"] = footprint["polygon"]
        costmap_params["footprint_padding"] = footprint["padding"]

    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    if (
        follow_path.get("plugin")
        == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    ):
        follow_path["desired_linear_vel"] = min(
            float(follow_path["desired_linear_vel"]),
            float(motion["max_vel_x"]),
        )
        follow_path["rotate_to_heading_angular_vel"] = min(
            float(follow_path["rotate_to_heading_angular_vel"]),
            float(motion["max_vel_theta"]),
        )
        follow_path["max_angular_accel"] = motion["acc_lim_theta"]
    else:
        follow_path["min_vel_x"] = motion["min_vel_x"]
        follow_path["max_vel_x"] = motion["max_vel_x"]
        follow_path["max_vel_y"] = motion["max_vel_y"]
        follow_path["acc_lim_x"] = motion["acc_lim_x"]
        follow_path["acc_lim_y"] = motion["acc_lim_y"]
        follow_path["acc_lim_theta"] = motion["acc_lim_theta"]
        follow_path["max_vel_theta"] = motion["max_vel_theta"]
        if motion["max_vel_y"] == 0.0:
            follow_path["min_vel_y"] = 0.0
            follow_path["vy_samples"] = 1

    smoother = params["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [
        motion["max_vel_x"],
        motion["max_vel_y"],
        motion["max_vel_theta"],
    ]
    smoother["min_velocity"] = [
        motion["min_vel_x"],
        -motion["max_vel_y"],
        -motion["max_vel_theta"],
    ]
    smoother["max_accel"] = [
        motion["acc_lim_x"],
        motion["acc_lim_y"],
        motion["acc_lim_theta"],
    ]
    smoother["max_decel"] = [
        -motion["acc_lim_x"],
        -motion["acc_lim_y"],
        -motion["acc_lim_theta"],
    ]
    smoother["odom_topic"] = topics["odom"]


def _adapter_parameters(profile):
    motion = profile["motion"]
    dds = profile["dds"]
    policy_command = profile.get("policy_command", {})
    return {
        "input_cmd_vel_topic": profile["topics"]["cmd_vel"],
        "transport": dds.get("transport", "udp"),
        "dds_topic": dds["topic"],
        "udp_host": dds.get("udp_host", "127.0.0.1"),
        "udp_port": int(dds.get("udp_port", 18080)),
        "publish_rate_hz": dds["publish_rate_hz"],
        "cmd_timeout_sec": dds["cmd_timeout_sec"],
        "default_height": dds["default_height"],
        "enable_lateral": motion["enable_lateral"],
        "invert_y": dds["invert_y"],
        "invert_yaw": dds["invert_yaw"],
        "min_vel_x": motion["min_vel_x"],
        "max_vel_x": motion["max_vel_x"],
        "max_vel_y": motion["max_vel_y"],
        "max_vel_theta": motion["max_vel_theta"],
        "policy_min_vel_x": policy_command.get("min_vel_x", -0.6),
        "policy_max_vel_x": policy_command.get("max_vel_x", 1.0),
        "policy_max_vel_y": policy_command.get("max_vel_y", 0.5),
        "policy_max_vel_theta": policy_command.get("max_vel_theta", 1.57),
    }
