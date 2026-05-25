# Humanoid Navigation

Nav2 bringup for humanoid navigation, first targeting Unitree G1 in Isaac Sim.

## V1 Launch

V1.0 is a simulator-only static-map Nav2 demo. It does not require depth or
RGBD obstacle updates.

```bash
source install/setup.bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=static_only \
  map:=/absolute/path/to/kitchen_g1_nav_map.yaml
```

The ROS adapter sends G1 policy commands to Unitree IsaacLab over UDP by
default, so the ROS Humble environment does not need `unitree_sdk2py`.
Start the Unitree sim scene with `--enable_nav_udp_cmd_bridge` so the sim
process can receive those packets in the `unitree_sim_lab` environment and
write the existing Wholebody run command channel.
The adapter treats Nav2 `Twist` values as physical velocity commands. It clips
them to the Nav2 motion limits, applies the Unitree sign convention, and uses
the G1 policy command range from `profiles/g1.yaml` only as a final safety
clamp.

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/.worktrees/unitree-g1-nav-task
conda run -n unitree_sim_lab python sim_main.py \
  --device cuda:0 \
  --no_render \
  --enable_cameras \
  --task Isaac-Kitchen-G129-Dex1-Wholebody \
  --robot_type g129 \
  --enable_nav_ros_clock \
  --enable_nav_ros_tf_odom \
  --enable_nav_udp_cmd_bridge \
  --nav_minimal_dds \
  --disable_image_server
```

Generate the map from the Unitree IsaacLab env before HUM-36 acceptance:

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/.worktrees/unitree-g1-nav-task
conda run -n unitree_sim_lab python sim_main.py \
  --device cuda:0 \
  --headless \
  --enable_cameras \
  --task Isaac-Kitchen-G129-Dex1-Wholebody \
  --robot_type g129 \
  --export_nav_static_map /absolute/path/to/kitchen_g1_nav_map.yaml
```

The package still carries `maps/g1_static_warehouse.yaml` as a launch/test
example, but it is not valid for the Unitree IsaacLab acceptance scene.

V1.5 RGBD 2D obstacle mode:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=obstacle_2d \
  map:=/absolute/path/to/kitchen_g1_nav_map.yaml
```

V1.5 optional voxel mode:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=voxel_3d \
  map:=/absolute/path/to/kitchen_g1_nav_map.yaml
```

Optional RViz:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=static_only \
  map:=/absolute/path/to/kitchen_g1_nav_map.yaml \
  rviz:=true
```

To reopen RViz without restarting Nav2, run:

```bash
ros2 launch humanoid_navigation humanoid_navigation_rviz.launch.py
```

Do not relaunch `humanoid_navigation.launch.py` only to reopen RViz. That starts
a second Nav2 stack with the same node and service names.

For `localization_mode:=sim_ground_truth`, do not use RViz `2D Pose Estimate`.
The global pose comes from the simulator TF chain. Send goals with RViz
`Nav2 Goal` or `2D Goal Pose`; both must publish a `PoseStamped` on
`/goal_pose`.

## Required Inputs

- TF: `map -> odom -> base_link`
- Odometry: `/odom`
- Smoothed Nav2 velocity output: `/cmd_vel`
- Nav2 static map generated from the active Unitree IsaacLab env and passed as
  `map:=/absolute/path/to/kitchen_g1_nav_map.yaml`
- Unitree sim UDP command bridge: `127.0.0.1:18080`, writing the existing
  Wholebody run command channel. DDS `rt/run_command/cmd` remains an optional
  legacy transport but is not the V1 default.
- Command diagnostics: `/g1/cmd_vel_debug_markers` for RViz arrows and
  `/g1/policy_cmd_debug` for the adapter-equivalent policy command vector.

V1.5 perception modes additionally require RGBD point cloud
`/g1/head_rgbd/points`.

## G1 Simulation Resources

Use `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/` as the source of truth for G1 simulation assets, policy command behavior, DDS examples, and Isaac Lab integration details.

When launching Isaac Sim for this project, use the conda environment `unitree_sim_lab`. Do not install packages into any conda environment without explicit user approval.

## Acceptance Checks

- `ros2 lifecycle get /bt_navigator` reports `active`.
- RViz shows the static map and planned path; TF, local costmap, global costmap,
  robot model, and RGBD point cloud displays are available but disabled by
  default to keep goal selection unambiguous.
- Sending a Nav2 goal produces `/cmd_vel`.
- `g1_cmd_vel_adapter` sends UDP JSON commands to the Unitree sim command
  bridge.
- Stopping Nav2 velocity output produces a zero policy command within
  `cmd_timeout_sec`.

## Command Direction Diagnostics

`humanoid_navigation.launch.py` starts `g1_cmd_vel_debug_visualizer` alongside
the G1 adapter. RViz displays `/g1/cmd_vel_debug_markers` by default:

- Green arrow: Nav2 `Twist` direction from `/cmd_vel`.
- Yellow arrow: adapter-equivalent G1 policy direction after clipping and
  Unitree axis conversion.
- Text marker: numeric `vx`, `vy`, and `wz` values for both layers.

Record the relevant command and pose topics while sending a goal:

```bash
ros2 bag record /cmd_vel /cmd_vel_nav /g1/policy_cmd_debug /odom /tf
```

## Notes

V1.0 uses a static global map and does not perform dynamic obstacle avoidance.
V1.5 adds local costmap updates from RGBD observations. Neither mode saves or
modifies the static map file.
