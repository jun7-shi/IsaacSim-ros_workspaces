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
  perception_mode:=static_only
```

If `map:=...` is omitted, the launch file uses the package-owned default map:

```text
share/humanoid_navigation/maps/g1_static_warehouse.yaml
```

An explicit static map is still accepted:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=static_only \
  map:=/absolute/path/to/map.yaml
```

V1.5 RGBD 2D obstacle mode:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=obstacle_2d
```

V1.5 optional voxel mode:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=voxel_3d
```

Optional RViz:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=static_only \
  rviz:=true
```

## Required Inputs

- TF: `map -> odom -> base_link`
- Odometry: `/odom`
- Smoothed Nav2 velocity output: `/cmd_vel_smoothed`
- Nav2 static map: `g1_static_warehouse.yaml` plus image file, or an explicit
  `map:=/absolute/path/to/map.yaml`
- Unitree DDS locomotion command receiver: `rt/run_command/cmd`

V1.5 perception modes additionally require RGBD point cloud
`/g1/head_rgbd/points`.

## G1 Simulation Resources

Use `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/` as the source of truth for G1 simulation assets, policy command behavior, DDS examples, and Isaac Lab integration details.

When launching Isaac Sim for this project, use the conda environment `unitree_sim_lab`. Do not install packages into any conda environment without explicit user approval.

## Acceptance Checks

- `ros2 lifecycle get /bt_navigator` reports `active`.
- RViz shows map, G1 footprint, TF, local costmap, global costmap, and planned path.
- Sending a Nav2 goal produces `/cmd_vel_smoothed`.
- `g1_cmd_vel_adapter` publishes DDS commands to `rt/run_command/cmd`.
- Stopping Nav2 velocity output produces zero DDS velocity within `cmd_timeout_sec`.

## Notes

V1.0 uses a static global map and does not perform dynamic obstacle avoidance.
V1.5 adds local costmap updates from RGBD observations. Neither mode saves or
modifies the static map file.
