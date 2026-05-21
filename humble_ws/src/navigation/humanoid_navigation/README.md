# Humanoid Navigation

Nav2 bringup for humanoid navigation, first targeting Unitree G1 in Isaac Sim.

## V1 Launch

```bash
source install/setup.bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=obstacle_2d \
  map:=/absolute/path/to/map.yaml
```

Optional voxel mode:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=voxel_3d \
  map:=/absolute/path/to/map.yaml
```

Optional RViz:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=obstacle_2d \
  map:=/absolute/path/to/map.yaml \
  rviz:=true
```

## Required Inputs

- TF: `map -> odom -> base_link`
- Odometry: `/odom`
- RGBD point cloud: `/g1/head_rgbd/points`
- Smoothed Nav2 velocity output: `/cmd_vel_smoothed`
- Nav2 static map: `map.yaml` plus image file
- Unitree DDS locomotion command receiver: `rt/run_command/cmd`

## G1 Simulation Resources

Use `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/` as the source of truth for G1 simulation assets, policy command behavior, DDS examples, and Isaac Lab integration details.

When launching Isaac Sim for this project, use the conda environment `unitree_sim_lab`. Do not install packages into any conda environment without explicit user approval.

## Acceptance Checks

- `ros2 lifecycle get /bt_navigator` reports `active`.
- RViz shows map, G1 footprint, TF, local costmap, global costmap, and planned path.
- RGBD obstacles appear in local costmap.
- Removing an obstacle clears it from local costmap.
- Sending a Nav2 goal produces `/cmd_vel_smoothed`.
- `g1_cmd_vel_adapter` publishes DDS commands to `rt/run_command/cmd`.
- Stopping Nav2 velocity output produces zero DDS velocity within `cmd_timeout_sec`.

## Notes

V1 uses a static global map and updates only runtime costmaps from RGBD observations. It does not save or modify the static map file.
