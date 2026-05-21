# Humanoid Navigation Design

Date: 2026-05-21

## Summary

Build a Nav2-based humanoid navigation stack for ROS 2 Humble, first targeting Unitree G1 in Isaac Sim. The first version uses a prebuilt static global map, RGBD point cloud updates for runtime costmaps, simulation ground-truth localization, and an adapter from Nav2 `cmd_vel` to the existing Unitree DDS locomotion command. Bumi support, true robot deployment, and RGBD SLAM/map saving are deferred to later phases but are accounted for through profiles and launch-time mode switches.

## Goals

- Create a new generic `humanoid_navigation` package.
- Support Unitree G1 as the first robot profile.
- Import a static global map through Nav2 `map_server`.
- Use RGBD observations to update Nav2 runtime costmaps.
- Provide two RGBD perception modes:
  - Default 2D costmap mode: `PointCloud2` into Nav2 `ObstacleLayer`.
  - Optional 3D voxel mode: `PointCloud2` into Nav2 `VoxelLayer`.
- Use simulation ground truth for V1 localization while keeping interfaces for SLAM localization or external TF.
- Convert Nav2 velocity output into Unitree DDS commands for the existing G1 locomotion policy.
- Default to non-holonomic Nav2 behavior while preserving an optional lateral velocity path for G1.

## Non-Goals

- V1 does not guarantee real G1 deployment.
- V1 does not build, save, or rewrite map files from RGBD observations.
- V1 does not implement RGBD SLAM.
- V1 does not add Bumi-specific behavior beyond keeping the package profile-ready.
- V1 does not implement a custom Nav2 costmap layer unless standard `ObstacleLayer` and `VoxelLayer` are insufficient.

## Phasing

### V1: G1 Simulation Navigation

V1 must run in Isaac Sim with Unitree G1. It loads a known static map, updates local costmaps from RGBD point clouds, uses simulation ground-truth localization, and sends Nav2 motion commands into the existing G1 velocity-command policy through Unitree DDS.

### V2: Real Robot and SLAM Localization

V2 adds low-speed real G1 validation, SLAM or VIO localization against a prebuilt static map, and optional map save/update workflows. Bumi is added as a new robot profile after the G1 interfaces are stable.

## Architecture

The system is organized around a generic `humanoid_navigation` package with robot-specific profiles. The package owns Nav2 launch files, parameter templates, perception mode parameters, RViz config, and the Unitree G1 command adapter.

Runtime flow:

```text
static map -> map_server -> global costmap StaticLayer
RGBD PointCloud2 -> filters/costmap observation source -> ObstacleLayer or VoxelLayer
simulation ground truth -> map/odom/base_link TF
Nav2 planner/controller -> cmd_vel -> velocity smoother -> g1_cmd_vel_adapter
g1_cmd_vel_adapter -> Unitree DDS rt/run_command/cmd -> G1 locomotion policy
```

The primary launch file exposes these mode switches:

- `robot_profile`: first value is `g1`.
- `perception_mode`: `obstacle_2d` by default, `voxel_3d` optional.
- `localization_mode`: `sim_ground_truth` by default, with `slam_localization` and `external_tf` reserved.
- `map`: path to Nav2-compatible `map.yaml`.
- `use_sim_time`: true by default for Isaac Sim.

## Package Layout

```text
humanoid_navigation/
  launch/
    humanoid_navigation.launch.py
  config/
    nav2_params.yaml
  profiles/
    g1.yaml
  params/
    perception_2d_obstacle.yaml
    perception_3d_voxel.yaml
    localization_sim_ground_truth.yaml
  humanoid_navigation/
    g1_cmd_vel_adapter.py
  rviz/
    humanoid_navigation.rviz
  package.xml
  setup.py
```

The package should use `ament_python` because V1 includes the Python `g1_cmd_vel_adapter.py` node. The main launch file reads the selected profile and injects frame names, topics, footprint, velocity limits, perception topic, and adapter settings into Nav2 and adapter parameters.

## Robot Profile

`profiles/g1.yaml` contains G1-specific values:

- `map_frame`: `map`
- `odom_frame`: `odom`
- `base_frame`: `base_link`
- `odom_topic`: profile-specific odometry topic, default `/odom`
- `rgbd_pointcloud_topic`: head RGBD `PointCloud2` topic
- `footprint`: humanoid navigation footprint or radius
- `max_vel_x`, `min_vel_x`, `max_vel_y`, `max_vel_theta`
- `acc_lim_x`, `acc_lim_y`, `acc_lim_theta`
- `enable_lateral`: false by default
- `dds_topic`: `rt/run_command/cmd`
- `default_height`: `0.8`
- `invert_y`: true by default to match the current keyboard command script
- `invert_yaw`: true by default to match the current keyboard command script
- `cmd_timeout_sec`: adapter timeout before zeroing velocity
- `adapter_publish_rate_hz`: fixed DDS publishing rate

Velocity limits for V1 should be more conservative than the keyboard policy range. Initial safe values are:

- `x`: `[-0.2, 0.5]` m/s
- `y`: `[0.0, 0.0]` m/s by default
- `yaw`: `[-0.8, 0.8]` rad/s

The existing keyboard script supports wider policy command ranges, but those ranges are not the default Nav2 limits.

## Static Map and Runtime Updates

V1 imports a static global map with Nav2 `map_server`. The global costmap includes `StaticLayer` as the source of the known world.

RGBD observations update runtime costmaps only:

- Local costmap receives RGBD obstacles by default.
- Global costmap can receive RGBD obstacles through a profile or launch option.
- The original static map file is not modified by V1.
- Map saving and map correction belong to V2.

This keeps global planning stable while allowing local avoidance from the head RGBD camera.

## Perception Modes

### Default: `obstacle_2d`

The RGBD camera provides or is converted to `sensor_msgs/PointCloud2`. Nav2 `ObstacleLayer` consumes the point cloud directly as an observation source and projects valid points into the 2D costmap.

Filtering requirements:

- Range limits prevent distant or invalid depth from polluting the costmap.
- Height limits exclude floor noise and high objects outside navigation relevance.
- Self-observation should be reduced with camera crop, height filtering, or a self-filter if the robot body appears in the point cloud.
- Clearing must be enabled so obstacles disappear after they leave the RGBD view.

This mode is the V1 default because it uses standard Nav2 behavior and does not require laser data.

### Optional: `voxel_3d`

The same `PointCloud2` source feeds Nav2 `VoxelLayer`. This keeps more vertical structure before projecting into the costmap.

This mode is included in V1 as an experimental mode. It must launch and show costmap updates, but V1 success is based on the default `obstacle_2d` mode.

## Localization Modes

### `sim_ground_truth`

V1 uses Isaac Sim ground truth or simulator-provided odometry/TF to maintain:

```text
map -> odom -> base_link
```

Nav2 must consistently see:

- `global_frame: map`
- `robot_base_frame: base_link`
- `odom_frame: odom`
- `odom_topic`: from the G1 profile

If Isaac Sim publishes the complete TF chain, the launch file only remaps parameters. If the chain is incomplete, a small bridge provides the missing `map -> odom` or odometry transform.

### `slam_localization`

Reserved for a later phase. It will localize against an existing static map with RGBD/VIO/SLAM components and publish the same Nav2-facing TF contract.

### `external_tf`

Reserved for externally supplied localization, such as motion capture or a separate robot state estimator.

## Command Adapter

`g1_cmd_vel_adapter.py` bridges Nav2 and the existing G1 policy command interface.

Inputs:

- `geometry_msgs/Twist` from `/cmd_vel` or `/cmd_vel_smoothed`

Output:

- Unitree SDK2 DDS `std_msgs/String_`
- DDS topic: `rt/run_command/cmd`
- Payload string: `[x_vel, y_vel, yaw_vel, height]`

The current G1 keyboard command script sends:

```text
[x_vel, -y_vel, -yaw_vel, height]
```

The adapter must expose `invert_y` and `invert_yaw` parameters so the sign convention is explicit and configurable.

Adapter behavior:

- Clip velocities to profile limits.
- Force `y = 0` when `enable_lateral` is false.
- Publish continuously at the configured rate because the receiver resets to a default command after reading.
- On command timeout, publish `[0.0, 0.0, 0.0, default_height]`.
- Exit clearly if DDS initialization fails.
- Log clipping and stale command events at a throttled rate.

## Nav2 Configuration

V1 uses standard Nav2 servers:

- `map_server`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `behavior_server`
- `velocity_smoother`
- `local_costmap`
- `global_costmap`
- `lifecycle_manager`

Controller defaults:

- DWB local planner in non-holonomic mode.
- `max_vel_y = 0.0` by default.
- Lateral velocity parameters remain in the template so a profile can enable them later.
- Backward and recovery behavior limits remain conservative for humanoid stability.

Recovery behavior:

- `spin` can remain enabled.
- `backup` is disabled by default or constrained to a very small speed/distance until tested with G1 policy stability.
- `wait` remains enabled.

## Error Handling and Safety

The adapter and launch configuration must fail loudly when required interfaces are missing.

Required checks:

- Missing `/cmd_vel` causes timeout and zero DDS velocity.
- Missing DDS initialization causes adapter node failure.
- Missing point cloud source leaves the costmap without RGBD updates and must be visible in logs/RViz.
- Missing TF causes Nav2 transform errors rather than silent operation.
- Over-limit velocity commands are clipped.
- Disabled lateral mode always publishes zero lateral velocity.

Simulation safety defaults:

- Conservative speed and acceleration limits.
- Explicit command timeout.
- Continuous zero command after Nav2 stops sending velocity.
- Clear RViz visibility for TF, footprint, costmaps, paths, and point cloud.

## Acceptance Criteria

V1 is accepted when all of these are true in Isaac Sim:

1. `humanoid_navigation` launches with `robot_profile=g1`, `localization_mode=sim_ground_truth`, and `perception_mode=obstacle_2d`.
2. Nav2 lifecycle nodes become active.
3. A supplied static map is visible in RViz and used by the global costmap.
4. The TF chain `map -> odom -> base_link` is available and stable.
5. The G1 footprint is visible in RViz.
6. RGBD point cloud observations update the local costmap.
7. Obstacles clear from the local costmap after leaving the RGBD view.
8. Sending a Nav2 goal produces `/cmd_vel` and a corresponding DDS command.
9. G1 moves toward the goal in simulation and reaches within the configured tolerance.
10. Stopping `/cmd_vel` causes the adapter to publish zero velocity within `cmd_timeout_sec`.
11. `perception_mode=voxel_3d` launches and produces visible costmap updates.
12. Changing the `map` argument loads a different static map without code changes.

## Implementation Notes

Do not edit `carter_navigation` for the humanoid stack. Existing Carter examples are useful references for Nav2 launch and parameter structure only.

The existing G1 DDS interface currently publishes commands to `rt/run_command/cmd` as a stringified four-element list. The adapter should reuse that interface instead of changing the policy receiver in V1.

If the RGBD camera only publishes depth images, add a standard ROS depth-to-point-cloud conversion stage before the Nav2 costmap observation source.

## References

- Existing Carter Nav2 configuration in `humble_ws/src/navigation/carter_navigation`.
- Existing G1 command publisher in `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/send_commands_keyboard.py`.
- Existing G1 command receiver in `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/action_provider/action_provider_wh_dds.py`.
