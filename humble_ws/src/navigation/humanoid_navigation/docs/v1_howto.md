# V1 Howto and Issue Log

Issue: HUM-36

Date: 2026-05-25

This document records the V1 Unitree G1 simulator navigation bringup. It is
intended as the handoff reference before starting V1.5 RGBD local perception.

## V1 Runtime Contract

V1 is a static-map simulator bringup:

```text
robot: Unitree G1 Wholebody in Isaac Sim
global map: nav_msgs/OccupancyGrid loaded by nav2_map_server
local updates: none; local costmap uses the static map
localization: simulator ground truth TF
TF: map -> odom -> base_link
odom: /odom
velocity command: /cmd_vel -> g1_cmd_vel_adapter -> Unitree sim UDP bridge
```

V1.5 should keep this Nav2-facing contract and add RGBD local obstacle updates.

## Launch Commands

Start the Unitree simulator from the Unitree navigation worktree:

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/.worktrees/unitree-g1-nav-task
conda run -n unitree_sim_lab python sim_main.py \
  --device cuda:0 \
  --enable_cameras \
  --task Isaac-Kitchen-G129-Dex1-Wholebody \
  --robot_type g129 \
  --enable_nav_ros_clock \
  --enable_nav_ros_tf_odom \
  --enable_nav_udp_cmd_bridge \
  --nav_minimal_dds \
  --disable_image_server
```

Start Nav2 and RViz:

```bash
cd /data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws
unset LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=static_only \
  use_sim_time:=true \
  map:=/absolute/path/to/kitchen_g1_nav_map.yaml \
  rviz:=true
```

Reopen only RViz without restarting Nav2:

```bash
ros2 launch humanoid_navigation humanoid_navigation_rviz.launch.py
```

## Issue Log

### 1. Scope: Nav2 Without LaserScan

Initial question: whether RGBD must be converted to LaserScan. The answer for
this project is no. V1 uses a static `OccupancyGrid` for planning. V1.5 should
feed RGBD point clouds into Nav2 costmap layers directly.

Attempts:

- Considered 3D voxel costmap and 2D projected depth map.
- Chose static global map plus later RGBD local updates.

Final V1 solution:

- `perception_mode:=static_only`
- `global_costmap`: static layer plus inflation.
- `local_costmap`: static layer plus inflation, because no depth camera data is
  available in V1.

### 2. Simulator Versus Hardware Scope

The early assumption was that sim and hardware only differ by ROS topics. The
extra work is the contract around TF, timing, localization ownership, camera
extrinsics, command limits, command transport, and safety.

Attempts:

- Kept V1 simulator-only.
- Deferred real G1 validation to a later stage.

Final V1 solution:

- `localization_mode:=sim_ground_truth` owns `map -> odom`.
- Real G1 validation remains documented in `docs/v2_backlog.md`.

### 3. NVIDIA Isaac Sim ROS Graphs

The requirement was to follow NVIDIA official Isaac Sim ROS tutorials, but use
scripted equivalents instead of manual GUI graph creation.

Attempts:

- Added scripted ROS clock graph.
- Added scripted TF/odom publishers from the simulator robot state.

Problems:

- ROS CLI commands can fail when Isaac Sim ROS libraries remain in
  `LD_LIBRARY_PATH`.
- `/clock` must be live before Nav2 lifecycle activation when
  `use_sim_time:=true`.

Final V1 solution:

- Start sim with `--enable_nav_ros_clock --enable_nav_ros_tf_odom`.
- Run ROS CLI/Nav2 from a clean ROS shell with `unset LD_LIBRARY_PATH`.

### 4. G1 Command Path

The existing Unitree policy already supports `[x, y, yaw, height]` commands.
The keyboard reference was:

```text
/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/send_commands_keyboard.py
```

Attempts:

- Direct DDS publishing was considered.
- UDP bridge was added so ROS does not need to run inside the Unitree/Isaac
  Python process.

Problems:

- Enabling broad DDS communication in sim caused a large frame-rate drop.
- Small yaw commands were below the locomotion policy's useful rotate-in-place
  range.

Final V1 solution:

- Nav2 publishes `/cmd_vel`.
- `g1_cmd_vel_adapter` clips and forwards commands to the Unitree sim UDP
  bridge at `127.0.0.1:18080`.
- The G1 profile applies a yaw floor only for rotate-in-place commands. Forward
  path-following yaw commands are not boosted.

### 5. RViz Lifecycle and Reopening

Problem: closing RViz and relaunching the full Nav2 launch file can start a
second Nav2 stack with duplicate node and service names, making Navigation and
Localization panels appear inactive.

Final V1 solution:

- `humanoid_navigation.launch.py` starts Nav2 and optionally RViz.
- `humanoid_navigation_rviz.launch.py` starts RViz only.
- To reopen RViz, use the RViz-only launch file.

### 6. Local Costmap Without RGBD

Problem: before adding the head depth camera and point cloud publisher, there
is no sensor source for a dynamic local costmap.

Attempts:

- Tested path planning with no local static layer.
- Added static layer to local costmap so RPP collision/cost behavior has a map
  source in V1.

Final V1 solution:

- `perception_static_only.yaml` adds a `static_layer` to the local costmap.
- Dynamic obstacle marking/clearing is deferred to V1.5.

### 7. Static Map Generation and Alignment

This was the hardest unresolved area.

Attempts:

- Used Isaac Sim occupancy map APIs from script.
- Generated visual overlay USD files to compare exported occupancy cells against
  the Kitchen scene.
- Tested selected prim exports for known Kitchen objects:
  `Kitchen_InsularShelf_01`, `Kitchen_Cabinet001_01`,
  `Kitchen_Cabinet002`, `Dishwasher054_01`, `Refrigerator001`, and
  `Kitchen_Wall001`.
- Fixed one X-axis mirror issue in exported overlays/maps.
- Added support for selected-prim debug exports to isolate individual objects.

Problems observed:

- Full Kitchen occupancy exports did not line up with the Isaac Sim scene in
  all areas.
- Some visible objects may not contribute expected collision geometry.
  `Kitchen_Wall001` needs further inspection; it may lack collision mesh or
  may be represented by child prims.
- Drag-dropping some generated USD overlays into Isaac Sim can trigger Kit USD
  drag-drop errors. Opening or referencing the USD through stage tools is more
  reliable.
- Script execution inside Isaac Sim Script Editor can crash if it mutates USD
  while other Kit async tasks are active.
- Selected object exports looked better than full-map exports, but the full
  algorithm is not complete.

Current V1 workaround:

- Use a manually reviewed static map artifact for acceptance.
- Keep selected-prim overlay tools as debugging aids outside the final ROS
  package.

V1.5 follow-up:

- Finish a deterministic map export pipeline with explicit bounds, collision
  filtering, coordinate convention tests, and overlay validation.
- Add a small regression scene where expected object extents are known.

### 8. Nav2 Path Tracking and Turning

Several controller options were tried.

Attempts:

- DWB was considered first, but RPP was simpler for static-map V1.
- RPP with `use_rotate_to_heading=true` followed paths better but could spin
  near the goal.
- RPP with `use_rotate_to_heading=false` stopped the final spin but degraded
  global-path following.
- Widening `yaw_goal_tolerance` to `3.14` avoided final heading control, but it
  removed the final orientation objective and was rejected.

Evidence:

- Recorded bags showed the bad behavior was controller-generated pure angular
  velocity, not a global planner loop.
- Humble RPP source shows `use_rotate_to_heading` controls both
  `shouldRotateToPath()` and `shouldRotateToGoalHeading()`.

Final V1 solution:

- Use `nav2_rotation_shim_controller::RotationShimController` as
  `FollowPath`.
- Set RPP as the shim `primary_controller`.
- Keep RPP `use_rotate_to_heading: false`.
- Let the shim handle rough path heading and final goal heading separately.
- Restore `yaw_goal_tolerance: 0.35`.
- Use `closed_loop: false` in the shim to avoid noisy or lagging odom angular
  velocity in the shim acceleration clamp.

### 9. RViz G1 Command Debug Overlay

Attempt:

- Added `g1_cmd_vel_debug_visualizer`, RViz markers, and policy-vector debug
  topics to compare Nav2 commands with adapter-equivalent G1 commands.

Problem:

- The feature did not provide a reliable enough signal during V1 testing and
  added noise to the launch/RViz surface.

Final V1 solution:

- Removed the RViz marker display, debug visualizer launch node, console script,
  debug module, and extra `std_msgs`/`visualization_msgs` dependencies.
- Keep command debugging as rosbag-based post-analysis:

```bash
ros2 bag record /cmd_vel /cmd_vel_nav /odom /tf
```

### 10. Build and Packaging Hygiene

Problem:

- `setup.py` used `glob("maps/*")`; generated directories under `maps/` caused
  `colcon build` to try copying a directory as a file.

Final V1 solution:

- `setup.py` filters package data with `Path(path).is_file()`.
- Generated debug outputs should remain untracked.

## V1 Acceptance Checklist

Before tagging V1:

1. Sim starts with clock, TF/odom, and UDP command bridge.
2. Nav2 lifecycle nodes reach `active`.
3. RViz shows map and global plan.
4. A short goal completes without repeated in-place spinning.
5. `/cmd_vel` reaches the G1 UDP adapter.
6. `colcon test --packages-select humanoid_navigation` passes.
7. `colcon build --packages-select humanoid_navigation` passes.
8. No generated bags, pycache, or overlay debug files are staged.

## V1.5 Preparation

The V1.5 branch should start from this contract and focus on:

1. Head RGBD camera and point cloud publication in Isaac Sim.
2. `perception_mode:=obstacle_2d` local costmap marking and clearing.
3. Costmap/map coordinate validation using overlays and known object extents.
4. Cleaner map export tooling for complete Kitchen maps.
5. Optional replacement of simulator ground truth localization with RGBD SLAM
   localization against a provided static map.
