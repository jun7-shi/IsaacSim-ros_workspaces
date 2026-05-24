# V1.0 Isaac Sim Static-Map Acceptance Notes

Issue: HUM-36

Date: 2026-05-22

## Goal

Validate the V1.0 Nav2 chain for Unitree G1 in Isaac Sim:

1. Start a mobile G1 Wholebody Isaac Sim scene.
2. Publish `/clock`, `/odom`, and `map -> odom -> base_link` through the
   scripted IsaacSim ROS Bridge graphs.
3. Launch `humanoid_navigation` with a static map, `sim_ground_truth`
   localization, and `perception_mode:=static_only`.
4. Send a short Nav2 goal and verify the G1 policy command path receives
   velocity commands through the Unitree sim UDP command bridge.

Depth-based obstacle updates are deferred to V1.5. V1.0 is expected to navigate
only in a known static map.

## Commands

Start the G1 Wholebody Isaac Sim scene from the Unitree navigation worktree:

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

For V1 static-map acceptance, `--no_render` avoids the renderer-dependent
camera observation path while still advancing the scripted ROS Bridge graphs.
Use `--headless` only when RGBD/point cloud publishing is enabled for V1.5.

Launch V1.0 navigation from this ROS workspace:

```bash
cd /data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=static_only \
  map:=/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/maps/kitchen_g1_nav_map.yaml
```

Generate that map from the Unitree IsaacLab env before running Nav2:

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/.worktrees/unitree-g1-nav-task
conda run -n unitree_sim_lab python sim_main.py \
  --device cuda:0 \
  --headless \
  --enable_cameras \
  --task Isaac-Kitchen-G129-Dex1-Wholebody \
  --robot_type g129 \
  --export_nav_static_map /data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/maps/kitchen_g1_nav_map.yaml
```

The exporter uses NVIDIA's Isaac Sim occupancy map API
`isaacsim.asset.gen.omap` against the live Kitchen task stage. For
`Isaac-Kitchen-G129-Dex1-Wholebody`, the exporter automatically bounds the map
to `/World/envs/env_0/Kitchen`, deactivates the task object, and disables robot
collision while generating the occupancy map. It does not deactivate the robot
prim because Kitchen camera and hand prims live under it.

The default movable exclusions are:

```text
deactivated: /World/envs/env_0/Object
collision-disabled: /World/envs/env_0/Robot
```

That prevents the G1, and any movable task object in other scenes, from being
baked into a static map.

Expected acceptance checks:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /local_costmap/local_costmap
ros2 lifecycle get /global_costmap/global_costmap
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /clock
ros2 topic hz /odom
ros2 topic echo /map --once
ros2 topic hz /cmd_vel
```

Run the final `/cmd_vel` check while sending a short RViz Nav2 goal.

## Confirmed Resources

The `unitree_sim_lab` conda environment exists and was checked without
installing new packages:

```text
python 3.11.15
isaacsim True
isaaclab True
rclpy False
cyclonedds True
unitree_sdk2py True
```

The Unitree G1 simulation project has mobile Wholebody tasks. The relevant V1.0
navigation task is `Isaac-Kitchen-G129-Dex1-Wholebody` in the Unitree
worktree branch `hum-nav-g1-nav-task`.

The current Unitree-side ROS bridge fixes are:

```text
fe4def9 fix: resolve G1 nav odometry chassis prim
5a09d46 fix: align nav map frame with G1 world pose
6daaefd fix: make G1 nav sim startup minimal
```

The Unitree worktree adds scripted equivalents of NVIDIA's official Isaac Sim
ROS2 graph shortcuts:

```text
Clock shortcut:
OnPlaybackTick -> IsaacReadSimulationTime -> ROS2PublishClock

TF/odometry shortcut:
Isaac Sim robot chassis prim -> ROS odometry and TF publishers
```

Enable `/clock` with `--enable_nav_ros_clock`; Nav2 launch uses
`use_sim_time:=true`, so `/clock` must be present during live acceptance.

Enable TF/odometry with `--enable_nav_ros_tf_odom`; it creates the ROS
localization contract expected by Nav2:

```text
map -> odom -> base_link
/odom
```

`/odom` is local odometry displacement from startup. Use:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

to view the robot pose in the global map frame.

`localization_mode:=sim_ground_truth` does not start AMCL. Do not use RViz
`2D Pose Estimate` for V1; the global pose comes from the Isaac Sim
`map -> odom -> base_link` TF chain.

The RViz config opens the Nav2 Navigation 2 panel, uses a top-down map view,
and keeps extra overlays disabled by default. Send a goal with either
`Nav2 Goal` or `2D Goal Pose`; both should result in a `PoseStamped` on
`/goal_pose`. If clicking RViz appears to do nothing, first verify the UI path
with:

```bash
ros2 topic echo /goal_pose --once
```

Then click-drag a goal on the map to set the goal orientation.

The Unitree project registers the Wholebody command DDS object when the task
contains `Wholebody` or `--enable_wholebody_dds` is set. For V1, ROS does not
publish Unitree DDS directly. `g1_cmd_vel_adapter` sends JSON UDP packets to
`127.0.0.1:18080`, and `--enable_nav_udp_cmd_bridge` in the Unitree sim process
writes those commands with `RunCommandDDS.write_run_command()`. The existing
Unitree Wholebody action provider consumes the same `[x, y, yaw, height]`
command shape. DDS `rt/run_command/cmd` remains available as an optional legacy
transport for later integration work.

The ROS adapter sends Nav2 `Twist` velocities directly as physical velocity
commands after clipping to the configured motion limits. It does not rescale a
partial Nav2 command to the full Unitree policy range; the policy range is only
a final safety clamp before sending `[x, y, yaw, height]`.

The Kitchen task loads `/data/jun7.shi/datasets/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd`
through the Unitree IsaacLab task `Isaac-Kitchen-G129-Dex1-Wholebody`.

## Latest Runtime Result

The V1.0 simulator and Nav2 chain was exercised with Isaac Sim and Nav2
together. Nav2 loaded the Kitchen static map, lifecycle nodes reached `active`,
`map -> base_link` TF was live, `/clock` and `/odom` were published, and a
short `/navigate_to_pose` goal succeeded while the G1 stayed upright.

The inherited shell can hit a ROS CLI `librcl_logging_spdlog.so` runtime issue
when Isaac Sim's ROS bridge libraries are left in `LD_LIBRARY_PATH`; run ROS CLI
checks from a clean ROS shell or unset `LD_LIBRARY_PATH` before sourcing
`/opt/ros/humble/setup.bash`.

| Check | Required by V1.0 | Current finding | Status |
| --- | --- | --- | --- |
| Static map | `map_server` loads a Unitree-env map | `kitchen_g1_nav_map.yaml` loaded from `Isaac-Kitchen-G129-Dex1-Wholebody` export | Passed |
| Sim time | `/clock` | `--enable_nav_ros_clock` creates an official-style ROS2 Bridge clock graph and `/clock` is visible to ROS | Passed |
| TF | `map -> odom -> base_link` | `--enable_nav_ros_tf_odom` publishes the TF chain and initializes `map -> odom` from the G1 start pose | Passed |
| Odometry | `/odom` | `/odom` is published during the live sim run | Passed |
| Nav2 lifecycle | active Nav2 lifecycle nodes | `bt_navigator`, `controller_server`, `planner_server`, `velocity_smoother`, and `map_server` reached `active` | Passed |
| G1 velocity command | UDP `127.0.0.1:18080` into Unitree sim run command memory | `/cmd_vel` reaches the UDP adapter and the sim consumes nonzero Wholebody run commands | Passed |
| G1 nav task | Unitree IsaacLab Kitchen scene with G1 Wholebody | `Isaac-Kitchen-G129-Dex1-Wholebody` starts with `--nav_minimal_dds --disable_image_server` | Passed |

## Remaining Follow-up Work

For longer routes, tune goal selection and controller limits so the G1 avoids
large in-place rotations near obstacles. The short V1.0 acceptance goal already
exercises the required command path.

V1.5 then enables the head depth bridge and validates local costmap marking and
clearing against runtime obstacles.
