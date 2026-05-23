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
  --enable_cameras \
  --task Isaac-Kitchen-G129-Dex1-Wholebody \
  --robot_type g129 \
  --enable_dex1_dds \
  --enable_nav_ros_clock \
  --enable_nav_ros_tf_odom \
  --enable_nav_udp_cmd_bridge
```

For an automated run without the Isaac Sim GUI, append `--headless`.

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
to `/World/envs/env_0/Kitchen` and does not deactivate `/World/envs/env_0/Robot`
because the task camera sensors live under that prim.

For non-Kitchen exports, it still deactivates the robot and task object if
those prims exist:

```text
/World/envs/env_0/Robot
/World/envs/env_0/Object
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

The Kitchen task loads `/data/jun7.shi/datasets/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd`
through the Unitree IsaacLab task `Isaac-Kitchen-G129-Dex1-Wholebody`.

## Blocked Acceptance Items

The full HUM-36 end-to-end acceptance is still pending a live run with Isaac
Sim and Nav2 together. The required V1.0 simulator interfaces now exist.

The inherited shell can hit a ROS CLI `librcl_logging_spdlog.so` runtime issue
when Isaac Sim's ROS bridge libraries are left in `LD_LIBRARY_PATH`; run ROS CLI
checks from a clean ROS shell or unset `LD_LIBRARY_PATH` before sourcing
`/opt/ros/humble/setup.bash`.

| Check | Required by V1.0 | Current finding | Status |
| --- | --- | --- | --- |
| Static map | `map_server` loads a Unitree-env map | Generate `kitchen_g1_nav_map.yaml` from `Isaac-Kitchen-G129-Dex1-Wholebody` using `--export_nav_static_map` | Ready for runtime generation |
| Sim time | `/clock` | `--enable_nav_ros_clock` creates an official-style ROS2 Bridge clock graph | Ready for runtime verification |
| TF | `map -> odom -> base_link` | `--enable_nav_ros_tf_odom` creates the TF chain and initializes `map -> odom` from the G1 start pose | Ready for runtime verification |
| Odometry | `/odom` | `--enable_nav_ros_tf_odom` creates the odometry publisher | Ready for runtime verification |
| Nav2 lifecycle | active Nav2 lifecycle nodes | Requires a full Isaac Sim plus Nav2 launch | Blocked on live acceptance |
| G1 velocity command | UDP `127.0.0.1:18080` into Unitree sim run command memory | ROS adapter no longer imports Unitree SDK; Unitree sim receives commands with `--enable_nav_udp_cmd_bridge` | Ready for integration |
| G1 nav task | Unitree IsaacLab Kitchen scene with G1 Wholebody | `Isaac-Kitchen-G129-Dex1-Wholebody` is registered in the Unitree worktree | Ready for integration |

## Required Follow-up Work

For the live V1.0 acceptance run, record:

1. Lifecycle state output for Nav2 nodes.
2. TF echo output for `map -> base_link`.
3. Topic rates for `/clock` and `/odom`.
4. RViz evidence that the static map, footprint, global plan, and local
   costmap render correctly.
5. Observation that `/cmd_vel` reaches the Unitree sim UDP bridge and
   updates the Wholebody run command path.

V1.5 then enables the head depth bridge and validates local costmap marking and
clearing against runtime obstacles.
