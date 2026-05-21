# V1 Isaac Sim Acceptance Notes

Issue: HUM-36

Date: 2026-05-21

## Goal

Validate the V1 Nav2 chain for Unitree G1 in Isaac Sim:

1. Start a mobile G1 Wholebody Isaac Sim scene.
2. Launch `humanoid_navigation` with a static map, `sim_ground_truth`
   localization, and the default `obstacle_2d` RGBD costmap mode.
3. Verify Nav2 lifecycle, TF, odometry, RGBD PointCloud2 input, local costmap
   marking and clearing, and G1 DDS velocity command delivery.

## Commands

Start the G1 Wholebody Isaac Sim scene from the Unitree project:

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab
conda run -n unitree_sim_lab python sim_main.py \
  --device cpu \
  --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody \
  --robot_type g129 \
  --enable_dex1_dds \
  --no_render
```

Launch V1 navigation from this ROS workspace:

```bash
cd /data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=obstacle_2d \
  map:=/absolute/path/to/map.yaml
```

Expected acceptance checks:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /local_costmap/local_costmap
ros2 lifecycle get /global_costmap/global_costmap
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /odom
ros2 topic hz /g1/head_rgbd/points
ros2 topic hz /cmd_vel_smoothed
```

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

The Unitree G1 simulation project has mobile Wholebody tasks. The relevant V1
task is `Isaac-Move-Cylinder-G129-Dex1-Wholebody`.

The Unitree project registers the Wholebody command DDS object when the task
contains `Wholebody` or `--enable_wholebody_dds` is set. Its subscriber listens
on `rt/run_command/cmd`, which matches the adapter implemented in this package.

The G1 camera presets include a front camera at:

```text
/World/envs/env_.*/Robot/d435_link/front_cam
```

## Blocked Acceptance Items

The full HUM-36 end-to-end acceptance is blocked by missing ROS-side simulation
interfaces in the referenced Unitree IsaacLab project:

| Check | Required by Nav2 V1 | Current Unitree project finding | Status |
| --- | --- | --- | --- |
| Static map | `map_server` loads `map:=...` | Navigation launch supports this, but no aligned sample map is provided by the Unitree scene | Blocked |
| TF | `map -> odom -> base_link` | No ROS TF publisher was found in the Unitree project | Blocked |
| Odometry | `/odom` | No ROS odometry publisher was found in the Unitree project | Blocked |
| RGBD costmap input | `/g1/head_rgbd/points` as `PointCloud2` | Camera pipeline currently exposes RGB image observations and shared-memory/image-server paths; no ROS `PointCloud2` publisher was found | Blocked |
| Nav2 lifecycle | active Nav2 lifecycle nodes | Cannot validate without ROS TF, `/odom`, map alignment, and RGBD input | Blocked |
| Local costmap updates | marking and clearing from RGBD `PointCloud2` | Costmap config is present, but there is no ROS point cloud input to drive it | Blocked |
| G1 velocity command | DDS `rt/run_command/cmd` | DDS topic exists and matches the adapter | Ready for integration |

There is also a local ROS CLI runtime issue in the current `/opt/ros/humble`
environment when running launch introspection:

```text
undefined symbol in librcl_logging_spdlog.so
```

That issue prevents using `ros2 launch --show-args` as a local verification
path in this shell. No conda packages were installed or modified.

## Required Follow-up Work

To unblock the real V1 acceptance run, the Isaac Sim side needs a ROS bridge
layer that publishes the exact Nav2-facing contract:

1. `map -> odom -> base_link` TF.
2. `/odom` as `nav_msgs/Odometry`.
3. `/g1/head_rgbd/points` as `sensor_msgs/PointCloud2`, generated from the G1
   head RGBD camera.
4. A static occupancy map aligned to the Isaac Sim scene, or a documented map
   generation step.

Once those interfaces exist, rerun the commands above and record:

1. Lifecycle state output for Nav2 nodes.
2. TF echo output for `map -> odom -> base_link`.
3. Topic rates for `/odom` and `/g1/head_rgbd/points`.
4. RViz evidence that local costmap obstacles mark and clear.
5. DDS observation that `/cmd_vel_smoothed` reaches `rt/run_command/cmd`.
