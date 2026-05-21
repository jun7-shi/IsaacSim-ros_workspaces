# V1 Isaac Sim Acceptance Notes

Issue: HUM-36

Date: 2026-05-22

## Goal

Validate the V1 Nav2 chain for Unitree G1 in Isaac Sim:

1. Start a mobile G1 Wholebody Isaac Sim scene.
2. Launch `humanoid_navigation` with a static map, `sim_ground_truth`
   localization, and the default `obstacle_2d` RGBD costmap mode.
3. Verify Nav2 lifecycle, TF, odometry, RGBD PointCloud2 input, local costmap
   marking and clearing, and G1 DDS velocity command delivery.

## Commands

Start the G1 Wholebody Isaac Sim scene from the Unitree navigation worktree:

```bash
cd /data/jun7.shi/code/poc/unitree/Manipulation/.worktrees/unitree-g1-nav-task
export HUMANOID_NAVIGATION_G1_NAV_USD=/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/assets/g1_nav/g1_29dof_with_dex1_nav_depth.usd
conda run -n unitree_sim_lab python sim_main.py \
  --device cpu \
  --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody-Nav \
  --robot_type g129 \
  --enable_dex1_dds \
  --enable_nav_ros_tf_odom \
  --enable_nav_ros_pointcloud \
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
  map:=/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/carter_navigation/maps/carter_warehouse_navigation.yaml
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
navigation task is `Isaac-Move-Cylinder-G129-Dex1-Wholebody-Nav`, added in the
Unitree worktree branch `hum-nav-g1-nav-task` at commit `b3da65c`.

The same Unitree worktree adds a TF/odometry ROS2 Bridge graph at commit
`a7f3f6c`. Enable it with `--enable_nav_ros_tf_odom`; it creates
`map -> odom -> base_link` and publishes `/odom` through Isaac Sim's built-in
`isaacsim.ros2.bridge`.

The Unitree worktree also adds the RGBD PointCloud2 ROS2 Bridge graph at commit
`a08e9c8`. Enable it with `--enable_nav_ros_pointcloud`; it creates a camera
render product for the G1 head RGBD camera and publishes
`/g1/head_rgbd/points` with `ROS2CameraHelper` in `depth_pcl` mode.

The Unitree project registers the Wholebody command DDS object when the task
contains `Wholebody` or `--enable_wholebody_dds` is set. Its subscriber listens
on `rt/run_command/cmd`, which matches the adapter implemented in this package.

The G1 navigation camera preset points at:

```text
/World/envs/env_.*/Robot/d435_link/head_d435_depth_camera
```

This package now owns the navigation-specific G1 USD layer:

```text
src/navigation/humanoid_navigation/assets/g1_nav/g1_29dof_with_dex1_nav_depth.usd
```

The layer references the original Unitree `g1-29dof_wholebody_dex1` robot asset
and adds a D435-style depth camera under `d435_link`. Unitree source assets are
not overwritten. The matching manifest is:

```text
src/navigation/humanoid_navigation/assets/g1_nav/manifest.yaml
```

For the first HUM-36 acceptance map, reuse the existing Carter warehouse map:

```text
src/navigation/carter_navigation/maps/carter_warehouse_navigation.yaml
```

The Unitree nav task points its robot USD path at the package-owned layer
through `HUMANOID_NAVIGATION_G1_NAV_USD`; if unset, the task defaults to the
same absolute path shown in the command above.

## Blocked Acceptance Items

The full HUM-36 end-to-end acceptance is still pending a live run. The
required Isaac Sim ROS-side interfaces now exist, but the current shell cannot
run ROS CLI acceptance commands because of the local `librcl_logging_spdlog.so`
runtime issue noted below:

| Check | Required by Nav2 V1 | Current Unitree project finding | Status |
| --- | --- | --- | --- |
| Static map | `map_server` loads `map:=...` | Navigation launch can reuse `src/navigation/carter_navigation/maps/carter_warehouse_navigation.yaml` for the first smoke acceptance | Ready for integration |
| TF | `map -> odom -> base_link` | `--enable_nav_ros_tf_odom` creates a ROS2 Bridge graph for `map -> odom -> base_link`; live `tf2_echo` still needs a full sim run | Ready for runtime verification |
| Odometry | `/odom` | `--enable_nav_ros_tf_odom` creates a ROS2 Bridge odometry publisher; live `ros2 topic hz /odom` still needs a full sim run | Ready for runtime verification |
| RGBD costmap input | `/g1/head_rgbd/points` as `PointCloud2` | `--enable_nav_ros_pointcloud` creates a ROS2 Bridge camera `depth_pcl` publisher; live `ros2 topic hz /g1/head_rgbd/points` still needs a full sim run | Ready for runtime verification |
| Nav2 lifecycle | active Nav2 lifecycle nodes | Cannot validate in this shell until the ROS CLI runtime issue is fixed and the full Isaac Sim plus Nav2 launch is run | Blocked in this shell |
| Local costmap updates | marking and clearing from RGBD `PointCloud2` | Costmap config and RGBD point cloud bridge are present; live RViz/costmap evidence still needs a full sim run | Ready for runtime verification |
| G1 velocity command | DDS `rt/run_command/cmd` | DDS topic exists and matches the adapter | Ready for integration |
| G1 nav task | Unitree IsaacLab task loads the package-owned RGBD USD | `Isaac-Move-Cylinder-G129-Dex1-Wholebody-Nav` is registered in the Unitree worktree | Ready for integration |

There is also a local ROS CLI runtime issue in the current `/opt/ros/humble`
environment when running launch introspection:

```text
undefined symbol in librcl_logging_spdlog.so
```

That issue prevents using `ros2 launch --show-args` as a local verification
path in this shell. No conda packages were installed or modified.

## Required Follow-up Work

The Isaac Sim side now has the ROS bridge layers needed for the first live V1
acceptance run:

1. `/odom` and `map -> odom -> base_link`, generated by
   `--enable_nav_ros_tf_odom`.
2. `/g1/head_rgbd/points` as `sensor_msgs/PointCloud2`, generated from the G1
   head RGBD camera by `--enable_nav_ros_pointcloud`.
3. The first acceptance run can use the existing Carter warehouse occupancy map.
   A G1-scene-aligned map remains a follow-up once navigation motion works.

For the live acceptance run, rerun the commands above and record:

1. Lifecycle state output for Nav2 nodes.
2. TF echo output for `map -> odom -> base_link`.
3. Topic rates for `/odom` and `/g1/head_rgbd/points`.
4. RViz evidence that local costmap obstacles mark and clear.
5. DDS observation that `/cmd_vel_smoothed` reaches `rt/run_command/cmd`.
