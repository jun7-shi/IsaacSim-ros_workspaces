# Unitree IsaacLab Static Map Generation

HUM-36 must use a static map generated from the actual Unitree IsaacLab
acceptance env. The packaged `g1_static_warehouse.yaml` is only a launch/test
example and is not aligned with `Isaac-Kitchen-G129-Dex1-Wholebody`.

## Export Command

Run this from the Unitree navigation worktree:

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

The command writes:

```text
src/navigation/humanoid_navigation/maps/kitchen_g1_nav_map.yaml
src/navigation/humanoid_navigation/maps/kitchen_g1_nav_map.pgm
```

## Defaults

The exporter uses NVIDIA's Isaac Sim occupancy map extension
`isaacsim.asset.gen.omap` and these defaults:

```text
Kitchen bound prim: /World/envs/env_0/Kitchen
cell size: 0.05 m
free origin: robot start x/y with z=0.1
z bounds: [0.05, 1.2]
Kitchen excluded prims: none
non-Kitchen default excluded prims:
  /World/envs/env_0/Robot
  /World/envs/env_0/Object
```

The exporter temporarily applies `UsdPhysics.CollisionAPI` to static meshes
before generation. This is needed for Kitchen assets that have visual geometry
but incomplete collision metadata; NVIDIA's occupancy map generator only sees
collision geometry.

Kitchen maps do not deactivate `/World/envs/env_0/Robot` because the Kitchen
task owns camera sensors under the robot prim. Non-Kitchen exports keep the
older robot/object exclusion defaults so movable assets are not baked into
static maps.

## Validate The Map

The generated PGM should contain occupied and free cells, not just free space.
For the current Kitchen export the observed PGM histogram was:

```text
{0: 2331, 205: 57, 254: 49902}
```

You can also validate that Nav2's map server loads it:

```bash
ROS_DOMAIN_ID=77 ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/maps/kitchen_g1_nav_map.yaml \
  -p topic_name:=map \
  -p frame_id:=map
```

Then configure/activate it from another shell in the same ROS domain and echo
`/map`.

## Use With Nav2

After export and rebuild:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=static_only \
  map:=/data/jun7.shi/code/poc/IsaacSim-ros_workspaces/.worktrees/nav2-humanoid-navigation/humble_ws/src/navigation/humanoid_navigation/maps/kitchen_g1_nav_map.yaml
```

Check alignment in RViz and with:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

If the map is shifted relative to the robot, adjust either the map generation
origin/bounds or the Unitree launch `--nav_ros_map_odom_translation` override.
