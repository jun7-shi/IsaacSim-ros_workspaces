# V2 Backlog

This file records V2 design decisions that should not change the V1
Nav2-facing contract unless explicitly called out.

## HUM-37: RGBD SLAM Localization and Map Update Path

### Nav2 Contract

V2 keeps the same interface expected by the current launch and params:

```text
frames: `map`, `odom`, `base_link`, `/odom`
rgbd observations: /g1/head_rgbd/points
default perception: obstacle_2d
```

The SLAM/localization component owns `map -> odom`. The robot state source owns
`odom -> base_link` and `/odom`. Nav2 continues to consume the static global map
through `map_server` and dynamic RGBD observations through the local costmap.

### Recommended Architecture Candidate

Use `slam_localization` as a mode name for an external RGBD SLAM or RGBD VIO
component that publishes `map -> odom` and consumes the G1 head RGBD stream.
Candidates to evaluate in V2:

1. RTAB-Map RGBD localization against a prior database or map.
2. Isaac Sim ground-truth localization bridge for simulation-only acceptance.
3. Vendor or robot-provided VIO/SLAM if it can publish the same ROS TF contract.

For V2 implementation, prefer a small mode-specific launch include around the
selected SLAM node instead of changing the Nav2 params. The include should be
selected by `localization_mode:=slam_localization`.

### Static-map Localization

The provided static occupancy map remains the global planning map. RGBD SLAM
must align its world frame to the same `map` frame before Nav2 starts planning.

Required sub-steps:

1. Load the static map with `nav2_map_server`.
2. Start the RGBD SLAM/localization component in localization-only mode.
3. Initialize or align the SLAM pose to the static map origin.
4. Publish `map -> odom` continuously.
5. Verify `map -> odom -> base_link` in TF before activating Nav2 lifecycle
   nodes.

### Map Save/Update

Separate map save/update from static-map localization. The first V2 target
should localize against a provided map. Map update can be added after that path
is stable.

Map save/update sub-steps:

1. Capture RGBD SLAM output or a generated occupancy grid as a separate map
   artifact.
2. Review map quality offline before replacing the static map used by Nav2.
3. Save maps through a dedicated SLAM/map-fusion workflow, not through the V1
   navigation launch path.
4. Keep runtime costmap clearing local and transient; do not merge local
   obstacle clearing into the global static map automatically.

### Risks and Follow-ups

Open risks:

1. The RGBD source must provide depth or `PointCloud2`, not only RGB images.
2. TF frame calibration between head camera, `base_link`, and odometry must be
   validated on both sim and real robot.
3. Static occupancy map coordinates must be aligned with the SLAM map frame.
4. Loop closure can shift `map -> odom`; Nav2 behavior during pose jumps needs
   low-speed validation.

Implementation follow-ups should be created once the specific SLAM backend is
chosen and its dependencies are approved.

## HUM-38: Real G1 Low-speed Navigation Validation

### Scope

Real G1 validation is a second-stage activity after simulation and bridge
contracts are stable. The first hardware target is low-speed navigation in a
restricted area with a static global map and RGBD local costmap updates.

### Interfaces to Confirm on Hardware

Before running Nav2 on the real G1, confirm these runtime interfaces:

1. Head RGBD topic name, expected to remain `/g1/head_rgbd/points` if a ROS
   bridge is added or already available.
2. Point cloud message type, frame ID, update rate, range limits, and timestamp
   source.
3. Head camera extrinsics from camera frame to `base_link`.
4. Localization source for `map -> odom`, either SLAM, VIO, motion capture, or
   a robot-provided localization bridge.
5. Odometry source for `/odom` and `odom -> base_link`.
6. Locomotion policy command limits accepted by the real robot command path.
7. DDS domain, network interface, and topic availability for
   `rt/run_command/cmd`.

### Low-speed Profile

Use a hardware-specific low-speed profile until the full safety envelope is
validated:

```text
max forward velocity: <= 0.20 m/s
max yaw velocity: <= 0.30 rad/s
max acceleration: conservative enough to avoid abrupt gait transitions
side velocity: disabled for V1 non-holonomic navigation
```

These limits should be enforced at both Nav2 and adapter/policy boundaries.

### Safety Requirements

Hardware validation must not start until these safety controls are in place:

1. Operator-controlled emergency stop tested before Nav2 activation.
2. fall detection or robot-provided protective stop enabled.
3. communication timeout that sends zero velocity if `/cmd_vel_smoothed` or DDS
   command updates stop.
4. Restricted test area with physical clearance around the robot.
5. Battery, thermal, and policy health monitoring visible to the operator.
6. A manual recovery procedure that does not depend on Nav2 remaining healthy.

### Bringup Checklist

1. Boot the robot without Nav2 and verify the locomotion policy can accept zero
   and low-speed commands.
2. Verify TF tree: `map -> odom -> base_link -> camera_frame`.
3. Verify `/odom` and `/g1/head_rgbd/points` rates.
4. Start Nav2 with `robot_profile:=g1`, the static map, and
   `perception_mode:=obstacle_2d`.
5. Send a short goal inside the restricted area.
6. Confirm the local costmap marks and clears obstacles before allowing a
   longer goal.
7. Record command, odometry, TF, and point cloud bags for post-run analysis.

### Open Risks

1. Real G1 camera topic names and camera extrinsics may differ from simulation.
2. The existing DDS policy command format may need tighter hardware-side limits.
3. SLAM localization drift can produce unsafe global plans if the map alignment
   is poor.
4. Footprint inflation may need to be increased to account for gait sway and
   arm configuration.
