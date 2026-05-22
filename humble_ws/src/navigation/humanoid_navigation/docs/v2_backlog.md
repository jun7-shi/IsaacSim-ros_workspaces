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

## HUM-46: Unitree Interface Adapter for Sim and Real Nav2 IO

### Goal

Build a Unitree-facing adapter so Nav2 can use the same ROS topic/frame
contract in simulator and on real G1 without depending directly on IsaacSim ROS
Bridge publishers for every runtime interface.

### Candidate Adapter Boundary

The adapter should publish the ROS interfaces Nav2 expects:

```text
frames: map -> odom -> base_link
odometry: /odom
velocity input: /cmd_vel_smoothed
policy command output: rt/run_command/cmd
optional depth input: /g1/head_rgbd/points
```

For simulator mode, the adapter can read Unitree simulator state and camera
interfaces from `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/`.
For hardware mode, it should use the matching real G1 state, image, and command
interfaces once they are confirmed.

### Why This Is V2

V1.0 is intentionally simulator-only and uses IsaacSim ROS Bridge for `/clock`,
TF, and `/odom`. V1.5 adds depth perception through the same simulator bridge.
The Unitree adapter is a larger integration layer whose value is sim/real
parity, so it belongs after the static-map simulator demo is accepted.

### Acceptance Requirements

1. The adapter publishes `/odom` and TF with frame names compatible with
   `profiles/g1.yaml`.
2. `/cmd_vel_smoothed` is clamped by the G1 profile limits before it reaches
   `rt/run_command/cmd`.
3. Command timeout sends zero velocity.
4. Simulator and hardware modes keep the same Nav2 launch arguments where
   feasible.
5. A migration note documents when to use the V1/V1.5 IsaacSim ROS Bridge path
   versus the V2 Unitree adapter path.

## HUM-39: Bumi Profile and Adapter Requirements

### Goal

Prepare Bumi support without changing the Nav2 `cmd_vel` contract or the
profile schema already used by G1.

### Profile Schema

`profiles/bumi.yaml` should use the same schema as `profiles/g1.yaml`:

```yaml
robot:
  name: bumi
  base_type: humanoid
frames:
  map: map
  odom: odom
  base_link: <bumi_base_frame>
topics:
  cmd_vel_in: /cmd_vel_smoothed
  odom: <bumi_odom_topic>
  rgbd_points: <bumi_rgbd_pointcloud_topic>
  dds_command: <only_if_needed>
motion_limits:
  max_vel_x: <confirmed_limit>
  max_vel_y: 0.0
  max_vel_theta: <confirmed_limit>
  max_accel_x: <confirmed_limit>
  max_accel_theta: <confirmed_limit>
footprint:
  points: <confirmed_2d_footprint>
localization:
  mode: <sim_ground_truth_or_slam_localization_or_external_tf>
adapter:
  type: <twist_passthrough_or_bumi_dedicated_adapter>
```

The schema must remain compatible with the existing profile loader so launch
logic can select `robot_profile:=g1` or `robot_profile:=bumi` without changing
Nav2 params.

### Adapter Decision

The first integration question is whether Bumi consumes `geometry_msgs/Twist`
directly:

1. If Bumi accepts `geometry_msgs/Twist`, use a twist passthrough adapter or no
   adapter beyond remapping `/cmd_vel_smoothed` to Bumi's command topic.
2. If Bumi uses a robot-specific command API, add a dedicated adapter that
   subscribes to `/cmd_vel_smoothed`, clamps the same profile limits, and
   publishes the Bumi-specific command.

In both cases, keep the Nav2 `cmd_vel` contract stable. Nav2 should not know
whether the downstream robot is G1 DDS or Bumi-specific control.

### Data Needed Before Implementation

The missing Bumi data is:

1. Base frame name and full TF tree.
2. Odometry topic name, message type, frame IDs, covariance behavior, and rate.
3. RGBD point cloud topic name, message type, frame ID, range, and rate.
4. Head camera extrinsics relative to `base_link`.
5. 2D footprint and inflation margin for walking sway.
6. Velocity and acceleration limits accepted by the locomotion stack.
7. Localization source and whether it publishes `map -> odom`.
8. Command interface: direct `geometry_msgs/Twist`, action API, DDS, service, or
   another transport.

### Follow-up Issues

Create implementation issues only after the missing Bumi data is available:

1. Add `profiles/bumi.yaml` with confirmed frames, topics, footprint, and
   limits.
2. Implement the Bumi adapter if direct `geometry_msgs/Twist` is not available.
3. Add Bumi launch/profile tests using the same profile-loader schema.
4. Run Bumi simulation or hardware smoke tests against the unchanged Nav2
   contract.
