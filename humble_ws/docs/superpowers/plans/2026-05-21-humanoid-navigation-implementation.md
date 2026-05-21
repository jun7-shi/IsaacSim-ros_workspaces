# Humanoid Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Nav2-based humanoid navigation package for Unitree G1 simulation with static map import, RGBD runtime costmap updates, simulation localization, and Unitree DDS velocity command output.

**Architecture:** Create a new `humanoid_navigation` `ament_python` package under `humble_ws/src/navigation`. Keep Nav2 configuration, robot profiles, perception modes, localization modes, and the G1 command adapter in separate files so Bumi and SLAM localization can be added later without changing the main launch contract.

**Tech Stack:** ROS 2 Humble, Nav2, `ament_python`, `launch_ros`, `rclpy`, `geometry_msgs`, Unitree SDK2 Python DDS, `pytest`, `launch_testing`, Isaac Sim.

---

## Git Policy

Every Linear issue created from this plan must end with at least one git commit that contains only the completed issue's scoped changes. If an issue requires multiple logical commits, keep each commit buildable and mention all commit hashes in the Linear issue completion note.

## Environment and G1 Resource Policy

- Use `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/` as the source of truth for Unitree G1 simulation assets, policies, DDS command examples, and Isaac Lab integration details.
- Launch Isaac Sim from the conda environment named `unitree_sim_lab`.
- Do not install packages into any conda environment without explicit user approval. If a dependency is missing, stop and ask with the exact package and command needed.

## File Structure

- Create `src/navigation/humanoid_navigation/package.xml`: ROS package metadata and dependencies.
- Create `src/navigation/humanoid_navigation/setup.py`: `ament_python` packaging and console script entry point.
- Create `src/navigation/humanoid_navigation/setup.cfg`: test/install configuration.
- Create `src/navigation/humanoid_navigation/resource/humanoid_navigation`: package marker.
- Create `src/navigation/humanoid_navigation/humanoid_navigation/__init__.py`: Python package marker.
- Create `src/navigation/humanoid_navigation/humanoid_navigation/profile_loader.py`: load and validate robot profile YAML.
- Create `src/navigation/humanoid_navigation/humanoid_navigation/g1_cmd_vel_adapter.py`: ROS Twist to Unitree DDS command adapter.
- Create `src/navigation/humanoid_navigation/launch/humanoid_navigation.launch.py`: main launch entry.
- Create `src/navigation/humanoid_navigation/config/nav2_params.yaml`: generic Nav2 parameter template.
- Create `src/navigation/humanoid_navigation/profiles/g1.yaml`: G1 frame, topic, velocity, perception, and DDS settings.
- Create `src/navigation/humanoid_navigation/params/perception_2d_obstacle.yaml`: default `PointCloud2` to `ObstacleLayer` settings.
- Create `src/navigation/humanoid_navigation/params/perception_3d_voxel.yaml`: optional `PointCloud2` to `VoxelLayer` settings.
- Create `src/navigation/humanoid_navigation/params/localization_sim_ground_truth.yaml`: localization mode parameters and TF expectations.
- Create `src/navigation/humanoid_navigation/rviz/humanoid_navigation.rviz`: RViz view for map, TF, point cloud, costmaps, paths, and footprint.
- Create `src/navigation/humanoid_navigation/test/test_profile_loader.py`: profile validation tests.
- Create `src/navigation/humanoid_navigation/test/test_g1_cmd_vel_adapter.py`: adapter conversion, clipping, timeout, and lateral-mode tests.
- Create `src/navigation/humanoid_navigation/test/test_launch_files.py`: launch and parameter file sanity tests.
- Create `src/navigation/humanoid_navigation/README.md`: operator workflow and acceptance checks.

## Task 1: Scaffold `humanoid_navigation` Package and Profile Loader

**Files:**
- Create: `src/navigation/humanoid_navigation/package.xml`
- Create: `src/navigation/humanoid_navigation/setup.py`
- Create: `src/navigation/humanoid_navigation/setup.cfg`
- Create: `src/navigation/humanoid_navigation/resource/humanoid_navigation`
- Create: `src/navigation/humanoid_navigation/humanoid_navigation/__init__.py`
- Create: `src/navigation/humanoid_navigation/humanoid_navigation/profile_loader.py`
- Create: `src/navigation/humanoid_navigation/profiles/g1.yaml`
- Test: `src/navigation/humanoid_navigation/test/test_profile_loader.py`

- [ ] **Step 1: Write profile loader tests**

Create `test/test_profile_loader.py` with tests for successful G1 profile loading and missing required keys.

```python
from pathlib import Path

import pytest

from humanoid_navigation.profile_loader import ProfileValidationError, load_profile


def test_load_g1_profile_has_required_navigation_contract():
    profile_path = Path(__file__).parents[1] / "profiles" / "g1.yaml"

    profile = load_profile(profile_path)

    assert profile["frames"]["map"] == "map"
    assert profile["frames"]["odom"] == "odom"
    assert profile["frames"]["base"] == "base_link"
    assert profile["perception"]["pointcloud_topic"]
    assert profile["dds"]["topic"] == "rt/run_command/cmd"
    assert profile["dds"]["default_height"] == 0.8
    assert profile["motion"]["enable_lateral"] is False


def test_load_profile_rejects_missing_required_section(tmp_path):
    profile_path = tmp_path / "bad.yaml"
    profile_path.write_text(
        "frames:\n"
        "  map: map\n"
        "  odom: odom\n"
        "  base: base_link\n",
        encoding="utf-8",
    )

    with pytest.raises(ProfileValidationError, match="missing required section"):
        load_profile(profile_path)
```

- [ ] **Step 2: Run profile tests and verify they fail**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q
```

Expected: package or import failure because `humanoid_navigation` and `profile_loader.py` do not exist yet.

- [ ] **Step 3: Create package metadata**

Create `package.xml` with `ament_python`, launch, Nav2, and test dependencies.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_navigation</name>
  <version>0.1.0</version>
  <description>Nav2-based humanoid navigation bringup for Unitree G1 and future humanoid profiles.</description>
  <maintainer email="jun7@example.com">jun7</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_common</exec_depend>
  <exec_depend>nav2_costmap_2d</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>yaml</exec_depend>

  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Create Python packaging files**

Create `setup.py`, `setup.cfg`, package marker files, and install data files.

```python
from glob import glob
from setuptools import find_packages, setup

package_name = "humanoid_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/profiles", glob("profiles/*.yaml")),
        (f"share/{package_name}/params", glob("params/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="jun7",
    maintainer_email="jun7@example.com",
    description="Nav2-based humanoid navigation bringup.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "g1_cmd_vel_adapter = humanoid_navigation.g1_cmd_vel_adapter:main",
        ],
    },
)
```

```ini
[develop]
script_dir=$base/lib/humanoid_navigation
[install]
install_scripts=$base/lib/humanoid_navigation
```

- [ ] **Step 5: Implement profile loader and G1 profile**

Create `profile_loader.py`.

```python
from pathlib import Path
from typing import Any

import yaml


class ProfileValidationError(ValueError):
    """Raised when a robot profile cannot satisfy the launch contract."""


REQUIRED_SECTIONS = ("frames", "topics", "perception", "motion", "dds")


def load_profile(path: str | Path) -> dict[str, Any]:
    profile_path = Path(path)
    with profile_path.open("r", encoding="utf-8") as stream:
        profile = yaml.safe_load(stream) or {}

    for section in REQUIRED_SECTIONS:
        if section not in profile:
            raise ProfileValidationError(f"missing required section: {section}")

    required_frames = ("map", "odom", "base")
    for frame in required_frames:
        if not profile["frames"].get(frame):
            raise ProfileValidationError(f"missing required frame: frames.{frame}")

    if not profile["perception"].get("pointcloud_topic"):
        raise ProfileValidationError("missing perception.pointcloud_topic")
    if not profile["dds"].get("topic"):
        raise ProfileValidationError("missing dds.topic")

    return profile
```

Create `profiles/g1.yaml`.

```yaml
frames:
  map: map
  odom: odom
  base: base_link

topics:
  odom: /odom
  cmd_vel: /cmd_vel_smoothed

perception:
  pointcloud_topic: /g1/head_rgbd/points
  obstacle_max_range: 4.0
  raytrace_max_range: 5.0
  min_obstacle_height: 0.05
  max_obstacle_height: 1.8
  enable_global_obstacles: false

motion:
  enable_lateral: false
  min_vel_x: -0.2
  max_vel_x: 0.5
  max_vel_y: 0.0
  max_vel_theta: 0.8
  acc_lim_x: 0.8
  acc_lim_y: 0.0
  acc_lim_theta: 1.2

footprint:
  polygon: "[[0.22, 0.18], [0.22, -0.18], [-0.18, -0.18], [-0.18, 0.18]]"
  padding: 0.08

dds:
  topic: rt/run_command/cmd
  default_height: 0.8
  invert_y: true
  invert_yaw: true
  cmd_timeout_sec: 0.25
  publish_rate_hz: 50.0
```

- [ ] **Step 6: Run profile tests and commit**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q
colcon test-result --verbose
```

Expected: `test_profile_loader.py` passes.

Commit:

```bash
git add src/navigation/humanoid_navigation
git commit -m "feat: scaffold humanoid navigation package"
```

## Task 2: Implement G1 `cmd_vel` to Unitree DDS Adapter

**Files:**
- Create: `src/navigation/humanoid_navigation/humanoid_navigation/g1_cmd_vel_adapter.py`
- Test: `src/navigation/humanoid_navigation/test/test_g1_cmd_vel_adapter.py`

- [ ] **Step 1: Write adapter unit tests**

Create tests for clipping, lateral disabling, sign inversion, and timeout fallback.

```python
from geometry_msgs.msg import Twist

from humanoid_navigation.g1_cmd_vel_adapter import AdapterConfig, CommandConverter


def make_twist(x=0.0, y=0.0, yaw=0.0):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.angular.z = yaw
    return msg


def test_converter_clips_and_inverts_unitree_axes():
    converter = CommandConverter(
        AdapterConfig(
            min_vel_x=-0.2,
            max_vel_x=0.5,
            max_vel_y=0.2,
            max_vel_theta=0.8,
            default_height=0.8,
            enable_lateral=True,
            invert_y=True,
            invert_yaw=True,
        )
    )

    command = converter.to_command(make_twist(x=1.0, y=0.1, yaw=2.0))

    assert command == [0.5, -0.1, -0.8, 0.8]


def test_converter_forces_zero_lateral_when_disabled():
    converter = CommandConverter(
        AdapterConfig(max_vel_y=0.2, enable_lateral=False, default_height=0.8)
    )

    command = converter.to_command(make_twist(y=0.2))

    assert command[1] == 0.0


def test_converter_zero_command_uses_default_height():
    converter = CommandConverter(AdapterConfig(default_height=0.8))

    assert converter.zero_command() == [0.0, 0.0, 0.0, 0.8]


def test_converter_formats_dds_payload_as_stringified_list():
    converter = CommandConverter(AdapterConfig(default_height=0.8))

    payload = converter.to_payload([0.1, 0.0, -0.2, 0.8])

    assert payload == "[0.1, 0.0, -0.2, 0.8]"
```

- [ ] **Step 2: Run adapter tests and verify they fail**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_g1_cmd_vel_adapter.py
```

Expected: FAIL because `AdapterConfig` and `CommandConverter` are not implemented.

- [ ] **Step 3: Implement conversion logic**

Implement pure conversion logic before DDS integration.

```python
from dataclasses import dataclass

from geometry_msgs.msg import Twist


@dataclass(frozen=True)
class AdapterConfig:
    min_vel_x: float = -0.2
    max_vel_x: float = 0.5
    max_vel_y: float = 0.0
    max_vel_theta: float = 0.8
    default_height: float = 0.8
    enable_lateral: bool = False
    invert_y: bool = True
    invert_yaw: bool = True


class CommandConverter:
    def __init__(self, config: AdapterConfig):
        self._config = config

    def to_command(self, msg: Twist) -> list[float]:
        x = self._clip(msg.linear.x, self._config.min_vel_x, self._config.max_vel_x)
        if self._config.enable_lateral:
            y = self._clip(msg.linear.y, -self._config.max_vel_y, self._config.max_vel_y)
        else:
            y = 0.0
        yaw = self._clip(
            msg.angular.z,
            -self._config.max_vel_theta,
            self._config.max_vel_theta,
        )

        if self._config.invert_y:
            y = -y
        if self._config.invert_yaw:
            yaw = -yaw

        return [round(x, 4), round(y, 4), round(yaw, 4), self._config.default_height]

    def zero_command(self) -> list[float]:
        return [0.0, 0.0, 0.0, self._config.default_height]

    def to_payload(self, command: list[float]) -> str:
        return str([float(value) for value in command])

    @staticmethod
    def _clip(value: float, minimum: float, maximum: float) -> float:
        return min(max(value, minimum), maximum)
```

- [ ] **Step 4: Implement ROS2 node with DDS publisher isolation**

Add a publisher wrapper so tests can exercise conversion without requiring Unitree SDK2.

```python
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class UnitreeDdsPublisher:
    def __init__(self, topic: str):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        ChannelFactoryInitialize(1)
        self._msg_type = String_
        self._publisher = ChannelPublisher(topic, String_)
        self._publisher.Init()

    def publish(self, payload: str) -> None:
        self._publisher.Write(self._msg_type(data=payload))


class G1CmdVelAdapter(Node):
    def __init__(self):
        super().__init__("g1_cmd_vel_adapter")
        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel_smoothed")
        self.declare_parameter("dds_topic", "rt/run_command/cmd")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_sec", 0.25)
        self.declare_parameter("default_height", 0.8)
        self.declare_parameter("enable_lateral", False)
        self.declare_parameter("invert_y", True)
        self.declare_parameter("invert_yaw", True)
        self.declare_parameter("min_vel_x", -0.2)
        self.declare_parameter("max_vel_x", 0.5)
        self.declare_parameter("max_vel_y", 0.0)
        self.declare_parameter("max_vel_theta", 0.8)

        config = AdapterConfig(
            min_vel_x=self.get_parameter("min_vel_x").value,
            max_vel_x=self.get_parameter("max_vel_x").value,
            max_vel_y=self.get_parameter("max_vel_y").value,
            max_vel_theta=self.get_parameter("max_vel_theta").value,
            default_height=self.get_parameter("default_height").value,
            enable_lateral=self.get_parameter("enable_lateral").value,
            invert_y=self.get_parameter("invert_y").value,
            invert_yaw=self.get_parameter("invert_yaw").value,
        )
        self._converter = CommandConverter(config)
        self._dds = UnitreeDdsPublisher(self.get_parameter("dds_topic").value)
        self._last_msg = None
        self._last_msg_time = None
        self._timeout_sec = self.get_parameter("cmd_timeout_sec").value

        self.create_subscription(
            Twist,
            self.get_parameter("input_cmd_vel_topic").value,
            self._on_cmd_vel,
            10,
        )
        self.create_timer(1.0 / self.get_parameter("publish_rate_hz").value, self._publish_tick)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_msg = msg
        self._last_msg_time = self.get_clock().now()

    def _publish_tick(self) -> None:
        if self._last_msg is None or self._is_stale():
            command = self._converter.zero_command()
        else:
            command = self._converter.to_command(self._last_msg)
        self._dds.publish(self._converter.to_payload(command))

    def _is_stale(self) -> bool:
        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        return elapsed > self._timeout_sec


def main(args=None):
    rclpy.init(args=args)
    node = G1CmdVelAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

- [ ] **Step 5: Run adapter tests and commit**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_g1_cmd_vel_adapter.py
colcon test-result --verbose
```

Expected: adapter tests pass.

Commit:

```bash
git add src/navigation/humanoid_navigation
git commit -m "feat: add G1 cmd_vel DDS adapter"
```

## Task 3: Add Nav2 Parameter Template and G1 Runtime Limits

**Files:**
- Create: `src/navigation/humanoid_navigation/config/nav2_params.yaml`
- Modify: `src/navigation/humanoid_navigation/profiles/g1.yaml`
- Test: `src/navigation/humanoid_navigation/test/test_launch_files.py`

- [ ] **Step 1: Write YAML sanity tests**

```python
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def load_yaml(relative_path):
    with (PACKAGE_ROOT / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_nav2_params_define_required_servers_and_costmaps():
    params = load_yaml("config/nav2_params.yaml")

    assert "bt_navigator" in params
    assert "controller_server" in params
    assert "planner_server" in params
    assert "local_costmap" in params
    assert "global_costmap" in params
    assert "map_server" in params


def test_nav2_defaults_are_non_holonomic_for_v1():
    params = load_yaml("config/nav2_params.yaml")
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["max_vel_y"] == 0.0
    assert follow_path["min_vel_y"] == 0.0
```

- [ ] **Step 2: Run YAML tests and verify they fail**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_launch_files.py
```

Expected: FAIL because `config/nav2_params.yaml` does not exist.

- [ ] **Step 3: Create Nav2 params from Carter pattern with humanoid-safe defaults**

Use existing Carter parameters as a reference, but set frames and topics to profile-compatible defaults:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 20
    default_server_timeout: 40

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: progress_checker
    goal_checker_plugin: [stopped_goal_checker]
    controller_plugins: [FollowPath]
    progress_checker:
      plugin: nav2_controller::SimpleProgressChecker
      required_movement_radius: 0.3
      movement_time_allowance: 12.0
    stopped_goal_checker:
      plugin: nav2_controller::StoppedGoalChecker
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.35
      stateful: true
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      debug_trajectory_details: true
      min_vel_x: -0.2
      max_vel_x: 0.5
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 0.8
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 0.8
      acc_lim_y: 0.0
      acc_lim_theta: 1.2
      decel_lim_x: -0.8
      decel_lim_y: 0.0
      decel_lim_theta: -1.2
      vx_samples: 12
      vy_samples: 1
      vtheta_samples: 16
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.3
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.1
      short_circuit_trajectory_evaluation: true
      stateful: true
      critics: [RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 24.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 20.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 24.0
      GoalDist.scale: 20.0
      RotateToGoal.scale: 24.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

Add remaining Nav2 server sections by adapting the existing Carter Nav2 layout, keeping `map_server`, `planner_server`, `smoother_server`, `behavior_server`, `velocity_smoother`, `local_costmap`, `global_costmap`, and lifecycle sections present.

- [ ] **Step 4: Run YAML tests and commit**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_launch_files.py
colcon test-result --verbose
```

Expected: YAML tests pass.

Commit:

```bash
git add src/navigation/humanoid_navigation
git commit -m "feat: add humanoid Nav2 parameter template"
```

## Task 4: Add Perception Modes for RGBD Costmap Updates

**Files:**
- Create: `src/navigation/humanoid_navigation/params/perception_2d_obstacle.yaml`
- Create: `src/navigation/humanoid_navigation/params/perception_3d_voxel.yaml`
- Modify: `src/navigation/humanoid_navigation/test/test_launch_files.py`

- [ ] **Step 1: Add perception parameter tests**

```python
def test_obstacle_2d_mode_uses_pointcloud_obstacle_layer():
    params = load_yaml("params/perception_2d_obstacle.yaml")
    plugins = params["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
    obstacle_layer = params["local_costmap"]["local_costmap"]["ros__parameters"]["rgbd_obstacle_layer"]

    assert "rgbd_obstacle_layer" in plugins
    assert obstacle_layer["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle_layer["pointcloud"]["data_type"] == "PointCloud2"


def test_voxel_3d_mode_uses_voxel_layer():
    params = load_yaml("params/perception_3d_voxel.yaml")
    plugins = params["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
    voxel_layer = params["local_costmap"]["local_costmap"]["ros__parameters"]["rgbd_voxel_layer"]

    assert "rgbd_voxel_layer" in plugins
    assert voxel_layer["plugin"] == "nav2_costmap_2d::VoxelLayer"
    assert voxel_layer["pointcloud"]["data_type"] == "PointCloud2"
```

- [ ] **Step 2: Run tests and verify they fail**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_launch_files.py
```

Expected: FAIL because perception params do not exist.

- [ ] **Step 3: Create default 2D obstacle mode**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: [rgbd_obstacle_layer, inflation_layer]
      rgbd_obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: true
        observation_sources: pointcloud
        pointcloud:
          topic: /g1/head_rgbd/points
          data_type: PointCloud2
          marking: true
          clearing: true
          obstacle_min_range: 0.05
          obstacle_max_range: 4.0
          raytrace_min_range: 0.05
          raytrace_max_range: 5.0
          min_obstacle_height: 0.05
          max_obstacle_height: 1.8
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        enabled: true
        cost_scaling_factor: 0.8
        inflation_radius: 0.55
```

- [ ] **Step 4: Create optional 3D voxel mode**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: [rgbd_voxel_layer, inflation_layer]
      rgbd_voxel_layer:
        plugin: nav2_costmap_2d::VoxelLayer
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.1
        z_voxels: 16
        max_obstacle_height: 1.8
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /g1/head_rgbd/points
          data_type: PointCloud2
          marking: true
          clearing: true
          obstacle_min_range: 0.05
          obstacle_max_range: 4.0
          raytrace_min_range: 0.05
          raytrace_max_range: 5.0
          min_obstacle_height: 0.05
          max_obstacle_height: 1.8
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        enabled: true
        cost_scaling_factor: 0.8
        inflation_radius: 0.55
```

- [ ] **Step 5: Run tests and commit**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_launch_files.py
colcon test-result --verbose
```

Expected: perception tests pass.

Commit:

```bash
git add src/navigation/humanoid_navigation
git commit -m "feat: add RGBD costmap perception modes"
```

## Task 5: Implement Main Launch with Profile, Map, Localization, and Perception Modes

**Files:**
- Create: `src/navigation/humanoid_navigation/launch/humanoid_navigation.launch.py`
- Create: `src/navigation/humanoid_navigation/params/localization_sim_ground_truth.yaml`
- Modify: `src/navigation/humanoid_navigation/test/test_launch_files.py`

- [ ] **Step 1: Add launch file existence and mode tests**

```python
def test_main_launch_file_exists():
    assert (PACKAGE_ROOT / "launch" / "humanoid_navigation.launch.py").exists()


def test_sim_ground_truth_localization_params_document_required_frames():
    params = load_yaml("params/localization_sim_ground_truth.yaml")

    assert params["localization_mode"] == "sim_ground_truth"
    assert params["required_tf"] == ["map", "odom", "base_link"]
```

- [ ] **Step 2: Run launch tests and verify they fail**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_launch_files.py
```

Expected: FAIL because launch and localization files do not exist.

- [ ] **Step 3: Create localization parameter file**

```yaml
localization_mode: sim_ground_truth
required_tf: [map, odom, base_link]
odom_topic: /odom
notes:
  - Isaac Sim should publish or bridge map->odom->base_link for V1.
  - slam_localization and external_tf are reserved launch modes.
```

- [ ] **Step 4: Create main launch file**

Implement launch arguments and include Nav2 bringup.

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("humanoid_navigation")
    nav2_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    perception_mode = LaunchConfiguration("perception_mode")

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
        }.items(),
    )

    g1_adapter = Node(
        package="humanoid_navigation",
        executable="g1_cmd_vel_adapter",
        name="g1_cmd_vel_adapter",
        output="screen",
        parameters=[os.path.join(package_dir, "profiles", "g1.yaml")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="g1"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(package_dir, "maps", "sample_map.yaml"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_dir, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("perception_mode", default_value="obstacle_2d"),
            DeclareLaunchArgument("localization_mode", default_value="sim_ground_truth"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            nav2_bringup,
            g1_adapter,
        ]
    )
```

This first launch version wires the stable entry points. In a later implementation pass, add `RewrittenYaml` substitutions so profile values override Nav2 and adapter parameters.

- [ ] **Step 5: Run tests and commit**

Run:

```bash
colcon test --packages-select humanoid_navigation --pytest-args -q test/test_launch_files.py
colcon test-result --verbose
```

Expected: launch tests pass.

Commit:

```bash
git add src/navigation/humanoid_navigation
git commit -m "feat: add humanoid navigation launch entrypoint"
```

## Task 6: Add RViz, Static Map Wiring, and Operator Documentation

**Files:**
- Create: `src/navigation/humanoid_navigation/rviz/humanoid_navigation.rviz`
- Create: `src/navigation/humanoid_navigation/README.md`
- Modify: `src/navigation/humanoid_navigation/launch/humanoid_navigation.launch.py`

- [ ] **Step 1: Add documentation checklist**

Create `README.md` with exact launch and validation commands:

```markdown
# Humanoid Navigation

## V1 Launch

```bash
source install/setup.bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=obstacle_2d \
  map:=/absolute/path/to/map.yaml
```

## Required Inputs

- TF: `map -> odom -> base_link`
- Odometry: `/odom`
- RGBD point cloud: `/g1/head_rgbd/points`
- Nav2 static map: `map.yaml` plus image file

## Acceptance Checks

- `ros2 lifecycle get /bt_navigator` reports `active`.
- RViz shows map, G1 footprint, TF, local costmap, global costmap, and planned path.
- RGBD obstacles appear in local costmap.
- Removing an obstacle clears it from local costmap.
- Sending a Nav2 goal produces `/cmd_vel_smoothed`.
- `g1_cmd_vel_adapter` publishes DDS commands to `rt/run_command/cmd`.
- Stopping Nav2 velocity output produces zero DDS velocity within `cmd_timeout_sec`.
```

- [ ] **Step 2: Add RViz config**

Create RViz config with displays for TF, Map, PointCloud2, local costmap, global costmap, path, footprint, and goal tool. Use the existing Carter RViz config as a reference and replace topics with humanoid defaults.

- [ ] **Step 3: Add RViz launch option**

Modify launch file to declare `rviz` and include `nav2_bringup/launch/rviz_launch.py` when enabled.

- [ ] **Step 4: Build package and commit**

Run:

```bash
colcon build --packages-select humanoid_navigation --symlink-install
```

Expected: build exits with code 0.

Commit:

```bash
git add src/navigation/humanoid_navigation
git commit -m "docs: add humanoid navigation operator workflow"
```

## Task 7: Run V1 Isaac Sim Acceptance

**Files:**
- Modify: `src/navigation/humanoid_navigation/README.md`
- Create: `src/navigation/humanoid_navigation/docs/v1_acceptance.md`

- [ ] **Step 1: Start Isaac Sim G1 scene**

Run the G1 Isaac Sim environment that publishes:

```text
map -> odom -> base_link
/odom
/g1/head_rgbd/points
```

Record the exact simulator command in `docs/v1_acceptance.md`.

- [ ] **Step 2: Launch humanoid navigation**

Run:

```bash
source install/setup.bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=obstacle_2d \
  map:=/absolute/path/to/map.yaml
```

Expected: Nav2 lifecycle nodes become active.

- [ ] **Step 3: Verify TF and topics**

Run:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /g1/head_rgbd/points
ros2 topic echo /cmd_vel_smoothed --once
```

Expected: TF is available, point cloud publishes at a stable rate, and Nav2 emits velocity after a goal is sent.

- [ ] **Step 4: Send Nav2 goal and record result**

Use RViz `Nav2 Goal` tool or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Expected: G1 moves toward the target and reaches the configured tolerance.

- [ ] **Step 5: Verify voxel mode launches**

Run:

```bash
ros2 launch humanoid_navigation humanoid_navigation.launch.py \
  robot_profile:=g1 \
  localization_mode:=sim_ground_truth \
  perception_mode:=voxel_3d \
  map:=/absolute/path/to/map.yaml
```

Expected: local costmap updates from RGBD observations without lifecycle startup failure.

- [ ] **Step 6: Commit acceptance notes**

Commit:

```bash
git add src/navigation/humanoid_navigation/README.md src/navigation/humanoid_navigation/docs/v1_acceptance.md
git commit -m "test: record G1 navigation acceptance run"
```

## Task 8: Prepare V2 Extension Backlog

**Files:**
- Create: `src/navigation/humanoid_navigation/docs/v2_backlog.md`

- [ ] **Step 1: Document SLAM localization path**

Add a section for `slam_localization` with the expected Nav2-facing contract:

```markdown
## SLAM Localization

The localization component must publish `map -> odom` and accept the same static map used by Nav2 `map_server`. It can use RGBD/VIO internally, but Nav2 must continue to consume `map`, `odom`, `base_link`, `/odom`, and costmap observations through the existing launch contract.
```

- [ ] **Step 2: Document real G1 validation delta**

Add real-robot requirements:

```markdown
## Real G1 Validation

- Confirm real RGBD topic names and frame IDs.
- Confirm camera extrinsics from head RGBD frame to `base_link`.
- Confirm locomotion policy command ranges on hardware.
- Add emergency stop, fall detection, communication timeout handling, and low-speed-only profile.
- Validate navigation in a cleared test area before enabling normal speed.
```

- [ ] **Step 3: Document Bumi profile requirements**

Add Bumi requirements:

```markdown
## Bumi Profile

The Bumi profile must define frames, odometry topic, RGBD point cloud topic, footprint, velocity limits, adapter type, and localization source. If Bumi does not use Unitree DDS, add a separate adapter node behind the same Nav2 `cmd_vel` contract.
```

- [ ] **Step 4: Commit V2 backlog**

Run:

```bash
git add src/navigation/humanoid_navigation/docs/v2_backlog.md
git commit -m "docs: add humanoid navigation V2 backlog"
```

## Self-Review

- Spec coverage:
  - Static map import: Task 3, Task 5, Task 7.
  - RGBD `ObstacleLayer` default mode: Task 4, Task 7.
  - RGBD `VoxelLayer` optional mode: Task 4, Task 7.
  - Simulation localization: Task 5, Task 7.
  - Unitree DDS adapter: Task 2.
  - G1 profile and future Bumi profile path: Task 1, Task 8.
  - V1/V2 boundary: Task 7, Task 8.
- Red-flag scan: no incomplete-marker tokens are intentionally left in this plan.
- Type consistency: `AdapterConfig`, `CommandConverter`, `load_profile`, and launch argument names are used consistently across tasks.
