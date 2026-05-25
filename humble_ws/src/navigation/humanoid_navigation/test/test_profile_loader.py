from pathlib import Path

import pytest

from humanoid_navigation.profile_loader import ProfileValidationError, load_profile


def test_load_g1_profile_has_required_navigation_contract():
    profile_path = Path(__file__).parents[1] / "profiles" / "g1.yaml"

    profile = load_profile(profile_path)

    assert profile["frames"]["map"] == "map"
    assert profile["frames"]["odom"] == "odom"
    assert profile["frames"]["base"] == "base_link"
    assert profile["topics"]["cmd_vel"] == "/cmd_vel"
    assert profile["perception"]["pointcloud_topic"]
    assert profile["dds"]["topic"] == "rt/run_command/cmd"
    assert profile["dds"]["transport"] == "udp"
    assert profile["dds"]["udp_host"] == "127.0.0.1"
    assert profile["dds"]["udp_port"] == 18080
    assert profile["dds"]["default_height"] == 0.8
    assert profile["motion"]["enable_lateral"] is False


def test_g1_profile_keeps_nav2_positive_yaw_positive_for_policy():
    profile_path = Path(__file__).parents[1] / "profiles" / "g1.yaml"

    profile = load_profile(profile_path)

    assert profile["dds"]["invert_yaw"] is False


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
