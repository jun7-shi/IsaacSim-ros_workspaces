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

    for frame in ("map", "odom", "base"):
        if not profile["frames"].get(frame):
            raise ProfileValidationError(f"missing required frame: frames.{frame}")

    if not profile["perception"].get("pointcloud_topic"):
        raise ProfileValidationError("missing perception.pointcloud_topic")
    if not profile["dds"].get("topic"):
        raise ProfileValidationError("missing dds.topic")

    return profile
