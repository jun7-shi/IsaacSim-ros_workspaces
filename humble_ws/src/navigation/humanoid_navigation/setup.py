from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "humanoid_navigation"


def files(pattern):
    return [path for path in glob(pattern) if Path(path).is_file()]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", files("launch/*.launch.py")),
        (f"share/{package_name}/config", files("config/*.yaml")),
        (f"share/{package_name}/profiles", files("profiles/*.yaml")),
        (f"share/{package_name}/params", files("params/*.yaml")),
        (f"share/{package_name}/maps", files("maps/*")),
        (f"share/{package_name}/rviz", files("rviz/*.rviz")),
        (f"share/{package_name}/docs", files("docs/*.md")),
        (f"share/{package_name}/assets/g1_nav", files("assets/g1_nav/*")),
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
