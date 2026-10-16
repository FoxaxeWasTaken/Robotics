from glob import glob

from setuptools import setup

package_name = "so101_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    zip_safe=True,
    tests_require=["pytest"],
    description="Top-level launch files, ROS parameters and RViz configuration of the SO-ARM101 stack.",
    license="Apache-2.0",
)
