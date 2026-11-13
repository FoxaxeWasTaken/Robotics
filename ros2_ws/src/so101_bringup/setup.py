from glob import glob

from setuptools import setup

package_name = "so101_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name, package_name + ".tools"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    zip_safe=True,
    tests_require=["pytest"],
    description="Launch files, ROS parameters, RViz configuration and sim tools of the SO-ARM101 stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "connect4_play = so101_bringup.connect4_play:main",
            "evaluate = so101_bringup.evaluate:main",
            "make_perception_dataset = so101_bringup.tools.make_perception_dataset:main",
            "robustness_demo = so101_bringup.robustness_demo:main",
        ],
    },
)
