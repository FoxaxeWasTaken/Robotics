from setuptools import setup

package_name = "so101_common"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    tests_require=["pytest"],
    description="Single source of truth of the SO-ARM101 (joint names, limits, gripper conversion, frames).",
    license="Apache-2.0",
)
