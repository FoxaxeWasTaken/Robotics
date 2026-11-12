from setuptools import setup

package_name = "so101_connect4"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/connect4_grid.yaml"]),
    ],
    zip_safe=True,
    tests_require=["pytest"],
    description="Phase 2, Connect Four: the game engine, the turns and the board reader (ADR-037).",
    license="Apache-2.0",
)
