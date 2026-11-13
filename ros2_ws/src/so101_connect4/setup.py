from glob import glob

from setuptools import setup

package_name = "so101_connect4"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    zip_safe=True,
    tests_require=["pytest"],
    description="Phase 2, Connect Four: the game engine, the turns and the board reader (ADR-037).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "board_reader = so101_connect4.board_reader_node:main",
            "game_node = so101_connect4.game_node:main",
        ]
    },
)
