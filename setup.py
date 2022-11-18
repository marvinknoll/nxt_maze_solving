from setuptools import setup
import os
from glob import glob

package_name = "nxt_maze_solving"
nxt_maze_solving_util = "nxt_maze_solving/util"
nxt_maze_solving_robots = "nxt_maze_solving/robots"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, nxt_maze_solving_util, nxt_maze_solving_robots],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "robot_configs"),
            glob(os.path.join("robot_configs", "*.yaml")),
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marvin",
    maintainer_email="knollmarvin6@gmail.com",
    description="TODO: Package description",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "maze_solver = nxt_maze_solving.maze_solver:main",
            "robot_benchmarking = nxt_maze_solving.robot_benchmarking:main",
        ],
    },
)
