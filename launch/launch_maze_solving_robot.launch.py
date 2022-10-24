import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import Shutdown


def generate_launch_description():
    nxt_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nxt_ros2"), "launch"
                ),
                "/default.launch.py",
            ]
        )
    )

    maze_solver_node = Node(
        package="nxt_maze_solving",
        executable="maze_solver",
        output="screen",
        on_exit=Shutdown(),
    )

    return LaunchDescription([nxt_ros2, maze_solver_node])
