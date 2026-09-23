import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Same builder chain as so101_moveit_config/launch/moveit.launch.py, plus the
    # MoveItCpp options MoveItPy needs.
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("so101_motion"),
                "config",
                "moveit_py.yaml",
            )
        )
        .to_moveit_configs()
    )

    cartesian_goal = Node(
        package="so101_motion",
        executable="cartesian_goal",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([cartesian_goal])
