import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

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
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="true with MuJoCo, false with the real robot",
            ),
            cartesian_goal,
        ]
    )
