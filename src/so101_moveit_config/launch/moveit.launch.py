from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # true for MuJoCo, false for the real robot
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("so101_moveit_config"),
            "config",
            "moveit.rviz",
        ]
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    static_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="true with MuJoCo, false with the real robot",
            ),
            static_world_tf,
            move_group,
            rviz,
        ]
    )
