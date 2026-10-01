import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    hardware = LaunchConfiguration("hardware")
    use_sim_time = ParameterValue(EqualsSubstitution(hardware, "mujoco"), value_type=bool)

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

    bringup_launch_dir = PathJoinSubstitution([FindPackageShare("so101_bringup"), "launch"])

    mujoco = IncludeLaunchDescription(
        PathJoinSubstitution([bringup_launch_dir, "mujoco.launch.py"]),
        condition=IfCondition(EqualsSubstitution(hardware, "mujoco")),
    )

    real = IncludeLaunchDescription(
        PathJoinSubstitution([bringup_launch_dir, "hardware.launch.py"]),
        condition=IfCondition(EqualsSubstitution(hardware, "real")),
        launch_arguments={
            "usb_port": LaunchConfiguration("usb_port"),
            "joint_config_file": LaunchConfiguration("joint_config_file"),
        }.items(),
    )

    motion_server = Node(
        package="so101_motion",
        executable="motion_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "confirm": ParameterValue(LaunchConfiguration("confirm"), value_type=bool),
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("so101_motion"), "config", "motion.rviz"]
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware",
                default_value="mujoco",
                choices=["mujoco", "real"],
                description="Arm to start: the MuJoCo simulation or the real arm",
            ),
            DeclareLaunchArgument(
                "usb_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B42133855-if00",
                description="Serial port of the follower arm's servo bus (hardware:=real)",
            ),
            DeclareLaunchArgument(
                "joint_config_file",
                default_value="",
                description="Optional Feetech per-joint YAML (hardware:=real)",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz to preview and confirm plans",
            ),
            DeclareLaunchArgument(
                "confirm",
                default_value="true",
                description="Wait for Next in RViz before executing; false executes right away",
            ),
            mujoco,
            real,
            motion_server,
            rviz,
        ]
    )
