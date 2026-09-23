from launch import LaunchDescription
from launch.actions import Shutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("so101_description"),
            "urdf",
            "so101.urdf.xacro",
        ]
    )

    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "controllers.yaml",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    mujoco_control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controllers_file),
        ],
        output="screen",
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            controllers_file,
        ],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--param-file",
            controllers_file,
        ],
        output="screen",
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--param-file",
            controllers_file,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            mujoco_control,
            joint_state_broadcaster,
            arm_controller,
            gripper_controller,
        ]
    )
