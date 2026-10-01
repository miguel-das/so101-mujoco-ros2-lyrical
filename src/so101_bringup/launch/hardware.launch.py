from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    usb_port = LaunchConfiguration("usb_port")
    joint_config_file = LaunchConfiguration("joint_config_file")

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
                    " hardware_type:=real",
                    " usb_port:=",
                    usb_port,
                    " joint_config_file:=",
                    joint_config_file,
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ParameterFile(controllers_file)],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
            "--param-file",
            controllers_file,
        ],
        output="screen",
    )

    default_joint_config_file = PathJoinSubstitution(
        [
            FindPackageShare("so101_bringup"),
            "config",
            "follower_joints.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "usb_port",
                # Follower arm board; /dev/ttyACM* numbering changes when the leader arm is also plugged in
                default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B42133855-if00",
                description="Serial port of the follower arm's servo bus",
            ),
            DeclareLaunchArgument(
                "joint_config_file",
                default_value=default_joint_config_file,
                description="Feetech per-joint YAML (homing_offset, PID, ...); empty string to use URDF params only",
            ),
            robot_state_publisher,
            control_node,
            controllers_spawner,
        ]
    )
