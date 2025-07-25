import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "emergency_stop_controller"
PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)


def generate_launch_description() -> LaunchDescription:
    launch_args = (
        DeclareLaunchArgument(
            "logger",
            default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Ros logger level",
        ),
    )

    ROS_PARAM_CONFIG = (osp.join(PACKAGE_DIR, "config", PACKAGE_NAME + ".yaml"),)
    emergency_stop_controller = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        arguments=[
            "--ros-args",
            "--log-level",
            ["aiformula_control.", PACKAGE_NAME, ":=", LaunchConfiguration("logger")],
        ],
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("sub_can", "/aiformula_sensing/vehicle_info"),
            ("pub_twist", "/aiformula_control/emergency_stop_controller/cmd_vel"),
        ],
    )

    return LaunchDescription(
        [
            *launch_args,
            emergency_stop_controller,
        ]
    )
