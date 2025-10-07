import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            "loggger",
            default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Logger level",
        ),
    )

    # --- IMU, GNSS --- #
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"), "launch/vectornav.launch.py"),
        ),
    )
    # --- Publish zero velocity if emergency stop is active --- #
    emergency_stop_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(
                get_package_share_directory("emergency_stop_controller"), "launch/emergency_stop_controller.launch.py"
            ),
        ),
    )
    motor_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("motor_controller"), "launch/motor_controller.launch.py"),
        ),
    )
    can_receiver_and_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"), "launch/socket_can_bridge.launch.py"),
        ),
    )

    # --- Command input from gamepad --- #
    gamepad_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"), "launch/gamepad_joy.launch.py"),
        ),
    )
    # --- Output velocity and angular velocity from gamepad --- #
    gamepad_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"), "launch/gamepad_teleop.launch.py"),
        ),
    )
    # --- Multiplex velocities --- #
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"), "launch/twist_mux.launch.py"),
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_runtime_monitor": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_args,
            vectornav,
            emergency_stop_controller,
            motor_controller,
            can_receiver_and_sender,
            gamepad_joy,
            gamepad_teleop,
            twist_mux,
        ]
    )
