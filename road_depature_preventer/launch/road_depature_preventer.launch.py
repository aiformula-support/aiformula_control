import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "road_depature_preventer"
    NODE_NAME = "road_depature_preventer"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    launch_args = (
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logger level (debug, info, warn, error, fatal)",
        ),
    )

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", PACKAGE_NAME + ".yaml"),
        osp.join(get_package_share_directory("sample_vehicle"), "config", "wheel.yaml"),
        osp.join(PACKAGE_DIR, "config", "remote_motor_controller.yaml"),
    )
    
    road_depature_preventer = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control/road_depature_preventer",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration('log_level')],
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("gnss", "/aiformula_sensing/vectornav/gnss"),
            ("pub_can", "/aiformula_control/motor_controller/reference_signal"),
            ("pub_twist", "/aiformula_control/emergency_stop_controller/cmd_vel"),
        ],
    )

    return LaunchDescription([
        *launch_args,
        road_depature_preventer,
    ])
