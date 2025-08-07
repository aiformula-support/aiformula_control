import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "vectornav"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    ROS_PARAM_CONFIG = osp.join(PACKAGE_DIR, "config", "vectornav.yaml")
    vectornav = Node(
        package=PACKAGE_NAME,
        executable="vectornav",
        name="vectornav",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
    )
    vn_sensor_msgs = Node(
        package=PACKAGE_NAME,
        executable="vn_sensor_msgs",
        name="vn_sensor_msgs",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
    )

    return LaunchDescription(
        [
            vectornav,
            vn_sensor_msgs,
        ]
    )
