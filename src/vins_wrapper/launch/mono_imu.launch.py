import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    vins_pkg = get_package_share_directory("vins_wrapper")

    default_vins_config = os.path.join(
        vins_pkg,
        "params",
        "vins_params",
        "gazebo-mono-imu-params.yaml"
    )

    default_ros_config = os.path.join(
        vins_pkg,
        "params",
        "ros_params",
        "gazebo-mono-imu-ros-params.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")
    vins_config_file = LaunchConfiguration("vins_config_file")
    ros_config_file = LaunchConfiguration("ros_config_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation clock"
    )

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Robot namespace"
    )

    declare_vins_config = DeclareLaunchArgument(
        "vins_config_file",
        default_value=default_vins_config,
        description="Full path to VINS config YAML"
    )

    declare_ros_config = DeclareLaunchArgument(
        "ros_config_file",
        default_value=default_ros_config,
        description="Full path to ROS params YAML"
    )

    vins_node = Node(
        package="vins_wrapper",
        executable="vins_wrapper_node",
        name="vins_estimator",
        namespace=robot_namespace,
        output="screen",
        parameters=[
            ros_config_file,
            {
                "use_sim_time": use_sim_time,
                "config_file": vins_config_file
            }
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_namespace,
        declare_vins_config,
        declare_ros_config,
        vins_node
    ])

