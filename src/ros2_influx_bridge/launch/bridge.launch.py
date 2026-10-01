from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="Absolute path to the bridge application YAML"),
        Node(package="ros2_influx_bridge", executable="bridge", output="screen",
             arguments=["run", "--config", LaunchConfiguration("config")]),
    ])
