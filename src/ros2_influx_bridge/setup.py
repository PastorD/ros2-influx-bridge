from glob import glob
from setuptools import find_packages, setup

setup(
    name="ros2_influx_bridge", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ros2_influx_bridge"]),
        ("share/ros2_influx_bridge", ["package.xml"]),
        ("share/ros2_influx_bridge/launch", glob("launch/*.launch.py")),
        ("share/ros2_influx_bridge/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML>=6.0", "influxdb-client>=1.48,<2"],
    python_requires=">=3.10", zip_safe=False,
    maintainer="ROS 2 Influx Bridge contributors", maintainer_email="maintainers@example.invalid",
    description="Message-agnostic YAML-configured ROS 2 to InfluxDB 2 telemetry bridge",
    license="Apache-2.0",
    entry_points={"console_scripts": ["bridge=ros2_influx_bridge.cli:main", "ros2-influx-bridge=ros2_influx_bridge.cli:main"]},
)
