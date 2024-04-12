from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # IMU 
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("wrp_ros2"),
                        "launch",
                        "peripheral",
                        "imu_sensor.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "sensor_model": "hipnuc",
            "device_path": "/dev/ttyUSB0",
            "baud_rate": "921600",
            "frame_id": "imu_link",
        }.items(),
    )

    # Livox Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("livox_ros_driver2"),
                        "launch_ROS2",
                        "mid360_pointcloud.launch.py",
                    ]
                )
            ]
        ),
    )

    # LIO-SAM
    liosam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("sample_launch"),
                        "launch",
                        "liosam.launch.py",
                    ]
                )
            ]
        )
    )

    return LaunchDescription(
        [
            imu_launch,
            lidar_launch,
            liosam_launch
        ]
    )
