from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("dart_description")

    xacro_file = PathJoinSubstitution(
        [
            description_share,
            "urdf",
            "dart_system.urdf.xacro",
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            description_share,
            "rviz",
            "description.rviz",
        ]
    )

    sensor_fov_config = PathJoinSubstitution(
        [description_share, "config", "sensor_fov.yaml"]
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    description_parameter = {"robot_description": robot_description}

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dart_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_fov_config",
                default_value=sensor_fov_config,
                description="Camera and lidar field-of-view visualization parameters",
            ),
            description_launch,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[description_parameter],
            ),
            Node(
                package="dart_description",
                executable="sensor_fov_visualizer.py",
                name="sensor_fov_visualizer",
                output="screen",
                parameters=[LaunchConfiguration("sensor_fov_config")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
