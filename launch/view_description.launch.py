from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("dart_description")

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

    site_file = LaunchConfiguration("site_file")

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    description_share,
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
        launch_arguments={"site_file": site_file}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "environment_config",
                default_value=PathJoinSubstitution(
                    [description_share, "config", "environment.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "sensor_fov_config",
                default_value=sensor_fov_config,
                description="Independent left/right camera field-of-view visualization parameters",
            ),
            DeclareLaunchArgument(
                "site_file",
                default_value=PathJoinSubstitution(
                    [description_share, "config", "site", "default.yaml"]
                ),
            ),
            description_launch,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                # Read the same /robot_description as RViz; do not evaluate a second xacro.
            ),
            Node(
                package="dart_description",
                executable="sensor_fov_visualizer.py",
                name="sensor_fov_visualizer",
                output="screen",
                parameters=[LaunchConfiguration("sensor_fov_config")],
            ),
            Node(
                package="dart_description",
                executable="environment_visualizer.py",
                name="environment_visualizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("environment_config"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
