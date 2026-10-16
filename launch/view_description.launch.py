from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MODEL_ARGS = ["mode", "team"]


def generate_launch_description():
    description_share = FindPackageShare("dart_description")

    rviz_config = PathJoinSubstitution(
        [
            description_share,
            "rviz",
            PythonExpression(["'robot.rviz' if '", LaunchConfiguration("mode"), "' == 'robot' else 'description.rviz'"]),
        ]
    )

    sensor_fov_config = PathJoinSubstitution(
        [description_share, "config", "sensor_fov.yaml"]
    )

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
        launch_arguments={"start_scene_state_publisher": "false", **{
            name: LaunchConfiguration(name) for name in MODEL_ARGS
        }}.items(),
    )

    return LaunchDescription(
        [
            *[DeclareLaunchArgument(name, default_value=(
                "scene" if name == "mode" else "red"
            )) for name in MODEL_ARGS],
            DeclareLaunchArgument(
                "sensor_fov_config",
                default_value=sensor_fov_config,
                description="Independent left/right camera field-of-view visualization parameters",
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
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
