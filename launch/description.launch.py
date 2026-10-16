"""Publish the mechanical rig by default, or the optional RM2027 analysis scene."""
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("robot", "scene"):
        raise ValueError("mode 必须为 robot/scene")
    team = LaunchConfiguration("team").perform(context)
    if team not in ("red", "blue"):
        raise ValueError("team 必须为 red/blue")
    mappings = {"team": team}
    filename = "dart_robot.urdf.xacro" if mode == "robot" else "dart_system.urdf.xacro"
    path = Path(get_package_share_directory("dart_description")) / "urdf" / filename
    description = xacro.process_file(str(path), mappings=mappings).toxml()
    nodes = [Node(package="robot_state_publisher", executable="robot_state_publisher",
                 name="robot_state_publisher", output="screen",
                 parameters=[{"robot_description": description,
                              "use_sim_time": LaunchConfiguration("use_sim_time")}])]
    if mode == "scene":
        nodes.append(Node(package="dart_description", executable="scene_joint_state_publisher.py",
                 name="scene_joint_state_publisher", output="screen",
                 condition=IfCondition(LaunchConfiguration("start_scene_state_publisher")),
                 parameters=[{"robot_description": description,
                              "use_sim_time": LaunchConfiguration("use_sim_time")}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="robot", choices=["robot", "scene"],
                              description="robot: mechanical TF only; scene: full field for analysis"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_scene_state_publisher", default_value="true",
                              description="Publish field mechanism visualization poses; no launcher yaw"),
        DeclareLaunchArgument("team", default_value="red", choices=["red", "blue"],
                              description="Own team; field_link remains red-side anchored"),
        OpaqueFunction(function=_launch_setup),
    ])
