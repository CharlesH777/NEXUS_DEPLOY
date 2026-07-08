"""Nav2 MPPI launch for the NEXUS real-robot deployment stack."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    root_dir = Path(__file__).resolve().parents[1]

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    global_frame = LaunchConfiguration("global_frame")
    robot_frame = LaunchConfiguration("robot_frame")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    vel_remappings = remappings + [("cmd_vel", cmd_vel_topic)]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(root_dir / "config" / "nav2_mppi_real_params.yaml"),
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Use /mppi/cmd_vel_raw when sand MPC is enabled.",
            ),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_footprint"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=vel_remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=vel_remappings,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": lifecycle_nodes},
                    {"bond_timeout": 10.0},
                    {"bond_connect_timeout": 30.0},
                    {"attempt_respawn_reconnection": True},
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "/usr/bin/python3",
                            str(root_dir / "scripts" / "continuous_navigator.py"),
                            "--ros-args",
                            "-p",
                            ["use_sim_time:=", use_sim_time],
                            "-p",
                            ["global_frame:=", global_frame],
                            "-p",
                            ["robot_frame:=", robot_frame],
                            "-p",
                            "preplan_distance:=3.0",
                            "-p",
                            "switch_distance:=1.5",
                        ],
                        output="screen",
                    ),
                ],
            ),
        ]
    )
