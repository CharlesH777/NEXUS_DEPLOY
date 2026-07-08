"""Top-level launch for the NEXUS real-robot deployment stack."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root_dir = Path(__file__).resolve().parents[1]

    use_sim_time = LaunchConfiguration("use_sim_time")
    stack_config = LaunchConfiguration("stack_config")
    nav2_params = LaunchConfiguration("nav2_params")
    launch_livox = LaunchConfiguration("launch_livox")
    enable_fastlio2 = LaunchConfiguration("enable_fastlio2")
    enable_elevation = LaunchConfiguration("enable_elevation")
    enable_traversability = LaunchConfiguration("enable_traversability")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    enable_sand_mpc = LaunchConfiguration("enable_sand_mpc")
    enable_exploration = LaunchConfiguration("enable_exploration")
    enable_exporter = LaunchConfiguration("enable_exporter")
    enable_rviz = LaunchConfiguration("enable_rviz")
    publish_livox_static_tf = LaunchConfiguration("publish_livox_static_tf")
    publish_map_to_odom = LaunchConfiguration("publish_map_to_odom")
    publish_base_footprint_to_base_link = LaunchConfiguration("publish_base_footprint_to_base_link")

    nav2_cmd_vel_topic = LaunchConfiguration("nav2_cmd_vel_topic")
    global_frame = LaunchConfiguration("global_frame")
    robot_frame = LaunchConfiguration("robot_frame")
    base_frame = LaunchConfiguration("base_frame")
    livox_frame = LaunchConfiguration("livox_frame")

    livox_config_default = PathJoinSubstitution(
        [FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"]
    )

    fastlio2_config_default = str(root_dir / "src" / "nexus_fastlio" / "config" / "fastlio2_real.yaml")

    livox_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=IfCondition(launch_livox),
        parameters=[
            {
                "xfer_format": ParameterValue(LaunchConfiguration("livox_xfer_format"), value_type=int),
                "multi_topic": ParameterValue(LaunchConfiguration("livox_multi_topic"), value_type=int),
                "data_src": 0,
                "publish_freq": ParameterValue(LaunchConfiguration("livox_publish_freq"), value_type=float),
                "output_data_type": 0,
                "frame_id": livox_frame,
                "user_config_path": LaunchConfiguration("livox_config"),
                "cmdline_input_bd_code": LaunchConfiguration("livox_bd_code"),
            }
        ],
    )

    base_footprint_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_to_base_link_static_tf",
        output="screen",
        condition=IfCondition(publish_base_footprint_to_base_link),
        arguments=[
            "--x",
            LaunchConfiguration("base_link_x"),
            "--y",
            LaunchConfiguration("base_link_y"),
            "--z",
            LaunchConfiguration("base_link_z"),
            "--roll",
            LaunchConfiguration("base_link_roll"),
            "--pitch",
            LaunchConfiguration("base_link_pitch"),
            "--yaw",
            LaunchConfiguration("base_link_yaw"),
            "--frame-id",
            robot_frame,
            "--child-frame-id",
            base_frame,
        ],
    )

    livox_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox_static_tf",
        output="screen",
        condition=IfCondition(publish_livox_static_tf),
        arguments=[
            "--x",
            LaunchConfiguration("livox_x"),
            "--y",
            LaunchConfiguration("livox_y"),
            "--z",
            LaunchConfiguration("livox_z"),
            "--roll",
            LaunchConfiguration("livox_roll"),
            "--pitch",
            LaunchConfiguration("livox_pitch"),
            "--yaw",
            LaunchConfiguration("livox_yaw"),
            "--frame-id",
            base_frame,
            "--child-frame-id",
            livox_frame,
        ],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        output="screen",
        condition=IfCondition(publish_map_to_odom),
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            global_frame,
            "--child-frame-id",
            "odom",
        ],
    )

    fastlio_lidar_adapter = Node(
        package="nexus_fastlio",
        executable="fastlio_lidar_adapter",
        name="fastlio_lidar_adapter",
        output="screen",
        condition=IfCondition(enable_fastlio2),
        parameters=[
            {
                "input_topic": LaunchConfiguration("fastlio2_lidar_input_topic"),
                "output_topic": LaunchConfiguration("fastlio2_lidar_output_topic"),
                "rotation_pitch_deg": ParameterValue(
                    LaunchConfiguration("fastlio2_lidar_rotation_pitch_deg"),
                    value_type=float,
                ),
                "target_frame_id": LaunchConfiguration("fastlio2_target_frame_id"),
            }
        ],
    )

    fastlio_imu_adapter = Node(
        package="nexus_fastlio",
        executable="fastlio_imu_adapter",
        name="fastlio_imu_adapter",
        output="screen",
        condition=IfCondition(enable_fastlio2),
        parameters=[
            {
                "input_topic": LaunchConfiguration("fastlio2_imu_input_topic"),
                "output_topic": LaunchConfiguration("fastlio2_imu_output_topic"),
                "linear_accel_scale": ParameterValue(
                    LaunchConfiguration("fastlio2_imu_linear_accel_scale"),
                    value_type=float,
                ),
                "rotation_pitch_deg": ParameterValue(
                    LaunchConfiguration("fastlio2_imu_rotation_pitch_deg"),
                    value_type=float,
                ),
                "target_frame_id": LaunchConfiguration("fastlio2_target_frame_id"),
            }
        ],
    )

    fastlio2_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration("fastlio2_bin"),
            "--ros-args",
            "-r",
            ["__ns:=", LaunchConfiguration("fastlio2_namespace")],
            "-r",
            ["/tf:=", LaunchConfiguration("fastlio2_tf_topic")],
            "-p",
            ["config_path:=", LaunchConfiguration("fastlio2_config")],
        ],
        name="fastlio2_lio_node",
        output="screen",
        condition=IfCondition(enable_fastlio2),
    )

    fastlio_odom_bridge = Node(
        package="nexus_fastlio",
        executable="fastlio_odom_bridge",
        name="fastlio_odom_bridge",
        output="screen",
        condition=IfCondition(enable_fastlio2),
        parameters=[
            {
                "input_odom_topic": LaunchConfiguration("fastlio2_odom_topic"),
                "output_odom_topic": LaunchConfiguration("odom_topic"),
                "output_pose_topic": LaunchConfiguration("pose_topic"),
                "output_frame_id": global_frame,
                "child_frame_id": robot_frame,
                "publish_tf": ParameterValue(
                    LaunchConfiguration("fastlio2_publish_odom_tf"),
                    value_type=bool,
                ),
                "publish_pose": ParameterValue(
                    LaunchConfiguration("fastlio2_publish_pose"),
                    value_type=bool,
                ),
            }
        ],
    )

    elevation_node = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        condition=IfCondition(enable_elevation),
        parameters=[stack_config, {"use_sim_time": use_sim_time}],
    )

    exporter_node = Node(
        package="nexus_elevation_mppi",
        executable="elevation_map_exporter",
        name="elevation_map_exporter",
        output="screen",
        condition=IfCondition(enable_exporter),
        parameters=[
            stack_config,
            {"use_sim_time": use_sim_time},
            {"output_dir": LaunchConfiguration("export_output_dir")},
        ],
    )

    traversability_node = Node(
        package="nexus_elevation_mppi",
        executable="traversability_to_map",
        name="traversability_to_map",
        output="screen",
        condition=IfCondition(enable_traversability),
        parameters=[stack_config, {"use_sim_time": use_sim_time}],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(root_dir / "launch" / "nav2_mppi_real.launch.py")),
        condition=IfCondition(enable_nav2),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "cmd_vel_topic": nav2_cmd_vel_topic,
            "global_frame": global_frame,
            "robot_frame": robot_frame,
        }.items(),
    )

    sand_mpc_node = Node(
        package="nexus_sand_mpc",
        executable="sand_mpc_compensator",
        name="sand_mpc_compensator",
        output="screen",
        condition=IfCondition(enable_sand_mpc),
        parameters=[stack_config, {"use_sim_time": use_sim_time}],
    )

    explorer_node = Node(
        package="nexus_elevation_mppi",
        executable="novelty_explorer",
        name="novelty_explorer",
        output="screen",
        condition=IfCondition(enable_exploration),
        parameters=[stack_config, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "stack_config",
                default_value=str(root_dir / "config" / "nexus_real_navigation_stack.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=str(root_dir / "config" / "nav2_mppi_real_params.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(root_dir / "config" / "nexus_real_navigation.rviz"),
            ),
            DeclareLaunchArgument("launch_livox", default_value="true"),
            DeclareLaunchArgument("enable_fastlio2", default_value="true"),
            DeclareLaunchArgument("enable_elevation", default_value="true"),
            DeclareLaunchArgument("enable_traversability", default_value="true"),
            DeclareLaunchArgument("enable_nav2", default_value="true"),
            DeclareLaunchArgument("enable_sand_mpc", default_value="true"),
            DeclareLaunchArgument("enable_exploration", default_value="true"),
            DeclareLaunchArgument("enable_exporter", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("publish_livox_static_tf", default_value="true"),
            DeclareLaunchArgument("publish_base_footprint_to_base_link", default_value="true"),
            DeclareLaunchArgument("publish_map_to_odom", default_value="false"),
            DeclareLaunchArgument(
                "nav2_cmd_vel_topic",
                default_value="/mppi/cmd_vel_raw",
                description="Set to /cmd_vel when enable_sand_mpc:=false.",
            ),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_footprint"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument("livox_config", default_value=livox_config_default),
            DeclareLaunchArgument("livox_xfer_format", default_value="1"),
            DeclareLaunchArgument("livox_multi_topic", default_value="0"),
            DeclareLaunchArgument("livox_publish_freq", default_value="10.0"),
            DeclareLaunchArgument("livox_bd_code", default_value="livox0000000001"),
            DeclareLaunchArgument("base_link_x", default_value="0.0"),
            DeclareLaunchArgument("base_link_y", default_value="0.0"),
            DeclareLaunchArgument("base_link_z", default_value="0.0"),
            DeclareLaunchArgument("base_link_roll", default_value="0.0"),
            DeclareLaunchArgument("base_link_pitch", default_value="0.0"),
            DeclareLaunchArgument("base_link_yaw", default_value="0.0"),
            DeclareLaunchArgument("livox_x", default_value="0.0"),
            DeclareLaunchArgument("livox_y", default_value="0.0"),
            DeclareLaunchArgument("livox_z", default_value="0.4"),
            DeclareLaunchArgument("livox_roll", default_value="0.0"),
            DeclareLaunchArgument("livox_pitch", default_value="0.0"),
            DeclareLaunchArgument("livox_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "fastlio2_bin",
                default_value=str(
                    root_dir
                    / "third_party"
                    / "FASTLIO2_ROS2"
                    / "install_nexus"
                    / "fastlio2"
                    / "lib"
                    / "fastlio2"
                    / "lio_node"
                ),
            ),
            DeclareLaunchArgument("fastlio2_config", default_value=fastlio2_config_default),
            DeclareLaunchArgument("fastlio2_namespace", default_value="/fastlio2"),
            DeclareLaunchArgument("fastlio2_tf_topic", default_value="/fastlio2/tf"),
            DeclareLaunchArgument("fastlio2_lidar_input_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("fastlio2_lidar_output_topic", default_value="/lidar_fastlio"),
            DeclareLaunchArgument("fastlio2_lidar_rotation_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("fastlio2_imu_input_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("fastlio2_imu_output_topic", default_value="/imu_fastlio"),
            DeclareLaunchArgument("fastlio2_imu_linear_accel_scale", default_value="1.0"),
            DeclareLaunchArgument("fastlio2_imu_rotation_pitch_deg", default_value="0.0"),
            DeclareLaunchArgument("fastlio2_target_frame_id", default_value="base_link"),
            DeclareLaunchArgument("fastlio2_odom_topic", default_value="/fastlio2/lio_odom"),
            DeclareLaunchArgument("fastlio2_publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("fastlio2_publish_pose", default_value="true"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("pose_topic", default_value="/pose"),
            DeclareLaunchArgument(
                "export_output_dir",
                default_value=str(root_dir / "output" / "elevation_maps"),
            ),
            map_to_odom_tf,
            base_footprint_to_base_link_tf,
            livox_static_tf,
            livox_node,
            fastlio_lidar_adapter,
            fastlio_imu_adapter,
            fastlio2_process,
            fastlio_odom_bridge,
            elevation_node,
            exporter_node,
            traversability_node,
            nav2_launch,
            sand_mpc_node,
            explorer_node,
            rviz_node,
        ]
    )
