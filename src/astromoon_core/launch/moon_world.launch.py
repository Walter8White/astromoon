# launch/fortress_spawn.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os, xacro, tempfile


def generate_launch_description():
    pkg = get_package_share_directory("astromoon_core")
    world = os.path.join(pkg, "worlds", "world.sdf")

    resource_paths = [
        pkg,
        os.path.join(pkg, "models"),
        os.path.join(pkg, "meshes"),
        os.path.join(pkg, "worlds"),
    ]
    resource_paths_str = ":".join(resource_paths)

    env_ign = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        resource_paths_str + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )
    env_gz = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        resource_paths_str + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    xacro_path = os.path.join(pkg, "urdf", "xacro", "rover.urdf.xacro")
    urdf_xml = xacro.process_file(xacro_path).toxml()
    tmp = tempfile.NamedTemporaryFile(prefix="rover_", suffix=".urdf", delete=False)
    tmp.write(urdf_xml.encode())
    tmp.flush()
    urdf_tmp = tmp.name

    gz = ExecuteProcess(cmd=["ign", "gazebo", world, "-v", "4"], output="screen")

    # TF: world -> map (identity)
    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        parameters=[{"use_sim_time": True}],
    )

    bridge_cmd_vel = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_cmd_vel",
        output="screen",
        arguments=[
            "/model/rover/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
        ],
        remappings=[
            ("/model/rover/cmd_vel", "/cmd_vel"),
        ],
    )

    bridge_odom = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_odom",
        output="screen",
        arguments=[
            "/model/rover/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
        ],
        remappings=[
            ("/model/rover/odometry", "/odom"),
        ],
    )

    # Gazebo global poses -> ROS TFMessage on a private topic (NOT /tf)
    # Prefer dynamic_pose/info for moving entities
    bridge_world_poses = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_world_poses",
        output="screen",
        arguments=[
            "/world/lunar_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
        ],
        ros_arguments=["-r", "/world/lunar_world/dynamic_pose/info:=/gz/pose_tf"],
    )

    unpause = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    "/world/lunar_world/control",
                    "--reqtype",
                    "ignition.msgs.WorldControl",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "3000",
                    "--req",
                    "pause: false",
                ],
                output="screen",
            )
        ],
    )

    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "rover", "-file", urdf_tmp, "-z", "5.3", "-world", "lunar_world"],
                output="screen",
            )
        ],
    )

    rover_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("astromoon_core"), "launch", "rover_bringup.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    odom_tf = Node(
        package="astromoon_core",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": True}, {"odom_topic": "/odom"}],
    )

    # Publishes map -> rover/odom using /gz/pose_tf (world truth) + /odom
    map_to_odom = Node(
        package="astromoon_core",
        executable="map_to_odom_broadcaster",
        name="map_to_odom_broadcaster",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"odom_topic": "/odom"},
            {"pose_tf_topic": "/gz/pose_tf"},   # <- TFMessage from Gazebo
            {"map_frame": "map"},
            {"odom_frame": "rover/odom"},
            {"base_frame": "rover/base_link"},
            {"world_frame": "world"},
        ],
    )

    return LaunchDescription(
        [
            env_ign,
            env_gz,
            gz,
            world_to_map,
            unpause,
            spawn,
            bridge_cmd_vel,
            bridge_odom,
            bridge_world_poses,
            rover_bringup,
            odom_tf,
            map_to_odom,
        ]
    )
