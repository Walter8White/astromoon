from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution([
        FindPackageShare("astromoon_nav"),
        "config",
        "nav2_params.yaml",
    ])

    # Nav2 nodes (no map_server, no amcl): odom-only bringup
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[params_file],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[params_file],
    )

    # Lifecycle manager to bring nodes to active state
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "planner_server",
                "controller_server",
                "smoother_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to Nav2 params file",
        ),

        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
