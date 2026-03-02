from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_mode(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in {"manual", "auto"}:
        raise RuntimeError("Invalid launch argument 'mode'. Expected 'manual' or 'auto'.")
    return []


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    mission = LaunchConfiguration("mission")
    use_rviz = LaunchConfiguration("use_rviz")

    is_auto = IfCondition(PythonExpression(["'", mode, "' == 'auto'"]))

    mission_file = PathJoinSubstitution([
        FindPackageShare("astromoon_missions"),
        "missions",
        [mission, TextSubstitution(text=".yaml")]
    ])

    # 1) World + rover
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("astromoon_core"),
                "launch",
                "moon_world.launch.py",
            ])
        ),
        launch_arguments={"mode": mode}.items(),
    )

    # 2) Nav2 in auto mode only
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("astromoon_nav"),
                "launch",
                "nav2.launch.py",
            ])
        ),
        condition=is_auto,
    )

    # 3) Mission nodes in auto mode only
    mission_manager = Node(
        package="astromoon_missions",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[
            {"mission_file": mission_file},
        ],
        condition=is_auto,
    )

    mission_referee = Node(
        package="astromoon_missions",
        executable="mission_referee",
        name="mission_referee",
        output="screen",
        parameters=[
            {"mission_file": mission_file},
        ],
        condition=is_auto,
    )

    # 4) RViz optional in both modes
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("astromoon_nav"),
                "rviz",
                "nav2.rviz",
            ])
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="auto",
            description="Launch mode: manual or auto.",
        ),
        OpaqueFunction(function=_validate_mode),
        DeclareLaunchArgument(
            "mission",
            default_value="m1_waypoint_traverse",
            description="Mission name (without .yaml). Used only in auto mode.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2 (true/false).",
        ),
        world_launch,
        nav2_launch,
        mission_manager,
        mission_referee,
        rviz,
    ])
