from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission = LaunchConfiguration("mission")      
    use_nav2 = LaunchConfiguration("use_nav2")    
    use_rviz = LaunchConfiguration("use_rviz")    

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
        )
    )

    # 2) Nav2 (optional)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("astromoon_nav"),
                "launch",
                "nav2.launch.py",
            ])
        ),
        condition=IfCondition(use_nav2),
    )

    # 3) Mission nodes
    mission_manager = Node(
        package="astromoon_missions",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[
            {"mission_file": mission_file},
            {"use_nav2": use_nav2},
        ],
    )

    mission_referee = Node(
        package="astromoon_missions",
        executable="mission_referee",
        name="mission_referee",
        output="screen",
        parameters=[
            {"mission_file": mission_file},
        ],
    )

    # 4) RViz optional 
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
            "mission",
            default_value="m0_bootcamp",
            description="Mission name (without .yaml). Example: m0_bootcamp",
        ),
        DeclareLaunchArgument(
            "use_nav2",
            default_value="false",
            description="Launch Nav2 stack (true/false).",
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
