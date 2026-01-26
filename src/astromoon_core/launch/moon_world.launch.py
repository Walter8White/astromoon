# launch/fortress_spawn.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os, xacro, tempfile

def generate_launch_description():
    pkg = get_package_share_directory('astromoon_core')
    world = os.path.join(pkg, 'worlds', 'world.sdf')

    # Resource paths (portable: fonctionne host + docker)
    resource_paths = [
        pkg,
        os.path.join(pkg, 'models'),
        os.path.join(pkg, 'meshes'),
        os.path.join(pkg, 'worlds'),
    ]
    resource_paths_str = ':'.join(resource_paths)

    # Append to existing paths (ne pas écraser)
    env_ign = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        resource_paths_str + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )
    env_gz = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        resource_paths_str + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # URDF du rover depuis le xacro
    xacro_path = os.path.join(pkg, 'urdf', 'xacro', 'rover.urdf.xacro')
    urdf_xml = xacro.process_file(xacro_path).toxml()
    tmp = tempfile.NamedTemporaryFile(prefix='rover_', suffix='.urdf', delete=False)
    tmp.write(urdf_xml.encode())
    tmp.flush()
    urdf_tmp = tmp.name

    # Lance Gazebo
    gz = ExecuteProcess(cmd=['ign', 'gazebo', world, '-v', '4'], output='screen')

    #gz-ros bridge for cmd_vel
    bridge_cmd_vel = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/model/rover/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
    ],
    output='screen',
    ros_arguments=['-r', '/model/rover/cmd_vel:=/cmd_vel'],
    )

    #gr-ros bridge for odometry
    bridge_odom = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='bridge_odom',
    output='screen',
    arguments=[
        '/model/rover/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
    ],
    ros_arguments=['-r', '/model/rover/odometry:=/odom'],
    )

    unpause = TimerAction(period=4.0, actions=[
        ExecuteProcess(
            cmd=[
                'ign', 'service',
                '-s', '/world/lunar_world/control',
                '--reqtype', 'ignition.msgs.WorldControl',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '3000',
                '--req', 'pause: false'
            ],
            output='screen'
        )
    ])


    # Spawner rover
    spawn = TimerAction(period=2.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'rover', '-file', urdf_tmp, '-z', '5.3', '-world', 'lunar_world'],
            output='screen'
        )
    ])

    rover_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("astromoon_core"),
                "launch",
                "rover_bringup.launch.py",
            ])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    odom_tf = Node(
        package="astromoon_core",
        executable="odom_tf_broadcaster",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"odom_topic": "/odom"},
            {"odom_frame": "odom"},
            {"base_frame": "base_link"},
        ],  
    )



    return LaunchDescription([env_ign, env_gz, gz, unpause, spawn, bridge_cmd_vel, bridge_odom, rover_bringup, odom_tf])



