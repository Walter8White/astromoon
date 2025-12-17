# launch/fortress_spawn.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
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

    # Spawner rover
    spawn = TimerAction(period=2.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'rover', '-file', urdf_tmp, '-z', '5.3', '-world', 'lunar_world'],
            output='screen'
        )
    ])

    return LaunchDescription([env_ign, env_gz, gz, spawn])


