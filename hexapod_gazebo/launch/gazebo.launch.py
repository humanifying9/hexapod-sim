import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_hexapod_gazebo = get_package_share_directory('hexapod_gazebo')
    pkg_hexapod_description = get_package_share_directory('hexapod_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_hexapod_gazebo, 'worlds', 'hexapod_world.sdf')

    # The URDF's mesh <geometry> tags use package:// URIs (which RViz/
    # robot_state_publisher resolve fine via ament_index). But spawning into
    # Gazebo goes through a URDF->SDF conversion that rewrites those into
    # model:// URIs, which Gazebo's own resource resolver can only find via
    # GZ_SIM_RESOURCE_PATH - it does not know about ROS package paths at all.
    # Without this, every mesh silently fails to load (falls back to no
    # visual, though collision/physics are unaffected since collision always
    # uses primitive boxes regardless of use_meshes).
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_hexapod_description))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hexapod', '-z', '0.2'],
        output='screen'
    )

    bridge_config = os.path.join(pkg_hexapod_gazebo, 'config', 'gz_bridge.yaml')

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        spawn_entity,
        ros_gz_bridge
    ])
