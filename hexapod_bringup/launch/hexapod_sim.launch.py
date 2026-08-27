import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_hexapod_description = get_package_share_directory('hexapod_description')
    pkg_hexapod_gazebo = get_package_share_directory('hexapod_gazebo')
    pkg_hexapod_control = get_package_share_directory('hexapod_control')

    # Xacro processing. controllers_file has to be passed explicitly here -
    # it has no useful default, and gz_ros2_control fails hard ("found an
    # empty parameters file") if it's left unset, which silently means the
    # controller_manager never starts and every controller spawner below
    # hangs forever waiting for a service that never comes up.
    controllers_yaml = os.path.join(pkg_hexapod_gazebo, 'config', 'ros2_controllers.yaml')
    xacro_file = os.path.join(pkg_hexapod_description, 'urdf', 'hexapod.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file, mappings={'controllers_file': controllers_yaml})
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hexapod_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gait_params = os.path.join(pkg_hexapod_control, 'config', 'gait.yaml')

    tripod_gait_node = Node(
        package='hexapod_control',
        executable='tripod_gait_controller',
        name='tripod_gait_controller',
        parameters=[gait_params],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        spawn_jsb,
        spawn_position_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_position_controller,
                on_exit=[tripod_gait_node],
            )
        ),
    ])
