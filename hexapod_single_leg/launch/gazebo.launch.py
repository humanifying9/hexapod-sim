"""
Launch file: brings up the full single-leg hexapod simulation stack.

┌── Launch Sequence ──────────────────────────────────────────┐
│  1. robot_state_publisher  → reads URDF, publishes TF      │
│  2. controller_manager     → manages ROS 2 controllers      │
│  3. Gazebo Sim (gz)        → physics engine + empty world  │
│  4. spawn_entity           → injects robot into Gazebo     │
│  5. joint_state_broadcaster spawn → publishes /joint_states│
│  6. hexapod_controller spawn → listens for trajectories    │
│  7. leg_controller (Python) → user-facing joint control    │
│                                                          │
│  Start with:  ros2 launch hexapod_single_leg gazebo.launch.py
└──────────────────────────────────────────────────────────┘
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():
    pkg_name = 'hexapod_single_leg'
    pkg_share = get_package_share_directory(pkg_name)

    # ── 1. Process the Xacro → URDF ──────────────────────────────
    # xacro is a macro language for URDF. It expands ${} expressions
    # and custom tags, producing a plain URDF string.
    xacro_file = os.path.join(pkg_share, 'urdf', 'hexapod_leg.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── 2. Robot State Publisher Node ─────────────────────────────
    # This node reads the robot_description and the /joint_states topic,
    # then computes the transform (TF) of every link → where each part
    # is in 3D space at this instant.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # ── 3. Controller Manager Node ────────────────────────────────
    # Loads ros2_controllers. It reads our YAML config, creates the
    # joint_state_broadcaster and hexapod_controller controllers,
    # and starts the update loop.
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_share, 'config', 'hexapod_single_leg.yaml'),
            {'use_sim_time': True},
        ]
    )

    # ── 4. Gazebo Sim (Harmonic / Gazebo Fortress) ────────────────
    # We use ros_gz_sim which bridges ROS 2 and Gazebo (the Ignition
    # fork). It loads an empty world with physics enabled.
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v 1 empty.sdf'
        }.items(),
    )

    # ── 5. Spawn Entity ───────────────────────────────────────────
    # Spawns our robot model into the Gazebo world using the
    # robot_description topic that robot_state_publisher set up.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hexapod_leg',
            '-z', '0.4',  # spawn 0.4m above ground
        ],
        output='screen'
    )

    # ── 6. Gazebo ROS Bridge Arguments ────────────────────────────
    # These bridge parameters tell ros2_control/Gazebo how to
    # communicate. In practice, the controller_manager node
    # communicates with Gazebo via the ros2_control plugin
    # embedded in Gazebo.

    # ── 7. Spawn Joint State Broadcaster ──────────────────────────
    # This controller reads joint positions from the simulation
    # and publishes them to /joint_states (which RViz uses).
    # We delay it slightly so the controller_manager has time
    # to register the hardware interfaces.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   '/controller_manager'],
        output='screen',
    )
    # Delay the spawner by 5 seconds to let Gazebo load first
    delayed_jsb_spawner = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # ── 8. Spawn Hexapod Trajectory Controller ────────────────────
    hexapod_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_controller', '--controller-manager',
                   '/controller_manager'],
        output='screen',
    )
    delayed_controller_spawner = TimerAction(
        period=6.0,
        actions=[hexapod_controller_spawner]
    )

    # ── 9. Our Custom Leg Controller Node ─────────────────────────
    # This is our Python node that publishes /joint_states and
    # listens for text commands on /leg/command.
    leg_controller_node = Node(
        package=pkg_name,
        executable='leg_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    delayed_leg_controller = TimerAction(
        period=3.0,
        actions=[leg_controller_node]
    )

    # ── 10. Bridge: forward /joint_states from Gazebo ───────────
    # ros2_control already publishes to /joint_states via the
    # joint_state_broadcaster, so we don't need an extra bridge.
    # However, we DO need a bridge for the Gazebo clock so that
    # use_sim_time works correctly.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── 11. Ensure Gazebo finishes loading before spawning ────────
    # We use an event handler: after spawn_entity exits, start
    # the controller spawners. This prevents race conditions.
    kill_controller_manager_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                delayed_leg_controller,
                delayed_jsb_spawner,
                delayed_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        gz_bridge,
        controller_manager_node,
        spawn_entity,
        kill_controller_manager_event,
    ])
