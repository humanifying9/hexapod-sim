#!/usr/bin/env python3
"""
leg_controller.py — ROS 2 node for controlling a single hexapod leg.

┌── What this node does ─────────────────────────────────────────┐
│  1. Publishes JointState messages to /joint_states at 30 Hz.   │
│     This tells Rviz/Gazebo the current angle of each joint.    │
│                                                                │
│  2. Subscribes to /leg/command (string) where you can type a   │
│     command like "home" or "wave".                              │
│                                                                │
│  3. Implements a sinusoidal "wave" demo so you can see the     │
│     tibia moving in simulation without any external input.     │
└──────────────────────────────────────────────────────────────────┘

Joint names (must match the URDF):
  - coxa_joint   (yaw, range ±90° = ±1.57 rad)
  - femur_joint  (pitch, range ±90° = ±1.57 rad)
  - tibia_joint  (pitch, range ±90° = ±1.57 rad)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# The three joints defined in hexapod_leg.urdf.xacro
JOINT_NAMES = ['coxa_joint', 'femur_joint', 'tibia_joint']

# Default "home" pose (all joints centered at 0 radians)
HOME_POSE = [0.0, 0.0, 0.0]

# A preset "ready" pose for walking
READY_POSE = [0.0, -0.5, 0.5]


class LegController(Node):
    """A simple ROS 2 node that publishes joint states and responds to commands."""

    def __init__(self):
        super().__init__('leg_controller')

        # Store current joint positions (radians)
        self.joint_positions = list(HOME_POSE)
        # Track whether we're in "wave" demo mode
        self._wave_mode = False

        # ── Publisher: /joint_states ──────────────────────────────
        # JointState is the standard ROS message for reporting joint
        # angles. robot_state_publisher consumes this topic to
        # compute the forward kinematics (i.e., where each link is).
        self._joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )

        # ── Subscriber: /leg/command ──────────────────────────────
        # Send a text command from the terminal:
        #   ros2 topic pub /leg/command std_msgs/String "data: 'wave'"
        #   ros2 topic pub /leg/command std_msgs/String "data: 'home'"
        #   ros2 topic pub /leg/command std_msgs/String "data: 'ready'"
        self._cmd_sub = self.create_subscription(
            String, '/leg/command', self._handle_command, 10
        )

        # ── Timer: publish at 30 Hz ────────────────────────────────
        self._timer = self.create_timer(1.0 / 30.0, self._publish_joint_states)

        self.get_logger().info(
            f'Leg controller node started. Joints: {JOINT_NAMES}. '
            f'Publish std_msgs/String "wave", "home", or "ready" to /leg/command.'
        )

    def _handle_command(self, msg: String):
        """Handle text commands from the /leg/command topic."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received command: "{cmd}"')

        if cmd == 'wave':
            self._wave_mode = True
            self.get_logger().info('Wave demo started! (sends sinusoidal tibia motion)')
        elif cmd == 'stop':
            self._wave_mode = False
        elif cmd == 'home':
            self._wave_mode = False
            self.joint_positions = list(HOME_POSE)
            self.get_logger().info(f'Joint positions set to HOME: {self.joint_positions}')
        elif cmd == 'ready':
            self._wave_mode = False
            self.joint_positions = list(READY_POSE)
            self.get_logger().info(f'Joint positions set to READY: {self.joint_positions}')
        else:
            self.get_logger().warn(
                f'Unknown command: "{cmd}". Try: wave, stop, home, ready'
            )

    def _publish_joint_states(self):
        """Send JointState messages at 30 Hz (called by the timer)."""
        # If in wave mode, compute a sinusoidal tibia position
        if self._wave_mode:
            t = self.get_clock().now().nanoseconds * 1e-9
            # Slow 5-second period sine wave for the tibia
            tibia_val = 1.0 * math.sin(t * 2 * math.pi / 5.0)
            self.joint_positions[2] = tibia_val  # index 2 = tibia_joint

        # Build and publish the JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self.joint_positions)

        self._joint_state_pub.publish(msg)


def main(args=None):
    """Entry point called by the `leg_controller` console script."""
    rclpy.init(args=args)
    node = LegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down leg controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
