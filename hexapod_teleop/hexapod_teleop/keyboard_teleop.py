"""
keyboard_teleop.py - WASD keyboard teleop for testing the hexapod's gait
before real input hardware exists.

Publishes geometry_msgs/Twist to /cmd_vel - the exact topic
hexapod_control/tripod_gait_controller already listens to. That's the whole
point of routing input through /cmd_vel: this node is a stand-in for now,
and the planned custom PCB controller (and eventually an onboard agent)
only need to publish the same message to the same topic later - nothing
downstream has to change.

Run with `ros2 run hexapod_teleop keyboard_teleop`, not `ros2 launch` - launch
does not forward the parent terminal's stdin to the process it starts, so a
launched copy always hits the "not a tty" guard below and exits immediately.

Keys (terminal running this node needs focus):
  w/s        forward / backward
  a/d        strafe left / right
  q/e        rotate left / right
  space, x   stop
  Ctrl-C     quit

Uses raw terminal mode to read one key at a time (the same approach ROS's
own teleop_twist_keyboard uses) - there's no key-up event from a terminal,
so "holding" a key relies on your OS's key-repeat resending it faster than
tripod_gait_controller's cmd_vel_timeout (0.5s by default in gait.yaml). If
walking stutters when you hold a key, your terminal's repeat rate is
probably slower than that.
"""

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# key -> (vx, vy, wz) direction, scaled by linear_speed/angular_speed below.
KEY_BINDINGS = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}
STOP_KEYS = (' ', 'x')
QUIT_KEYS = ('\x03',)  # Ctrl-C


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed', 0.05)
        self.declare_parameter('angular_speed', 0.4)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx) * self.linear_speed
        msg.linear.y = float(vy) * self.linear_speed
        msg.angular.z = float(wz) * self.angular_speed
        self.cmd_pub.publish(msg)


def _read_key(fd, saved_settings) -> str:
    tty.setraw(fd)
    key = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, saved_settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error(
            'keyboard_teleop needs an interactive terminal (stdin is not a tty)')
        node.destroy_node()
        rclpy.shutdown()
        return

    fd = sys.stdin.fileno()
    saved_settings = termios.tcgetattr(fd)
    print('WASD to move, Q/E to turn, space/x to stop, Ctrl-C to quit.')

    try:
        while rclpy.ok():
            key = _read_key(fd, saved_settings)
            if key in QUIT_KEYS:
                break
            elif key in STOP_KEYS:
                node.publish(0, 0, 0)
            elif key in KEY_BINDINGS:
                node.publish(*KEY_BINDINGS[key])
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved_settings)
        node.publish(0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
