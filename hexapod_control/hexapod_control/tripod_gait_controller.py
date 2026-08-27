"""
tripod_gait_controller.py - drives the hexapod's tripod gait from /cmd_vel.

The six legs split into two tripods that alternate: fl+mr+rl (group A) and
fr+ml+rr (group B) - one triangle of feet is always planted while the other
swings forward, which is what keeps the robot statically stable throughout
the gait. Each leg's foot traces a stance/swing cycle in its own leg-local
frame - identical maths for all six, per leg_kinematics.py - which
LegKinematics.inverse() turns into joint angles. All 18 angles are published
in one Float64MultiArray to /joint_group_position_controller/commands, in
the exact joint order hexapod_gazebo/config/ros2_controllers.yaml declares.

============================= TURNING CMD_VEL INTO A STEP =============================
Body-frame cmd_vel (vx, vy, wz) has to become a per-leg step vector:

  1. wz contributes a tangential velocity at each leg's mount point,
     v = wz x r (r = the leg's (mount_x, mount_y) offset from the body
     centre - see hexapod_description/urdf/hexapod.urdf.xacro).
  2. The foot needs to move backward relative to the body while it's
     planted (that's what pushes the body forward), so the target foot
     velocity is the NEGATIVE of the body-point velocity above.
  3. That body-frame vector gets rotated by -mount_yaw to land in the leg's
     own local frame, where +x points radially outward at that leg's splay
     angle (see leg.xacro - this is why one IK solver works for all six legs
     with no left/right mirroring).

Over one stance phase (half the gait period) the foot needs to travel the
full step, so half the step displacement is local_velocity * period / 4.
The trajectory then interpolates linearly from +half_step to -half_step
during stance (foot drags backward, on the ground) and back from -half_step
to +half_step during swing (foot lifts on a sine arc and returns forward),
matching up continuously at both ends of the cycle.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from hexapod_control.leg_kinematics import LegGeometry, LegKinematics

# (mount_x, mount_y, mount_yaw_rad) for each leg - mirrors the <xacro:leg .../>
# calls in hexapod_description/urdf/hexapod.urdf.xacro. Keep in sync.
LEG_MOUNTS = {
    'fl': (0.080, 0.060, math.radians(40)),
    'ml': (0.000, 0.060, math.radians(90)),
    'rl': (-0.080, 0.060, math.radians(140)),
    'fr': (0.080, -0.060, -math.radians(40)),
    'mr': (0.000, -0.060, -math.radians(90)),
    'rr': (-0.080, -0.060, -math.radians(140)),
}

# Publish order must match hexapod_gazebo/config/ros2_controllers.yaml.
LEG_ORDER = ['fl', 'ml', 'rl', 'fr', 'mr', 'rr']

TRIPOD_A = {'fl', 'mr', 'rl'}  # the remaining three (fr, ml, rr) are tripod B


class TripodGaitController(Node):

    def __init__(self):
        super().__init__('tripod_gait_controller')

        self.declare_parameter('coxa_length', 0.045)
        self.declare_parameter('femur_length', 0.075)
        self.declare_parameter('tibia_length', 0.130)
        self.declare_parameter('coxa_tilt_deg', 55.0)
        self.declare_parameter('stance_reach', 0.080)
        self.declare_parameter('stance_drop', 0.110)
        self.declare_parameter('gait_period', 1.0)
        self.declare_parameter('step_height', 0.030)
        self.declare_parameter('max_step_length', 0.050)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        geom = LegGeometry(
            coxa_length=self.get_parameter('coxa_length').value,
            femur_length=self.get_parameter('femur_length').value,
            tibia_length=self.get_parameter('tibia_length').value,
            coxa_tilt=math.radians(self.get_parameter('coxa_tilt_deg').value),
        )
        self.ik = LegKinematics(geom)

        self.period = self.get_parameter('gait_period').value
        self.step_height = self.get_parameter('step_height').value
        self.max_step_length = self.get_parameter('max_step_length').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # Neutral standing foot position, leg-local frame. Same for all six
        # legs - see leg_kinematics.py's docstring on why that's valid.
        stance_reach = self.get_parameter('stance_reach').value
        stance_drop = self.get_parameter('stance_drop').value
        self.neutral_xyz = (geom.coxa_length + stance_reach, 0.0, -stance_drop)

        self._cmd = (0.0, 0.0, 0.0)  # vx, vy, wz in the body frame
        self._last_cmd_time = self.get_clock().now()
        self._phase = 0.0

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)

        self._dt = 1.0 / self.get_parameter('control_rate').value
        self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f'tripod_gait_controller up: period={self.period}s, '
            f'neutral foot={self.neutral_xyz}, '
            f'control_rate={1.0 / self._dt:.0f}Hz')

    def _on_cmd_vel(self, msg: Twist):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _on_timer(self):
        # Safety: a stale /cmd_vel (publisher died, teleop disconnected)
        # decays to standing still instead of the last command forever.
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        vx, vy, wz = self._cmd if age < self.cmd_vel_timeout else (0.0, 0.0, 0.0)

        # Below this, treat cmd_vel as "not moving" - hold every foot at the
        # neutral stance instead of still cycling legs through a swing arc
        # with zero stride length, which would just lift and replant them in
        # place for no reason. The phase freezes here too, so motion always
        # resumes with a clean gait cycle rather than partway through one.
        standing_still = abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(wz) < 1e-4
        if not standing_still:
            self._phase = (self._phase + self._dt / self.period) % 1.0

        commands = []
        for leg in LEG_ORDER:
            mount_x, mount_y, mount_yaw = LEG_MOUNTS[leg]
            group_offset = 0.0 if leg in TRIPOD_A else 0.5
            local_phase = (self._phase + group_offset) % 1.0

            # Foot velocity (relative to body) needed to realize this cmd_vel
            # at this leg's mount point - see the module docstring.
            body_vx = -(vx - wz * mount_y)
            body_vy = -(vy + wz * mount_x)

            c, s = math.cos(-mount_yaw), math.sin(-mount_yaw)
            local_vx = c * body_vx - s * body_vy
            local_vy = s * body_vx + c * body_vy

            half_step_x = _clamp(local_vx * self.period / 4.0, self.max_step_length)
            half_step_y = _clamp(local_vy * self.period / 4.0, self.max_step_length)

            x0, y0, z0 = self.neutral_xyz
            if standing_still:
                dx = dy = dz = 0.0
            elif local_phase < 0.5:
                # Stance: foot planted, dragging from +half_step to -half_step.
                t = local_phase / 0.5
                dx, dy, dz = half_step_x * (1 - 2 * t), half_step_y * (1 - 2 * t), 0.0
            else:
                # Swing: foot lifts and returns from -half_step to +half_step.
                t = (local_phase - 0.5) / 0.5
                dx, dy = -half_step_x * (1 - 2 * t), -half_step_y * (1 - 2 * t)
                dz = self.step_height * math.sin(math.pi * t)

            q1, q2, q3 = self.ik.inverse(x0 + dx, y0 + dy, z0 + dz)
            commands.extend([q1, q2, q3])

        msg = Float64MultiArray()
        msg.data = commands
        self.joint_pub.publish(msg)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = TripodGaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
