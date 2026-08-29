"""
Forward and inverse kinematics for one hexapod leg.

Pure maths, no ROS. That keeps it unit-testable (see test/test_kinematics.py,
which checks FK(IK(p)) == p over a grid of reachable points).

======================= THE GEOMETRY =======================

Everything happens in the LEG-LOCAL frame: the origin is the coxa joint, +x
points out along the leg at coxa angle zero, +z is up. The leg's mounting yaw on
the body is applied outside this class, which is why all six legs share one
solver with no left/right mirroring.

The chain, matching hexapod_description/urdf/leg.xacro exactly:

    coxa joint   at origin,       rotates q1 about +z
    coxa link    length Lc along +x
    femur joint  at (Lc, 0, 0),   frame PRE-PITCHED by beta, then rotates q2 about +y
    femur link   length Lf along +x
    tibia joint  at (Lf, 0, 0),   rotates q3 about +y
    tibia link   length Lt along +x  ->  foot at the tip

`beta` is the 55 degree mechanical tilt built into the femur bracket. It is not
a joint - it is a constant offset baked into the URDF as rpy="0 55deg 0" on the
femur joint origin. Its one consequence is that the femur servo angle q2 is
measured from an already-tilted frame, so the femur's true angle below
horizontal is (beta + q2), not q2.

We therefore work with two "absolute" angles measured down from horizontal:

    phi2 = beta + q2              femur, below horizontal
    phi3 = beta + q2 + q3         tibia, below horizontal

and the leg reduces to an ordinary planar 2-link arm in (r, h), where r is the
horizontal reach from the coxa axis and h is the drop below it:

    r = Lc + Lf*cos(phi2) + Lt*cos(phi3)
    h =      Lf*sin(phi2) + Lt*sin(phi3)

Rotating a point (L,0,0) about +y by q sends it to (L*cos q, 0, -L*sin q), which
is why a POSITIVE angle lowers the foot and h carries a positive sign.

======================= BRANCH SELECTION =======================

A 2-link arm has two solutions ("knee up" / "knee down"). We always take the
NEGATIVE q3 branch, because the real servos run -150..+30 degrees: the tibia
folds back underneath the femur. Taking the positive branch would produce
angles the hardware physically cannot reach.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LegGeometry:
    """Link lengths and the fixed femur tilt, all in metres / radians.

    Defaults mirror hexapod_description/urdf/common.xacro. If you re-measure the
    robot, change them there and in hexapod_control/config/gait.yaml.
    """

    coxa_length: float = 0.0764
    femur_length: float = 0.0859
    tibia_length: float = 0.1194
    coxa_tilt: float = math.radians(50.0)

    @property
    def max_reach(self) -> float:
        """Furthest the foot can get from the coxa axis, in the r/h plane."""
        return self.coxa_length + self.femur_length + self.tibia_length

    @property
    def min_reach(self) -> float:
        """Closest the foot can fold toward the coxa axis."""
        return abs(self.femur_length - self.tibia_length)


class LegKinematics:
    """Solves one leg. Stateless - safe to share across all six."""

    def __init__(self, geometry: LegGeometry | None = None):
        self.geom = geometry or LegGeometry()

    # ------------------------------------------------------------------
    # forward
    # ------------------------------------------------------------------
    def forward(self, q1: float, q2: float, q3: float) -> tuple[float, float, float]:
        """Joint angles -> foot position (x, y, z) in the leg-local frame."""
        g = self.geom
        phi2 = g.coxa_tilt + q2
        phi3 = g.coxa_tilt + q2 + q3

        # Reach and drop in the leg's vertical plane.
        r = g.coxa_length + g.femur_length * math.cos(phi2) + g.tibia_length * math.cos(phi3)
        h = g.femur_length * math.sin(phi2) + g.tibia_length * math.sin(phi3)

        return (r * math.cos(q1), r * math.sin(q1), -h)

    # ------------------------------------------------------------------
    # inverse
    # ------------------------------------------------------------------
    def inverse(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Foot position (x, y, z) in the leg-local frame -> joint angles.

        Targets outside the reachable annulus are clamped to its edge rather
        than raising, so a gait that briefly over-reaches degrades into a
        straightened leg instead of crashing the controller mid-step.
        """
        g = self.geom

        # Coxa simply points the leg's vertical plane at the target.
        q1 = math.atan2(y, x)

        # Collapse into that plane. r is measured from the FEMUR joint, so the
        # coxa length comes off the front.
        r = math.hypot(x, y) - g.coxa_length
        h = -z

        dist = math.hypot(r, h)

        # Clamp into the reachable annulus. The small epsilon keeps us off the
        # exact singularity at full extension, where the Jacobian blows up.
        eps = 1e-4
        lo = g.min_reach + eps
        hi = g.femur_length + g.tibia_length - eps
        if dist > hi or dist < lo:
            clamped = min(max(dist, lo), hi)
            if dist < 1e-9:
                # Degenerate: target sits on the femur axis. Push it straight
                # down, which is always a sane fallback for a walking robot.
                r, h = 0.0, clamped
            else:
                scale = clamped / dist
                r, h = r * scale, h * scale
            dist = clamped

        # Cosine rule for the femur-tibia interior angle.
        cos_q3 = (dist * dist - g.femur_length ** 2 - g.tibia_length ** 2) / (
            2.0 * g.femur_length * g.tibia_length
        )
        cos_q3 = min(1.0, max(-1.0, cos_q3))

        # Negative branch: the tibia folds back under the femur, matching the
        # -150..+30 degree servo range.
        q3 = -math.acos(cos_q3)

        # Femur angle below horizontal, then remove the fixed 55 degree tilt to
        # get the actual servo command.
        phi2 = math.atan2(h, r) - math.atan2(
            g.tibia_length * math.sin(q3),
            g.femur_length + g.tibia_length * math.cos(q3),
        )
        q2 = phi2 - g.coxa_tilt

        return (q1, q2, q3)
