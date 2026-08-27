"""FK(IK(p)) == p over a grid of reachable points, as promised by
leg_kinematics.py's module docstring."""

import math

from hexapod_control.leg_kinematics import LegGeometry, LegKinematics


def test_round_trip_over_reachable_grid():
    geom = LegGeometry()
    ik = LegKinematics(geom)

    # Parametrize by (dist, theta) in the leg's r/h plane so every point is
    # guaranteed inside the reachable annulus - margin keeps clear of the
    # clamp boundary, where FK(IK(p)) intentionally would not equal p.
    margin = 0.005
    lo, hi = geom.min_reach + margin, geom.femur_length + geom.tibia_length - margin

    for coxa_deg in range(-60, 61, 20):
        for dist in (lo, (lo + hi) / 2, hi):
            for theta_deg in (10, 45, 80):
                coxa_angle = math.radians(coxa_deg)
                theta = math.radians(theta_deg)
                r, h = dist * math.cos(theta), dist * math.sin(theta)

                x = (geom.coxa_length + r) * math.cos(coxa_angle)
                y = (geom.coxa_length + r) * math.sin(coxa_angle)
                z = -h

                q1, q2, q3 = ik.inverse(x, y, z)
                fx, fy, fz = ik.forward(q1, q2, q3)

                assert math.isclose(fx, x, abs_tol=1e-6)
                assert math.isclose(fy, y, abs_tol=1e-6)
                assert math.isclose(fz, z, abs_tol=1e-6)


def test_out_of_reach_target_clamps_instead_of_raising():
    ik = LegKinematics()
    # Absurdly far target - must clamp into the reachable annulus, not raise.
    q1, q2, q3 = ik.inverse(10.0, 0.0, 0.0)
    fx, fy, fz = ik.forward(q1, q2, q3)
    assert math.hypot(fx, fy) < 1.0
