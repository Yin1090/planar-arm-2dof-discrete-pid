"""Unit tests for 2-DOF planar robotic arm forward kinematics."""

from __future__ import annotations

import math

import pytest

from planar_arm_pid.kinematics import (
    ArmGeometry,
    forward_kinematics,
    link_polyline,
    workspace_bounds,
)


def test_forward_kinematics_analytical_poses() -> None:
    geom = ArmGeometry(l1=5.0, l2=5.0)

    p_zero = forward_kinematics(0.0, 0.0, geom)
    assert (p_zero.x1, p_zero.y1) == pytest.approx((5.0, 0.0), abs=1e-12)
    assert (p_zero.x, p_zero.y) == pytest.approx((10.0, 0.0), abs=1e-12)

    p_up = forward_kinematics(90.0, 0.0, geom)
    assert (p_up.x, p_up.y) == pytest.approx((0.0, 10.0), abs=1e-12)

    p_bent = forward_kinematics(90.0, -90.0, geom)
    assert (p_bent.x1, p_bent.y1) == pytest.approx((0.0, 5.0), abs=1e-12)
    assert (p_bent.x, p_bent.y) == pytest.approx((5.0, 5.0), abs=1e-12)

    p_30_30 = forward_kinematics(30.0, 30.0, geom)
    expected_xy = 5.0 * math.cos(math.radians(30.0)) + 5.0 * math.cos(math.radians(60.0))
    assert expected_xy == pytest.approx(6.830127018922194, abs=1e-12)
    assert p_30_30.x == pytest.approx(expected_xy, abs=1e-12)
    assert p_30_30.y == pytest.approx(expected_xy, abs=1e-12)


def test_link_polyline_starts_at_origin_and_ends_at_effector() -> None:
    pose = forward_kinematics(30.0, 30.0, ArmGeometry(l1=5.0, l2=5.0))
    xs, ys = link_polyline(pose)
    assert xs == [0.0, pose.x1, pose.x]
    assert ys == [0.0, pose.y1, pose.y]


def test_workspace_bounds_symmetric_and_asymmetric() -> None:
    assert workspace_bounds(ArmGeometry(l1=5.0, l2=5.0)) == (0.0, 10.0)
    assert workspace_bounds(ArmGeometry(l1=6.0, l2=2.0)) == (4.0, 8.0)


@pytest.mark.parametrize("l1,l2", [(0.0, 5.0), (-1.0, 5.0), (5.0, 0.0), (5.0, -2.0), (math.nan, 5.0)])
def test_invalid_link_lengths_raise_value_error(l1: float, l2: float) -> None:
    with pytest.raises(ValueError):
        ArmGeometry(l1=l1, l2=l2)
