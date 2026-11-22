"""2-DOF planar robotic arm forward kinematics and workspace geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass


def _validate_finite(name: str, value: float) -> float:
    val = float(value)
    if not math.isfinite(val):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return val


@dataclass(frozen=True)
class ArmGeometry:
    """Link lengths for a planar 2R robotic arm."""

    l1: float = 5.0
    l2: float = 5.0

    def __post_init__(self) -> None:
        l1_val = _validate_finite("l1", self.l1)
        l2_val = _validate_finite("l2", self.l2)
        if l1_val <= 0.0:
            raise ValueError(f"l1 must be positive, got {self.l1!r}")
        if l2_val <= 0.0:
            raise ValueError(f"l2 must be positive, got {self.l2!r}")
        object.__setattr__(self, "l1", l1_val)
        object.__setattr__(self, "l2", l2_val)


@dataclass(frozen=True)
class ArmPose:
    """Cartesian elbow and end-effector coordinates for given joint angles."""

    theta1_deg: float
    theta2_deg: float
    x1: float
    y1: float
    x: float
    y: float
    reach_radius: float


def forward_kinematics(
    theta1_deg: float,
    theta2_deg: float,
    geometry: ArmGeometry | None = None,
) -> ArmPose:
    """Compute 2-DOF planar forward kinematics from joint angles in degrees."""
    geom = geometry if geometry is not None else ArmGeometry()
    t1_deg = _validate_finite("theta1_deg", theta1_deg)
    t2_deg = _validate_finite("theta2_deg", theta2_deg)

    t1_rad = math.radians(t1_deg)
    t2_rad = math.radians(t2_deg)
    t12_rad = t1_rad + t2_rad

    x1 = geom.l1 * math.cos(t1_rad)
    y1 = geom.l1 * math.sin(t1_rad)
    x = x1 + geom.l2 * math.cos(t12_rad)
    y = y1 + geom.l2 * math.sin(t12_rad)
    reach = math.hypot(x, y)

    return ArmPose(
        theta1_deg=t1_deg,
        theta2_deg=t2_deg,
        x1=x1,
        y1=y1,
        x=x,
        y=y,
        reach_radius=reach,
    )


def link_polyline(pose: ArmPose) -> tuple[list[float], list[float]]:
    """Return ([0, x1, X], [0, y1, Y]) polyline vectors for XY plotting."""
    return ([0.0, pose.x1, pose.x], [0.0, pose.y1, pose.y])


def workspace_bounds(geometry: ArmGeometry) -> tuple[float, float]:
    """Return (inner_radius, outer_radius) of the reachable planar annulus."""
    return (abs(geometry.l1 - geometry.l2), geometry.l1 + geometry.l2)
