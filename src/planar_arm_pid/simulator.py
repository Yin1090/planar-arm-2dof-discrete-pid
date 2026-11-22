"""Closed-loop 2-DOF discrete PID + forward kinematics simulator."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Sequence

import numpy as np

from planar_arm_pid.kinematics import ArmGeometry, ArmPose, forward_kinematics
from planar_arm_pid.pid import DiscretePIDController, PIDGains


def _validate_positive_finite(name: str, value: float) -> float:
    val = float(value)
    if not math.isfinite(val) or val <= 0.0:
        raise ValueError(f"{name} must be positive and finite, got {value!r}")
    return val


@dataclass(frozen=True)
class SimulationConfig:
    """Configuration for dual-joint discrete PID and 2-DOF arm simulation."""

    geometry: ArmGeometry = ArmGeometry()
    gains1: PIDGains = PIDGains()
    gains2: PIDGains = PIDGains()
    dt: float = 0.01
    dt2: float | None = None
    theta1_init_deg: float = 0.0
    theta2_init_deg: float = 0.0

    def __post_init__(self) -> None:
        dt_val = _validate_positive_finite("dt", self.dt)
        object.__setattr__(self, "dt", dt_val)
        if self.dt2 is not None:
            dt2_val = _validate_positive_finite("dt2", self.dt2)
            object.__setattr__(self, "dt2", dt2_val)
        t1 = float(self.theta1_init_deg)
        t2 = float(self.theta2_init_deg)
        if not (math.isfinite(t1) and math.isfinite(t2)):
            raise ValueError("Initial joint angles must be finite")
        object.__setattr__(self, "theta1_init_deg", t1)
        object.__setattr__(self, "theta2_init_deg", t2)

    @property
    def effective_dt2(self) -> float:
        return self.dt if self.dt2 is None else self.dt2


@dataclass(frozen=True)
class SimulationStep:
    """Single discrete simulation sample."""

    step: int
    time_s: float
    theta1_ref_deg: float
    theta2_ref_deg: float
    theta1_actual_deg: float
    theta2_actual_deg: float
    error1_deg: float
    error2_deg: float
    u1: float
    u2: float
    x1: float
    y1: float
    x: float
    y: float


@dataclass(frozen=True)
class JointMetrics:
    """Summary step/tracking performance metrics for one joint."""

    final_angle_deg: float
    final_error_deg: float
    peak_angle_deg: float
    overshoot_pct: float
    settling_time_s: float | None
    rmse_deg: float
    iae_deg_s: float


@dataclass(frozen=True)
class WaypointSegment:
    """Constant-reference waypoint segment for multi-phase trajectories."""

    theta1_ref_deg: float
    theta2_ref_deg: float
    duration_s: float
    reset_before: bool = False

    def __post_init__(self) -> None:
        t1 = float(self.theta1_ref_deg)
        t2 = float(self.theta2_ref_deg)
        if not (math.isfinite(t1) and math.isfinite(t2)):
            raise ValueError("Waypoint reference angles must be finite")
        dur = _validate_positive_finite("duration_s", self.duration_s)
        object.__setattr__(self, "theta1_ref_deg", t1)
        object.__setattr__(self, "theta2_ref_deg", t2)
        object.__setattr__(self, "duration_s", dur)


@dataclass(frozen=True)
class SimulationResult:
    """Complete trajectory, per-joint performance metrics, and final pose."""

    config: SimulationConfig
    steps: list[SimulationStep]
    joint1_metrics: JointMetrics
    joint2_metrics: JointMetrics
    final_pose: ArmPose

    def to_dict(self) -> dict[str, Any]:
        last = self.steps[-1]
        return {
            "config": {
                "l1": self.config.geometry.l1,
                "l2": self.config.geometry.l2,
                "dt": self.config.dt,
                "dt2": self.config.effective_dt2,
                "gains1": asdict(self.config.gains1),
                "gains2": asdict(self.config.gains2),
                "theta1_init_deg": self.config.theta1_init_deg,
                "theta2_init_deg": self.config.theta2_init_deg,
            },
            "num_steps": len(self.steps),
            "duration_s": last.time_s,
            "final_step": asdict(last),
            "final_pose": asdict(self.final_pose),
            "joint1_metrics": asdict(self.joint1_metrics),
            "joint2_metrics": asdict(self.joint2_metrics),
        }


def compute_joint_metrics(
    times_s: Sequence[float],
    refs_deg: Sequence[float],
    angles_deg: Sequence[float],
    errors_deg: Sequence[float],
    dt: float,
    initial_angle_deg: float = 0.0,
) -> JointMetrics:
    """Compute final value, overshoot, 2%-band settling time, RMSE, and IAE."""
    if not angles_deg:
        raise ValueError("Cannot compute metrics on an empty trajectory")

    angles_arr = np.asarray(angles_deg, dtype=float)
    errors_arr = np.asarray(errors_deg, dtype=float)
    final_ref = float(refs_deg[-1])
    final_angle = float(angles_arr[-1])
    final_error = float(errors_arr[-1])

    delta = final_ref - float(initial_angle_deg)
    if abs(delta) > 1e-9:
        sign = 1.0 if delta > 0.0 else -1.0
        excursions = (angles_arr - final_ref) * sign
        peak_idx = int(np.argmax(excursions))
        peak_angle = float(angles_arr[peak_idx])
        max_excursion = float(excursions[peak_idx])
        overshoot_pct = max(0.0, (max_excursion / abs(delta)) * 100.0)
        tol = 0.02 * abs(delta)
    else:
        peak_idx = int(np.argmax(np.abs(angles_arr - final_ref)))
        peak_angle = float(angles_arr[peak_idx])
        overshoot_pct = 0.0
        tol = 0.02

    within = np.abs(angles_arr - final_ref) <= tol
    settling_time_s: float | None = None
    if bool(within[-1]):
        outside_indices = np.flatnonzero(~within)
        if outside_indices.size == 0:
            settling_time_s = float(times_s[0])
        else:
            first_settled_idx = int(outside_indices[-1]) + 1
            if first_settled_idx < len(times_s):
                settling_time_s = float(times_s[first_settled_idx])

    rmse_deg = float(np.sqrt(np.mean(errors_arr ** 2)))
    iae_deg_s = float(np.sum(np.abs(errors_arr)) * dt)

    return JointMetrics(
        final_angle_deg=final_angle,
        final_error_deg=final_error,
        peak_angle_deg=peak_angle,
        overshoot_pct=overshoot_pct,
        settling_time_s=settling_time_s,
        rmse_deg=rmse_deg,
        iae_deg_s=iae_deg_s,
    )


def _finalize_result(
    config: SimulationConfig,
    steps: list[SimulationStep],
) -> SimulationResult:
    times = [s.time_s for s in steps]
    r1 = [s.theta1_ref_deg for s in steps]
    r2 = [s.theta2_ref_deg for s in steps]
    a1 = [s.theta1_actual_deg for s in steps]
    a2 = [s.theta2_actual_deg for s in steps]
    e1 = [s.error1_deg for s in steps]
    e2 = [s.error2_deg for s in steps]

    m1 = compute_joint_metrics(times, r1, a1, e1, config.dt, config.theta1_init_deg)
    m2 = compute_joint_metrics(
        times, r2, a2, e2, config.effective_dt2, config.theta2_init_deg
    )
    last = steps[-1]
    final_pose = forward_kinematics(
        last.theta1_actual_deg,
        last.theta2_actual_deg,
        config.geometry,
    )
    return SimulationResult(
        config=config,
        steps=steps,
        joint1_metrics=m1,
        joint2_metrics=m2,
        final_pose=final_pose,
    )


def simulate_step(
    theta1_ref_deg: float = 30.0,
    theta2_ref_deg: float = 30.0,
    duration_s: float = 15.0,
    config: SimulationConfig | None = None,
    reset_midway_step: int | None = None,
) -> SimulationResult:
    """Run closed-loop dual-joint step response simulation for k = 0..round(duration_s/dt)."""
    cfg = config if config is not None else SimulationConfig()
    ref1 = float(theta1_ref_deg)
    ref2 = float(theta2_ref_deg)
    if not (math.isfinite(ref1) and math.isfinite(ref2)):
        raise ValueError("Reference angles must be finite")
    dur = _validate_positive_finite("duration_s", duration_s)

    n_intervals = int(round(dur / cfg.dt))
    if n_intervals < 1:
        raise ValueError("duration_s must cover at least one sample dt")
    total_steps = n_intervals + 1

    if reset_midway_step is not None and not (0 <= reset_midway_step < total_steps):
        raise ValueError(
            f"reset_midway_step must be in [0, {total_steps - 1}], got {reset_midway_step}"
        )

    pid1 = DiscretePIDController(cfg.gains1, cfg.dt, cfg.theta1_init_deg)
    pid2 = DiscretePIDController(cfg.gains2, cfg.effective_dt2, cfg.theta2_init_deg)

    steps: list[SimulationStep] = []
    for k in range(total_steps):
        if reset_midway_step is not None and k == reset_midway_step:
            pid1.reset(cfg.theta1_init_deg)
            pid2.reset(cfg.theta2_init_deg)

        t1 = pid1.step(ref1)
        t2 = pid2.step(ref2)
        pose = forward_kinematics(t1.theta_new, t2.theta_new, cfg.geometry)
        steps.append(
            SimulationStep(
                step=k,
                time_s=round(k * cfg.dt, 10),
                theta1_ref_deg=ref1,
                theta2_ref_deg=ref2,
                theta1_actual_deg=t1.theta_new,
                theta2_actual_deg=t2.theta_new,
                error1_deg=t1.error,
                error2_deg=t2.error,
                u1=t1.u,
                u2=t2.u,
                x1=pose.x1,
                y1=pose.y1,
                x=pose.x,
                y=pose.y,
            )
        )

    return _finalize_result(cfg, steps)


def simulate_waypoints(
    waypoints: Sequence[WaypointSegment],
    config: SimulationConfig | None = None,
) -> SimulationResult:
    """Run closed-loop simulation across sequential waypoint segments."""
    if not waypoints:
        raise ValueError("waypoints must contain at least one WaypointSegment")

    cfg = config if config is not None else SimulationConfig()
    pid1 = DiscretePIDController(cfg.gains1, cfg.dt, cfg.theta1_init_deg)
    pid2 = DiscretePIDController(cfg.gains2, cfg.effective_dt2, cfg.theta2_init_deg)

    steps: list[SimulationStep] = []
    global_step = 0

    for seg in waypoints:
        n_seg = int(round(seg.duration_s / cfg.dt))
        if n_seg < 1:
            raise ValueError(
                f"Segment duration {seg.duration_s} must be >= dt ({cfg.dt})"
            )
        if seg.reset_before:
            pid1.reset(cfg.theta1_init_deg)
            pid2.reset(cfg.theta2_init_deg)

        for _ in range(n_seg):
            t1 = pid1.step(seg.theta1_ref_deg)
            t2 = pid2.step(seg.theta2_ref_deg)
            pose = forward_kinematics(t1.theta_new, t2.theta_new, cfg.geometry)
            steps.append(
                SimulationStep(
                    step=global_step,
                    time_s=round(global_step * cfg.dt, 10),
                    theta1_ref_deg=seg.theta1_ref_deg,
                    theta2_ref_deg=seg.theta2_ref_deg,
                    theta1_actual_deg=t1.theta_new,
                    theta2_actual_deg=t2.theta_new,
                    error1_deg=t1.error,
                    error2_deg=t2.error,
                    u1=t1.u,
                    u2=t2.u,
                    x1=pose.x1,
                    y1=pose.y1,
                    x=pose.x,
                    y=pose.y,
                )
            )
            global_step += 1

    return _finalize_result(cfg, steps)
