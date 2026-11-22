"""Discrete-time PID controller matching the LabVIEW Shift-Register recurrence."""

from __future__ import annotations

import math
from dataclasses import dataclass


def _validate_finite(name: str, value: float) -> float:
    val = float(value)
    if not math.isfinite(val):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return val


@dataclass(frozen=True)
class PIDGains:
    """Discrete PID gains for a single joint."""

    kp: float = 15.0
    ki: float = 0.1
    kd: float = 0.2

    def __post_init__(self) -> None:
        object.__setattr__(self, "kp", _validate_finite("kp", self.kp))
        object.__setattr__(self, "ki", _validate_finite("ki", self.ki))
        object.__setattr__(self, "kd", _validate_finite("kd", self.kd))


@dataclass
class JointPIDState:
    """Shift-register state persisted across While Loop iterations."""

    theta_deg: float = 0.0
    error_prev_deg: float = 0.0
    integral: float = 0.0
    step_index: int = 0


@dataclass(frozen=True)
class PIDStepTerms:
    """Intermediate and output signals computed in one discrete PID iteration."""

    error_pre_update: float
    p_term: float
    i_state: float
    i_term: float
    d_term: float
    u: float
    theta_new: float
    error: float

    @property
    def error_in(self) -> float:
        return self.error_pre_update

    @property
    def error_out(self) -> float:
        return self.error


class DiscretePIDController:
    """Single-joint discrete PID controller with Euler integral plant update.

    Exact LabVIEW Shift-Register recurrence per iteration:
        1. e = theta_ref - theta_prev
        2. P = Kp * e
        3. I_new = I_prev + e * dt
        4. I_out = Ki * I_new
        5. D = Kd * (e - e_prev) / dt
        6. u = P + I_out + D
        7. theta_new = theta_prev + u * dt
        8. error = theta_ref - theta_new
    """

    def __init__(
        self,
        gains: PIDGains | None = None,
        dt: float = 0.01,
        initial_theta_deg: float = 0.0,
    ) -> None:
        self.gains = gains if gains is not None else PIDGains()
        dt_val = _validate_finite("dt", dt)
        if dt_val <= 0.0:
            raise ValueError(f"dt must be positive, got {dt!r}")
        self.dt = dt_val
        init_theta = _validate_finite("initial_theta_deg", initial_theta_deg)
        self.state = JointPIDState(theta_deg=init_theta)

    def reset(self, initial_theta_deg: float = 0.0) -> None:
        """Reset shift-register state (theta_prev, e_prev, integral, step_index)."""
        init_theta = _validate_finite("initial_theta_deg", initial_theta_deg)
        self.state = JointPIDState(
            theta_deg=init_theta,
            error_prev_deg=0.0,
            integral=0.0,
            step_index=0,
        )

    def step(self, theta_ref_deg: float) -> PIDStepTerms:
        """Advance the discrete PID loop and plant by one sample dt."""
        ref = _validate_finite("theta_ref_deg", theta_ref_deg)
        dt = self.dt
        gains = self.gains
        theta_prev = self.state.theta_deg
        e_prev = self.state.error_prev_deg
        i_prev = self.state.integral

        e = ref - theta_prev
        p_term = gains.kp * e
        i_new = i_prev + e * dt
        i_out = gains.ki * i_new
        d_term = gains.kd * (e - e_prev) / dt
        u = p_term + i_out + d_term
        theta_new = theta_prev + u * dt
        error = ref - theta_new

        self.state.theta_deg = theta_new
        self.state.error_prev_deg = e
        self.state.integral = i_new
        self.state.step_index += 1

        return PIDStepTerms(
            error_pre_update=e,
            p_term=p_term,
            i_state=i_new,
            i_term=i_out,
            d_term=d_term,
            u=u,
            theta_new=theta_new,
            error=error,
        )
