"""Unit tests for the discrete-time PID controller (LabVIEW Shift-Register recurrence)."""

from __future__ import annotations

import math

import pytest

from planar_arm_pid.pid import DiscretePIDController, PIDGains


def test_first_step_exact_labview_recurrence() -> None:
    ctrl = DiscretePIDController(PIDGains(kp=15.0, ki=0.1, kd=0.2), dt=0.01)
    terms = ctrl.step(30.0)

    assert terms.error_pre_update == pytest.approx(30.0, abs=1e-12)
    assert terms.p_term == pytest.approx(450.0, abs=1e-12)
    assert terms.i_state == pytest.approx(0.3, abs=1e-12)
    assert terms.i_term == pytest.approx(0.03, abs=1e-12)
    assert terms.d_term == pytest.approx(600.0, abs=1e-12)
    assert terms.u == pytest.approx(1050.03, abs=1e-12)
    assert terms.theta_new == pytest.approx(10.5003, abs=1e-12)
    assert terms.error == pytest.approx(19.4997, abs=1e-12)

    assert ctrl.state.step_index == 1
    assert ctrl.state.theta_deg == pytest.approx(10.5003, abs=1e-12)
    assert ctrl.state.error_prev_deg == pytest.approx(30.0, abs=1e-12)
    assert ctrl.state.integral == pytest.approx(0.3, abs=1e-12)


def test_reset_restores_shift_register_state_and_replays_identically() -> None:
    ctrl = DiscretePIDController(PIDGains(kp=15.0, ki=0.1, kd=0.2), dt=0.01)
    first_pass = [ctrl.step(30.0) for _ in range(50)]

    assert ctrl.state.step_index == 50
    assert ctrl.state.integral > 1.0

    ctrl.reset()
    assert ctrl.state.theta_deg == 0.0
    assert ctrl.state.error_prev_deg == 0.0
    assert ctrl.state.integral == 0.0
    assert ctrl.state.step_index == 0

    second_pass = [ctrl.step(30.0) for _ in range(50)]
    for t1, t2 in zip(first_pass, second_pass):
        assert t1.theta_new == pytest.approx(t2.theta_new, abs=1e-15)
        assert t1.error == pytest.approx(t2.error, abs=1e-15)


def test_zero_gains_produce_zero_control_output() -> None:
    ctrl = DiscretePIDController(PIDGains(kp=0.0, ki=0.0, kd=0.0), dt=0.01)
    for _ in range(25):
        terms = ctrl.step(45.0)
        assert terms.u == 0.0
        assert terms.theta_new == 0.0
        assert terms.error == 45.0


def test_negative_target_tracks_symmetrically() -> None:
    ctrl = DiscretePIDController(PIDGains(kp=15.0, ki=0.1, kd=0.2), dt=0.01)
    for _ in range(1000):
        terms = ctrl.step(-45.0)
    assert terms.theta_new == pytest.approx(-45.018736, abs=1e-4)
    assert terms.error == pytest.approx(0.018736, abs=1e-4)


@pytest.mark.parametrize("bad_dt", [0.0, -0.01, math.nan, math.inf, -math.inf])
def test_invalid_dt_raises_value_error(bad_dt: float) -> None:
    with pytest.raises(ValueError, match="dt"):
        DiscretePIDController(PIDGains(), dt=bad_dt)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"kp": math.nan},
        {"ki": math.inf},
        {"kd": -math.inf},
    ],
)
def test_non_finite_gains_raise_value_error(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError, match="finite"):
        PIDGains(**kwargs)


def test_non_finite_step_reference_raises_value_error() -> None:
    ctrl = DiscretePIDController()
    with pytest.raises(ValueError, match="theta_ref_deg"):
        ctrl.step(float("nan"))
