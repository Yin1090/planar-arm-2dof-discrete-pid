"""Integration tests for closed-loop simulation and deterministic CSV datasets."""

from __future__ import annotations

from pathlib import Path

import pytest

from planar_arm_pid.datasets import (
    CSV_COLUMNS,
    default_baseline_config,
    default_multi_waypoint_segments,
    load_trajectory_csv,
    verify_datasets,
    write_trajectory_csv,
)
from planar_arm_pid.kinematics import ArmGeometry
from planar_arm_pid.pid import PIDGains
from planar_arm_pid.simulator import (
    SimulationConfig,
    WaypointSegment,
    simulate_step,
    simulate_waypoints,
)

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_baseline_benchmark_matches_labview_report_values() -> None:
    res = simulate_step(30.0, 30.0, duration_s=15.0, config=default_baseline_config())
    assert len(res.steps) == 1501

    s0 = res.steps[0]
    assert s0.step == 0
    assert s0.time_s == 0.0
    assert s0.theta1_actual_deg == pytest.approx(10.5003, abs=1e-9)
    assert s0.error1_deg == pytest.approx(19.4997, abs=1e-9)

    s_peak = res.steps[115]
    assert s_peak.theta1_actual_deg == pytest.approx(30.013243, abs=1e-5)

    s_1230 = res.steps[1230]
    assert s_1230.time_s == pytest.approx(12.30, abs=1e-9)
    assert s_1230.theta1_actual_deg == pytest.approx(30.012300, abs=1e-5)
    assert s_1230.theta2_actual_deg == pytest.approx(30.012300, abs=1e-5)
    assert s_1230.error1_deg == pytest.approx(-0.012300, abs=1e-5)
    assert s_1230.error2_deg == pytest.approx(-0.012300, abs=1e-5)
    assert s_1230.x == pytest.approx(6.827731, abs=1e-5)
    assert s_1230.y == pytest.approx(6.832130, abs=1e-5)
    assert s_1230.x == pytest.approx(6.83, abs=5e-3)
    assert s_1230.y == pytest.approx(6.83, abs=5e-3)

    assert res.joint1_metrics.settling_time_s is not None
    assert res.joint1_metrics.settling_time_s < 1.0
    assert res.joint1_metrics.overshoot_pct < 0.1


def test_long_horizon_asymptotic_convergence() -> None:
    res = simulate_step(30.0, 30.0, duration_s=750.0, config=default_baseline_config())
    last = res.steps[-1]
    assert last.theta1_actual_deg == pytest.approx(30.0, abs=1.5e-4)
    assert last.theta2_actual_deg == pytest.approx(30.0, abs=1.5e-4)
    assert last.x == pytest.approx(6.830127, abs=1e-4)
    assert last.y == pytest.approx(6.830127, abs=1e-4)


def test_independent_joint_gains_and_reset_midway() -> None:
    cfg = SimulationConfig(
        geometry=ArmGeometry(5.0, 5.0),
        gains1=PIDGains(15.0, 0.1, 0.2),
        gains2=PIDGains(8.0, 0.05, 0.1),
        dt=0.01,
    )
    res = simulate_step(30.0, 30.0, duration_s=6.0, config=cfg, reset_midway_step=300)
    assert res.steps[299].theta1_actual_deg != pytest.approx(res.steps[299].theta2_actual_deg, abs=1e-3)
    assert res.steps[300].theta1_actual_deg == pytest.approx(res.steps[0].theta1_actual_deg, abs=1e-12)
    assert res.steps[300].theta2_actual_deg == pytest.approx(res.steps[0].theta2_actual_deg, abs=1e-12)


def test_multi_waypoint_tracking_and_reset_before() -> None:
    res = simulate_waypoints(default_multi_waypoint_segments(), default_baseline_config())
    assert len(res.steps) == 1200

    assert res.steps[299].theta1_actual_deg == pytest.approx(30.0, abs=0.05)
    assert res.steps[599].theta1_actual_deg == pytest.approx(60.0, abs=0.05)
    assert res.steps[599].theta2_actual_deg == pytest.approx(-30.0, abs=0.05)
    assert res.steps[899].theta1_actual_deg == pytest.approx(90.0, abs=0.05)
    assert res.steps[899].theta2_actual_deg == pytest.approx(-45.0, abs=0.05)
    assert res.steps[1199].theta1_actual_deg == pytest.approx(0.0, abs=0.05)
    assert res.steps[1199].theta2_actual_deg == pytest.approx(0.0, abs=0.05)

    segs_reset = [
        WaypointSegment(30.0, 30.0, 1.0),
        WaypointSegment(30.0, 30.0, 1.0, reset_before=True),
    ]
    res_reset = simulate_waypoints(segs_reset, default_baseline_config())
    assert res_reset.steps[100].theta1_actual_deg == pytest.approx(
        res_reset.steps[0].theta1_actual_deg, abs=1e-12
    )


def test_committed_datasets_match_fresh_simulation_and_roundtrip(tmp_path: Path) -> None:
    data_dir = REPO_ROOT / "data"
    baseline_csv = data_dir / "baseline_step_30deg.csv"
    waypoint_csv = data_dir / "multi_waypoint_trajectory.csv"

    assert baseline_csv.is_file()
    assert waypoint_csv.is_file()

    baseline_rows = load_trajectory_csv(baseline_csv)
    waypoint_rows = load_trajectory_csv(waypoint_csv)
    assert len(baseline_rows) == 1501
    assert len(waypoint_rows) == 1200

    header_line = baseline_csv.read_text(encoding="utf-8").splitlines()[0].split(",")
    assert header_line == CSV_COLUMNS

    roundtrip_path = write_trajectory_csv(baseline_rows, tmp_path / "roundtrip.csv")
    reloaded = load_trajectory_csv(roundtrip_path)
    assert len(reloaded) == 1501
    assert reloaded[1230].theta1_actual_deg == pytest.approx(
        baseline_rows[1230].theta1_actual_deg, abs=1e-12
    )

    verification = verify_datasets(data_dir=data_dir, atol=1e-5)
    assert verification["verified"] is True
    assert verification["datasets"]["baseline_step_30deg.csv"]["matched"] is True
    assert verification["datasets"]["multi_waypoint_trajectory.csv"]["matched"] is True
