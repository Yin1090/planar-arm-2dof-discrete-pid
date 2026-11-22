"""Deterministic CSV dataset generation, loading, and verification."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Sequence

from planar_arm_pid.kinematics import ArmGeometry, forward_kinematics
from planar_arm_pid.pid import PIDGains
from planar_arm_pid.simulator import (
    SimulationConfig,
    SimulationResult,
    SimulationStep,
    WaypointSegment,
    compute_joint_metrics,
    simulate_step,
    simulate_waypoints,
)

CSV_COLUMNS: list[str] = [
    "step",
    "time_s",
    "theta1_ref_deg",
    "theta2_ref_deg",
    "theta1_actual_deg",
    "theta2_actual_deg",
    "error1_deg",
    "error2_deg",
    "u1",
    "u2",
    "x1",
    "y1",
    "x",
    "y",
]

BASELINE_FILENAME = "baseline_step_30deg.csv"
MULTI_WAYPOINT_FILENAME = "multi_waypoint_trajectory.csv"


def default_baseline_config() -> SimulationConfig:
    """Return the calibrated LabVIEW FINAL2.vi baseline configuration."""
    return SimulationConfig(
        geometry=ArmGeometry(l1=5.0, l2=5.0),
        gains1=PIDGains(kp=15.0, ki=0.1, kd=0.2),
        gains2=PIDGains(kp=15.0, ki=0.1, kd=0.2),
        dt=0.01,
    )


def default_multi_waypoint_segments() -> list[WaypointSegment]:
    """Return the 4-phase 12.0 s multi-waypoint reference sequence (1200 rows at dt=0.01)."""
    return [
        WaypointSegment(theta1_ref_deg=30.0, theta2_ref_deg=30.0, duration_s=3.0),
        WaypointSegment(theta1_ref_deg=60.0, theta2_ref_deg=-30.0, duration_s=3.0),
        WaypointSegment(theta1_ref_deg=90.0, theta2_ref_deg=-45.0, duration_s=3.0),
        WaypointSegment(theta1_ref_deg=0.0, theta2_ref_deg=0.0, duration_s=3.0),
    ]


def write_trajectory_csv(
    source: SimulationResult | Sequence[SimulationStep],
    path: Path,
) -> Path:
    """Write simulation trajectory steps to UTF-8 CSV with high numeric precision."""
    steps = source.steps if isinstance(source, SimulationResult) else list(source)
    if not steps:
        raise ValueError("Cannot write an empty trajectory to CSV")

    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_COLUMNS)
        for s in steps:
            writer.writerow(
                [
                    s.step,
                    f"{s.time_s:.4f}",
                    f"{s.theta1_ref_deg:.8f}",
                    f"{s.theta2_ref_deg:.8f}",
                    f"{s.theta1_actual_deg:.8f}",
                    f"{s.theta2_actual_deg:.8f}",
                    f"{s.error1_deg:.8f}",
                    f"{s.error2_deg:.8f}",
                    f"{s.u1:.8f}",
                    f"{s.u2:.8f}",
                    f"{s.x1:.8f}",
                    f"{s.y1:.8f}",
                    f"{s.x:.8f}",
                    f"{s.y:.8f}",
                ]
            )
    return out_path


def load_trajectory_csv(path: Path) -> list[SimulationStep]:
    """Load and validate a trajectory CSV into SimulationStep objects."""
    in_path = Path(path)
    if not in_path.is_file():
        raise FileNotFoundError(f"Trajectory CSV not found: {in_path}")

    with in_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != CSV_COLUMNS:
            raise ValueError(
                f"Invalid CSV columns in {in_path}: expected {CSV_COLUMNS}, got {reader.fieldnames}"
            )
        steps: list[SimulationStep] = []
        for row in reader:
            steps.append(
                SimulationStep(
                    step=int(row["step"]),
                    time_s=float(row["time_s"]),
                    theta1_ref_deg=float(row["theta1_ref_deg"]),
                    theta2_ref_deg=float(row["theta2_ref_deg"]),
                    theta1_actual_deg=float(row["theta1_actual_deg"]),
                    theta2_actual_deg=float(row["theta2_actual_deg"]),
                    error1_deg=float(row["error1_deg"]),
                    error2_deg=float(row["error2_deg"]),
                    u1=float(row["u1"]),
                    u2=float(row["u2"]),
                    x1=float(row["x1"]),
                    y1=float(row["y1"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                )
            )
    if not steps:
        raise ValueError(f"Trajectory CSV is empty: {in_path}")
    return steps


def result_from_steps(
    steps: Sequence[SimulationStep],
    geometry: ArmGeometry | None = None,
) -> SimulationResult:
    """Reconstruct a SimulationResult wrapper from loaded CSV steps."""
    if not steps:
        raise ValueError("Cannot build SimulationResult from empty steps")
    geom = geometry if geometry is not None else ArmGeometry()
    dt = steps[1].time_s - steps[0].time_s if len(steps) > 1 else 0.01
    if dt <= 0.0:
        dt = 0.01
    cfg = SimulationConfig(geometry=geom, dt=dt)

    times = [s.time_s for s in steps]
    r1 = [s.theta1_ref_deg for s in steps]
    r2 = [s.theta2_ref_deg for s in steps]
    a1 = [s.theta1_actual_deg for s in steps]
    a2 = [s.theta2_actual_deg for s in steps]
    e1 = [s.error1_deg for s in steps]
    e2 = [s.error2_deg for s in steps]

    m1 = compute_joint_metrics(times, r1, a1, e1, dt, 0.0)
    m2 = compute_joint_metrics(times, r2, a2, e2, dt, 0.0)
    last = steps[-1]
    final_pose = forward_kinematics(last.theta1_actual_deg, last.theta2_actual_deg, geom)
    return SimulationResult(
        config=cfg,
        steps=list(steps),
        joint1_metrics=m1,
        joint2_metrics=m2,
        final_pose=final_pose,
    )


def generate_baseline_dataset(out_path: Path = Path("data") / BASELINE_FILENAME) -> Path:
    """Generate the 1501-row (15.0 s) baseline 30 deg step response CSV."""
    result = simulate_step(
        theta1_ref_deg=30.0,
        theta2_ref_deg=30.0,
        duration_s=15.0,
        config=default_baseline_config(),
    )
    return write_trajectory_csv(result, out_path)


def generate_multi_waypoint_dataset(
    out_path: Path = Path("data") / MULTI_WAYPOINT_FILENAME,
) -> Path:
    """Generate the 1200-row (12.0 s) 4-phase multi-waypoint trajectory CSV."""
    result = simulate_waypoints(
        default_multi_waypoint_segments(),
        config=default_baseline_config(),
    )
    return write_trajectory_csv(result, out_path)


def generate_all_datasets(data_dir: Path = Path("data")) -> dict[str, Path]:
    """Generate all committed reference CSV datasets in data_dir."""
    target_dir = Path(data_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    baseline_path = generate_baseline_dataset(target_dir / BASELINE_FILENAME)
    waypoint_path = generate_multi_waypoint_dataset(target_dir / MULTI_WAYPOINT_FILENAME)
    return {
        "baseline": baseline_path,
        "multi_waypoint": waypoint_path,
    }


def _compare_steps(
    expected: Sequence[SimulationStep],
    actual: Sequence[SimulationStep],
    atol: float,
) -> dict[str, Any]:
    if len(expected) != len(actual):
        return {
            "matched": False,
            "expected_rows": len(expected),
            "actual_rows": len(actual),
            "max_abs_diff": None,
            "reason": f"Row count mismatch: expected {len(expected)}, got {len(actual)}",
        }

    float_fields = CSV_COLUMNS[1:]
    max_diff = 0.0
    worst_field = ""
    worst_step = -1

    for exp_row, act_row in zip(expected, actual):
        if exp_row.step != act_row.step:
            return {
                "matched": False,
                "expected_rows": len(expected),
                "actual_rows": len(actual),
                "max_abs_diff": None,
                "reason": f"Step index mismatch: expected {exp_row.step}, got {act_row.step}",
            }
        for field in float_fields:
            diff = abs(getattr(exp_row, field) - getattr(act_row, field))
            if diff > max_diff:
                max_diff = diff
                worst_field = field
                worst_step = exp_row.step

    matched = max_diff <= atol
    info: dict[str, Any] = {
        "matched": matched,
        "rows": len(actual),
        "max_abs_diff": max_diff,
        "worst_field": worst_field,
        "worst_step": worst_step,
    }
    if not matched:
        info["reason"] = (
            f"Max diff {max_diff:.3e} on {worst_field} at step {worst_step} exceeds atol={atol}"
        )
    return info


def verify_datasets(
    data_dir: Path = Path("data"),
    atol: float = 1e-5,
) -> dict[str, Any]:
    """Re-run simulations and verify committed CSVs match within tolerance."""
    target_dir = Path(data_dir)
    expected_baseline = simulate_step(
        theta1_ref_deg=30.0,
        theta2_ref_deg=30.0,
        duration_s=15.0,
        config=default_baseline_config(),
    ).steps
    expected_waypoints = simulate_waypoints(
        default_multi_waypoint_segments(),
        config=default_baseline_config(),
    ).steps

    specs = {
        BASELINE_FILENAME: expected_baseline,
        MULTI_WAYPOINT_FILENAME: expected_waypoints,
    }

    results: dict[str, Any] = {}
    all_ok = True
    for filename, exp_steps in specs.items():
        csv_path = target_dir / filename
        if not csv_path.is_file():
            results[filename] = {
                "matched": False,
                "path": str(csv_path),
                "reason": "Missing dataset file",
            }
            all_ok = False
            continue
        try:
            loaded = load_trajectory_csv(csv_path)
            comparison = _compare_steps(exp_steps, loaded, atol=atol)
            comparison["path"] = str(csv_path)
            results[filename] = comparison
            if not comparison["matched"]:
                all_ok = False
        except Exception as exc:  # pragma: no cover - defensive error capture
            results[filename] = {
                "matched": False,
                "path": str(csv_path),
                "reason": str(exc),
            }
            all_ok = False

    return {
        "verified": all_ok,
        "atol": atol,
        "datasets": results,
    }
