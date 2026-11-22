"""Command-line interface for the 2-DOF planar arm discrete PID simulator."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

from planar_arm_pid.datasets import (
    default_baseline_config,
    default_multi_waypoint_segments,
    load_trajectory_csv,
    result_from_steps,
    verify_datasets,
    write_trajectory_csv,
)
from planar_arm_pid.kinematics import ArmGeometry
from planar_arm_pid.pid import PIDGains
from planar_arm_pid.plotting import generate_report_figures, launch_interactive_ui
from planar_arm_pid.simulator import SimulationConfig, simulate_step, simulate_waypoints


def _emit(payload: dict[str, Any], as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, indent=2))
        return
    for key, val in payload.items():
        print(f"{key}: {val}")


def _add_json_flag(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--json",
        action="store_true",
        default=False,
        help="Emit machine-readable JSON output.",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="planar-arm-pid",
        description="2-DOF Planar Arm Discrete PID & Forward Kinematics Simulator",
    )
    _add_json_flag(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    p_bench = sub.add_parser(
        "benchmark",
        help="Run the calibrated 30 deg LabVIEW FINAL2.vi baseline benchmark.",
    )
    p_bench.add_argument("--duration", type=float, default=15.0)
    p_bench.add_argument("--output-csv", type=Path, default=None)
    p_bench.add_argument("--out-plot-dir", type=Path, default=None)
    _add_json_flag(p_bench)

    p_sim = sub.add_parser(
        "simulate",
        help="Run a custom dual-joint step response simulation.",
    )
    p_sim.add_argument("--theta1", type=float, default=30.0)
    p_sim.add_argument("--theta2", type=float, default=30.0)
    p_sim.add_argument("--l1", type=float, default=5.0)
    p_sim.add_argument("--l2", type=float, default=5.0)
    p_sim.add_argument("--kp", "--kp1", dest="kp1", type=float, default=15.0)
    p_sim.add_argument("--ki", "--ki1", dest="ki1", type=float, default=0.1)
    p_sim.add_argument("--kd", "--kd1", dest="kd1", type=float, default=0.2)
    p_sim.add_argument("--kp2", type=float, default=None)
    p_sim.add_argument("--ki2", type=float, default=None)
    p_sim.add_argument("--kd2", type=float, default=None)
    p_sim.add_argument("--dt", type=float, default=0.01)
    p_sim.add_argument("--dt2", type=float, default=None)
    p_sim.add_argument("--duration", type=float, default=15.0)
    p_sim.add_argument("--out-csv", type=Path, default=None)
    p_sim.add_argument("--out-plot-dir", type=Path, default=None)
    _add_json_flag(p_sim)

    p_wp = sub.add_parser(
        "waypoints",
        help="Run the 4-phase multi-waypoint trajectory simulation.",
    )
    p_wp.add_argument("--l1", type=float, default=5.0)
    p_wp.add_argument("--l2", type=float, default=5.0)
    p_wp.add_argument("--kp", type=float, default=15.0)
    p_wp.add_argument("--ki", type=float, default=0.1)
    p_wp.add_argument("--kd", type=float, default=0.2)
    p_wp.add_argument("--dt", type=float, default=0.01)
    p_wp.add_argument("--out-csv", type=Path, default=None)
    p_wp.add_argument("--out-plot-dir", type=Path, default=None)
    _add_json_flag(p_wp)

    p_plot = sub.add_parser(
        "plot",
        help="Render joint-response and planar-arm PNG plots from an existing CSV.",
    )
    p_plot.add_argument("--input-csv", type=Path, required=True)
    p_plot.add_argument("--out-dir", type=Path, required=True)
    p_plot.add_argument("--prefix", type=str, default="arm")
    p_plot.add_argument("--l1", type=float, default=5.0)
    p_plot.add_argument("--l2", type=float, default=5.0)
    _add_json_flag(p_plot)

    p_ver = sub.add_parser(
        "verify-datasets",
        help="Re-simulate and verify committed CSV datasets in data/.",
    )
    p_ver.add_argument("--data-dir", type=Path, default=Path("data"))
    p_ver.add_argument("--atol", type=float, default=1e-5)
    _add_json_flag(p_ver)

    p_ui = sub.add_parser(
        "interactive",
        help="Launch optional Tkinter interactive PID tuning window.",
    )
    _add_json_flag(p_ui)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    as_json = bool(getattr(args, "json", False))

    if args.command == "benchmark":
        cfg = default_baseline_config()
        res = simulate_step(30.0, 30.0, duration_s=args.duration, config=cfg)
        payload: dict[str, Any] = res.to_dict()
        idx_1230 = min(1230, len(res.steps) - 1)
        s_1230 = res.steps[idx_1230]
        payload["report_checkpoint"] = {
            "step": s_1230.step,
            "time_s": s_1230.time_s,
            "theta1_actual_deg": round(s_1230.theta1_actual_deg, 6),
            "theta2_actual_deg": round(s_1230.theta2_actual_deg, 6),
            "error1_deg": round(s_1230.error1_deg, 6),
            "error2_deg": round(s_1230.error2_deg, 6),
            "x": round(s_1230.x, 6),
            "y": round(s_1230.y, 6),
        }
        if args.output_csv is not None:
            csv_path = write_trajectory_csv(res, args.output_csv)
            payload["output_csv"] = str(csv_path)
        if args.out_plot_dir is not None:
            figs = generate_report_figures(res, args.out_plot_dir, prefix="baseline")
            payload["plots"] = {k: str(v) for k, v in figs.items()}
        _emit(payload, as_json)
        return 0

    if args.command == "simulate":
        gains1 = PIDGains(kp=args.kp1, ki=args.ki1, kd=args.kd1)
        gains2 = PIDGains(
            kp=args.kp1 if args.kp2 is None else args.kp2,
            ki=args.ki1 if args.ki2 is None else args.ki2,
            kd=args.kd1 if args.kd2 is None else args.kd2,
        )
        cfg = SimulationConfig(
            geometry=ArmGeometry(l1=args.l1, l2=args.l2),
            gains1=gains1,
            gains2=gains2,
            dt=args.dt,
            dt2=args.dt2,
        )
        res = simulate_step(
            theta1_ref_deg=args.theta1,
            theta2_ref_deg=args.theta2,
            duration_s=args.duration,
            config=cfg,
        )
        payload = res.to_dict()
        if args.out_csv is not None:
            payload["output_csv"] = str(write_trajectory_csv(res, args.out_csv))
        if args.out_plot_dir is not None:
            figs = generate_report_figures(res, args.out_plot_dir, prefix="simulate")
            payload["plots"] = {k: str(v) for k, v in figs.items()}
        _emit(payload, as_json)
        return 0

    if args.command == "waypoints":
        gains = PIDGains(kp=args.kp, ki=args.ki, kd=args.kd)
        cfg = SimulationConfig(
            geometry=ArmGeometry(l1=args.l1, l2=args.l2),
            gains1=gains,
            gains2=gains,
            dt=args.dt,
        )
        res = simulate_waypoints(default_multi_waypoint_segments(), config=cfg)
        payload = res.to_dict()
        if args.out_csv is not None:
            payload["output_csv"] = str(write_trajectory_csv(res, args.out_csv))
        if args.out_plot_dir is not None:
            figs = generate_report_figures(res, args.out_plot_dir, prefix="waypoints")
            payload["plots"] = {k: str(v) for k, v in figs.items()}
        _emit(payload, as_json)
        return 0

    if args.command == "plot":
        steps = load_trajectory_csv(args.input_csv)
        res = result_from_steps(steps, ArmGeometry(l1=args.l1, l2=args.l2))
        figs = generate_report_figures(res, args.out_dir, prefix=args.prefix)
        payload = {
            "input_csv": str(args.input_csv),
            "rows": len(steps),
            "plots": {k: str(v) for k, v in figs.items()},
        }
        _emit(payload, as_json)
        return 0

    if args.command == "verify-datasets":
        report = verify_datasets(data_dir=args.data_dir, atol=args.atol)
        _emit(report, as_json)
        return 0 if report["verified"] else 1

    if args.command == "interactive":
        rc = launch_interactive_ui()
        if as_json:
            _emit({"status": "ok" if rc == 0 else "headless_unavailable", "exit_code": rc}, True)
        return rc

    parser.error(f"Unknown command: {args.command}")
    return 2
