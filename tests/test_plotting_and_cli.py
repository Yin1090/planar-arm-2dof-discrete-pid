"""Tests for headless Matplotlib plotting and CLI subcommands."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from planar_arm_pid.datasets import default_baseline_config, default_multi_waypoint_segments
from planar_arm_pid.plotting import (
    generate_report_figures,
    plot_arm_trajectory,
    plot_joint_response,
)
from planar_arm_pid.simulator import simulate_step, simulate_waypoints

REPO_ROOT = Path(__file__).resolve().parents[1]
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _run_cli(args: list[str], cwd: Path = REPO_ROOT) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    return subprocess.run(
        [sys.executable, "-m", "planar_arm_pid", *args],
        cwd=str(cwd),
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def test_headless_plot_generation_writes_valid_pngs(tmp_path: Path) -> None:
    res = simulate_waypoints(default_multi_waypoint_segments(), default_baseline_config())
    resp_path = plot_joint_response(res, tmp_path / "resp.png")
    traj_path = plot_arm_trajectory(res, tmp_path / "traj.png")
    bundle = generate_report_figures(res, tmp_path / "bundle", prefix="arm")

    for path in (resp_path, traj_path, bundle["response"], bundle["trajectory"]):
        data = path.read_bytes()
        assert data.startswith(PNG_MAGIC)
        assert len(data) > 5000


def test_cli_benchmark_and_verify_datasets_json() -> None:
    proc_bench = _run_cli(["benchmark", "--json"])
    assert proc_bench.returncode == 0, proc_bench.stderr
    bench_data = json.loads(proc_bench.stdout)
    assert bench_data["num_steps"] == 1501
    assert abs(bench_data["report_checkpoint"]["theta1_actual_deg"] - 30.0123) < 1e-4
    assert abs(bench_data["report_checkpoint"]["x"] - 6.827731) < 1e-4

    proc_ver = _run_cli(["verify-datasets", "--data-dir", str(REPO_ROOT / "data"), "--json"])
    assert proc_ver.returncode == 0, proc_ver.stderr
    ver_data = json.loads(proc_ver.stdout)
    assert ver_data["verified"] is True


def test_cli_simulate_waypoints_plot_and_missing_dataset_failure(tmp_path: Path) -> None:
    sim_csv = tmp_path / "sim.csv"
    sim_plots = tmp_path / "sim_plots"
    proc_sim = _run_cli(
        [
            "simulate",
            "--theta1",
            "45",
            "--theta2",
            "-30",
            "--duration",
            "4.0",
            "--out-csv",
            str(sim_csv),
            "--out-plot-dir",
            str(sim_plots),
            "--json",
        ]
    )
    assert proc_sim.returncode == 0, proc_sim.stderr
    sim_payload = json.loads(proc_sim.stdout)
    assert sim_payload["num_steps"] == 401
    assert sim_csv.is_file()
    assert Path(sim_payload["plots"]["response"]).read_bytes().startswith(PNG_MAGIC)

    wp_csv = tmp_path / "wp.csv"
    proc_wp = _run_cli(["waypoints", "--out-csv", str(wp_csv), "--json"])
    assert proc_wp.returncode == 0, proc_wp.stderr
    assert json.loads(proc_wp.stdout)["num_steps"] == 1200
    assert wp_csv.is_file()

    plot_dir = tmp_path / "csv_plots"
    proc_plot = _run_cli(
        [
            "plot",
            "--input-csv",
            str(wp_csv),
            "--out-dir",
            str(plot_dir),
            "--prefix",
            "wp",
            "--json",
        ]
    )
    assert proc_plot.returncode == 0, proc_plot.stderr
    plot_payload = json.loads(proc_plot.stdout)
    assert plot_payload["rows"] == 1200
    assert Path(plot_payload["plots"]["trajectory"]).read_bytes().startswith(PNG_MAGIC)

    empty_dir = tmp_path / "empty_data"
    empty_dir.mkdir()
    proc_fail = _run_cli(["verify-datasets", "--data-dir", str(empty_dir), "--json"])
    assert proc_fail.returncode == 1
    assert json.loads(proc_fail.stdout)["verified"] is False
