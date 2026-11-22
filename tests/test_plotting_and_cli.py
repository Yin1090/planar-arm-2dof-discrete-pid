"""Tests for headless Matplotlib plotting and CLI subcommands."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import types
from pathlib import Path
from unittest.mock import patch

import planar_arm_pid.plotting as plotting
from planar_arm_pid.cli import main as cli_main
from planar_arm_pid.datasets import default_baseline_config, default_multi_waypoint_segments
from planar_arm_pid.kinematics import ArmGeometry
from planar_arm_pid.pid import PIDGains
from planar_arm_pid.plotting import (
    generate_report_figures,
    plot_arm_trajectory,
    plot_joint_response,
)
from planar_arm_pid.simulator import SimulationConfig, simulate_step, simulate_waypoints

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


def test_cli_pre_subcommand_json_and_clean_error_handling(tmp_path: Path) -> None:
    proc_pre = _run_cli(["--json", "benchmark"])
    assert proc_pre.returncode == 0, proc_pre.stderr
    pre_payload = json.loads(proc_pre.stdout)
    assert pre_payload["num_steps"] == 1501

    proc_bad_dt = _run_cli(["simulate", "--dt", "0"])
    assert proc_bad_dt.returncode == 1
    assert "Traceback" not in proc_bad_dt.stderr
    assert proc_bad_dt.stderr.strip().startswith("Error:")

    proc_bad_l1_json = _run_cli(["--json", "simulate", "--l1", "-5"])
    assert proc_bad_l1_json.returncode == 1
    assert "Traceback" not in proc_bad_l1_json.stderr
    err_payload = json.loads(proc_bad_l1_json.stdout)
    assert "error" in err_payload

    missing_csv = tmp_path / "does_not_exist.csv"
    proc_missing_csv = _run_cli(
        ["plot", "--input-csv", str(missing_csv), "--out-dir", str(tmp_path), "--json"]
    )
    assert proc_missing_csv.returncode == 1
    assert "Traceback" not in proc_missing_csv.stderr
    assert "error" in json.loads(proc_missing_csv.stdout)


def test_cli_verify_datasets_fails_on_nan_tampered_csv(tmp_path: Path) -> None:
    data_dir = REPO_ROOT / "data"
    nan_dir = tmp_path / "nan_data"
    nan_dir.mkdir()
    lines = (data_dir / "baseline_step_30deg.csv").read_text(encoding="utf-8").splitlines()
    fields = lines[101].split(",")
    fields[4] = "nan"
    lines[101] = ",".join(fields)
    (nan_dir / "baseline_step_30deg.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (nan_dir / "multi_waypoint_trajectory.csv").write_text(
        (data_dir / "multi_waypoint_trajectory.csv").read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    proc = _run_cli(["verify-datasets", "--data-dir", str(nan_dir), "--json"])
    assert proc.returncode == 1
    payload = json.loads(proc.stdout)
    assert payload["verified"] is False
    assert payload["datasets"]["baseline_step_30deg.csv"]["matched"] is False


def test_interactive_ui_wires_gains_initial_config_and_handles_invalid_inputs() -> None:
    class _FakeVar:
        def __init__(self, value: float | str = 0.0) -> None:
            self._value = value

        def get(self) -> float | str:
            return self._value

        def set(self, value: float | str) -> None:
            self._value = value

    class _FakeTclError(Exception):
        pass

    captured_buttons: list[tuple[str, object]] = []
    captured_vars: list[_FakeVar] = []
    status_vars: list[_FakeVar] = []
    captured_configs: list[SimulationConfig | None] = []
    draw_calls = 0

    class _FakeWidget:
        def pack(self, **kwargs: object) -> None:
            return None

        def grid(self, **kwargs: object) -> None:
            return None

    class _FakeRoot:
        def title(self, text: str) -> None:
            return None

        def mainloop(self) -> None:
            return None

    def _make_double_var(value: float = 0.0) -> _FakeVar:
        v = _FakeVar(value)
        captured_vars.append(v)
        return v

    def _make_string_var(value: str = "") -> _FakeVar:
        v = _FakeVar(value)
        status_vars.append(v)
        return v

    def _make_button(*args: object, **kwargs: object) -> _FakeWidget:
        captured_buttons.append((str(kwargs.get("text", "")), kwargs.get("command")))
        return _FakeWidget()

    class _FakeCanvas:
        def __init__(self, fig: object, master: object = None) -> None:
            self._widget = _FakeWidget()

        def get_tk_widget(self) -> _FakeWidget:
            return self._widget

        def draw(self) -> None:
            nonlocal draw_calls
            draw_calls += 1

    fake_tk = types.ModuleType("tkinter")
    fake_tk.LEFT = "left"
    fake_tk.RIGHT = "right"
    fake_tk.Y = "y"
    fake_tk.BOTH = "both"
    fake_tk.TclError = _FakeTclError
    fake_tk.Tk = _FakeRoot
    fake_tk.DoubleVar = _make_double_var
    fake_tk.StringVar = _make_string_var

    fake_ttk = types.ModuleType("tkinter.ttk")
    fake_ttk.Frame = lambda *a, **k: _FakeWidget()
    fake_ttk.Label = lambda *a, **k: _FakeWidget()
    fake_ttk.Entry = lambda *a, **k: _FakeWidget()
    fake_ttk.Button = _make_button
    fake_tk.ttk = fake_ttk

    fake_backend = types.ModuleType("matplotlib.backends.backend_tkagg")
    fake_backend.FigureCanvasTkAgg = _FakeCanvas

    real_simulate_step = plotting.simulate_step

    def _spy_simulate_step(*args: object, **kwargs: object) -> object:
        captured_configs.append(kwargs.get("config"))  # type: ignore[arg-type]
        return real_simulate_step(*args, **kwargs)  # type: ignore[arg-type]

    init_cfg = SimulationConfig(
        geometry=ArmGeometry(l1=7.0, l2=3.0),
        gains1=PIDGains(kp=22.0, ki=0.4, kd=0.35),
        gains2=PIDGains(kp=22.0, ki=0.4, kd=0.35),
        dt=0.02,
        dt2=0.02,
        theta1_init_deg=5.0,
        theta2_init_deg=-5.0,
    )

    with patch.dict(
        sys.modules,
        {
            "tkinter": fake_tk,
            "tkinter.ttk": fake_ttk,
            "matplotlib.backends.backend_tkagg": fake_backend,
        },
    ), patch.object(plotting, "simulate_step", side_effect=_spy_simulate_step):
        rc = plotting.launch_interactive_ui(initial_config=init_cfg)
        assert rc == 0
        assert len(captured_configs) == 1
        assert captured_configs[0] == init_cfg

        # vars_map order: theta1, theta2, kp, ki, kd, duration
        captured_vars[2].set(9.5)
        captured_vars[3].set(0.25)
        captured_vars[4].set(0.15)
        captured_vars[5].set(2.0)

        run_cb = captured_buttons[0][1]
        assert callable(run_cb)
        run_cb()

        assert len(captured_configs) == 2
        updated_cfg = captured_configs[1]
        assert updated_cfg is not None
        assert updated_cfg.geometry == ArmGeometry(l1=7.0, l2=3.0)
        assert updated_cfg.dt == 0.02
        assert updated_cfg.theta1_init_deg == 5.0
        assert updated_cfg.theta2_init_deg == -5.0
        assert updated_cfg.gains1 == PIDGains(kp=9.5, ki=0.25, kd=0.15)
        assert updated_cfg.gains2 == PIDGains(kp=9.5, ki=0.25, kd=0.15)
        assert draw_calls == 2

        # Invalid duration triggers ValueError -> caught gracefully, status label updated, no redraw
        captured_vars[5].set(-1.0)
        run_cb()
        assert draw_calls == 2
        assert str(status_vars[0].get()).startswith("Input error:")

        # Invalid Tk entry triggers TclError -> caught gracefully without crashing
        def _raise_tcl() -> float:
            raise _FakeTclError("bad double")

        captured_vars[2].get = _raise_tcl  # type: ignore[method-assign]
        run_cb()
        assert str(status_vars[0].get()).startswith("Input error:")
        assert draw_calls == 2

    # Also verify cli_main(['interactive', '--json']) handles headless TclError cleanly
    with patch.dict(sys.modules, {"tkinter": None}):
        assert cli_main(["interactive", "--json"]) == 1
