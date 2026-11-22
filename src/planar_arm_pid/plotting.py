"""Headless Matplotlib plotting and optional Tkinter interactive UI."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from planar_arm_pid.kinematics import forward_kinematics, link_polyline, workspace_bounds
from planar_arm_pid.simulator import SimulationConfig, SimulationResult, simulate_step


def plot_joint_response(
    result: SimulationResult,
    out_path: Path,
    title: str | None = None,
) -> Path:
    """Render a 2-panel joint angle & tracking error PNG via headless FigureCanvasAgg."""
    target = Path(out_path)
    target.parent.mkdir(parents=True, exist_ok=True)

    times = [s.time_s for s in result.steps]
    r1 = [s.theta1_ref_deg for s in result.steps]
    r2 = [s.theta2_ref_deg for s in result.steps]
    a1 = [s.theta1_actual_deg for s in result.steps]
    a2 = [s.theta2_actual_deg for s in result.steps]
    e1 = [s.error1_deg for s in result.steps]
    e2 = [s.error2_deg for s in result.steps]

    fig = Figure(figsize=(10.0, 6.4), dpi=160)
    ax_ang, ax_err = fig.subplots(2, 1, sharex=True)

    ax_ang.plot(times, r1, linestyle="--", linewidth=1.4, color="#64748b", label="Theta1_ref")
    ax_ang.plot(times, a1, linewidth=2.0, color="#0f766e", label="Theta1_actual")
    ax_ang.plot(times, r2, linestyle=":", linewidth=1.6, color="#94a3b8", label="Theta2_ref")
    ax_ang.plot(times, a2, linewidth=1.8, color="#d97706", alpha=0.9, label="Theta2_actual")
    ax_ang.set_ylabel("Joint Angle (deg)")
    ax_ang.set_title(
        title or "2-DOF Planar Arm Discrete PID Joint Response",
        fontsize=12,
        fontweight="bold",
    )
    ax_ang.grid(True, linestyle="--", alpha=0.35)
    ax_ang.legend(loc="best", ncol=2, frameon=True, fontsize=8.5)

    ax_err.plot(times, e1, linewidth=1.6, color="#0f766e", label="Error 1 (ref - actual)")
    ax_err.plot(times, e2, linewidth=1.5, color="#dc2626", alpha=0.85, label="Error 2 (ref - actual)")
    ax_err.axhline(0.0, color="#334155", linewidth=0.9, linestyle="-.")
    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Tracking Error (deg)")
    ax_err.grid(True, linestyle="--", alpha=0.35)
    ax_err.legend(loc="best", frameon=True, fontsize=8.5)

    fig.tight_layout()
    FigureCanvasAgg(fig).print_png(str(target))
    return target


def plot_arm_trajectory(
    result: SimulationResult,
    out_path: Path,
    snapshot_count: int = 10,
    title: str | None = None,
) -> Path:
    """Render 2-DOF planar workspace trajectory and arm link snapshots to PNG."""
    target = Path(out_path)
    target.parent.mkdir(parents=True, exist_ok=True)

    geom = result.config.geometry
    r_inner, r_outer = workspace_bounds(geom)
    xs = [s.x for s in result.steps]
    ys = [s.y for s in result.steps]

    fig = Figure(figsize=(7.6, 7.2), dpi=160)
    ax = fig.add_subplot(111)

    angles = np.linspace(0.0, 2.0 * math.pi, 360)
    ax.plot(
        r_outer * np.cos(angles),
        r_outer * np.sin(angles),
        linestyle="--",
        linewidth=1.1,
        color="#94a3b8",
        label=f"Workspace Outer (R={r_outer:.1f})",
    )
    if r_inner > 1e-6:
        ax.plot(
            r_inner * np.cos(angles),
            r_inner * np.sin(angles),
            linestyle=":",
            linewidth=1.0,
            color="#cbd5e1",
            label=f"Workspace Inner (r={r_inner:.1f})",
        )

    ax.plot(xs, ys, linewidth=1.8, color="#0284c7", label="End-Effector (X, Y) Path")

    n_steps = len(result.steps)
    count = max(2, min(snapshot_count, n_steps))
    indices = np.unique(np.linspace(0, n_steps - 1, count, dtype=int))
    for idx_pos, step_idx in enumerate(indices[:-1]):
        step = result.steps[int(step_idx)]
        pose = forward_kinematics(step.theta1_actual_deg, step.theta2_actual_deg, geom)
        px, py = link_polyline(pose)
        alpha = 0.16 + 0.35 * (idx_pos / max(1, len(indices) - 1))
        ax.plot(
            px,
            py,
            linewidth=1.4,
            color="#64748b",
            alpha=alpha,
            marker="o",
            markersize=3.0,
        )

    final_px, final_py = link_polyline(result.final_pose)
    disp_x = 0.0 if abs(result.final_pose.x) < 5e-3 else result.final_pose.x
    disp_y = 0.0 if abs(result.final_pose.y) < 5e-3 else result.final_pose.y
    ax.plot(
        final_px,
        final_py,
        linewidth=3.0,
        color="#0f766e",
        marker="o",
        markersize=6.5,
        label=f"Final Arm Pose (X={disp_x:.2f}, Y={disp_y:.2f})",
    )
    ax.scatter([0.0], [0.0], s=65, color="#1e293b", zorder=5, label="Base (0, 0)")

    pad = r_outer * 1.15
    ax.set_xlim(-pad, pad)
    ax.set_ylim(-pad, pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title(
        title or "2-DOF Planar Arm Workspace & End-Effector Trajectory",
        fontsize=12,
        fontweight="bold",
    )
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(loc="lower left", fontsize=8.0, frameon=True)

    fig.tight_layout()
    FigureCanvasAgg(fig).print_png(str(target))
    return target


def generate_report_figures(
    result: SimulationResult,
    out_dir: Path,
    prefix: str = "arm",
) -> dict[str, Path]:
    """Generate both joint-response and planar-workspace PNGs into out_dir."""
    target_dir = Path(out_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    resp_path = plot_joint_response(result, target_dir / f"{prefix}_response.png")
    traj_path = plot_arm_trajectory(result, target_dir / f"{prefix}_trajectory.png")
    return {
        "response": resp_path,
        "trajectory": traj_path,
    }


def launch_interactive_ui(initial_config: SimulationConfig | None = None) -> int:
    """Launch an optional Tkinter + Matplotlib interactive PID tuning window."""
    try:
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    except Exception as exc:  # pragma: no cover - environment dependent
        print(f"Interactive UI unavailable (Tkinter import failed): {exc}", file=sys.stderr)
        return 1

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(
            f"Interactive UI cannot open display (headless environment): {exc}",
            file=sys.stderr,
        )
        return 1

    cfg = initial_config if initial_config is not None else SimulationConfig()
    matplotlib.use("TkAgg", force=False)
    root.title("2-DOF Planar Arm Discrete PID Simulator")

    controls = ttk.Frame(root, padding=10)
    controls.pack(side=tk.LEFT, fill=tk.Y)

    vars_map = {
        "theta1": tk.DoubleVar(value=30.0),
        "theta2": tk.DoubleVar(value=30.0),
        "kp": tk.DoubleVar(value=cfg.gains1.kp),
        "ki": tk.DoubleVar(value=cfg.gains1.ki),
        "kd": tk.DoubleVar(value=cfg.gains1.kd),
        "duration": tk.DoubleVar(value=15.0),
    }

    for row_idx, (label, var) in enumerate(vars_map.items()):
        ttk.Label(controls, text=label).grid(row=row_idx, column=0, sticky="w", pady=3)
        ttk.Entry(controls, textvariable=var, width=10).grid(
            row=row_idx, column=1, padx=6, pady=3
        )

    fig = Figure(figsize=(8.0, 5.2), dpi=100)
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    def _run_update() -> None:
        fig.clear()
        res = simulate_step(
            theta1_ref_deg=vars_map["theta1"].get(),
            theta2_ref_deg=vars_map["theta2"].get(),
            duration_s=vars_map["duration"].get(),
        )
        ax1, ax2 = fig.subplots(1, 2)
        times = [s.time_s for s in res.steps]
        ax1.plot(times, [s.theta1_actual_deg for s in res.steps], label="Theta1")
        ax1.plot(times, [s.theta2_actual_deg for s in res.steps], label="Theta2")
        ax1.set_title("Joint Response")
        ax1.legend()
        ax1.grid(True)

        px, py = link_polyline(res.final_pose)
        ax2.plot([s.x for s in res.steps], [s.y for s in res.steps], label="Path")
        ax2.plot(px, py, marker="o", linewidth=2.5, label="Final Pose")
        ax2.set_aspect("equal", adjustable="box")
        ax2.set_title("XY Planar Arm")
        ax2.legend()
        ax2.grid(True)
        canvas.draw()

    ttk.Button(controls, text="Run", command=_run_update).grid(
        row=len(vars_map), column=0, columnspan=2, pady=10
    )
    _run_update()
    root.mainloop()
    return 0
