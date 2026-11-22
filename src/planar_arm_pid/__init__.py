"""Discrete-time dual-joint PID controller and 2-DOF planar arm simulator."""

from planar_arm_pid.datasets import (
    CSV_COLUMNS,
    default_baseline_config,
    default_multi_waypoint_segments,
    generate_all_datasets,
    generate_baseline_dataset,
    generate_multi_waypoint_dataset,
    load_trajectory_csv,
    verify_datasets,
    write_trajectory_csv,
)
from planar_arm_pid.kinematics import (
    ArmGeometry,
    ArmPose,
    forward_kinematics,
    link_polyline,
    workspace_bounds,
)
from planar_arm_pid.pid import (
    DiscretePIDController,
    JointPIDState,
    PIDGains,
    PIDStepTerms,
)
from planar_arm_pid.simulator import (
    JointMetrics,
    SimulationConfig,
    SimulationResult,
    SimulationStep,
    WaypointSegment,
    compute_joint_metrics,
    simulate_step,
    simulate_waypoints,
)

__version__ = "1.0.0"

__all__ = [
    "ArmGeometry",
    "ArmPose",
    "CSV_COLUMNS",
    "DiscretePIDController",
    "JointMetrics",
    "JointPIDState",
    "PIDGains",
    "PIDStepTerms",
    "SimulationConfig",
    "SimulationResult",
    "SimulationStep",
    "WaypointSegment",
    "__version__",
    "compute_joint_metrics",
    "default_baseline_config",
    "default_multi_waypoint_segments",
    "forward_kinematics",
    "generate_all_datasets",
    "generate_baseline_dataset",
    "generate_multi_waypoint_dataset",
    "link_polyline",
    "load_trajectory_csv",
    "simulate_step",
    "simulate_waypoints",
    "verify_datasets",
    "workspace_bounds",
    "write_trajectory_csv",
]
