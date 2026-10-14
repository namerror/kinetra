"""Commanding the Go2, and the bring-up that establishes the command path works.

`limits`, `stages` and `motion_log` import no ROS and no Unitree SDK, so the
ceilings, the velocity profile, the readiness gate, the abort predicates and the
log format are all unit tested without a robot. That split is deliberate:
`capture.py` keeps its risky logic inside a ROS node and is therefore untestable
(`docs/SNAPSHOTS.md`), and repeating that here would leave the code that decides
how fast a robot moves near people covered by nothing.

`motion_check` is the only module that opens a DDS participant, and it is
deliberately NOT re-exported here: importing it pulls in `unitree_sdk2py`, which
must never share a process with `rclpy`. Both claim CycloneDDS domain 0, and
whichever calls `dds_create_domain` second fails.
"""

from kinetra.robot.limits import (
    AbortCriteria,
    AbortMonitor,
    AbortVerdict,
    CeilingViolation,
    MotionLimits,
    ReadinessVerdict,
    StateSignature,
    StreamHealth,
    Velocity,
    assess_readiness,
    build_profile,
    clamp_or_refuse,
    clamp_velocity,
    describe_return_code,
    displacement_between,
    integrate_profile,
    motion_onset,
    path_length,
    profile_at,
    ramp_duration_s,
    settle_time,
    wrap_angle,
)
from kinetra.robot.motion_log import (
    CommandRecord,
    Measurement,
    MotionLog,
    StageResult,
    load_motion_log,
    run_id,
)
from kinetra.robot.stages import (
    STAGE_NAMES,
    STAGES,
    ScheduledCommand,
    StageSpec,
    build_schedule,
    describe_schedule,
    refusals,
    schedule_bounds,
)

__all__ = [
    "STAGES",
    "STAGE_NAMES",
    "AbortCriteria",
    "AbortMonitor",
    "AbortVerdict",
    "CeilingViolation",
    "CommandRecord",
    "Measurement",
    "MotionLimits",
    "MotionLog",
    "ReadinessVerdict",
    "ScheduledCommand",
    "StageResult",
    "StageSpec",
    "StateSignature",
    "StreamHealth",
    "Velocity",
    "assess_readiness",
    "build_profile",
    "build_schedule",
    "clamp_or_refuse",
    "clamp_velocity",
    "describe_return_code",
    "describe_schedule",
    "displacement_between",
    "integrate_profile",
    "load_motion_log",
    "motion_onset",
    "path_length",
    "profile_at",
    "ramp_duration_s",
    "refusals",
    "run_id",
    "schedule_bounds",
    "settle_time",
    "wrap_angle",
]
