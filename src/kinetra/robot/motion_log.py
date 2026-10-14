"""The on-disk account of one stage run.

Imports no ROS and no Unitree SDK. A commanded move cannot be replayed, so this
file is the only account of it there will ever be, and its central obligation is
to not overstate what happened: every recorded call carries a `proves` string
saying what its return code does and does not establish. `SportClient.Move` is
fire-and-forget -- a `0` means the local DDS write succeeded and nothing more --
so a log that reported "32 commands sent, all returned 0" without that caveat
would read as evidence the robot moved, which it is not.

Layout follows `perception/snapshot.py`: one directory per run, each dataclass
owns its `to_dict()`, and `save()` writes `motion.json` with `indent=2`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from kinetra.robot.limits import (
    AbortVerdict,
    MotionLimits,
    ReadinessVerdict,
    StateSignature,
    Velocity,
    describe_return_code,
)
from kinetra.robot.stages import ScheduledCommand, StageSpec

MOTION_FILENAME = "motion.json"
# Logs go to the NVMe for the same reason snapshots do: `/` had 16 GB free at the
# October survey, and `.gitignore` does not cover this filename, so the default
# location is the thing keeping a run out of a public repository.
DEFAULT_OUTPUT_ROOT = "/mnt/data/kinetra/motion"
SCHEMA = "kinetra.motion/1"

# What a return code from each call does and does not prove. Attached to every
# record so that no later reader -- including us, months from now -- can mistake
# a 0 from Move() for robot-side evidence.
RETURN_CODE_NOTES: dict[str, str] = {
    "SportClient.Move": (
        "fire-and-forget (_CallNoReply): 0 means the local DDS write succeeded and "
        "nothing more. It is NOT evidence the robot received, accepted or acted on "
        "the command. The evidence for that is motion_onset_s."),
    "SportClient.StopMove": "blocking round-trip: the code is returned by the robot.",
    "SportClient.GetServerApiVersion": (
        "blocking round-trip, and it works before Init() because api id 1 bypasses "
        "the registration table: the code is robot-side."),
    "MotionSwitcherClient.CheckMode": "blocking round-trip: the code is robot-side.",
    "SportClient.Damp": "blocking round-trip: the code is returned by the robot.",
}


def run_id(stage_name: str, taken: datetime, label: str = "") -> str:
    """`20261014T153118_902_yaw`, following `capture.py`'s naming precedent.

    The stage name comes last so that a directory listing reads as a bring-up
    history rather than as an undifferentiated pile of runs.
    """
    suffix = f"_{label}" if label else ""
    return (f"{taken.strftime('%Y%m%dT%H%M%S')}_{taken.microsecond // 1000:03d}"
            f"_{stage_name}{suffix}")


@dataclass(frozen=True)
class CommandRecord:
    """One SDK call, with timestamps either side so its cost is measurable.

    The before/after pair is not redundant: `Move()` waits for a matched
    subscriber in 0.1 s steps before writing, so when the robot is unreachable a
    single call can block for the whole `SetTimeout`. Nobody has measured that
    on this hardware, and this is where the measurement lands.
    """

    index: int
    call: str
    phase: str
    sent_at: float
    sent_monotonic: float
    returned_monotonic: float
    velocity: Velocity | None
    return_code: int | None
    blocking: bool
    error: str | None = None

    @property
    def call_duration_s(self) -> float:
        return self.returned_monotonic - self.sent_monotonic

    @property
    def proves(self) -> str:
        return RETURN_CODE_NOTES.get(self.call, "unknown call; no claim is made")

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "call": self.call,
            "phase": self.phase,
            "sent_at": self.sent_at,
            "sent_monotonic": self.sent_monotonic,
            "call_duration_s": self.call_duration_s,
            "velocity": None if self.velocity is None else self.velocity.to_dict(),
            "return_code": self.return_code,
            "return_code_meaning": describe_return_code(self.return_code),
            "blocking": self.blocking,
            "error": self.error,
            "proves": self.proves,
        }


@dataclass(frozen=True)
class Measurement:
    """What the robot actually did, as measured rather than as commanded."""

    translation_m: float | None = None
    rotation_rad: float | None = None
    path_length_m: float | None = None
    motion_onset_s: float | None = None
    peak_speed_mps: float | None = None
    peak_yaw_rate_rps: float | None = None
    stop_translation_m: float | None = None
    stop_rotation_rad: float | None = None
    settle_time_s: float | None = None
    # At a first bring-up the tape measure is ground truth and the odometry is
    # the thing under test, not the other way round. Odometry here reports an
    # all-zero covariance and its drift has never been measured
    # (docs/OPEN_QUESTIONS.md 2.8), so an unchecked odometry figure is a claim,
    # not a measurement.
    tape_translation_m: float | None = None
    tape_rotation_deg: float | None = None
    operator_direction: str = ""

    def to_dict(self) -> dict[str, Any]:
        ratio: float | None = None
        if (self.tape_translation_m is not None and self.translation_m is not None
                and self.tape_translation_m > 0.0):
            ratio = self.translation_m / self.tape_translation_m
        return {
            "translation_m": self.translation_m,
            "rotation_rad": self.rotation_rad,
            "path_length_m": self.path_length_m,
            "motion_onset_s": self.motion_onset_s,
            "motion_onset_note": ("None means the robot never moved; that is a finding, "
                                  "not an error, and it is the only evidence a Move() did "
                                  "anything"),
            "peak_speed_mps": self.peak_speed_mps,
            "peak_yaw_rate_rps": self.peak_yaw_rate_rps,
            "stop_translation_m": self.stop_translation_m,
            "stop_rotation_rad": self.stop_rotation_rad,
            "settle_time_s": self.settle_time_s,
            "tape_translation_m": self.tape_translation_m,
            "tape_rotation_deg": self.tape_rotation_deg,
            "odometry_over_tape": ratio,
            "ground_truth_note": ("the tape measure is ground truth here and the odometry "
                                  "is under test: position and velocity come from the same "
                                  "onboard estimator, which reports zero covariance"),
            "operator_direction": self.operator_direction,
        }


@dataclass(frozen=True)
class StageResult:
    """The outcome of one run, in terms that cannot be mistaken for more."""

    outcome: str
    refusals: tuple[str, ...] = ()
    abort: AbortVerdict | None = None
    commands_sent: int = 0
    moves_written: int = 0
    blocking_calls_answered: int = 0
    measurement: Measurement = field(default_factory=Measurement)
    stop_confirmed: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "outcome": self.outcome,
            "refusals": list(self.refusals),
            "abort": None if self.abort is None else self.abort.to_dict(),
            "commands_sent": self.commands_sent,
            "moves_written": self.moves_written,
            "moves_written_note": ("local DDS writes, not acknowledgements: see each "
                                   "command's 'proves' field"),
            "blocking_calls_answered": self.blocking_calls_answered,
            "stop_confirmed": self.stop_confirmed,
            "measurement": self.measurement.to_dict(),
        }


@dataclass
class MotionLog:
    """Everything about one stage run. Mutable, because a run fills it in.

    Deliberately savable at any point: the `watchdog-kill` stage is killed with
    SIGKILL, which runs no handler, so a log that only existed at the end is
    exactly the log that stage could never produce.
    """

    run_id: str
    stage: StageSpec
    limits: MotionLimits
    dry_run: bool
    started_at: float
    schedule: tuple[ScheduledCommand, ...] = ()
    schedule_bounds: dict[str, Any] = field(default_factory=dict)
    readiness: ReadinessVerdict | None = None
    state_before: StateSignature | None = None
    state_after: StateSignature | None = None
    state_settled: StateSignature | None = None
    state_trace: list[StateSignature] = field(default_factory=list)
    commands: list[CommandRecord] = field(default_factory=list)
    result: StageResult = field(default_factory=lambda: StageResult("incomplete"))
    clock: dict[str, Any] = field(default_factory=dict)
    environment: dict[str, Any] = field(default_factory=dict)
    notes: dict[str, Any] = field(default_factory=dict)
    finished_at: float | None = None

    def to_metadata(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "schema": SCHEMA,
            "dry_run": self.dry_run,
            "stage": self.stage.to_dict(),
            "limits": self.limits.to_dict(),
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "schedule": [c.to_dict() for c in self.schedule],
            "schedule_bounds": self.schedule_bounds,
            "readiness": None if self.readiness is None else self.readiness.to_dict(),
            "state_before": None if self.state_before is None else self.state_before.to_dict(),
            "state_after": None if self.state_after is None else self.state_after.to_dict(),
            "state_settled": (None if self.state_settled is None
                              else self.state_settled.to_dict()),
            "state_trace": [s.to_dict() for s in self.state_trace],
            "state_trace_note": ("decimated from 500 Hz over the stage window plus the "
                                 "post-log period; this is the evidence for onset, stop "
                                 "distance and settle time"),
            "commands": [c.to_dict() for c in self.commands],
            "result": self.result.to_dict(),
            "clock": self.clock,
            "environment": self.environment,
            "notes": self.notes,
        }

    def save(self, directory: str | Path) -> Path:
        """Write `motion.json` and return the directory. Overwrites in place."""
        out = Path(directory)
        out.mkdir(parents=True, exist_ok=True)
        (out / MOTION_FILENAME).write_text(json.dumps(self.to_metadata(), indent=2) + "\n")
        return out


def load_motion_log(directory: str | Path) -> dict[str, Any]:
    """Read a run back off-robot, as plain JSON.

    Returns the dict rather than a rehydrated `MotionLog`: a log is read to be
    audited, and reconstructing the dataclasses would quietly drop any field a
    later schema added. The one thing worth enforcing is that the file is ours.
    """
    path = Path(directory)
    if path.is_dir():
        path = path / MOTION_FILENAME
    try:
        payload = json.loads(path.read_text())
    except OSError as exc:
        raise OSError(f"cannot read a motion log at {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path} is not valid JSON: {exc}") from exc
    if not isinstance(payload, dict) or "run_id" not in payload or "stage" not in payload:
        raise ValueError(f"{path} does not look like a kinetra motion log")
    return payload
