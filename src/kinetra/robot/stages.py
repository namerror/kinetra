"""The bring-up stages, as data, plus the briefing an operator reads first.

Imports no ROS and no Unitree SDK. A stage is a promise about what will be sent
before anything is sent, so these tests can check the catalogue cannot promise
something the ceilings in `limits` forbid.

There is no stage sequencer, deliberately: exactly one stage runs per
invocation. A bring-up step the operator did not individually authorise is not a
bring-up step.

The order the stages are meant to be run in, and why each exists, is in
`docs/MOTION.md`. Two of them answer questions the obvious single-stage design
could not: `yaw-estop` disarms the predicates that would otherwise make *us*
stop the robot during an operator interrupt -- leaving us unable to tell whether
the remote stopped it or we did -- and `watchdog-quiet` asks whether the robot
stops when commands merely cease, which costs nothing because we keep every stop
mechanism, before `watchdog-kill` destroys our own stop path on purpose.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

from kinetra.robot.limits import (
    AbortCriteria,
    MotionLimits,
    Velocity,
    build_profile,
    clamp_velocity,
    integrate_profile,
    ramp_duration_s,
)

# How long to keep logging state after the last command. Long enough to measure
# the stop, and in yaw-estop long enough to catch the robot RESUMING our Move
# stream when the operator releases the stick, which is a distinct and important
# outcome the stage must be able to report.
DEFAULT_POST_LOG_S = 2.0
ESTOP_POST_LOG_S = 3.0


@dataclass(frozen=True)
class StageSpec:
    """One bring-up stage."""

    name: str
    summary: str
    target: Velocity
    window_s: float
    sends_motion: bool
    blocking_calls: tuple[str, ...]
    expectation: str
    abort: AbortCriteria
    clearance_note: str
    operator_actions: tuple[str, ...] = ()
    # Motion stages refuse to run until `handshake` has recorded a standing
    # signature, because the foot-force gate threshold is otherwise a guess.
    # Enforced in code rather than in a runbook: that is what makes
    # "handshake first" a property of the script instead of a hope.
    requires_signature: bool = False
    # watchdog-kill additionally requires evidence that the remote can actually
    # interrupt us, because by construction nothing of ours survives SIGKILL.
    requires_estop_evidence: bool = False
    # Stop publishing at this offset but stay alive, fully able to StopMove.
    quiet_from_s: float | None = None
    # Send SIGKILL to ourselves at this offset.
    self_kill_at_s: float | None = None
    post_log_s: float = DEFAULT_POST_LOG_S
    notes: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "summary": self.summary,
            "target": self.target.to_dict(),
            "window_s": self.window_s,
            "sends_motion": self.sends_motion,
            "blocking_calls": list(self.blocking_calls),
            "expectation": self.expectation,
            "abort": self.abort.to_dict(),
            "clearance_note": self.clearance_note,
            "operator_actions": list(self.operator_actions),
            "requires_signature": self.requires_signature,
            "requires_estop_evidence": self.requires_estop_evidence,
            "quiet_from_s": self.quiet_from_s,
            "self_kill_at_s": self.self_kill_at_s,
            "post_log_s": self.post_log_s,
            "notes": list(self.notes),
        }


@dataclass(frozen=True)
class ScheduledCommand:
    """One planned tick, so the whole schedule is printable before the run."""

    index: int
    offset_s: float
    phase: str
    velocity: Velocity

    def to_dict(self) -> dict[str, Any]:
        return {"index": self.index, "offset_s": self.offset_s,
                "phase": self.phase, "velocity": self.velocity.to_dict()}


_YAW_TARGET = Velocity(wz=0.30)
_FORWARD_TARGET = Velocity(vx=0.15)

# Geometric caps per stage, tighter than the global ceilings where the stage's
# own geometry allows it. An in-place yaw that translates 0.15 m has drifted off
# its spot and we want to know.
_YAW_ABORT = AbortCriteria(max_translation_m=0.15, max_rotation_rad=0.70)
_FORWARD_ABORT = AbortCriteria(max_translation_m=0.35, max_rotation_rad=0.35)

STAGES: dict[str, StageSpec] = {
    "handshake": StageSpec(
        name="handshake",
        summary="prove the command path is real; no motion is commanded",
        target=Velocity(),
        window_s=0.0,
        sends_motion=False,
        blocking_calls=("SportClient.GetServerApiVersion", "MotionSwitcherClient.CheckMode",
                        "SportClient.StopMove"),
        expectation="three round-trips answer with robot-side codes; the robot does not move",
        abort=AbortCriteria(max_translation_m=0.05, max_rotation_rad=0.10),
        clearance_note="the robot should be standing, but it is not asked to move",
        operator_actions=("stand the robot up with the handheld remote before running this",),
        post_log_s=1.0,
        notes=(
            "This sends one command, StopMove, which is safe but is not nothing.",
            "GetServerApiVersion runs BEFORE Init(): api id 1 is below "
            "RPC_INTERNAL_API_ID_MAX, so it bypasses the registration table and "
            "separates 'unreachable' from 'not registered' into two findings.",
            "Run it with the robot STANDING: its real deliverable is the first "
            "standing foot_force signature this project has ever had, and every "
            "motion stage's gate threshold depends on it.",
        ),
    ),
    "yaw": StageSpec(
        name="yaw",
        summary="rotate in place to the left at +0.30 rad/s, then stop",
        target=_YAW_TARGET,
        window_s=2.0,
        sends_motion=True,
        blocking_calls=("SportClient.StopMove",),
        expectation="the body rotates left in place, all four feet stay loaded, "
                    "translation stays near zero",
        abort=_YAW_ABORT,
        clearance_note="1.5 m clear radius: in-place rotation drifts, so a footprint "
                       "is not enough",
        requires_signature=True,
        notes=(
            "Tested before any forward motion because a stop failure during an "
            "in-place rotation cannot carry the robot anywhere.",
            "+wz is asserted to be LEFT by the lab bridge's documentation and is "
            "UNVERIFIED on this robot; confirming it is part of the point.",
        ),
    ),
    "yaw-estop": StageSpec(
        name="yaw-estop",
        summary="rotate in place while the operator interrupts with the remote",
        target=_YAW_TARGET,
        window_s=2.0,
        sends_motion=True,
        blocking_calls=("SportClient.StopMove",),
        expectation="the remote overrides a live commanding process; the robot stops "
                    "while we are still publishing Move",
        # operator_interrupt, no_onset and overspeed are DISARMED here on purpose.
        # Armed, they would fire the instant the remote took over and we would stop
        # the robot ourselves -- after which we could never distinguish "the remote
        # stopped it" from "we stopped ourselves", which is the whole question.
        abort=AbortCriteria(max_translation_m=0.15, max_rotation_rad=0.70,
                            disarmed=("no_onset", "overspeed", "operator_interrupt",
                                      "feet_unloaded")),
        clearance_note="1.5 m clear radius, operator within arm's reach of the robot",
        operator_actions=(
            "interrupt with the handheld remote about halfway through the rotation",
            "then report what the robot did, and whether it resumed when you let go",
        ),
        requires_signature=True,
        post_log_s=ESTOP_POST_LOG_S,
        notes=(
            "We keep publishing Move through the interrupt until the window ends. "
            "That is the actual question: whether the remote outranks a live "
            "commanding process, not whether it outranks silence.",
            "State is logged for 3 s afterwards because the robot RESUMING our Move "
            "stream when the stick is released is a distinct outcome that matters.",
            "This run's log is the evidence watchdog-kill requires.",
        ),
    ),
    "forward": StageSpec(
        name="forward",
        summary="walk forward at 0.15 m/s for a 2.0 s window, then stop",
        target=_FORWARD_TARGET,
        window_s=2.0,
        sends_motion=True,
        blocking_calls=("SportClient.StopMove",),
        expectation="the robot walks forward about 0.22 m and stops; yaw stays near zero",
        abort=_FORWARD_ABORT,
        clearance_note="clearance is computed from the stop distance the yaw stages "
                       "measured, and the operator is asked to confirm it",
        requires_signature=True,
        notes=(
            "A safety and stop-distance test, not an odometry calibration: at this "
            "distance no drift number would be trustworthy (OPEN_QUESTIONS 2.8).",
        ),
    ),
    "watchdog-quiet": StageSpec(
        name="watchdog-quiet",
        summary="stop publishing Move mid-rotation while staying alive and able to stop",
        target=_YAW_TARGET,
        window_s=2.0,
        sends_motion=True,
        blocking_calls=("SportClient.StopMove",),
        expectation="either the robot stops on its own when commands cease, or it "
                    "keeps rotating and we stop it",
        abort=_YAW_ABORT,
        clearance_note="1.5 m clear radius",
        requires_signature=True,
        quiet_from_s=1.0,
        post_log_s=ESTOP_POST_LOG_S,
        notes=(
            "This answers most of OPEN_QUESTIONS 1.7 at ZERO added risk: we simply "
            "stop talking, and we keep every stop mechanism we have.",
            "Run it before watchdog-kill. If the robot stops in 0.3 s, the kill "
            "variant becomes a confirmation rather than a discovery.",
        ),
    ),
    "watchdog-kill": StageSpec(
        name="watchdog-kill",
        summary="SIGKILL this process mid-rotation, destroying our own stop path",
        target=_YAW_TARGET,
        window_s=2.0,
        sends_motion=True,
        blocking_calls=(),
        expectation="unknown by design: either the robot stops on its own, or the "
                    "handheld remote is the only thing that will stop it",
        abort=_YAW_ABORT,
        clearance_note="1.5 m clear radius, spotter's thumb ON the remote stop",
        operator_actions=(
            "hold the remote with your thumb on the stop for the whole run",
            "stop the robot yourself if it is still rotating a second after the kill",
        ),
        requires_signature=True,
        requires_estop_evidence=True,
        self_kill_at_s=1.0,
        post_log_s=0.0,
        notes=(
            "Yaw only, never forward. If there is no receiver-side watchdog the last "
            "Move persists, and a persisting in-place rotation is bounded in "
            "displacement where a persisting walk is not.",
            "The kill is scheduled by the script rather than typed by the operator: "
            "it lands deterministically mid-motion, and it keeps both the operator's "
            "hands on the remote.",
            "The log is flushed every tick, because SIGKILL runs no handler and a log "
            "written only at the end is exactly the log this stage cannot produce.",
        ),
    ),
}

STAGE_NAMES: tuple[str, ...] = tuple(sorted(STAGES))


def build_schedule(stage: StageSpec, limits: MotionLimits) -> tuple[ScheduledCommand, ...]:
    """The planned ticks for a stage, phase-labelled."""
    if not stage.sends_motion or stage.window_s <= 0.0:
        return ()
    profile = build_profile(stage.target, stage.window_s, limits)
    ramp = ramp_duration_s(stage.target, stage.window_s, limits)
    commands: list[ScheduledCommand] = []
    for index, velocity in enumerate(profile):
        offset = index * limits.tick_s
        # Ordered so the zero at t=0 reads as the start of the ramp rather than
        # as a stop: only a tick at or past the end of the window is a stop.
        if offset >= stage.window_s:
            phase = "stop"
        elif offset < ramp:
            phase = "ramp-up"
        elif offset <= stage.window_s - ramp:
            phase = "hold"
        else:
            phase = "ramp-down"
        commands.append(ScheduledCommand(index, offset, phase, velocity))
    return tuple(commands)


def schedule_bounds(stage: StageSpec, schedule: Sequence[ScheduledCommand],
                    limits: MotionLimits) -> dict[str, Any]:
    """What the schedule commands, as an upper bound rather than a prediction."""
    profile = [c.velocity for c in schedule]
    translation, lateral, rotation = integrate_profile(profile, limits.tick_s)
    return {
        "ticks": len(schedule),
        "duration_s": len(schedule) * limits.tick_s,
        "peak_speed_mps": max((v.speed_mps for v in profile), default=0.0),
        "peak_yaw_rate_rps": max((abs(v.wz) for v in profile), default=0.0),
        "commanded_translation_m": translation,
        "commanded_lateral_m": lateral,
        "commanded_rotation_rad": rotation,
        "translation_cap_m": stage.abort.max_translation_m,
        "rotation_cap_rad": stage.abort.max_rotation_rad,
        "note": ("forward-Euler upper bound on commanded travel, assuming the robot "
                 "tracks every command perfectly and instantly; not a prediction"),
    }


def refusals(stage: StageSpec, limits: MotionLimits) -> tuple[str, ...]:
    """Every reason this stage must not run, before any robot state is read.

    An empty tuple is the only permission to send.
    """
    reasons: list[str] = []

    if stage.window_s > limits.max_window_s:
        reasons.append(f"window {stage.window_s} s exceeds the ceiling "
                       f"{limits.max_window_s} s")

    _, changes = clamp_velocity(stage.target, limits)
    if changes:
        reasons.append("target is outside the envelope: " + "; ".join(changes))

    schedule = build_schedule(stage, limits)
    if stage.sends_motion:
        bounds = schedule_bounds(stage, schedule, limits)
        if bounds["commanded_translation_m"] > stage.abort.max_translation_m:
            reasons.append(
                f"commanded translation {bounds['commanded_translation_m']:.3f} m exceeds "
                f"the stage cap {stage.abort.max_translation_m:.3f} m")
        if bounds["commanded_translation_m"] > limits.max_translation_m:
            reasons.append(
                f"commanded translation {bounds['commanded_translation_m']:.3f} m exceeds "
                f"the hard cap {limits.max_translation_m:.3f} m")
        if bounds["commanded_rotation_rad"] > stage.abort.max_rotation_rad:
            reasons.append(
                f"commanded rotation {bounds['commanded_rotation_rad']:.3f} rad exceeds "
                f"the stage cap {stage.abort.max_rotation_rad:.3f} rad")
        if not schedule:
            reasons.append("stage claims to send motion but builds an empty schedule")

    for offset_name in ("quiet_from_s", "self_kill_at_s"):
        offset = getattr(stage, offset_name)
        if offset is not None and not 0.0 < offset < stage.window_s:
            reasons.append(f"{offset_name}={offset} is not inside the "
                           f"{stage.window_s} s window")

    return tuple(reasons)


def _format_ticks(schedule: Sequence[ScheduledCommand], limits: MotionLimits) -> list[str]:
    """Print the ramps tick by tick and elide long runs of identical holds."""
    lines: list[str] = []
    index = 0
    while index < len(schedule):
        command = schedule[index]
        run_end = index
        while (run_end + 1 < len(schedule)
               and schedule[run_end + 1].phase == command.phase
               and schedule[run_end + 1].velocity == command.velocity):
            run_end += 1
        lines.append(
            f"  tick {command.index:3d}  t+{command.offset_s:5.3f} s  {command.phase:<9s}"
            f"  vx {command.velocity.vx:+.2f}  vy {command.velocity.vy:+.2f}"
            f"  wz {command.velocity.wz:+.2f}")
        hidden = run_end - index
        if hidden > 1:
            lines.append(f"  ...        {hidden} identical {command.phase} ticks omitted")
            index = run_end
        index += 1
    return lines


def describe_schedule(stage: StageSpec, limits: MotionLimits,
                      schedule: Sequence[ScheduledCommand], *, dry_run: bool,
                      operator: str = "", spotter: str = "") -> str:
    """The briefing the operator reads before anything is sent.

    Printed identically in a dry run and a live run, so that reading it twice is
    part of the safety case rather than a formality.
    """
    mode = "DRY RUN" if dry_run else "LIVE -- THE ROBOT WILL MOVE" if stage.sends_motion \
        else "LIVE -- no motion commanded"
    bounds = schedule_bounds(stage, schedule, limits)
    width = 14

    def row(label: str, text: str) -> str:
        return f"{label:<{width}s}{text}"

    lines = [f"========== kinetra motion check  stage {stage.name}  {mode} ==========",
             row("intent", stage.summary)]

    if stage.sends_motion:
        suppressed = " -- suppressed: a dry run sends nothing" if dry_run else ""
        lines.append(row("sends motion", f"yes{suppressed}"))
    else:
        lines.append(row("sends motion", "no motion is commanded, but StopMove IS a command"))

    lines.append(row("limits", f"vx <= {limits.max_forward_mps:.2f} m/s  "
                               f"vy <= {limits.max_lateral_mps:.2f} m/s  "
                               f"wz <= {limits.max_yaw_rps:.2f} rad/s  "
                               f"accel {limits.accel_mps2:.2f} m/s^2 / "
                               f"{limits.yaw_accel_rps2:.2f} rad/s^2"))

    if schedule:
        lines.append(row("schedule", f"{len(schedule)} ticks of SportClient.Move at "
                                     f"{limits.command_rate_hz:.1f} Hz over "
                                     f"{bounds['duration_s']:.2f} s, re-issued every tick"))
        lines.extend(_format_ticks(schedule, limits))
    for call in stage.blocking_calls:
        lines.append(f"  call       {call}()  blocking, the return code is robot-side")
    if not schedule and not stage.blocking_calls:
        lines.append(row("schedule", "nothing is sent"))

    lines.append(row("bounds", f"commanded rotation {bounds['commanded_rotation_rad']:.3f} rad "
                               f"of {bounds['rotation_cap_rad']:.3f} rad cap"))
    lines.append(row("", f"commanded translation {bounds['commanded_translation_m']:.3f} m "
                         f"of {bounds['translation_cap_m']:.3f} m cap -- open-loop upper "
                         "bound, not a prediction"))
    lines.append(row("gate", f"all four feet >= {limits.min_foot_force:g} force "
                             "(UNCALIBRATED), error_code 0, state fresh and advancing"))
    lines.append(row("", "mode and body_height are recorded, never asserted: both read "
                         "'standing-ish' while the robot was lying down"))
    lines.append(row("abort if", "state stops arriving or goes stale, travel or rotation "
                                 "past the cap, body tilt, overspeed, all feet unloaded, "
                                 "or no motion after a command"))
    if stage.abort.disarmed:
        lines.append(row("", f"DISARMED for this stage: {', '.join(stage.abort.disarmed)}"))
    lines.append(row("on abort", "Move(0,0,0) then StopMove; the robot stays standing and "
                                 "you sit it down with the remote"))
    lines.append(row("expect", stage.expectation))
    lines.append(row("clearance", stage.clearance_note))
    lines.append(row("", "nothing stops this robot but the operator: obstacle avoidance "
                         "reports enabled while its lidar is dead"))

    if stage.operator_actions:
        lines.append(row("you must", stage.operator_actions[0]))
        for action in stage.operator_actions[1:]:
            lines.append(row("", action))
    lines.append(row("people", f"operator {operator or '(unset)'}  "
                               f"spotter {spotter or '(unset)'}"))
    for note in stage.notes:
        lines.append(row("note", note))

    lines.append("=" * 62)
    if dry_run:
        lines.append("dry run: no command is sent. This proves the script, not the robot.")
    return "\n".join(lines)
