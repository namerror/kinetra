"""What the robot is allowed to do, and how we decide it is doing it.

Imports no ROS and no Unitree SDK, so every ceiling, the velocity profile, the
readiness gate, the abort predicates and the measurement helpers are unit tested
without a robot. `motion_check` is the only module that opens a DDS participant,
and it contains no arithmetic of its own: it asks this module for every decision.

Three properties are deliberate and should survive future edits:

* **A request above a ceiling is refused, never silently clamped.** Clamping
  would let an operator ask for 0.4 m/s, receive 0.15, and read a success
  message. `clamp_velocity` reports what it changed; `clamp_or_refuse` raises.
* **The velocity profile is a pure function of elapsed time**, not an
  accumulator, so it cannot drift and cannot be corrupted by a slow iteration.
* **Nothing is gated on a value whose meaning is unverified.** `mode` and
  `body_height` are recorded and never compared to a literal: the survey saw
  `mode 7` and `body_height 0.32` while the robot was lying down with unloaded
  feet (`docs/ROBOT_INVENTORY.md` section 5), so neither discriminates standing.
  Readiness is decided from foot force, attitude, error code and freshness --
  quantities whose physical meaning is not in doubt.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence

# -- hard ceilings ----------------------------------------------------------
#
# These are Kinetra's limits. The pre-existing lab bridge's defaults (0.4 m/s
# forward, 0.1 back, 0.05 lateral, 0.6 rad/s yaw) are another project's
# settings for a robot whose motion path had also never run on hardware
# (docs/ROBOT_INVENTORY.md section 8), and every value here is below them.
#
# They sit AT the first bring-up test values, so no stage can be made more
# aggressive without editing this file and re-reading docs/PROJECT_SPEC.md.

# A slow walk a spotter can out-pace. Nothing stops this robot but the operator:
# obstacle avoidance reports enabled while the lidar feeding it is dead
# (docs/OPEN_QUESTIONS.md 1.3).
HARD_MAX_FORWARD_MPS = 0.15
# Zero means REFUSE, never "clamp to a stop". The robot has no rearward sensing,
# and lateral is the least-exercised axis in Unitree's own stack. Turn-then-walk
# composes to every pose a reverse or lateral move would reach, so there is no
# safety case for carrying the risk. Either one gets its own bring-up stage, with
# its own spotter, if it is ever actually needed.
HARD_MAX_BACKWARD_MPS = 0.0
HARD_MAX_LATERAL_MPS = 0.0
# ~17 deg/s: a body turn a spotter sees clearly inside a second.
HARD_MAX_YAW_RPS = 0.30
# Total commanded window, both ramps included -- not just the hold.
#
# Raised from 2.0 s on 2026-10-15 so an operator-interrupt stage can run long
# enough to be interrupted: at 0.30 rad/s a 2.0 s window leaves under 1.5 s of
# steady rotation, which is not enough time to reach for the remote and see
# what happened. Note what this ceiling does and does not bound: the window is
# a CLOCK bound and bounds nothing physical on its own. What the robot may
# actually travel is bounded by HARD_MAX_TRANSLATION_M and
# HARD_MAX_ROTATION_RAD below, which are unchanged, are checked against
# measured odometry rather than against the profile, and are checked against
# the commanded profile again in `stages.refusals` before anything is sent. So
# a longer window is only reachable by commanding a SLOWER motion, which is
# exactly the trade an interrupt stage wants. A 6.0 s window at the full
# 0.30 rad/s yaw would command 1.74 rad and is refused.
HARD_MAX_WINDOW_S = 6.0

# Geometric bounds, checked against measured odometry rather than against the
# profile. They trust no clock, no loop rate and no velocity command, which is
# why they catch the two failures a time-based bound structurally cannot see:
# the robot moving faster than commanded, and the robot not stopping when the
# profile ended.
HARD_MAX_TRANSLATION_M = 0.35
HARD_MAX_ROTATION_RAD = 0.70

# Below 10 Hz a dropped tick is a long gap in a fire-and-forget command stream;
# above 50 Hz we only load DDS, since Move() is re-issued rather than queued.
MIN_COMMAND_RATE_HZ = 10.0
HARD_MAX_COMMAND_RATE_HZ = 50.0
DEFAULT_COMMAND_RATE_HZ = 20.0

# Gentle enough that the ramp is visible in the logs, which is what makes the
# motion-onset latency measurable -- and onset is the only evidence we get that
# a Move() did anything at all.
DEFAULT_ACCEL_MPS2 = 0.3
DEFAULT_YAW_ACCEL_RPS2 = 0.6
# So nobody can configure a step input by asking for a very short ramp.
MIN_RAMP_S = 0.2

# /sportmodestate publishes at 500 Hz, so a healthy sample is ~2 ms old.
# 0.30 s is ~150 missed messages: a real stall, not jitter.
DEFAULT_MAX_STATE_AGE_S = 0.30
# Stricter before we commit to moving than while we are already moving.
GATE_MAX_STATE_AGE_S = 0.10
GATE_MIN_SAMPLES = 500
GATE_MIN_RATE_HZ = 200.0

# A standing Go2 on a flat floor sits within a couple of degrees. 0.20 rad
# (~11.5 deg) catches both "lying on its side" and "the floor is not flat".
GATE_MAX_TILT_RAD = 0.20
ABORT_TILT_RAD = 0.20
# Already going over. Far outside the gate and unreachable by level-floor
# walking, so the one automatic escalation to Damp() cannot fire in normal use.
FALL_TILT_RAD = 0.60

# CALIBRATED October 14, 2026, against both physical states on this robot.
#
# Lying: foot_force [8, 13, 12, 8] summing to 41, and [7, 14, 14, 7] on
# October 7; the widest lying reading on a single foot is 17.
# Standing: the lightest foot wandered over 25-31 and the sum over 133-142,
# across five runs and a 5000-sample window, with the robot simply standing
# still. So the lightest foot separates the two states by only about 2x and
# drifts, while the TOTAL separates them by 3.3x and is well behaved.
#
# Then the readings kept falling, on a continuously standing robot AND on a
# freshly stood one: the lightest foot went 29, 30, 28, 25, 22, 19 and the total
# 141 down to 121, while `body_height` held at exactly 0.320 throughout. Load
# that falls while the geometry does not move is not a stance change, so this is
# very probably foot-force sensor drift, plausibly thermal -- the IMU read 79 C
# at the survey. See docs/OPEN_QUESTIONS.md 8.3.
#
# That leaves the lightest foot almost useless for telling the states apart:
# worst standing 19 against worst lying 17. The TOTAL still separates cleanly,
# 121 standing against 49 lying, so it carries the whole standing decision and
# this per-foot floor is demoted to what it can actually do -- catch a foot
# bearing nothing at all. 8 is below every lying reading on purpose: deciding
# lying is not this check's job.
#
# Units unknown, and specific to this robot. Both values are recorded in every
# log so no run can be read without knowing what gated it, and both are now
# operator-overridable because their basis is known to drift.
DEFAULT_MIN_FOOT_FORCE = 8.0
FOOT_FORCE_LYING_OBSERVED = (7, 14, 14, 7)
# The widest single-foot reading ever seen while lying down. Any derived
# threshold must stay clear of this, or the gate would pass a lying robot.
FOOT_FORCE_LYING_MAX_OBSERVED = 17
FOOT_FORCE_STANDING_MIN_OBSERVED = 25
# THE standing check, and the only one of the two that still works. Across the
# whole of 2026-10-14: lying totalled 24-49, standing 121-141. 90 sits 1.8x
# above the worst lying reading and 26% below the worst standing one, which
# leaves about 31 counts of room for the drift above before it would refuse a
# standing robot again.
#
# It must stay above 4 x the per-foot floor to do any work at all; an earlier
# 90 was dead code against a per-foot floor of 25, which already guaranteed 100.
# Against a floor of 8 the minimum reachable total is 32, so this bites.
FOOT_FORCE_SUM_MIN = 90.0
# A lifted leg, or a robot leaning on something, shows up as one foot far below
# the mean even when the total looks plausible. Standing measured 0.79 here, so
# 0.35 leaves ample room.
FOOT_FORCE_BALANCE_RATIO = 0.35

# Velocity below this counts as stopped, matching the bridge's deadband.
DEFAULT_DEADBAND = 0.02
# At-rest thresholds for confirming a stop actually happened.
AT_REST_SPEED_MPS = 0.03
AT_REST_YAW_RPS = 0.05
# Detection thresholds for "the robot started moving".
ONSET_SPEED_MPS = 0.03
ONSET_YAW_RPS = 0.05

# Measured faster than this above what we commanded means the robot is doing
# something we did not ask for. Absolute margins rather than ratios: leg
# odometry is noisy, and a ratio is meaningless near zero.
OVERSPEED_MARGIN_MPS = 0.15
OVERSPEED_MARGIN_RPS = 0.30

# Return codes documented in the SDK (rpc/internal.py, go2/sport/sport_api.py).
# Anything else coming back from the robot is undocumented and is treated as the
# most dangerous case rather than the most convenient one.
KNOWN_RETURN_CODES: dict[int, str] = {
    0: "ok",
    3001: "RPC_ERR_UNKNOWN",
    3102: "RPC_ERR_CLIENT_SEND -- the local DDS write failed",
    3103: "RPC_ERR_CLIENT_API_NOT_REG -- Init() was not called",
    3104: "RPC_ERR_CLIENT_API_TIMEOUT",
    3105: "RPC_ERR_CLIENT_API_NOT_MATCH",
    3106: "RPC_ERR_CLIENT_API_DATA",
    3107: "RPC_ERR_CLIENT_LEASE_INVALID",
    3201: "RPC_ERR_SERVER_SEND",
    3202: "RPC_ERR_SERVER_INTERNAL",
    3203: "RPC_ERR_SERVER_API_NOT_IMPL",
    3204: "RPC_ERR_SERVER_API_PARAMETER",
    3205: "RPC_ERR_SERVER_LEASE_DENIED",
    3206: "RPC_ERR_SERVER_NOT_EXIST",
    3207: "RPC_ERR_SERVER_EXIST",
    4101: "SPORT_ERR_CLIENT_POINT_PATH",
    4201: "SPORT_ERR_SERVER_OVERTIME",
    4202: "SPORT_ERR_SERVER_NOT_INIT",
}


class CeilingViolation(ValueError):
    """A velocity reached the send chokepoint outside the approved envelope.

    Raised rather than clamped: by the time a command is about to be written,
    a value outside the envelope means the profile is wrong, and sending a
    quietly corrected version of a wrong command is how a bug becomes a move.
    """


def describe_return_code(code: int | None) -> str:
    """A human-readable note for a return code, flagging unknown ones loudly."""
    if code is None:
        return "no code -- the call raised or was never answered"
    known = KNOWN_RETURN_CODES.get(code)
    return known if known is not None else f"UNDOCUMENTED code {code} -- treated as unsafe"


def wrap_angle(radians: float) -> float:
    """Wrap to (-pi, pi], so a yaw crossing pi is not read as a near-full turn."""
    return math.remainder(radians, math.tau)


@dataclass(frozen=True)
class Velocity:
    """One sport-mode velocity command, in the robot's body frame.

    The sign convention -- +vx forward, +vy left, +wz left -- is asserted by the
    lab bridge's documentation and is UNVERIFIED on this robot
    (docs/OPEN_QUESTIONS.md 1.2). Confirming it is part of what the yaw and
    forward stages are for, so nothing here may assume it is already known.
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def __post_init__(self) -> None:
        for name in ("vx", "vy", "wz"):
            value = getattr(self, name)
            if not math.isfinite(value):
                # Never clamped to zero: a NaN is a programming error, and
                # zeroing it would send a stop while reporting a move.
                raise ValueError(f"{name} must be finite, got {value!r}")

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.vx, self.vy)

    def is_stopped(self, deadband: float = DEFAULT_DEADBAND) -> bool:
        return (abs(self.vx) < deadband and abs(self.vy) < deadband
                and abs(self.wz) < deadband)

    def scaled(self, factor: float) -> Velocity:
        return Velocity(self.vx * factor, self.vy * factor, self.wz * factor)

    def to_dict(self) -> dict[str, float]:
        return {"vx": self.vx, "vy": self.vy, "wz": self.wz}


STOPPED = Velocity()


@dataclass(frozen=True)
class MotionLimits:
    """The approved envelope for one run. Mirrors docs/PROJECT_SPEC.md.

    Validated on construction, so a CLI value above a hard ceiling fails at
    parse time rather than being clamped at send time.
    """

    max_forward_mps: float = HARD_MAX_FORWARD_MPS
    max_backward_mps: float = HARD_MAX_BACKWARD_MPS
    max_lateral_mps: float = HARD_MAX_LATERAL_MPS
    max_yaw_rps: float = HARD_MAX_YAW_RPS
    max_window_s: float = HARD_MAX_WINDOW_S
    max_translation_m: float = HARD_MAX_TRANSLATION_M
    max_rotation_rad: float = HARD_MAX_ROTATION_RAD
    accel_mps2: float = DEFAULT_ACCEL_MPS2
    yaw_accel_rps2: float = DEFAULT_YAW_ACCEL_RPS2
    command_rate_hz: float = DEFAULT_COMMAND_RATE_HZ
    min_foot_force: float = DEFAULT_MIN_FOOT_FORCE
    min_foot_force_sum: float = FOOT_FORCE_SUM_MIN
    max_state_age_s: float = DEFAULT_MAX_STATE_AGE_S
    deadband: float = DEFAULT_DEADBAND

    _CEILINGS = (
        ("max_forward_mps", HARD_MAX_FORWARD_MPS),
        ("max_backward_mps", HARD_MAX_BACKWARD_MPS),
        ("max_lateral_mps", HARD_MAX_LATERAL_MPS),
        ("max_yaw_rps", HARD_MAX_YAW_RPS),
        ("max_window_s", HARD_MAX_WINDOW_S),
        ("max_translation_m", HARD_MAX_TRANSLATION_M),
        ("max_rotation_rad", HARD_MAX_ROTATION_RAD),
        ("command_rate_hz", HARD_MAX_COMMAND_RATE_HZ),
    )

    def __post_init__(self) -> None:
        for name, ceiling in self._CEILINGS:
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite, got {value!r}")
            if value < 0.0:
                raise ValueError(f"{name} must not be negative, got {value!r}")
            if value > ceiling:
                raise ValueError(
                    f"{name}={value!r} exceeds the hard ceiling {ceiling!r}. "
                    "Raising it means editing limits.py and docs/PROJECT_SPEC.md, "
                    "which is deliberate: this is a bring-up envelope, not a preference.")
        if self.command_rate_hz < MIN_COMMAND_RATE_HZ:
            raise ValueError(
                f"command_rate_hz={self.command_rate_hz!r} is below "
                f"{MIN_COMMAND_RATE_HZ} Hz; Move() is fire-and-forget and must be "
                "re-issued often enough that one dropped tick is not a long gap")
        for name in ("accel_mps2", "yaw_accel_rps2", "min_foot_force",
                     "min_foot_force_sum", "max_state_age_s", "deadband"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive, got {value!r}")

    @property
    def tick_s(self) -> float:
        return 1.0 / self.command_rate_hz

    def to_dict(self) -> dict[str, Any]:
        return {
            "max_forward_mps": self.max_forward_mps,
            "max_backward_mps": self.max_backward_mps,
            "max_lateral_mps": self.max_lateral_mps,
            "max_yaw_rps": self.max_yaw_rps,
            "max_window_s": self.max_window_s,
            "max_translation_m": self.max_translation_m,
            "max_rotation_rad": self.max_rotation_rad,
            "accel_mps2": self.accel_mps2,
            "yaw_accel_rps2": self.yaw_accel_rps2,
            "command_rate_hz": self.command_rate_hz,
            "min_foot_force": self.min_foot_force,
            "min_foot_force_sum": self.min_foot_force_sum,
            "min_foot_force_note": (
                "measured 2026-10-14: lying totalled 24-49, standing 121-141, and the "
                "standing reading drifts downward even after a fresh stand (probably "
                "sensor drift). min_foot_force_sum decides standing; min_foot_force only "
                "catches a foot bearing nothing. Units unknown"),
            "max_state_age_s": self.max_state_age_s,
            "deadband": self.deadband,
        }


def clamp_velocity(request: Velocity, limits: MotionLimits) -> tuple[Velocity, tuple[str, ...]]:
    """Clamp into the envelope, returning the result and what it changed.

    A non-finite component raises instead: `Velocity` refuses to hold one.
    """
    changes: list[str] = []
    vx, vy, wz = request.vx, request.vy, request.wz

    if vx > limits.max_forward_mps:
        changes.append(f"vx {vx:+.3f} -> {limits.max_forward_mps:+.3f} (forward ceiling)")
        vx = limits.max_forward_mps
    elif vx < -limits.max_backward_mps:
        limit = -limits.max_backward_mps
        reason = "reverse is refused" if limits.max_backward_mps == 0.0 else "reverse ceiling"
        changes.append(f"vx {vx:+.3f} -> {limit:+.3f} ({reason})")
        vx = limit

    if abs(vy) > limits.max_lateral_mps:
        limit = math.copysign(limits.max_lateral_mps, vy)
        reason = "lateral is refused" if limits.max_lateral_mps == 0.0 else "lateral ceiling"
        changes.append(f"vy {vy:+.3f} -> {limit:+.3f} ({reason})")
        vy = limit

    if abs(wz) > limits.max_yaw_rps:
        limit = math.copysign(limits.max_yaw_rps, wz)
        changes.append(f"wz {wz:+.3f} -> {limit:+.3f} (yaw ceiling)")
        wz = limit

    return Velocity(vx, vy, wz), tuple(changes)


def clamp_or_refuse(request: Velocity, limits: MotionLimits) -> Velocity:
    """The single chokepoint every command passes through before it is sent.

    Raises `CeilingViolation` if anything needed clamping. Reaching here out of
    envelope means the profile is wrong, not that the operator asked for too
    much -- that was already refused at parse time.
    """
    clamped, changes = clamp_velocity(request, limits)
    if changes:
        raise CeilingViolation("; ".join(changes))
    return clamped


def ramp_duration_s(target: Velocity, window_s: float, limits: MotionLimits) -> float:
    """How long each ramp of the trapezoid takes, for this target.

    Long enough to respect the acceleration limits and `MIN_RAMP_S`, and never
    more than half the window -- at which point the profile is a triangle and
    the target is never actually reached.
    """
    needed = max(
        abs(target.vx) / limits.accel_mps2,
        abs(target.vy) / limits.accel_mps2,
        abs(target.wz) / limits.yaw_accel_rps2,
    )
    return min(max(needed, MIN_RAMP_S), window_s / 2.0)


def profile_at(elapsed_s: float, target: Velocity, window_s: float,
               limits: MotionLimits) -> Velocity:
    """The commanded velocity at one instant: a trapezoid, ramped at both ends.

    A pure function of elapsed time rather than an accumulator, so a slow
    iteration cannot make it drift, and the whole schedule can be printed before
    the run. Both ends ramp so that the nominal end of a stage is a commanded
    zero reached gradually -- `StopMove()` is then confirmation, not the
    mechanism by which the robot stops.
    """
    ramp = ramp_duration_s(target, window_s, limits)
    if elapsed_s <= 0.0 or elapsed_s >= window_s:
        return STOPPED
    if elapsed_s < ramp:
        return target.scaled(elapsed_s / ramp)
    if elapsed_s <= window_s - ramp:
        return target
    return target.scaled((window_s - elapsed_s) / ramp)


def build_profile(target: Velocity, window_s: float,
                  limits: MotionLimits) -> tuple[Velocity, ...]:
    """The whole tick-by-tick schedule, always ending at exactly zero.

    The final exact `STOPPED` matters: a ramp that merely approaches zero would
    leave the robot tracking a small velocity forever.
    """
    if window_s <= 0.0:
        return (STOPPED,)
    ticks = int(math.ceil(window_s / limits.tick_s))
    profile = [profile_at(i * limits.tick_s, target, window_s, limits) for i in range(ticks)]
    profile.append(STOPPED)
    return tuple(profile)


def integrate_profile(profile: Sequence[Velocity], tick_s: float) -> tuple[float, float, float]:
    """``(translation_m, lateral_m, rotation_rad)`` commanded by a profile.

    Forward-Euler path length on absolute values, so it is an upper bound on
    commanded travel rather than a prediction of where the robot ends up: it
    assumes the robot tracks every command perfectly and instantly, which is
    exactly the pessimistic assumption a travel cap wants.
    """
    translation = sum(abs(v.vx) for v in profile) * tick_s
    lateral = sum(abs(v.vy) for v in profile) * tick_s
    rotation = sum(abs(v.wz) for v in profile) * tick_s
    return translation, lateral, rotation


@dataclass(frozen=True)
class StreamHealth:
    """Whether the state stream is good enough to trust, over a warm-up window.

    `stamp_advancing` is not redundant with freshness: a driver republishing one
    frozen sample gives new arrival times forever and passes a naive age check.
    """

    samples: int
    observed_rate_hz: float
    stamp_advancing: bool

    def to_dict(self) -> dict[str, Any]:
        return {"samples": self.samples, "observed_rate_hz": self.observed_rate_hz,
                "stamp_advancing": self.stamp_advancing}


@dataclass(frozen=True)
class StateSignature:
    """One `rt/sportmodestate` sample, with nothing interpreted.

    `received_monotonic` is what every freshness check uses. The robot's own
    `stamp` runs ~2687 s behind the Jetson and drifts at ~90 ppm
    (docs/OPEN_QUESTIONS.md 3.1), so checking freshness against it would be
    checking against a 44-minute-old clock. The stamp's only jobs are feeding
    `ClockOffsetEstimator` and being recorded.
    """

    received_at: float
    received_monotonic: float
    stamp_robot_clock: float
    stamp_local_clock: float | None
    mode: int
    gait_type: int
    error_code: int
    progress: float
    body_height_m: float
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    yaw_speed_rps: float
    rpy_rad: tuple[float, float, float]
    foot_force: tuple[int, int, int, int]
    range_obstacle: tuple[float, float, float, float]

    @property
    def yaw_rad(self) -> float:
        return self.rpy_rad[2]

    @property
    def roll_rad(self) -> float:
        return self.rpy_rad[0]

    @property
    def pitch_rad(self) -> float:
        return self.rpy_rad[1]

    @property
    def tilt_rad(self) -> float:
        return max(abs(self.roll_rad), abs(self.pitch_rad))

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.velocity[0], self.velocity[1])

    def loaded_feet(self, threshold: float) -> int:
        return sum(1 for f in self.foot_force if f >= threshold)

    def is_at_rest(self) -> bool:
        return self.speed_mps < AT_REST_SPEED_MPS and abs(self.yaw_speed_rps) < AT_REST_YAW_RPS

    def is_moving(self) -> bool:
        return self.speed_mps >= ONSET_SPEED_MPS or abs(self.yaw_speed_rps) >= ONSET_YAW_RPS

    def age_s(self, now_monotonic: float) -> float:
        return now_monotonic - self.received_monotonic

    def to_dict(self) -> dict[str, Any]:
        return {
            "received_at": self.received_at,
            "received_monotonic": self.received_monotonic,
            "stamp_robot_clock": self.stamp_robot_clock,
            "stamp_local_clock": self.stamp_local_clock,
            "mode": self.mode,
            "gait_type": self.gait_type,
            "error_code": self.error_code,
            "progress": self.progress,
            "body_height_m": self.body_height_m,
            "position": list(self.position),
            "velocity": list(self.velocity),
            "yaw_speed_rps": self.yaw_speed_rps,
            "rpy_rad": list(self.rpy_rad),
            "foot_force": list(self.foot_force),
            "range_obstacle": list(self.range_obstacle),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> StateSignature:
        return cls(
            received_at=float(d["received_at"]),
            received_monotonic=float(d["received_monotonic"]),
            stamp_robot_clock=float(d["stamp_robot_clock"]),
            stamp_local_clock=(None if d.get("stamp_local_clock") is None
                               else float(d["stamp_local_clock"])),
            mode=int(d["mode"]),
            gait_type=int(d["gait_type"]),
            error_code=int(d["error_code"]),
            progress=float(d["progress"]),
            body_height_m=float(d["body_height_m"]),
            position=tuple(float(x) for x in d["position"]),          # type: ignore[arg-type]
            velocity=tuple(float(x) for x in d["velocity"]),          # type: ignore[arg-type]
            yaw_speed_rps=float(d["yaw_speed_rps"]),
            rpy_rad=tuple(float(x) for x in d["rpy_rad"]),            # type: ignore[arg-type]
            foot_force=tuple(int(x) for x in d["foot_force"]),        # type: ignore[arg-type]
            range_obstacle=tuple(float(x) for x in d["range_obstacle"]),  # type: ignore[arg-type]
        )


@dataclass(frozen=True)
class ReadinessVerdict:
    """Whether a stage may arm, and everything observed while deciding."""

    ready: bool
    reasons: tuple[str, ...]
    observations: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"ready": self.ready, "reasons": list(self.reasons),
                "observations": self.observations}


def assess_readiness(state: StateSignature | None, stream: StreamHealth | None,
                     now_monotonic: float, limits: MotionLimits,
                     *, require_motion_ready: bool = True) -> ReadinessVerdict:
    """The empirical standing gate. Returns EVERY reason, not just the first.

    All of them, because an operator who fixes one blocker and re-runs only to
    hit the next has been told half of what the script already knew.
    """
    reasons: list[str] = []
    observations: dict[str, Any] = {
        "min_foot_force_required": limits.min_foot_force,
        "min_foot_force_note": (
            "calibrated 2026-10-14: lying 8-17 per foot (41 total), standing 25-45 "
            "(133-142 total). The total decides standing; this floor catches a lifted "
            "leg. Units unknown"),
    }

    if state is None:
        reasons.append("no rt/sportmodestate message has arrived at all")
        return ReadinessVerdict(False, tuple(reasons), observations)

    age = state.age_s(now_monotonic)
    observations.update({
        "mode": state.mode,
        "mode_note": ("recorded, never asserted: there is no sport-mode enumeration "
                      "anywhere, and mode 7 was observed while lying down"),
        "gait_type": state.gait_type,
        "error_code": state.error_code,
        "progress": state.progress,
        "body_height_m": state.body_height_m,
        "body_height_note": ("recorded, never gated on: it read 0.32 while the robot was "
                             "lying with unloaded feet, so it does not discriminate standing"),
        "foot_force": list(state.foot_force),
        "loaded_feet": state.loaded_feet(limits.min_foot_force),
        "foot_force_sum": sum(state.foot_force),
        "foot_force_sum_required": limits.min_foot_force_sum,
        "roll_rad": state.roll_rad,
        "pitch_rad": state.pitch_rad,
        "speed_mps": state.speed_mps,
        "yaw_speed_rps": state.yaw_speed_rps,
        "state_age_s": age,
        "range_obstacle": list(state.range_obstacle),
        "range_obstacle_note": ("logged, never used as proximity data: it reads a flat 2.0, "
                                "saturated or placeholder (docs/OPEN_QUESTIONS.md 2.7)"),
    })

    if stream is None:
        reasons.append("the state stream was never assessed")
    else:
        observations["stream"] = stream.to_dict()
        if stream.samples < GATE_MIN_SAMPLES:
            reasons.append(f"only {stream.samples} state samples in the warm-up, "
                           f"need {GATE_MIN_SAMPLES}")
        if stream.observed_rate_hz < GATE_MIN_RATE_HZ:
            reasons.append(f"state rate {stream.observed_rate_hz:.1f} Hz is below "
                           f"{GATE_MIN_RATE_HZ:.0f} Hz")
        if not stream.stamp_advancing:
            reasons.append("the state stamp is not advancing -- the stream is frozen, "
                           "even though samples keep arriving")

    if age > GATE_MAX_STATE_AGE_S:
        reasons.append(f"newest state is {age * 1e3:.0f} ms old, limit "
                       f"{GATE_MAX_STATE_AGE_S * 1e3:.0f} ms")

    if state.error_code != 0:
        reasons.append(f"error_code is {state.error_code}, not 0")

    if not require_motion_ready:
        return ReadinessVerdict(not reasons, tuple(reasons), observations)

    unloaded = [i for i, f in enumerate(state.foot_force) if f < limits.min_foot_force]
    if unloaded:
        reasons.append(
            f"feet {unloaded} are below {limits.min_foot_force:g} force "
            f"(foot_force {list(state.foot_force)}) -- the robot is not standing on them")
    total = sum(state.foot_force)
    if total < limits.min_foot_force_sum:
        reasons.append(
            f"total foot force {total} is below {limits.min_foot_force_sum:g}; on this "
            f"robot lying totalled 24-49 and standing 121-141")

    if not unloaded:
        mean = total / 4.0
        if min(state.foot_force) < FOOT_FORCE_BALANCE_RATIO * mean:
            reasons.append(
                f"foot_force {list(state.foot_force)} is lopsided: the lightest foot is "
                f"below {FOOT_FORCE_BALANCE_RATIO:g} of the mean, so a leg is lifted or "
                "the robot is leaning on something")

    if state.tilt_rad > GATE_MAX_TILT_RAD:
        reasons.append(f"body tilt {state.tilt_rad:.3f} rad exceeds "
                       f"{GATE_MAX_TILT_RAD} rad (roll {state.roll_rad:+.3f}, "
                       f"pitch {state.pitch_rad:+.3f})")

    if not state.is_at_rest():
        reasons.append(f"robot is already moving: speed {state.speed_mps:.3f} m/s, "
                       f"yaw rate {state.yaw_speed_rps:+.3f} rad/s")

    return ReadinessVerdict(not reasons, tuple(reasons), observations)


@dataclass(frozen=True)
class AbortCriteria:
    """Per-stage bounds on what the robot may do while a stage is in flight.

    `armed` lets a stage disarm specific predicates by name. The yaw-estop stage
    needs that: there, `operator_interrupt` and `no_onset` firing would make US
    stop the robot, and we would then be unable to tell whether the remote
    stopped it or we did.
    """

    max_translation_m: float = HARD_MAX_TRANSLATION_M
    max_rotation_rad: float = HARD_MAX_ROTATION_RAD
    max_tilt_rad: float = ABORT_TILT_RAD
    fall_tilt_rad: float = FALL_TILT_RAD
    max_state_age_s: float = DEFAULT_MAX_STATE_AGE_S
    min_foot_force: float = DEFAULT_MIN_FOOT_FORCE
    disarmed: tuple[str, ...] = ()

    def is_armed(self, predicate: str) -> bool:
        return predicate not in self.disarmed

    def to_dict(self) -> dict[str, Any]:
        return {"max_translation_m": self.max_translation_m,
                "max_rotation_rad": self.max_rotation_rad,
                "max_tilt_rad": self.max_tilt_rad,
                "fall_tilt_rad": self.fall_tilt_rad,
                "max_state_age_s": self.max_state_age_s,
                "min_foot_force": self.min_foot_force,
                "disarmed": list(self.disarmed)}


# Dwell windows. At 500 Hz a single sample is noise; these are the durations a
# condition must persist before it counts. The hard geometric and liveness
# predicates have no dwell: by the time they are true, waiting is the wrong move.
DWELL_TILT_S = 0.10
DWELL_FALL_S = 0.20
DWELL_OVERSPEED_S = 0.15
DWELL_FEET_S = 0.20
DWELL_NO_ONSET_S = 0.60
# `error_code` is not a fault field alone. Measured over two runs on
# 2026-10-14: it goes 0 -> 2 within 5 ms of the first Move, exactly as `mode`
# goes 1 -> 3 and the robot enters a trot, and it stays at 2 for as long as the
# robot keeps walking -- continuously, 829 samples with no break -- clearing
# about 0.6 s after StopMove once the robot settles.
#
# The first run appeared to show a 0.656 s transient, but only because that run
# aborted at 0.052 s and the robot stopped. So no dwell can work here: 2 is set
# for the whole of any motion stage, and a dwell merely moves the spurious abort
# later. Code 2 is a status flag meaning "walking" and never aborts.
#
# An UNKNOWN nonzero code still aborts, on this dwell, because an unexplained
# code is the dangerous case. 1.0 s is long enough that a momentary flicker
# during a mode change does not kill a run, and short enough to stop a latching
# fault inside a 2 s window.
DWELL_ERROR_CODE_S = 1.0
# Codes explained by observation rather than documentation. Each is recorded in
# every log but never aborts, because each is known to accompany a normal state
# rather than report a failure. A code NOT listed here aborts on the dwell
# above: this list is what has been seen AND explained, nothing more.
BENIGN_ERROR_CODES = {
    2: "set for the duration of locomotion (mode 3); clears when the robot settles",
}


@dataclass(frozen=True)
class AbortVerdict:
    """Why a stage must stop, and whether the fault is bad enough to de-energise."""

    reason: str
    predicate: str
    escalate_to_damp: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {"reason": self.reason, "predicate": self.predicate,
                "escalate_to_damp": self.escalate_to_damp}


class AbortMonitor:
    """Evaluates the abort predicates over a stream of samples.

    Stateful, because the dwell windows need history, but it holds no I/O and no
    clock of its own: every call is given `now_monotonic`. That is what lets the
    whole abort surface be tested without a robot, which matters because this is
    the code we most need working and least want to run for the first time live.
    """

    def __init__(self, start: StateSignature, criteria: AbortCriteria) -> None:
        self._start = start
        self._criteria = criteria
        self._baseline_error_code = start.error_code
        self._since: dict[str, float] = {}
        self._verdict: AbortVerdict | None = None
        # Every code seen is reported even when none of them aborted, so a
        # tolerated transient still leaves a trace in the log.
        self._error_codes_seen: set[int] = set()
        # The last time motion was OBSERVED, not whether the newest sample
        # happens to show it. A trot oscillates yaw rate and speed through zero
        # several times a second -- on real data, 25 of 40 samples in an 80 ms
        # window of a genuine trot read as stationary -- so an instantaneous
        # check false-positives on a robot that is plainly walking.
        self._last_moving: float | None = None

    @property
    def verdict(self) -> AbortVerdict | None:
        """The first tripped criterion. First cause wins; later ones are noise."""
        return self._verdict

    @property
    def baseline_error_code(self) -> int:
        return self._baseline_error_code

    @property
    def error_codes_seen(self) -> tuple[int, ...]:
        """Every code observed to differ from the baseline, aborting or not."""
        return tuple(sorted(self._error_codes_seen))

    def _dwelled(self, key: str, now: float, condition: bool, dwell_s: float) -> bool:
        if not condition:
            self._since.pop(key, None)
            return False
        started = self._since.setdefault(key, now)
        return (now - started) >= dwell_s

    def _trip(self, predicate: str, reason: str, *, damp: bool = False) -> AbortVerdict:
        if self._verdict is None:
            self._verdict = AbortVerdict(reason, predicate, escalate_to_damp=damp)
        return self._verdict

    def observe(self, state: StateSignature | None, now_monotonic: float,
                commanded: Velocity, *, commanded_since_s: float | None = None,
                return_code: int | None = 0) -> AbortVerdict | None:
        """Fold one sample in and report the abort verdict, if any."""
        if self._verdict is not None:
            return self._verdict

        c = self._criteria

        if return_code not in (0, None):
            return self._trip("rpc_error",
                              f"a command returned {return_code}: "
                              f"{describe_return_code(return_code)}")

        if state is None:
            # Without state there are no abort predicates at all -- we are flying
            # blind, and a bring-up that has lost sight of the robot must stop.
            return self._trip("state_missing", "state stopped arriving entirely")

        age = state.age_s(now_monotonic)
        if age > c.max_state_age_s:
            return self._trip("state_stale",
                              f"state is {age * 1e3:.0f} ms old, limit "
                              f"{c.max_state_age_s * 1e3:.0f} ms")

        # A change, not a nonzero value: a nonzero baseline is plausible on this
        # robot with the lidar dead, and code that demands 0 here is code that
        # gets edited to ignore the field. But it needs a dwell, because the
        # field also flags normal transitions -- see DWELL_ERROR_CODE_S.
        changed = state.error_code != self._baseline_error_code
        if changed:
            self._error_codes_seen.add(state.error_code)
        # An explained code is recorded and tolerated however long it persists.
        unexplained = changed and state.error_code not in BENIGN_ERROR_CODES
        if self._dwelled("error_code", now_monotonic, unexplained, DWELL_ERROR_CODE_S):
            return self._trip("error_code_change",
                              f"error_code changed from {self._baseline_error_code} "
                              f"to {state.error_code}, which is undocumented and has "
                              f"persisted {DWELL_ERROR_CODE_S:g} s")

        travel, rotation = displacement_between(self._start, state)
        if travel > c.max_translation_m:
            return self._trip("displacement",
                              f"travelled {travel:.3f} m, cap {c.max_translation_m:.3f} m")
        if abs(rotation) > c.max_rotation_rad:
            return self._trip("yaw_excursion",
                              f"rotated {rotation:+.3f} rad, cap "
                              f"{c.max_rotation_rad:.3f} rad")

        if self._dwelled("fall", now_monotonic, state.tilt_rad > c.fall_tilt_rad, DWELL_FALL_S):
            # The one automatic escalation: the robot is already going over, so
            # continued actuation is itself the hazard.
            return self._trip("fall",
                              f"body tilt {state.tilt_rad:.3f} rad exceeds "
                              f"{c.fall_tilt_rad} rad -- the robot is going over",
                              damp=True)

        if c.is_armed("attitude") and self._dwelled(
                "attitude", now_monotonic, state.tilt_rad > c.max_tilt_rad, DWELL_TILT_S):
            return self._trip("attitude",
                              f"body tilt {state.tilt_rad:.3f} rad exceeds "
                              f"{c.max_tilt_rad} rad")

        if c.is_armed("overspeed"):
            over_lin = state.speed_mps > commanded.speed_mps + OVERSPEED_MARGIN_MPS
            over_yaw = abs(state.yaw_speed_rps) > abs(commanded.wz) + OVERSPEED_MARGIN_RPS
            if self._dwelled("overspeed", now_monotonic, over_lin or over_yaw,
                             DWELL_OVERSPEED_S):
                return self._trip("overspeed",
                                  f"measured {state.speed_mps:.3f} m/s and "
                                  f"{state.yaw_speed_rps:+.3f} rad/s against commanded "
                                  f"{commanded.speed_mps:.3f} m/s and "
                                  f"{commanded.wz:+.3f} rad/s")

        if c.is_armed("feet_unloaded"):
            # All four, not any one: individual feet unload every gait cycle, so
            # a per-foot predicate would false-trigger continuously while walking.
            all_unloaded = all(f < c.min_foot_force for f in state.foot_force)
            if self._dwelled("feet", now_monotonic, all_unloaded, DWELL_FEET_S):
                return self._trip("feet_unloaded",
                                  f"all four feet are unloaded (foot_force "
                                  f"{list(state.foot_force)})")

        if state.is_moving():
            self._last_moving = now_monotonic

        if (c.is_armed("no_onset") and commanded_since_s is not None
                and not commanded.is_stopped(self._criteria_deadband())):
            # The only predicate that catches "Move() returned 0 and the robot
            # ignored it". It aborts rather than retries: publishing into an
            # unresponsive robot is how a surprise lurch happens when whatever
            # was blocking clears.
            #
            # Judged on when motion was last SEEN rather than on the newest
            # sample, because a gait cycles through zero and would otherwise
            # abort a walking robot.
            quiet_for = (commanded_since_s if self._last_moving is None
                         else now_monotonic - self._last_moving)
            if commanded_since_s >= DWELL_NO_ONSET_S and quiet_for >= DWELL_NO_ONSET_S:
                seen = ("" if self._last_moving is None
                        else f", and none in the last {quiet_for:.2f} s")
                return self._trip("no_onset",
                                  f"commanded motion for {commanded_since_s:.2f} s and the "
                                  f"robot has not moved{seen}; a Move() return of 0 is only "
                                  "a local DDS write")

        return None

    def _criteria_deadband(self) -> float:
        return DEFAULT_DEADBAND


def displacement_between(start: StateSignature, end: StateSignature) -> tuple[float, float]:
    """``(distance_m, rotation_rad)`` between two samples, in the odometry frame.

    Planar: a change in body height is crouching, not travel. Rotation is
    wrapped, so a yaw crossing pi is not reported as a near-full turn.
    """
    dx = end.position[0] - start.position[0]
    dy = end.position[1] - start.position[1]
    return math.hypot(dx, dy), wrap_angle(end.yaw_rad - start.yaw_rad)


def path_length(samples: Sequence[StateSignature]) -> float:
    """Total planar path length through a run of samples.

    Reported alongside net displacement because a robot that wanders out and
    back has moved, and the net figure alone would hide it.
    """
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        total += math.hypot(b.position[0] - a.position[0], b.position[1] - a.position[1])
    return total


def motion_onset(samples: Sequence[StateSignature],
                 first_command_monotonic: float) -> float | None:
    """Seconds from the first command until the robot was observably moving.

    `None` means it never moved. That is a finding, not an error: `Move()` is
    fire-and-forget, so this is the only evidence that a command did anything.
    """
    for sample in samples:
        if sample.received_monotonic < first_command_monotonic:
            continue
        if sample.is_moving():
            return sample.received_monotonic - first_command_monotonic
    return None


def settle_time(samples: Sequence[StateSignature], from_monotonic: float,
                dwell_s: float = 0.20) -> float | None:
    """Seconds from `from_monotonic` until the robot was at rest and stayed there."""
    resting_since: float | None = None
    for sample in samples:
        if sample.received_monotonic < from_monotonic:
            continue
        if sample.is_at_rest():
            if resting_since is None:
                resting_since = sample.received_monotonic
            elif sample.received_monotonic - resting_since >= dwell_s:
                return resting_since - from_monotonic
        else:
            resting_since = None
    return None
