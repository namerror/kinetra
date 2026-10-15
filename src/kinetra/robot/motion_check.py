"""Staged bounded-motion bring-up for the Go2. One stage per invocation.

    python3 -m kinetra.robot.motion_check --list-stages
    python3 -m kinetra.robot.motion_check --stage yaw --dry-run
    python3 -m kinetra.robot.motion_check --stage yaw --operator you --spotter them

This is the first code in the repository that commands the robot, and the
`unitree_sdk2py` motion path has never run on this hardware by anyone -- the
pre-existing lab bridge's own README says its SDK calls were only exercised in
`--dry-run` on a laptop. So this is a bring-up, not a routine step, and the
defaults are chosen accordingly: nothing is sent without a typed confirmation,
the ceilings in `limits` cannot be raised from the command line, and the
operator holds the handheld remote throughout.

**This module must never share a process with `rclpy`.** Both claim CycloneDDS
domain 0, and whichever calls `dds_create_domain` second fails. State therefore
comes from the SDK's own `ChannelSubscriber`, not from ROS.

The rule that keeps this file reviewable: **no arithmetic and no conditional on
robot state lives here.** Every decision is a call into `limits` or `stages`,
and every output a call into `motion_log`, all three of which import neither ROS
nor the SDK and are covered by `tests/`.

Two SDK facts shape everything below, both read from the installed source:

* `SportClient.Move` is `_CallNoReply`. It returns 0 when the **local** DDS
  write succeeded and nothing more, so it is never evidence the robot moved.
  `motion_onset_s` in the log is that evidence.
* `Move` can block for the whole `SetTimeout`: the writer waits for a matched
  subscriber in 0.1 s steps before writing. With the SDK default of 1.0 s a
  20 Hz loop becomes a 1 Hz loop at the exact moment the robot goes
  unreachable, which is why the timeout differs per phase and why the watchdog
  lives in its own thread.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from kinetra.perception.clock import ClockOffsetEstimator
from kinetra.robot.limits import (
    AbortMonitor,
    AbortVerdict,
    CeilingViolation,
    MotionLimits,
    StateSignature,
    StreamHealth,
    Velocity,
    assess_readiness,
    clamp_or_refuse,
    describe_return_code,
    displacement_between,
    motion_onset,
    path_length,
    profile_at,
    settle_time,
)
from kinetra.robot.motion_log import (
    DEFAULT_OUTPUT_ROOT,
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
    StageSpec,
    build_schedule,
    describe_schedule,
    refusals,
    schedule_bounds,
)

DEFAULT_NETWORK_INTERFACE = "eno1"
DEFAULT_STATE_TOPIC = "rt/sportmodestate"
DEFAULT_SIGNATURE_NAME = "standing-signature.json"

# Per-phase SDK timeouts. A blocking round-trip costs up to ~2x its timeout
# (the write's match-wait, then the reply wait), so the loop's is short enough
# that one bad tick cannot swallow the whole command window.
HANDSHAKE_TIMEOUT_S = 1.0
LOOP_TIMEOUT_S = 0.2
SHUTDOWN_TIMEOUT_S = 0.5

# The DDS reader drops the NEWEST sample when its queue is full, so a long queue
# on a 500 Hz topic accumulates stale backlog and throws away current state.
STATE_QUEUE_LEN = 4
STATE_BUFFER = 4096

WATCHDOG_TICK_S = 0.01
# A loop that has not reached the top of an iteration for this long is stuck
# inside an SDK call, and the watchdog stops the robot itself.
LOOP_STALL_S = 0.5
DEADLINE_SLACK_S = 1.0
SHUTDOWN_BUDGET_S = 4.0
STOP_CONFIRM_S = 1.0
STOP_ATTEMPTS = 3

EXIT_OK = 0
EXIT_NO_DATA = 1
EXIT_REFUSED = 2
EXIT_ABORTED = 3
EXIT_TRANSPORT = 4
EXIT_INCONCLUSIVE = 5

# Set by the signal handler, which must do nothing else: an SDK call from a
# handler can reenter the very call it interrupted, and `logging` or `print`
# takes a lock the interrupted frame may already hold.
_interrupted = threading.Event()
_interrupt_count = 0


def _install_signal_handlers() -> None:
    def handler(signum: int, frame: Any) -> None:
        global _interrupt_count
        _interrupt_count += 1
        _interrupted.set()
        if _interrupt_count >= 3:
            # By the third the operator has clearly decided the remote is the
            # better tool, and the bounded stop sequence has had its chance.
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


# -- the command backend ----------------------------------------------------

@runtime_checkable
class MotionBackend(Protocol):
    """Exactly the four calls this script is allowed to make.

    A Protocol with explicit methods rather than a `__getattr__` stub: the lab
    bridge's dry-run client answered any attribute with 0, which made its
    handshake pass unconditionally, hid every nonzero path, and would have
    raised on the real `GetServerApiVersion` tuple. A dry run whose job is to
    de-risk a first hardware run must never be more permissive than the real
    thing.
    """

    def server_api_version(self) -> tuple[int, str | None]: ...
    def initialise(self) -> None: ...
    def check_mode(self) -> tuple[int, Any]: ...
    def move(self, velocity: Velocity) -> int: ...
    def stop_move(self) -> int: ...
    def set_timeout(self, timeout_s: float) -> None: ...


class LiveBackend:
    """The real thing. The only class in the repository that moves the robot."""

    def __init__(self, interface: str) -> None:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient)
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        self._sport = SportClient()
        self._sport.SetTimeout(HANDSHAKE_TIMEOUT_S)
        self._switcher = MotionSwitcherClient()
        self._switcher.SetTimeout(HANDSHAKE_TIMEOUT_S)
        self._initialised = False

    def server_api_version(self) -> tuple[int, str | None]:
        # Deliberately before Init(): api id 1 is below RPC_INTERNAL_API_ID_MAX,
        # so it bypasses the registration table. That separates "cannot reach
        # the robot" from "did not register the API" into two distinct findings
        # instead of one ambiguous 3103.
        return self._sport.GetServerApiVersion()

    def initialise(self) -> None:
        self._sport.Init()
        self._switcher.Init()
        self._initialised = True

    def _require_init(self) -> None:
        """Fail here rather than letting the SDK answer 3103 forever.

        `Init()` registers every api id; `__CheckApi` rejects an unregistered
        one without touching the wire, so a loop that skipped it publishes
        nothing at all while every call looks like an ordinary error code. This
        turns that into one unambiguous failure before anything is attempted.
        """
        if not self._initialised:
            raise RuntimeError(
                "SportClient.Init() was not called, so no command can reach the robot: "
                "every api id above 100 would be rejected locally with 3103")

    def check_mode(self) -> tuple[int, Any]:
        return self._switcher.CheckMode()

    def move(self, velocity: Velocity) -> int:
        self._require_init()
        return self._sport.Move(velocity.vx, velocity.vy, velocity.wz)

    def stop_move(self) -> int:
        self._require_init()
        return self._sport.StopMove()

    def set_timeout(self, timeout_s: float) -> None:
        self._sport.SetTimeout(timeout_s)


class DryRunBackend:
    """Mirrors the real signatures, including the tuple shapes, and sends nothing.

    Return codes are configurable so the nonzero-handling half of the code -- the
    half that only matters on hardware -- can be exercised before hardware, and
    `rpc_latency_s` makes blocking calls actually sleep so the stall detector is
    tested against the SDK's real match-wait cost.
    """

    def __init__(self, *, rpc_latency_s: float = 0.0, move_latency_s: float = 0.0,
                 move_code: int = 0, stop_code: int = 0, api_code: int = 0,
                 mode_code: int = 0, move_raises: bool = False) -> None:
        self._move_raises = move_raises
        self._latency = rpc_latency_s
        self._move_latency = move_latency_s
        self._move_code = move_code
        self._stop_code = stop_code
        self._api_code = api_code
        self._mode_code = mode_code
        self.sent: list[Velocity] = []
        self.initialised = False

    def _blocking(self) -> None:
        if self._latency > 0.0:
            time.sleep(self._latency)

    def server_api_version(self) -> tuple[int, str | None]:
        self._blocking()
        return self._api_code, ("1.0.0.1" if self._api_code == 0 else None)

    def initialise(self) -> None:
        self.initialised = True

    def check_mode(self) -> tuple[int, Any]:
        self._blocking()
        return self._mode_code, {"name": "normal", "dry_run": True}

    def move(self, velocity: Velocity) -> int:
        # Move() is not free on real hardware: the writer waits for a matched
        # subscriber in 0.1 s steps before writing, so an unreachable robot
        # turns a sub-millisecond call into a SetTimeout-long one. Injecting
        # that here is what lets the loop-stall watchdog be tested at all.
        if self._move_latency > 0.0:
            time.sleep(self._move_latency)
        if self._move_raises:
            raise RuntimeError("injected failure inside Move")
        self.sent.append(velocity)
        return self._move_code

    def stop_move(self) -> int:
        self._blocking()
        return self._stop_code

    def set_timeout(self, timeout_s: float) -> None:
        return None


# -- state ------------------------------------------------------------------

def _stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StateMonitor:
    """Buffers `rt/sportmodestate` and publishes the newest sample.

    The handler does the minimum and returns: at 500 Hz on a Jetson in
    `MODE_30W`, formatting or serialising in the callback would be real load,
    and the DDS reader drops the newest sample when it falls behind.
    """

    def __init__(self, topic: str = DEFAULT_STATE_TOPIC) -> None:
        self.topic = topic
        self.clock = ClockOffsetEstimator()
        self._lock = threading.Lock()
        self._latest: StateSignature | None = None
        self._trace: deque[StateSignature] = deque(maxlen=STATE_BUFFER)
        self._count = 0
        self._first_monotonic: float | None = None
        self._last_stamp: float | None = None
        self._stamp_advanced = False
        self._subscriber: Any = None

    def start(self) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

        self._subscriber = ChannelSubscriber(self.topic, SportModeState_)
        self._subscriber.Init(self._on_state, STATE_QUEUE_LEN)

    def close(self) -> None:
        if self._subscriber is not None:
            try:
                self._subscriber.Close()
            except Exception:
                pass

    def _on_state(self, msg: Any) -> None:
        arrival, monotonic = time.time(), time.monotonic()
        stamp = _stamp_seconds(msg.stamp)
        self.clock.observe(arrival_s=arrival, stamp_s=stamp)
        sample = StateSignature(
            received_at=arrival,
            received_monotonic=monotonic,
            stamp_robot_clock=stamp,
            stamp_local_clock=self.clock.to_local(stamp) if self.clock.trustworthy else None,
            mode=int(msg.mode),
            gait_type=int(msg.gait_type),
            error_code=int(msg.error_code),
            progress=float(msg.progress),
            body_height_m=float(msg.body_height),
            position=(float(msg.position[0]), float(msg.position[1]),
                      float(msg.position[2])),
            velocity=(float(msg.velocity[0]), float(msg.velocity[1]),
                      float(msg.velocity[2])),
            yaw_speed_rps=float(msg.yaw_speed),
            rpy_rad=(float(msg.imu_state.rpy[0]), float(msg.imu_state.rpy[1]),
                     float(msg.imu_state.rpy[2])),
            foot_force=(int(msg.foot_force[0]), int(msg.foot_force[1]),
                        int(msg.foot_force[2]), int(msg.foot_force[3])),
            range_obstacle=(float(msg.range_obstacle[0]), float(msg.range_obstacle[1]),
                            float(msg.range_obstacle[2]), float(msg.range_obstacle[3])),
        )
        with self._lock:
            if self._first_monotonic is None:
                self._first_monotonic = monotonic
            if self._last_stamp is not None and stamp > self._last_stamp:
                self._stamp_advanced = True
            self._last_stamp = stamp
            self._count += 1
            self._latest = sample
            self._trace.append(sample)

    @property
    def latest(self) -> StateSignature | None:
        with self._lock:
            return self._latest

    def trace(self) -> list[StateSignature]:
        with self._lock:
            return list(self._trace)

    def health(self) -> StreamHealth:
        with self._lock:
            count, first, advanced = self._count, self._first_monotonic, self._stamp_advanced
        span = (time.monotonic() - first) if first is not None else 0.0
        rate = count / span if span > 0.0 else 0.0
        return StreamHealth(samples=count, observed_rate_hz=rate, stamp_advancing=advanced)

    def wait_for_first(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.latest is not None:
                return True
            time.sleep(0.01)
        return False

    def collect(self, duration_s: float) -> None:
        if duration_s > 0.0:
            time.sleep(duration_s)


class OfflineStateMonitor(StateMonitor):
    """Synthetic standing state, for exercising the script with no robot at all.

    It proves much less than a connected dry run -- nothing about the interface,
    the subscription, the real rates, or the real gate -- and says so.
    """

    def __init__(self, topic: str = "offline", *, responsive: bool = False) -> None:
        super().__init__(topic)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        # When responsive, the simulated robot tracks what we command, which is
        # what makes the completed path, the onset latency and the self-kill
        # reachable off-robot. Left off, it models a robot that takes every
        # command and ignores it -- the case `no_onset` exists to catch.
        self._responsive = responsive
        self._commanded = Velocity()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def set_commanded(self, velocity: Velocity) -> None:
        self._commanded = velocity

    def start(self) -> None:
        def run() -> None:
            stamp = 1000.0
            while not self._stop.is_set():
                stamp += 0.002
                if self._responsive:
                    self._yaw += self._commanded.wz * 0.002
                    self._x += self._commanded.vx * math.cos(self._yaw) * 0.002
                    self._y += self._commanded.vx * math.sin(self._yaw) * 0.002
                self._synthesise(stamp)
                time.sleep(0.002)

        self._thread = threading.Thread(target=run, name="offline-state", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _synthesise(self, stamp: float) -> None:
        arrival, monotonic = time.time(), time.monotonic()
        self.clock.observe(arrival_s=arrival, stamp_s=stamp)
        sample = StateSignature(
            received_at=arrival, received_monotonic=monotonic,
            stamp_robot_clock=stamp,
            stamp_local_clock=self.clock.to_local(stamp) if self.clock.trustworthy else None,
            mode=7, gait_type=1, error_code=0, progress=0.0, body_height_m=0.32,
            position=(self._x, self._y, 0.32),
            velocity=(self._commanded.vx if self._responsive else 0.0, 0.0, 0.0),
            yaw_speed_rps=self._commanded.wz if self._responsive else 0.0,
            rpy_rad=(0.0, 0.0, self._yaw), foot_force=(170, 165, 172, 168),
            range_obstacle=(2.0, 2.0, 2.0, 2.0))
        with self._lock:
            if self._first_monotonic is None:
                self._first_monotonic = monotonic
            if self._last_stamp is not None and stamp > self._last_stamp:
                self._stamp_advanced = True
            self._last_stamp = stamp
            self._count += 1
            self._latest = sample
            self._trace.append(sample)


# -- the watchdog -----------------------------------------------------------

class Watchdog:
    """Stops the robot when the command loop cannot.

    Its own thread, because the lab bridge's stale detector lives inside the
    loop it guards: a stall inside an SDK call delays the very remedy for that
    stall. Non-daemon, because a daemon thread is killed at interpreter
    shutdown, which is the worst possible moment to lose the stop path.
    """

    def __init__(self, monitor: StateMonitor, abort: AbortMonitor,
                 deadline_monotonic: float) -> None:
        self._monitor = monitor
        self._abort = abort
        self._deadline = deadline_monotonic
        self._heartbeat = time.monotonic()
        self._lock = threading.Lock()
        self.tripped = threading.Event()
        self.verdict: AbortVerdict | None = None
        self.owns_stop = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="watchdog", daemon=False)

    def start(self) -> None:
        self._thread.start()

    def beat(self) -> None:
        """Called at the TOP of each loop iteration.

        The top, not the bottom: a heartbeat written after a blocking call
        cannot report that the call is blocking.
        """
        with self._lock:
            self._heartbeat = time.monotonic()

    def _trip(self, verdict: AbortVerdict) -> None:
        if self.verdict is None:
            self.verdict = verdict
        self.tripped.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                heartbeat = self._heartbeat
            if now > self._deadline:
                self._trip(AbortVerdict(
                    f"stage deadline passed by {now - self._deadline:.2f} s", "deadline"))
                self.owns_stop.set()
            elif now - heartbeat > LOOP_STALL_S:
                self._trip(AbortVerdict(
                    f"the command loop has not ticked for {now - heartbeat:.2f} s, so it "
                    "is stuck inside an SDK call", "loop_stall"))
                # The writer is not documented thread-safe, but a possibly
                # duplicated stop is strictly better than no stop. Do not
                # "fix" this with a lock the stalled call would hold.
                self.owns_stop.set()
            if self.tripped.is_set():
                break
            self._stop.wait(WATCHDOG_TICK_S)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


# -- the run ----------------------------------------------------------------

def _guarded(label: str, call: Any) -> tuple[int | None, str | None, float, float]:
    """Run one SDK call, never letting it take the process down with it.

    `StopMove` can raise, and it can block for the whole timeout, so every
    shutdown call is wrapped individually: a process hanging in `finally` is
    worse than one that exits, because the operator is watching the terminal.
    """
    started = time.monotonic()
    try:
        result = call()
    except Exception as exc:  # noqa: BLE001 - a stop must survive any failure
        return None, f"{type(exc).__name__}: {exc}", started, time.monotonic()
    if isinstance(result, tuple):
        code = int(result[0])
    elif result is None:
        code = 0
    else:
        code = int(result)
    return code, None, started, time.monotonic()


class StageRunner:
    """Executes exactly one stage and fills in its log."""

    def __init__(self, stage: StageSpec, limits: MotionLimits, backend: MotionBackend,
                 monitor: StateMonitor, log: MotionLog, *, dry_run: bool) -> None:
        self.stage = stage
        self.limits = limits
        self.backend = backend
        self.monitor = monitor
        self.log = log
        self.dry_run = dry_run
        self.out_dir: Path | None = None
        self._index = 0

    def _record(self, call: str, phase: str, velocity: Velocity | None,
                code: int | None, error: str | None, started: float, ended: float,
                *, blocking: bool) -> CommandRecord:
        record = CommandRecord(
            index=self._index, call=call, phase=phase,
            sent_at=time.time() - (time.monotonic() - started),
            sent_monotonic=started, returned_monotonic=ended,
            velocity=velocity, return_code=code, blocking=blocking, error=error)
        self._index += 1
        self.log.commands.append(record)
        return record

    def _outcome(self, name: str) -> str:
        """Tag a dry run's outcome so it can never be read as a real one.

        `dry_run: true` is already in the log, but an outcome of plain
        "completed" would survive a grep over outcomes and look like evidence
        that the robot did something.
        """
        return f"dry-run-{name}" if self.dry_run else name

    def _flush(self) -> None:
        if self.out_dir is not None:
            self.log.save(self.out_dir)

    def handshake(self) -> tuple[int, list[str]]:
        """The three round-trips that prove the path is real. No motion."""
        problems: list[str] = []

        code, error, t0, t1 = _guarded("api", self.backend.server_api_version)
        self._record("SportClient.GetServerApiVersion", "handshake", None, code, error,
                     t0, t1, blocking=True)
        print(f"  GetServerApiVersion -> {code}  {describe_return_code(code)}"
              f"  ({(t1 - t0) * 1e3:.1f} ms, before Init)")
        if code != 0:
            problems.append(f"GetServerApiVersion returned {code}: the robot is not "
                            "reachable on this interface")

        code, error, t0, t1 = _guarded("init", self.backend.initialise)
        self._record("SportClient.Init", "handshake", None, code, error, t0, t1,
                     blocking=True)
        if error is not None:
            problems.append(f"Init() raised: {error}")

        code, error, t0, t1 = _guarded("mode", self.backend.check_mode)
        self._record("MotionSwitcherClient.CheckMode", "handshake", None, code, error,
                     t0, t1, blocking=True)
        print(f"  CheckMode           -> {code}  {describe_return_code(code)}"
              f"  ({(t1 - t0) * 1e3:.1f} ms)")
        if code != 0:
            problems.append(f"CheckMode returned {code}: the robot's motion mode could "
                            "not be confirmed")

        code, error, t0, t1 = _guarded("stop", self.backend.stop_move)
        self._record("SportClient.StopMove", "handshake", None, code, error, t0, t1,
                     blocking=True)
        print(f"  StopMove            -> {code}  {describe_return_code(code)}"
              f"  ({(t1 - t0) * 1e3:.1f} ms)")
        if code != 0:
            problems.append(f"StopMove returned {code}: the registered command path "
                            "did not answer")

        answered = sum(1 for c in self.log.commands if c.blocking and c.return_code == 0)
        self.log.result = StageResult(
            self._outcome("completed" if not problems else "aborted"),
            commands_sent=len(self.log.commands), blocking_calls_answered=answered,
            refusals=tuple(problems))
        return (EXIT_OK if not problems else EXIT_TRANSPORT), problems

    def run_motion(self) -> int:
        """The command loop, the watchdog, and the stop."""
        stage, limits = self.stage, self.limits
        start_state = self.monitor.latest
        if start_state is None:
            return EXIT_NO_DATA

        criteria = stage.abort
        if self.dry_run:
            # `no_onset` asks whether the robot ignored a command we sent. In a
            # dry run we send nothing, so "it did not move" is a tautology, not
            # a finding, and leaving it armed would abort every dry run at 0.6 s
            # and hide the loop timing the rest of the window would measure.
            # Everything else stays armed: if the robot moves while we are
            # sending nothing, that is very much worth knowing.
            criteria = replace(criteria, disarmed=tuple(
                set(criteria.disarmed) | {"no_onset"}))
        abort = AbortMonitor(start_state, criteria)

        # Register the API before anything is commanded. Owned by the runner so
        # that both stage paths get it: it used to live only in `handshake()`
        # and in main's dry-run branch, which meant a live motion stage ran
        # with an unregistered client and published nothing at all.
        code, error, t0, t1 = _guarded("init", self.backend.initialise)
        self._record("SportClient.Init", "handshake", None, code, error, t0, t1,
                     blocking=True)
        if error is not None:
            self.log.result = StageResult(
                self._outcome("aborted"), commands_sent=len(self.log.commands),
                abort=AbortVerdict(f"could not initialise the command path: {error}",
                                   "init_failed"))
            return EXIT_TRANSPORT
        window = stage.window_s
        deadline = time.monotonic() + window + DEADLINE_SLACK_S
        watchdog = Watchdog(self.monitor, abort, deadline)

        self.backend.set_timeout(LOOP_TIMEOUT_S)
        moves = 0
        first_command_monotonic: float | None = None
        commanded_since: float | None = None
        verdict: AbortVerdict | None = None
        went_quiet = False
        killed = False

        watchdog.start()
        t_start = time.monotonic()
        try:
            while True:
                watchdog.beat()
                now = time.monotonic()
                elapsed = now - t_start
                if elapsed >= window:
                    break

                if _interrupted.is_set():
                    verdict = AbortVerdict("the operator interrupted with Ctrl-C",
                                           "operator_ctrl_c")
                    break
                if watchdog.tripped.is_set():
                    verdict = watchdog.verdict
                    break

                if stage.self_kill_at_s is not None and elapsed >= stage.self_kill_at_s:
                    # Flush first: SIGKILL runs no handler, so a log written only
                    # at the end is exactly the log this stage cannot produce.
                    self.log.result = StageResult(
                        self._outcome("awaiting-kill"),
                        commands_sent=len(self.log.commands),
                        moves_written=moves)
                    self._flush()
                    print(f"\n  SIGKILL at t+{elapsed:.2f} s -- this process dies now, "
                          f"mid-motion, by design.\n  WATCH THE ROBOT. Stop it with the "
                          f"remote if it keeps rotating.\n")
                    sys.stdout.flush()
                    killed = True
                    os.kill(os.getpid(), signal.SIGKILL)

                if stage.quiet_from_s is not None and elapsed >= stage.quiet_from_s:
                    # Nothing is published from here on. A simulated robot
                    # holds its last commanded velocity, which is the honest
                    # stand-in for "we do not know what the real one will do".
                    if not went_quiet:
                        went_quiet = True
                        print(f"  t+{elapsed:.2f} s: no longer publishing Move. Still "
                              "alive and still able to StopMove.")
                    time.sleep(limits.tick_s)
                    continue

                commanded = profile_at(elapsed, stage.target, window, limits)
                try:
                    commanded = clamp_or_refuse(commanded, limits)
                except CeilingViolation as exc:
                    verdict = AbortVerdict(f"a command left the envelope: {exc}",
                                           "ceiling_violation")
                    break

                tracker = getattr(self.monitor, "set_commanded", None)
                if tracker is not None:
                    tracker(commanded)
                code, error, t0, t1 = _guarded("move", lambda: self.backend.move(commanded))
                self._record("SportClient.Move", "window", commanded, code, error, t0, t1,
                             blocking=False)
                moves += 1
                if error is not None:
                    # A raised call reports no code, and `None` reads as "no code
                    # yet" to the abort monitor, so this is checked here instead.
                    verdict = AbortVerdict(f"a command raised: {error}", "move_raised")
                    break
                if first_command_monotonic is None:
                    # The FIRST Move of the stage, whatever its magnitude. A
                    # Move(0,0,0) is not a no-op: measured 2026-10-14, it puts
                    # the robot into locomotion and starts the gait 23 ms later,
                    # 27 ms before the first non-zero tick. Timing onset from
                    # the first non-zero command skipped the command the robot
                    # actually reacted to and reported ~1.4 ms, which is not a
                    # response latency at all.
                    first_command_monotonic = t0
                if commanded.is_stopped(limits.deadband):
                    commanded_since = None
                elif commanded_since is None:
                    commanded_since = t0

                sample = self.monitor.latest
                since = None if commanded_since is None else time.monotonic() - commanded_since
                verdict = abort.observe(sample, time.monotonic(), commanded,
                                        commanded_since_s=since, return_code=code)
                if verdict is not None:
                    break

                slept = limits.tick_s - (time.monotonic() - now)
                if slept > 0.0:
                    time.sleep(slept)
        finally:
            if not killed:
                watchdog.close()

        # A tolerated transient must still be visible: it aborted nothing, so
        # the log is the only place it would ever show up.
        self.log.notes["error_codes_seen"] = list(abort.error_codes_seen)
        self.log.notes["error_code_baseline"] = abort.baseline_error_code
        if abort.error_codes_seen:
            self.log.notes["error_code_note"] = (
                "codes differing from the baseline during the stage. They did not abort "
                "it, so each is either explained (see BENIGN_ERROR_CODES in "
                "kinetra.robot.limits) or cleared within the dwell window")
            print(f"  error_code changed during the stage: "
                  f"{list(abort.error_codes_seen)} (cleared, did not abort)")

        exit_code = self._finish(start_state, verdict, moves, first_command_monotonic)
        return exit_code

    def _stop_sequence(self, phase: str) -> bool:
        """Zero, then stop, then confirm. Each call individually guarded."""
        budget_end = time.monotonic() + SHUTDOWN_BUDGET_S
        self.backend.set_timeout(SHUTDOWN_TIMEOUT_S)

        code, error, t0, t1 = _guarded("move0", lambda: self.backend.move(Velocity()))
        self._record("SportClient.Move", phase, Velocity(), code, error, t0, t1,
                     blocking=False)

        confirmed = False
        for attempt in range(STOP_ATTEMPTS):
            if time.monotonic() > budget_end:
                break
            code, error, t0, t1 = _guarded("stop", self.backend.stop_move)
            self._record("SportClient.StopMove", phase, None, code, error, t0, t1,
                         blocking=True)
            print(f"  StopMove -> {code}  {describe_return_code(code)}"
                  f"  ({(t1 - t0) * 1e3:.1f} ms)")
            settle_end = min(time.monotonic() + STOP_CONFIRM_S, budget_end)
            while time.monotonic() < settle_end:
                sample = self.monitor.latest
                if sample is not None and sample.is_at_rest():
                    confirmed = True
                    break
                time.sleep(0.02)
            if confirmed:
                break

        if not confirmed:
            print("\n  ***********************************************************",
                  "\n  *  THE STOP WAS NOT CONFIRMED.  USE THE REMOTE NOW.       *",
                  "\n  ***********************************************************\n",
                  file=sys.stderr)
        return confirmed

    def _finish(self, start_state: StateSignature, verdict: AbortVerdict | None,
                moves: int, first_command_monotonic: float | None) -> int:
        confirmed = self._stop_sequence("stop" if verdict is None else "abort")

        if verdict is not None and verdict.escalate_to_damp:
            # The single automatic escalation: the robot is already going over,
            # so continued actuation is itself the hazard.
            print("  the robot is going over: de-energising with Damp()", file=sys.stderr)
            code, error, t0, t1 = _guarded("damp", getattr(self.backend, "damp", lambda: 0))
            self._record("SportClient.Damp", "abort", None, code, error, t0, t1,
                         blocking=True)

        self.monitor.collect(self.stage.post_log_s)
        trace = self.monitor.trace()
        end_state = self.monitor.latest
        self.log.state_after = end_state
        self.log.state_settled = end_state
        self.log.state_trace = trace

        measurement = self._measure(start_state, end_state, trace, first_command_monotonic)
        answered = sum(1 for c in self.log.commands if c.blocking and c.return_code == 0)

        if verdict is not None:
            outcome, exit_code = "aborted", EXIT_ABORTED
        elif moves and measurement.motion_onset_s is None:
            # Every write succeeded locally and the robot never moved. Without a
            # distinct outcome this exits 0 and gets written up as a success.
            outcome, exit_code = "inconclusive", EXIT_INCONCLUSIVE
        else:
            outcome, exit_code = "completed", EXIT_OK

        self.log.result = StageResult(
            self._outcome(outcome), abort=verdict, commands_sent=len(self.log.commands),
            moves_written=moves, blocking_calls_answered=answered,
            measurement=measurement, stop_confirmed=confirmed)
        return exit_code

    def _measure(self, start: StateSignature, end: StateSignature | None,
                 trace: list[StateSignature],
                 first_command_monotonic: float | None) -> Measurement:
        if end is None:
            return Measurement()
        translation, rotation = displacement_between(start, end)
        onset = (None if first_command_monotonic is None
                 else motion_onset(trace, first_command_monotonic))
        last_move = next((c for c in reversed(self.log.commands)
                          if c.call == "SportClient.Move"), None)
        stop_from = last_move.sent_monotonic if last_move is not None else None
        after = [s for s in trace
                 if stop_from is not None and s.received_monotonic >= stop_from]
        stop_translation = stop_rotation = None
        if after:
            stop_translation, stop_rotation = displacement_between(after[0], after[-1])
        return Measurement(
            translation_m=translation, rotation_rad=rotation,
            path_length_m=path_length(trace), motion_onset_s=onset,
            peak_speed_mps=max((s.speed_mps for s in trace), default=0.0),
            peak_yaw_rate_rps=max((abs(s.yaw_speed_rps) for s in trace), default=0.0),
            stop_translation_m=stop_translation, stop_rotation_rad=stop_rotation,
            settle_time_s=(None if stop_from is None else settle_time(trace, stop_from)))


def _describe(log: MotionLog, saved_to: Path | None) -> str:
    """One line per run, in the house style."""
    result, m = log.result, log.result.measurement
    parts = [log.run_id, f"stage {log.stage.name}",
             "DRY RUN" if log.dry_run else result.outcome]
    if result.moves_written:
        parts.append(f"{result.moves_written} moves written (local writes only)")
    if result.blocking_calls_answered:
        parts.append(f"{result.blocking_calls_answered} round-trips answered")
    if m.motion_onset_s is not None:
        parts.append(f"onset {m.motion_onset_s * 1e3:.1f} ms")
    elif result.moves_written:
        parts.append("NO MOTION OBSERVED")
    if m.rotation_rad is not None:
        parts.append(f"rot {m.rotation_rad:+.2f} rad")
    if m.translation_m is not None:
        parts.append(f"trans {m.translation_m:+.2f} m")
    if m.stop_translation_m is not None:
        parts.append(f"stop {m.stop_translation_m:+.2f} m")
    if result.abort is not None:
        parts.append(f"\n    ABORTED ({result.abort.predicate}): {result.abort.reason}")
    for refusal in result.refusals:
        parts.append(f"\n    REFUSED: {refusal}")
    parts.append(f"-> {saved_to}" if saved_to else "(not written)")
    return "  ".join(filter(None, parts))


# -- the standing signature -------------------------------------------------

def write_signature(path: Path, state: StateSignature, clock: dict[str, Any]) -> None:
    """Record what this robot looks like standing, so the gate stops guessing."""
    payload = {
        "recorded_at": state.received_at,
        "foot_force": list(state.foot_force),
        "foot_force_min": min(state.foot_force),
        "mode": state.mode,
        "gait_type": state.gait_type,
        "body_height_m": state.body_height_m,
        "rpy_rad": list(state.rpy_rad),
        "error_code": state.error_code,
        "clock": clock,
        "note": ("the first standing signature measured on this robot. mode and "
                 "body_height are recorded for reference only: both read the same "
                 "while it was lying down with unloaded feet."),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def read_signature(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) and "foot_force_min" in payload else None


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    selector = p.add_mutually_exclusive_group(required=True)
    selector.add_argument("--stage", choices=STAGE_NAMES,
                          help="which bring-up stage to run; exactly one per invocation")
    selector.add_argument("--list-stages", action="store_true",
                          help="print the stage catalogue and schedules, send nothing")
    p.add_argument("--dry-run", action="store_true",
                   help="brief, gate and time the run against live state, but send nothing")
    p.add_argument("--offline", action="store_true",
                   help="synthesise state instead of subscribing; proves far less")
    p.add_argument("--operator", default="", help="who is at the keyboard")
    p.add_argument("--spotter", default="", help="who holds the handheld remote")
    p.add_argument("--area", default="", help='where this ran, e.g. "lab-cleared-2m"')
    p.add_argument("--label", default="", help="tag for this run, added to the run id")
    p.add_argument("--note", default="", help="operator note stored in the log")
    p.add_argument("--out", default=DEFAULT_OUTPUT_ROOT,
                   help="output root, keep off the repo tree (default: %(default)s)")
    p.add_argument("--signature",
                   help=f"standing signature file (default: <out>/{DEFAULT_SIGNATURE_NAME})")
    p.add_argument("--estop-evidence",
                   help="a yaw-estop run directory, required by watchdog-kill")
    p.add_argument("--network-interface", default=DEFAULT_NETWORK_INTERFACE,
                   help="interface the Go2 is on; its traffic is not on the wireless "
                        "LAN (default: %(default)s)")
    p.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC,
                   help="DDS state topic (default: %(default)s)")
    p.add_argument("--command-rate", type=float, default=None,
                   help="Move() ticks per second; Move is fire-and-forget and must be "
                        "re-issued every tick")
    p.add_argument("--window", type=float, default=None,
                   help="override the stage's commanded window in seconds, both ramps "
                        "included. Useful to give yourself more time to interrupt "
                        "yaw-estop. The geometric caps are NOT relaxed with it, so a "
                        "longer window is only accepted together with a slower "
                        "--yaw-rate; the refusal says what it would have commanded")
    p.add_argument("--yaw-rate", type=float, default=None,
                   help="override the stage's commanded yaw rate in rad/s; only for a "
                        "stage that already yaws, and never above the ceiling")
    p.add_argument("--min-foot-force", type=float, default=None,
                   help="per-foot force required before motion; catches a foot bearing "
                        "nothing, and does not decide standing")
    p.add_argument("--min-foot-force-sum", type=float, default=None,
                   help="total foot force required before motion; this is what decides "
                        "standing, and its basis is known to drift")
    p.add_argument("--warmup", type=float, default=2.0,
                   help="seconds of state collected before the briefing, so the clock "
                        "offset is measured (default: %(default)s)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="seconds to wait for the first state message (default: %(default)s)")
    p.add_argument("--clearance", type=float, default=None,
                   help="measured clear distance ahead in m, required by --stage forward")
    p.add_argument("--tape-translation", type=float, default=None,
                   help="optional: how far the robot actually moved in m, measured by "
                        "you. The odometry is under test, so this is the ground truth")
    p.add_argument("--tape-rotation", type=float, default=None,
                   help="optional: how far the robot actually turned in degrees, "
                        "measured by you against a floor mark; signed, left positive")
    p.add_argument("--direction", default="",
                   help='what you saw, e.g. "rotated left" -- recorded verbatim')
    p.add_argument("--allow-unready", action="store_true",
                   help="record the readiness failure and continue; handshake only")
    p.add_argument("--sim-rpc-latency", type=float, default=0.0,
                   help="make dry-run blocking calls sleep this long, to exercise the "
                        "stall detector (default: %(default)s)")
    p.add_argument("--sim-motion", action="store_true",
                   help="make the offline robot track what is commanded, so the "
                        "completed path and the onset measurement are reachable")
    p.add_argument("--sim-move-latency", type=float, default=0.0,
                   help="make dry-run Move() sleep this long, to exercise the loop-stall "
                        "watchdog (default: %(default)s)")
    p.add_argument("--sim-move-code", type=int, default=0,
                   help="return code for dry-run Move(), for fault injection")
    p.add_argument("--sim-move-raise", action="store_true",
                   help="make dry-run Move() raise, to exercise the abort path for a "
                        "call that fails with no return code at all")
    p.add_argument("--sim-stop-code", type=int, default=0,
                   help="return code for dry-run StopMove(), for fault injection")
    return p


def _apply_stage_overrides(stage: StageSpec, args: argparse.Namespace) -> StageSpec:
    """Apply `--window` and `--yaw-rate` to the stage before anything is checked.

    Deliberately NOT a way around the envelope. The overridden stage is the
    stage from here on: `refusals` re-derives the commanded profile from it and
    refuses the run if the longer window pushes commanded rotation or
    translation past the caps, the briefing prints the real schedule, and
    `stage.to_dict()` puts the override in the log. So the operator can trade
    speed for time within a fixed geometric bound, and cannot buy more travel.
    """
    changes: dict[str, Any] = {}

    if args.window is not None:
        if not stage.sends_motion:
            raise ValueError(f"--window does not apply to --stage {stage.name}: it "
                             "commands no motion window")
        if not math.isfinite(args.window) or args.window <= 0.0:
            raise ValueError(f"--window must be positive, got {args.window!r}")
        changes["window_s"] = args.window

    if args.yaw_rate is not None:
        if stage.target.wz == 0.0:
            raise ValueError(f"--yaw-rate does not apply to --stage {stage.name}: its "
                             "target does not yaw, and this is not the way to make it")
        if not math.isfinite(args.yaw_rate) or args.yaw_rate == 0.0:
            raise ValueError(f"--yaw-rate must be non-zero and finite, got "
                             f"{args.yaw_rate!r}")
        changes["target"] = replace(stage.target, wz=args.yaw_rate)

    if not changes:
        return stage
    return replace(stage, notes=stage.notes + (
        "OVERRIDDEN from the command line: "
        + ", ".join(f"{k}={v}" for k, v in sorted(changes.items())),), **changes)


def _print_catalogue(limits: MotionLimits) -> None:
    for name in STAGE_NAMES:
        stage = STAGES[name]
        schedule = build_schedule(stage, limits)
        print(describe_schedule(stage, limits, schedule, dry_run=True))
        print()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    limits_kwargs: dict[str, Any] = {}
    if args.command_rate is not None:
        limits_kwargs["command_rate_hz"] = args.command_rate
    if args.min_foot_force is not None:
        limits_kwargs["min_foot_force"] = args.min_foot_force
    if args.min_foot_force_sum is not None:
        limits_kwargs["min_foot_force_sum"] = args.min_foot_force_sum
    try:
        limits = MotionLimits(**limits_kwargs)
    except ValueError as exc:
        print(f"refused: {exc}", file=sys.stderr)
        return EXIT_REFUSED

    if args.list_stages:
        _print_catalogue(limits)
        return EXIT_OK

    stage = STAGES[args.stage]
    try:
        stage = _apply_stage_overrides(stage, args)
    except ValueError as exc:
        print(f"refused: {exc}", file=sys.stderr)
        return EXIT_REFUSED
    out_root = Path(args.out)
    signature_path = (Path(args.signature) if args.signature
                      else out_root / DEFAULT_SIGNATURE_NAME)

    # -- refusals that need no robot ---------------------------------------
    blockers = list(refusals(stage, limits))
    signature = read_signature(signature_path)
    if stage.requires_signature and signature is None:
        blockers.append(
            f"no standing signature at {signature_path}. Run --stage handshake with the "
            "robot standing first: it records this robot's standing foot force and "
            "calibrates the gate, so a motion stage never runs on an assumed stance")
    if stage.requires_estop_evidence:
        if not args.estop_evidence:
            blockers.append("--estop-evidence is required: nothing of ours survives "
                            "SIGKILL, so the remote must already be known to override "
                            "us. Note what a yaw-estop run can prove -- the override "
                            "holds only while the stick is held (OPEN_QUESTIONS 8.13), "
                            "so do not release it during this stage")
        else:
            try:
                evidence = load_motion_log(args.estop_evidence)
            except (OSError, ValueError) as exc:
                evidence = None
                blockers.append(f"--estop-evidence is unreadable: {exc}")
            if evidence is not None and evidence.get("stage", {}).get("name") != "yaw-estop":
                blockers.append("--estop-evidence must be a yaw-estop run")
    if stage.name == "forward" and not args.dry_run and args.clearance is None:
        blockers.append("--clearance is required for the forward stage: the required "
                        "distance is computed from the stop distance the yaw stages "
                        "measured, not hardcoded")
    if stage.sends_motion and not args.dry_run:
        if not args.operator or not args.spotter:
            blockers.append("--operator and --spotter are both required for a live "
                            "motion run")

    # The gate threshold is NOT derived from the signature. Two attempts at that
    # were both wrong -- half the lightest standing foot lands below a lying
    # reading, and 0.85 of a good stance refuses a merely adequate one -- and the
    # mistake was conceptual: the per-foot floor exists to catch a lifted leg,
    # which reads near zero whatever the stance, so fitting it to one snapshot
    # was answering the wrong question. The thresholds are measured constants in
    # `limits`, and the signature's job is to prove handshake was run standing
    # and to record what standing looked like.

    schedule = build_schedule(stage, limits)
    bounds = schedule_bounds(stage, schedule, limits)
    started = time.time()
    log = MotionLog(
        run_id=run_id(stage.name, datetime.fromtimestamp(started), args.label),
        stage=stage, limits=limits, dry_run=args.dry_run, started_at=started,
        schedule=schedule, schedule_bounds=bounds)
    log.environment = {
        "operator": args.operator, "spotter": args.spotter, "area": args.area,
        "network_interface": args.network_interface, "state_topic": args.state_topic,
        "offline": args.offline,
        "transport": ("synthetic" if args.offline else
                      "unitree_sdk2py ChannelSubscriber + SportClient, CycloneDDS domain 0"),
        "rclpy_imported": "rclpy" in sys.modules,
        "rclpy_note": ("rclpy and unitree_sdk2py both claim CycloneDDS domain 0, so the "
                       "second dds_create_domain in a process fails; this module never "
                       "imports rclpy"),
        "signature_path": str(signature_path), "signature": signature,
        "clearance_m": args.clearance, "argv": sys.argv[1:],
        "python": sys.version.split()[0],
        "cdds_log": "/tmp/cdds.LOG (written by the SDK whenever an interface is named)",
    }
    log.notes = {"operator_note": args.note}

    print(describe_schedule(stage, limits, schedule, dry_run=args.dry_run,
                            operator=args.operator, spotter=args.spotter))
    print()

    if blockers:
        for reason in blockers:
            print(f"refused: {reason}", file=sys.stderr)
        log.result = StageResult("refused", refusals=tuple(blockers))
        saved = log.save(out_root / log.run_id)
        print(_describe(log, saved))
        return EXIT_REFUSED

    # -- connect ------------------------------------------------------------
    monitor: StateMonitor
    if args.offline:
        monitor = OfflineStateMonitor(responsive=args.sim_motion)
        print("offline: state is synthetic. This proves nothing about the interface, "
              "the subscription, the real rates, or the real gate.")
    else:
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            # The interface is named explicitly on purpose: with None the SDK
            # autodetermines and may bind the wireless NIC, silently never
            # reaching the robot. Its own inline XML also overrides the
            # CYCLONEDDS_URI that pins eno1 for ROS.
            ChannelFactoryInitialize(0, args.network_interface)
        except Exception as exc:  # noqa: BLE001
            print(f"cannot open a DDS participant on {args.network_interface}: {exc}",
                  file=sys.stderr)
            print("Is the SDK's CycloneDDS on LD_LIBRARY_PATH? "
                  "`source ~/repos/MMC/setup.sh` is enough on this Jetson.",
                  file=sys.stderr)
            return EXIT_TRANSPORT
        monitor = StateMonitor(args.state_topic)

    _install_signal_handlers()
    exit_code = EXIT_OK
    saved: Path | None = None
    try:
        monitor.start()
        if not monitor.wait_for_first(args.timeout):
            print(f"no message on {monitor.topic} within {args.timeout:.0f} s",
                  file=sys.stderr)
            print("Is the robot powered and on this interface?", file=sys.stderr)
            return EXIT_NO_DATA
        monitor.collect(args.warmup)

        state = monitor.latest
        health = monitor.health()
        verdict = assess_readiness(state, health, time.monotonic(), limits,
                                   require_motion_ready=stage.sends_motion)
        log.readiness = verdict
        log.state_before = state
        log.clock = {"robot_vs_local": monitor.clock.report(),
                     "note": ("measured per run; the Go2 offset drifts ~90 ppm and is "
                              "never hardcoded or cached")}
        print(f"state: {health.samples} samples at {health.observed_rate_hz:.0f} Hz, "
              f"stamp advancing {health.stamp_advancing}")
        print(f"clock: {monitor.clock.report()}")
        if state is not None:
            print(f"gate:  mode {state.mode}  error_code {state.error_code}  "
                  f"foot_force {list(state.foot_force)}  sum {sum(state.foot_force)}"
                  f"/{limits.min_foot_force_sum:g}  "
                  f"loaded {state.loaded_feet(limits.min_foot_force)}/4  "
                  f"per-foot floor {limits.min_foot_force:g}")
        for reason in verdict.reasons:
            print(f"  not ready: {reason}", file=sys.stderr)

        if not verdict.ready and not args.allow_unready:
            log.result = StageResult("refused", refusals=verdict.reasons)
            return EXIT_REFUSED

        # handshake's real deliverable: what this robot looks like standing.
        # Only ever written from a live measurement. A dry run or an offline run
        # must not produce the artifact that unlocks every motion stage, and
        # synthetic foot forces are not a measurement of anything.
        if (stage.name == "handshake" and state is not None
                and not args.dry_run and not args.offline):
            if min(state.foot_force) >= limits.min_foot_force:
                write_signature(signature_path, state, monitor.clock.report())
                print(f"standing signature written to {signature_path}")
            else:
                print("not writing a standing signature: the feet are not loaded, so "
                      "this is a lying-down reading", file=sys.stderr)

        if args.dry_run:
            backend: Any = DryRunBackend(rpc_latency_s=args.sim_rpc_latency,
                                         move_latency_s=args.sim_move_latency,
                                         move_code=args.sim_move_code,
                                         stop_code=args.sim_stop_code,
                                         move_raises=args.sim_move_raise)
        else:
            if not sys.stdin.isatty():
                print("refused: a live run needs a typed confirmation on a terminal",
                      file=sys.stderr)
                log.result = StageResult("refused", refusals=("stdin is not a terminal",))
                return EXIT_REFUSED
            typed = input(f"Type '{stage.name}' to send these commands for real: ").strip()
            if typed != stage.name:
                print("operator declined", file=sys.stderr)
                log.result = StageResult("refused", refusals=("operator declined",))
                return EXIT_REFUSED
            log.environment["confirmed_by"] = "typed-stage-name"
            backend = LiveBackend(args.network_interface)

        runner = StageRunner(stage, limits, backend, monitor, log, dry_run=args.dry_run)
        runner.out_dir = out_root / log.run_id

        print()
        if stage.sends_motion:
            exit_code = runner.run_motion()
        else:
            exit_code, _ = runner.handshake()

        if args.tape_translation is not None or args.tape_rotation is not None \
                or args.direction:
            log.result = replace(log.result, measurement=replace(
                log.result.measurement, tape_translation_m=args.tape_translation,
                tape_rotation_deg=args.tape_rotation,
                operator_direction=args.direction))
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        exit_code = EXIT_ABORTED
    finally:
        monitor.close()
        log.finished_at = time.time()
        log.state_trace = log.state_trace or monitor.trace()
        try:
            saved = log.save(out_root / log.run_id)
        except OSError as exc:
            print(f"could not write the log: {exc}", file=sys.stderr)
        print()
        print(_describe(log, saved))

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
