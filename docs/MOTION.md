# Bounded motion

A **stage** is one bounded, pre-declared move, run on its own, with the operator holding the handheld remote.
`kinetra.robot.motion_check` runs exactly one stage per invocation, prints what it is about to send before
sending it, and sends nothing without a typed confirmation. It is the first code in this repository that
commands the robot.

**Status, October 14, 2026: `handshake` and `yaw` have run on hardware.** `yaw` completed its full window
(`20261014T174008_123_yaw`): `+wz` is left, the robot delivers about 0.55 of the commanded rotation,
actuation latency is 23 ms, and motion continues ~0.5 s after `StopMove` is acknowledged. `yaw-estop`,
`forward`, `watchdog-quiet` and `watchdog-kill` have **not** been run, so anything below about those is a
question the stage exists to answer rather than a result.

```
src/kinetra/robot/
    limits.py        ceilings, the velocity profile, the readiness gate, the abort predicates
    stages.py        the stage catalogue, the schedule builder, the operator briefing
    motion_log.py    the frozen value objects and the on-disk log
    motion_check.py  the only module that opens a DDS participant
```

The first three import neither ROS nor the Unitree SDK, so the clamping, the ramp, the gate, the abort
criteria and the log format are unit tested without a robot. That split is the point: `capture.py` keeps its
risky logic inside a ROS node and is therefore untested (`SNAPSHOTS.md`), and the code deciding how fast a
robot moves near people should not be in that position.

## The stages

Run them in this order. Each refuses to run until its prerequisite exists, in code rather than in a runbook.

| Stage | What it sends | What it answers |
| --- | --- | --- |
| `handshake` | no motion; three round-trips | Is the command path real? What does this robot look like standing? |
| `yaw` | `+0.30 rad/s`, 2.0 s window, ~26° | Is `+wz` left? What is the stop distance? |
| `yaw-estop` | `+0.12 rad/s`, 5.0 s window, ~33°, through an operator interrupt | Does the remote override a *live commanding process*? |
| `forward` | `0.15 m/s`, 2.0 s window, ~0.22 m | Does it walk and stop where we think? |
| `watchdog-quiet` | stops publishing mid-rotation | Does the robot stop when commands merely cease? |
| `watchdog-kill` | SIGKILLs itself mid-rotation | Does anything robot-side stop a move when we die? |

Three of these are not the obvious design, and the reasons matter.

**`handshake` runs with the robot standing, and it is not motion-free.** `StopMove()` is a command, and what
sport mode does with it on a lying robot in `mode 7` is unverified. Its real deliverable is the **first
standing `foot_force` signature** ever measured on this robot, written to `<out>/standing-signature.json`;
every motion stage refuses to run without it, so a motion stage can never run on an assumed stance. It also calls
`GetServerApiVersion()` **before** `Init()`: api id 1 is below `RPC_INTERNAL_API_ID_MAX`, so it bypasses the
registration table, which separates "cannot reach the robot" from "did not register the API" into two
findings instead of one ambiguous `3103`.

**`yaw-estop` is a separate stage from `yaw`, and it disarms `no_onset`, `overspeed` and
`operator_interrupt`.** Armed, those fire the instant the remote takes over, *we* stop the robot, and we are
then unable to distinguish "the remote stopped it" from "we stopped ourselves" — which is the entire
question. So this stage keeps publishing `Move` through the interrupt until the window ends, and keeps
logging for 3 s afterwards, because the robot **resuming** our command stream when the stick is released is a
distinct and important outcome.

It also rotates **slower and longer than `yaw`** — `0.12 rad/s` over `5.0 s`, against `0.30 rad/s` over
`2.0 s`. The binding constraint is the 0.70 rad rotation cap, not the clock, so the same bounded rotation
spent more slowly buys 4.8 s of steady motion to interrupt instead of 1.5 s, and an interrupt test the
operator cannot comfortably interrupt measures the operator rather than the robot. Either number can be
changed for one run with `--window` and `--yaw-rate`; the caps are not relaxed with them, so asking for a
longer window at an unchanged rate is refused before anything is sent:

```
python3 -m kinetra.robot.motion_check --stage yaw-estop --window 5.5 --yaw-rate 0.11 \
    --operator you --spotter them --dry-run
```

### What `yaw-estop` found, October 15, 2026

Four live runs, operator Leon. Both halves of the result matter, and the second half was the surprise:

- **The remote overrides a live commanding process, totally.** While the stick is held, our `Move` stream
  produces nothing. One run answered a full 5.0 s of commanded rotation with 0.022 rad/s of measured yaw.
- **The override does not latch.** Release the stick and our stream has the robot back inside the same
  window, at the usual ~0.5 of the commanded rate. So an interrupt is a *hold*, not a stop, and it costs a
  human's continuous attention for as long as it lasts (`OPEN_QUESTIONS.md` 8.13).
- **Driving the robot away instead of stopping it aborts the run.** One run was pushed forward at 0.632 m/s
  and the translation cap fired at 0.149 m of the 0.150 m allowance — measured odometry catching what no
  command-side check could, since this stage disarms `overspeed` and `operator_interrupt` on purpose
  (`OPEN_QUESTIONS.md` 8.14).
- **Holding the robot still aborts nothing**, and the run reports `completed`. Read `motion_onset_s` before
  believing a completed run moved: on that run it was 5.609 s, past the end of the window.

The operator consequence: **if you interrupt, keep holding and sit the robot down with the remote.** Letting
go hands the robot back to whatever was commanding it.

**`watchdog-quiet` comes before `watchdog-kill`.** Simply ceasing to publish, while staying alive and fully
able to `StopMove()`, answers most of the receiver-watchdog question at **zero added risk**. If the robot
stops in 0.3 s, the kill variant becomes a confirmation rather than a discovery. `watchdog-kill` then yaws
rather than walks — if the last `Move` persists, a rotation is bounded in displacement where a walk is not —
schedules its own `SIGKILL` so it lands deterministically mid-motion and the operator keeps both hands on the
remote, requires `--estop-evidence` from a `yaw-estop` run, and flushes its log every tick because SIGKILL
runs no handler.

## Running one

```bash
source ~/repos/MMC/setup.sh        # puts the SDK's CycloneDDS on LD_LIBRARY_PATH

PYTHONPATH=src python3 -m kinetra.robot.motion_check --list-stages
PYTHONPATH=src python3 -m kinetra.robot.motion_check --stage yaw --dry-run
PYTHONPATH=src python3 -m kinetra.robot.motion_check --stage yaw \
    --operator <name> --spotter <name> --area lab-cleared-2m --tape-translation 0.21
```

A live run asks you to type the stage name verbatim. There is no `--yes`: a stage name cannot be produced by
a stray keypress or a `yes |` pipe, and a motion command should not be auto-confirmable. A non-TTY stdin is
refused for the same reason.

**A dry run stays connected.** It opens the real participant, subscribes to the real topic, runs the real
readiness gate against live state, runs the real loop, watchdog and abort path at real wall-clock timing, and
diverts only `move` and `stop_move`. That is the only way to validate the most misconfiguration-prone step
and to measure actual loop jitter on this Jetson at `MODE_30W`. `--offline` synthesises state instead and
says plainly that it proves much less; `--sim-motion` makes that synthetic robot track what is commanded, so
the success path is reachable with no robot at all.

Fault injection, so that every abort path runs before hardware does:
`--sim-move-code 3102` (local write failed), `--sim-stop-code 3104`, `--sim-move-latency 0.7` (exercises the
loop-stall watchdog), `--sim-rpc-latency 0.9`.

## Limits

Normative copy in [PROJECT_SPEC.md](PROJECT_SPEC.md); `limits.py` is the executable one.

| | |
| --- | --- |
| Forward | `≤ 0.15 m/s` |
| Yaw | `≤ 0.30 rad/s` |
| Reverse, lateral | **refused** — no CLI flag exists to raise them |
| Commanded window | `≤ 2.0 s`, both ramps included |
| Commanded travel | `≤ 0.35 m` |
| Rate, state staleness | 20 Hz, 0.30 s |
| Abort terminal state | `StopMove()`; the robot stays standing |

A value above a ceiling is **refused, not clamped**. Clamping would let an operator ask for 0.4 m/s, receive
0.15, and read a success message. Raising one means editing `limits.py` and this document, which is
deliberate.

## Reading a run back

One directory per run under `/mnt/data/kinetra/motion/`, named `20261014T153118_902_yaw`, holding
`motion.json`. It records the stage, the limits, the full planned schedule and its integrated bound, the
readiness verdict with everything observed while deciding, state before/after/settled plus a decimated trace,
every command, the measurements, the abort reason, the clock report, and who was in the room. **A refused run
writes the same file** — a refusal is evidence.

```python
from kinetra.robot import load_motion_log
log = load_motion_log("/mnt/data/kinetra/motion/20261014T153118_902_yaw")
log["result"]["measurement"]["motion_onset_s"]   # None means the robot never moved
```

### Why `Move()` returning 0 proves nothing

`SportClient.Move` is `_CallNoReply` (`rpc/client_base.py:49-56`): it returns `0` when the **local** DDS write
succeeded and nothing more. It is not evidence the robot received, accepted or acted on the command. Every
command record therefore carries a `proves` string saying so, `moves_written` is labelled as writes rather
than acknowledgements, and the actual evidence is `motion_onset_s`. A run where every write succeeded and the
robot never moved exits **5, inconclusive** — without that, such a run exits 0 and gets written up as a
success.

The blocking calls do return robot-side codes: `StopMove`, `GetServerApiVersion`, `CheckMode`. An undocumented
nonzero code is treated as the most dangerous case, not the most convenient one.

Independent confirmation, from a second terminal, is the only way to see our command on the wire:

```bash
ros2 topic echo /api/sport/request     # our Move appears as api_id 1008
ros2 topic echo /api/sport/response    # robot-side status.code, keyed by identity.id
```

## Exit codes

`0` completed or dry run · `1` no state within the timeout · `2` refused (ceiling, cap, gate, operator
declined, missing operator/spotter, non-TTY) · `3` aborted mid-stage · `4` SDK or transport failure · `5`
inconclusive: motion commanded, every write succeeded locally, no motion ever observed.

## Known limits

The project-wide register is [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md). These are the ones that affect anyone
*running* a stage.

- **`motion_check.py` must never share a process with `rclpy`.** Both claim CycloneDDS domain 0, and the
  second `dds_create_domain` fails. This is why state comes from the SDK's `ChannelSubscriber` and not from
  ROS, and why `capture` and `motion_check` cannot be combined.
- **Standing is decided by the foot-force TOTAL, and that signal drifts.** Measured 2026-10-14: lying totals
  24-49, standing 117-141, and the standing reading decays over an hour of use without a fresh stand
  resetting it, while `body_height` does not move — so it is probably sensor drift, not posture. The gate is
  therefore the **total (≥ 90)**, with a per-foot floor of **8** that only catches a foot bearing nothing;
  the lightest foot is useless for standing-vs-lying, since worst standing (16) overlaps worst lying (17).
  Both floors are overridable (`--min-foot-force`, `--min-foot-force-sum`) because their basis moves, and
  both are logged. If a standing total under ~100 shows up, fix the basis rather than lowering the floor —
  see `OPEN_QUESTIONS.md` 8.3 for the relative design that should replace it.
- **`mode` and `body_height` are recorded and never asserted.** `body_height` read 0.32 while the robot lay
  with unloaded feet. `mode` is worse than useless as a gate: it read 7 lying, and **both 6 and 1 while
  physically standing**, changing between them without the robot moving. There is no sport-mode enumeration
  anywhere in the SDK or the ROS packages.
- **Integrated travel is an upper bound, not a prediction.** It assumes the robot tracks every command
  perfectly and instantly, which is what a travel cap wants and is not what will happen.
- **Odometry is the thing under test, not the ruler.** `position` and `velocity` come from the same onboard
  estimator, which reports an all-zero covariance and whose drift has never been measured. Each motion stage
  takes `--tape-translation` and reports odometry, tape and their ratio. At a bring-up the tape is ground
  truth.
- **`Move()` can block for the whole `SetTimeout`.** The writer waits for a matched subscriber in 0.1 s steps
  before writing, so an unreachable robot turns a sub-millisecond call into a timeout-long one. The command
  loop therefore runs at `SetTimeout` 0.2 s, and the watchdog lives in its own non-daemon thread so that a
  loop stuck inside an SDK call cannot delay its own remedy.
- **`StopMove` returning 0 is an acknowledgement, not a stop.** Measured 2026-10-14: acknowledged in 1.4 ms,
  and the robot kept moving for a further **0.82 s**. Clearance must allow for post-stop travel, and
  `settle_time_s` / `stop_translation_m` are measured from the last command rather than from the
  acknowledgement.
- **`error_code` is a status field.** It goes `0 -> 2` on entering locomotion and stays there for as long as
  the robot walks, clearing ~0.6 s after it settles. Code 2 therefore never aborts and is recorded in
  `notes.error_codes_seen`; an undocumented code still aborts after a 1.0 s dwell.
- **A commanded rotation is a request, not a displacement.** Measured twice: ratio 0.52 and 0.55, the second
  corroborated by an operator's independent ~15 deg reading against the odometry's 14.1 deg. The peak yaw
  rate meanwhile *overshoots* (0.370 against 0.30 rad/s commanded). Do not treat a bounded action as the
  motion executed -- see `OPEN_QUESTIONS.md` 8.11.
- **`Move(0, 0, 0)` is not a no-op**: it enters locomotion and starts the gait 23 ms later. `StopMove` is
  what leaves velocity tracking, and it must always follow. Onset is therefore timed from the first `Move`
  of any magnitude.
- **Nothing stops this robot but the operator.** `obstaclesAvoidSwitch` reports `true` while the L1 lidar
  feeding it is dead, and `range_obstacle` reads a flat `2.0`. Neither is used as proximity data.
- **`/tmp/cdds.LOG` cannot be disabled** when an interface is named: the SDK passes its own inline XML to
  `dds_create_domain`. It grew to 13 KB in one connected dry run. Keep an eye on `/`, which has 16 GB free.
- **The SDK prints `[Reader] take sample error` on teardown.** Observed on every connected run, harmless so
  far, and not ours to fix.
- **`motion.json` is not covered by `.gitignore` by name alone**, so the default output root living on
  `/mnt/data` is what keeps a run out of a public repository. `motion/` and `**/motion.json` are now ignored
  as well.
