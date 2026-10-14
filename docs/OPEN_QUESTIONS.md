# Open questions, limitations, and missing information

**Last updated: October 7, 2026.** This is the authoritative register of what Kinetra does *not* know or
cannot yet do. `docs/ROBOT_INVENTORY.md` records what was measured; this file records what is missing, why it
matters, and what would close it.

Tags: **[BLOCKING]** stops other work · **[RISK]** could invalidate the demo · **[LIMIT]** known and accepted,
documented so nobody is surprised · **[UNKNOWN]** not yet looked at.

Every claim below is marked *measured*, *assumed*, or *untested*. Nothing here may be upgraded to "verified"
without a live check — a catalogue entry, a model card, or a config flag is not proof (`AGENTS.md`).

---

## 1. Before the robot moves at all

**1.1 No operator emergency stop has been demonstrated. [BLOCKING]**
*Untested.* The pre-existing bridge's documentation says the Unitree handheld remote overrides the SDK, but
nobody has shown that on this robot. `/wirelesscontroller` at least makes remote input observable.
*Closes with:* a spotter holding the remote, interrupting a commanded move, and the stop being measured.
*Progress, October 14:* `kinetra.robot.motion_check --stage yaw-estop` is built for exactly this and
disarms the predicates that would otherwise make *us* stop the robot during the interrupt -- armed, they fire
the moment the remote takes over and the run can no longer distinguish "the remote stopped it" from "we
stopped ourselves". The stage has not been run.

**1.2 The motion path has never run on hardware — by anyone. [BLOCKING]**
*Untested.* `/home/jetson/repos/robot_control`'s own README states the real `unitree_sdk2py` calls and the
RealSense capture were only exercised in `--dry-run` on a laptop. The first commanded move will be that path's
first real test, so it must be treated as a bring-up, not a routine step.
*Closes with:* a single short move in a cleared area with a spotter, directions confirmed (+vx forward,
+vy left, +wz left), and a measured stop distance.
*Progress, October 14 -- the RPC path is verified; motion is not.* `kinetra.robot.motion_check` exists, with
174 unit tests over the ceilings, the profile, the gate and the abort predicates.

`--stage handshake` has now **run live against the robot**, and all three round-trips were answered by the
robot with code 0:

| call | code | what that establishes |
| --- | --- | --- |
| `SportClient.GetServerApiVersion` | 0 | the robot is reachable on `eno1`, *before* `Init()` and so independent of API registration |
| `MotionSwitcherClient.CheckMode` | 0 | its motion mode is readable and reports normal sport mode |
| `SportClient.StopMove` | 0 | the **registered** command path works: the robot received a command of ours and answered it |

This is the first time any code in this project has commanded this robot, and it is the first evidence the
`unitree_sdk2py` path works on this hardware at all -- the lab bridge's README says its SDK calls had only
ever run in `--dry-run` on a laptop.

*Progress, October 14 -- the robot moves on our command.* Run `20261014T172046_078_yaw` sent the first
velocity command in this project's history. The robot entered `mode 3` within 5 ms and began a trot, which
`foot_force` shows unambiguously as alternating diagonal pairs. So `Move` reaches the robot and moves its
legs.

*A bounded rotation has now been executed, October 14.* Run `20261014T174008_123_yaw` **completed**: 41
ticks, no abort, stop confirmed. Results:

| | |
| --- | --- |
| direction | `+wz` produced **+0.246 rad of positive yaw** -- counter-clockwise, i.e. left, as the lab bridge's docs claim. Confirmed on two runs. |
| achieved vs commanded | 14.1° against 25.7° commanded, ratio 0.55, corroborated by the operator's ~15° eyeball (section 8.11) |
| actuation latency | 23 ms from the first `Move` to observable motion (section 8.12) |
| stop | `StopMove` acknowledged in 1.7 ms; motion continued **0.503 s**, settled 0.506 s after the last command |
| stayed in place | net translation +0.013 m, though the path length was 0.132 m -- it wandered and came back |
| body attitude | max tilt 0.029 rad throughout |

**So the `+wz` half of this entry is closed.** The remaining criteria are `+vx` forward, which needs the
`forward` stage, and a stop distance measured from *translation* rather than rotation. `+vy` is refused
outright, so it will never be confirmed.

One number worth carrying forward: the total foot force fell to **50** during the gait, well under the
pre-arm gate of 90. The readiness gate is evaluated only before arming, which is correct -- a gate applied
during locomotion would refuse a walking robot every cycle.

## 2. Sensing

**2.1 The L1 lidar produces no data. [BLOCKING]**
*Measured.* `error_state: 6`, `cloud_frequency 0.0`, `cloud_size 0`, and zero messages on
`/utlidar/cloud_deskewed`, `/utlidar/height_map_array`, `/utlidar/imu`. Height map, voxel map, `/uslam/*`,
`/lio_sam_ros2/mapping/odometry`, and Unitree's topological-graph interface (`/qt_*`) must all be assumed
unavailable. `error_state: 6` is not decoded.
*Closes with:* a time-boxed triage — `/utlidar/switch`, power, cabling, a robot power cycle — then either a
working lidar or a written decision to run D455-only. Do not spend more than an hour before deciding.

**2.2 Nothing relates the camera to the robot body. [BLOCKING]**
*Measured.* `/tf_static` carries only intra-camera frames. No URDF, no robot state publisher, no
camera-to-`base_link` transform anywhere. So a depth point can be computed in the camera optical frame but
**cannot be placed in the robot or odometry frame** — which blocks place memory, candidate locations, and any
claim about where a person is.
*Closes with:* measuring the D455's mount pose and height by hand and publishing a static transform.

**2.3 Depth on dark clothing is the single biggest unexamined risk. [RISK]**
*Untested, and specific to this project.* The headline demo is a person **wearing black, lying on the floor**.
The D455 derives depth from stereo assisted by an IR projector (`depth_module.emitter_enabled = 1`), and dark,
IR-absorbing fabric is exactly the case where stereo depth degrades. A lying person is also seen at a
**grazing angle** from robot height, which degrades it further. If depth over the target is mostly dropouts,
range-to-candidate and the "go closer and verify" step both weaken.
*Closes with:* the visibility set — a dark-clothed target lying at 1/2/3/4 m, `depth_stats()` and the valid
fraction *over the target region specifically*, not over the whole frame.

**2.4 Colour exposure on a dark target is untested. [RISK]**
*Measured config, untested effect.* Auto-exposure is on for both streams (`rgb_camera.exposure 166`,
`depth_module.exposure 8500`). A dark-clothed person against a bright floor may be under-exposed into a
silhouette, which is a poor input to an attribute-verifying vision model. The survey's single frame
(mean 108, std 44) says nothing about the demo area's lighting.
*Closes with:* the same visibility set, inspected for exposure, plus a check of whether fixed exposure helps.

**2.5 Resolution may be too low to recognise a person at range. [RISK]**
*Measured.* Both streams run 640×480 at 30 Hz, as set by the launch file; the D455 supports more. At 4 m a
lying person occupies a modest pixel area, and hosted vision models generally do better with more detail.
*Closes with:* testing recognition at the working distances before fixing the profile; raise resolution for
stopped snapshots if it helps, since 30 Hz is not needed when the robot is stationary.

**2.6 Only one of the two configured cameras exists. [LIMIT]**
*Measured.* The launch file starts `D455_1` (serial `213622301134`) and `D455_2` (`213622300726`), but only
`213622300726` is physically attached. `/camera/D455_1` is a live node publishing nothing, which is a trap for
anyone who subscribes to it and waits.
*Closes with:* attaching the second camera or reducing the launch file to one.

**2.7 `range_obstacle` may not be a working sensor. [UNKNOWN]**
*Measured.* It read a flat `[2.0, 2.0, 2.0, 2.0]` — saturated or placeholder. It must not be used as proximity
data until it has been seen to change in response to a real obstacle.

**2.8 Odometry reports no uncertainty, and its drift is unmeasured. [LIMIT]**
*Measured.* `/utlidar/robot_odom` publishes at 245 Hz with an **all-zero covariance**. Drift over a short move
has never been measured, which is the number place memory and revisit detection depend on.
*Closes with:* driving a known short loop and comparing start and end poses.

**2.9 Unsampled or idle sensors. [UNKNOWN]**
`/frontvideostream` (the Go2's built-in front camera) has never been sampled — format, rate, and quality
unknown, though it is a plausible second view. A VectorNav IMU workspace is built and an FT232 USB-serial
adapter is present, but no node runs and no `/vectornav` topic exists.

## 3. Timing

**3.1 The robot clock offset drifts at ~90 ppm and must never be hardcoded. [LIMIT]**
*Measured.* Go2 stamps run ≈2628.6 s behind the Jetson, and that offset **moves**. Sampled in 30 s buckets
over 5.5 minutes:

```
t=30s   2628.5886      t=210s  2628.5992      t=330s  2628.6156
t=120s  2628.5996      t=270s  2628.6070
```

Net **+27 ms over 300 s — about 90 ppm, extrapolating to +0.32 s per hour** (the last two minutes were steeper,
nearer 140 ppm). So this is a genuine relative clock drift, not a constant with noise. A value measured at the
start of a mission would be ~5 ms wrong after a minute and ~300 ms wrong after an hour, which is larger than
the pose-pairing limit.

This is why `clock.py` re-measures continuously over a rolling 256-sample window rather than storing a
constant. At 245 Hz that window spans ~1 s, over which 90 ppm of drift contributes ~0.1 ms — far below the
4–5 ms of latency jitter already present, so the estimate tracks the drift for free. **Never write 2628.6 into
code, and never cache the offset across a mission.**

*Residual risk:* the estimator does not expire samples by age, so if odometry stalled for a long time the
offset would go stale at 0.32 s/hour. In practice a stall also stops the pose buffer, so the snapshot's own
`capture_offset_s` check catches it first.

**3.2 Camera header stamps are not a wall-clock reference, and they jump. [LIMIT]**
*Measured.* RealSense colour stamps run **ahead** of local arrival. Within any one measurement the gap is
steady and oscillates by a few milliseconds (−0.170 to −0.179 s across 5.5 minutes, +10 ppm net), but it also
**steps between measurements**: it was −0.084 s at 17:33 and −0.174 s twenty-five minutes later, a ~90 ms jump
that looks like a driver time-remapping event rather than smooth drift.

This is harmless here only because `captured_at` uses local **arrival** time, and colour and depth are paired
against *each other*, where their stamps agree exactly. Anyone comparing a camera header stamp to wall-clock
time will be wrong by ~0.1–0.2 s, and by a different amount after a re-sync.

**3.3 Timing across reboots and full missions is still unmeasured. [UNKNOWN]**
The drift above was measured over 5.5 minutes on a stationary robot. Unmeasured: whether the rate holds over a
full mission, how the offset changes across a robot power cycle (it will differ, which is why it is never
cached), whether the camera's stamp jumps are periodic, and whether any of this shifts once the robot is
walking and the CPU is loaded.

*Observed October 14, 2026, and a warning about how NOT to measure this:* the offset read 2687.29 s at 10:40
and 2688.88 s at 15:03. That pair is **not** a drift measurement, because the Jetson rebooted between them
and its system clock also stepped (`last -x reboot` records the boot at 11:18 while `uptime` implies 14:45).
A reboot or an NTP correction on either side moves the offset discontinuously, so any two readings separated
by one are incomparable. The same applies to comparing the October 7 figure (2628.60 s) with any later one.
Drift can only be measured within a single uninterrupted window, which is what `clock.py` does over its
rolling window and why the offset is never cached across a mission.

## 4. Snapshot capture — known limits

Also summarised in `docs/SNAPSHOTS.md` for API users.

**4.1 `capture.py` has no automated test. [LIMIT]** It needs ROS 2 and a live camera, so only the ROS-free
half (`snapshot`, `clock`) is covered by the 24 unit tests. The node has been exercised by hand against the
live camera, including its failure paths, but nothing guards it against regression.
*Closes with:* a recorded-bag or fake-publisher test, once there is a bag worth replaying.

**4.2 Depth scale is 1 mm per count by assumption.** `depth_module.depth_units` is unset, so librealsense's
default applies, and the observed median agrees. Recorded per snapshot as `depth_scale_m`, overridable with
`--depth-scale`, so a configuration change cannot silently rescale stored observations.

**4.3 Distortion is not undone** in `deproject`. The colour stream is nearly rectified (|k1| ≈ 0.055) and depth
is used for coarse judgements well inside the pose uncertainty it is combined with. Revisit if depth is ever
used for anything fine-grained.

**4.4 `valid_fraction` is rounded to four decimals,** so a frame with a handful of bad pixels reports `1.0`.
Irrelevant at the ~82 % valid rates actually observed, but it is not a strict "no dropouts" guarantee.

**4.5 Single view, no point cloud, no undistortion.** A snapshot is one camera's RGB-D pair. Multi-view fusion,
point-cloud export, and rectified output are all absent by choice, not by accident.

**4.6 `--out` must stay off the repository tree.** `/` had 16 GB free; `/mnt/data` has 1.5 TB. `.gitignore`
blocks snapshot files, but the real constraint is that identifiable images stay out of the public repository
unless release is approved (`docs/HACKATHON.md`).

## 5. Models and Nebius — nothing is verified

**5.1 No Nebius access has ever been exercised. [BLOCKING]**
*Measured only that the endpoint is reachable* (`https://api.studio.nebius.com/v1/models` → 401). No
credentials are configured, no `.env` exists, and the `openai` package is not installed. Unknown: whether
`nvidia/Nemotron-3_5-Lightning` is available to this account, its latency, whether it honours structured
output, and its decision quality on real place-memory state.
*Closes with:* a key, a client, a model listing, and one real text inference with recorded latency and format.
This also satisfies the hackathon's qualifying-technology requirement, so it cannot slip.

**5.2 Whether *any* available model can tell a lying person in black from a standing one is unverified — and
it is the project's core premise. [RISK]**
*Untested.* This is a **model** question, not just a camera question, and the two must be tested together. No
hosted vision model has been confirmed to accept images on this account, let alone to distinguish the target
attributes. A person detector trained on standing pedestrians should not be assumed to recognise a lying one.
*Closes with:* real snapshots from §2.3 sent to candidate models, scoring attribute discrimination and the
standing-distractor rejection specifically.

**5.3 The vision model is undecided. [UNKNOWN]** Nebius's public workbench defaults to `MiniMaxAI/MiniMax-M3`;
`nvidia/Cosmos-Reason2-2B` on Nebius AI Cloud is the NVIDIA alternative, with a documented 24 GB GPU minimum
and unknown serving cost and integration time. A retirement notice already removed one previously announced
model, so availability must be checked live.

**5.4 Local inference feasibility is untested. [UNKNOWN]** The Jetson runs at `MODE_30W`, not maximum
performance, and `torch` 2.8.0 is installed. No model has been benchmarked on it, and no decision has been made
about what runs locally versus hosted.

## 6. Evaluation

**6.1 Nothing is implemented. [BLOCKING for the submission]** No trial definitions, no metrics code, no
fixed-order baseline, no logging schema beyond the snapshot format. `docs/EVALUATION.md` is a plan.

**6.2 Trial count and matching are undecided. [UNKNOWN]** How many matched runs, how the area is rearranged
between them, and what counts as a confirmed find versus a false confirmation are all unsettled. Absent-target
trials are required by the spec but undesigned.

**6.3 Battery limits the trial budget, and that budget is unmeasured. [UNKNOWN]** 46 % charge with 5 cycles on
the pack at survey time, discharging at 656 mA. Runtime per trial has never been measured, so a matched trial
set cannot yet be planned. One IMU reported 79 °C, which is unexplained and worth watching.

## 7. Platform and process

**7.1 The exact Go2 variant and firmware are unidentified. [UNKNOWN]** `/lowstate` reports `sn: [0, 0]` and
`version: [0, 0]`; the only version strings found were the lidar's (`1.0.0.39`) and the BMS's (1.16). Whether
this is a Go2 EDU — which determines what low-level control is permitted at all — is not established from the
robot itself.

**7.2 The ROS environment is not reproducible from a fresh shell. [LIMIT]** `~/.bashrc` does not source ROS
(the line is commented out; there is an `sr` alias). The working environment came from interactive sourcing, so
any service, cron job, or systemd unit will need it made explicit.

**7.3 `CYCLONEDDS_URI` pins no interface. [UNKNOWN]** It is set but its `<Interfaces>` block is empty. Go2
traffic arrives on `eno1` while the LAN is on wireless, so this should be understood before adding our own DDS
participants.

**7.4 The public-release path for robot control is undecided. [BLOCKING for the submission]** The existing
bridge is lab code outside the release boundary and must not be copied in. Kinetra therefore needs either its
own minimal public adapter or an approved documented interface to the private one. Either way the public
repository must stand on its own through mock or public components.

**7.5 Twenty-three days remain and nothing runs end to end. [RISK]** Deadline is October 30, 2026, 10:00 PDT.
The submission needs a working physical demo, a public repository, a video under three minutes containing at
least one minute of real Go2 operation, and service feedback. Demo and video preparation must not be left to
the final days.

---

## What I would close first

1. **§5.1 + §5.2 together** — a key, a client, and the visibility set through a real vision model. Pure
   software plus a stationary robot, and it is the premise everything else rests on.
2. **§2.1 lidar triage, time-boxed to an hour**, then a written decision either way.
3. **§2.2 camera transform** — a tape measure and a static transform publisher.
4. **§1.1–1.4 safety** — before any autonomy, in a cleared area with a spotter.

## 8. Bounded motion — known limits

Added October 14, 2026, with `src/kinetra/robot/`. The operator-facing version is in
[MOTION.md](MOTION.md).

**8.1 `Move()` returning 0 is not evidence of anything on the robot. [LIMIT]**
*Measured in source.* `SportClient.Move` is `_CallNoReply` (`rpc/client_base.py:49-56`): it returns 0 when
the **local** DDS write succeeded, 3102 if it failed, 3103 if `Init()` was skipped. It is never an
acknowledgement. Every logged command therefore carries a `proves` string, `moves_written` is labelled as
writes, and a run where every write succeeded and no motion was observed exits **5, inconclusive** rather
than 0. The actual evidence is `motion_onset_s`.

**8.2 `Move()` can block for the whole `SetTimeout`. [LIMIT]**
*Measured in source.* `core/channel.py:130-141` waits for a matched subscriber in 0.1 s steps before writing,
so an unreachable robot turns a sub-millisecond call into a timeout-long one — and a blocking round-trip
costs up to twice its timeout. The command loop therefore uses 0.2 s, shutdown 0.5 s, handshake 1.0 s, and
the watchdog runs in its own non-daemon thread so a loop stuck inside an SDK call cannot delay its own
remedy. Per-call durations are recorded in every log.

*Measured October 14, 2026, by a connected dry run of the `yaw` stage* (run `20261014T170831_718_yaw`, with
the real subscriber at 500 Hz and the real watchdog thread running, but `move`/`stop_move` diverted):

| | |
| --- | --- |
| tick interval | median 50.11 ms against a 50.00 ms target, mean 50.27, max 54.63 |
| jitter | stdev 0.72 ms, worst deviation 4.63 ms |
| ticks over 55 ms | none, across 41 ticks |
| window span | 2.011 s for a 2.0 s window |
| state samples buffered | 3012 |

So the loop holds its period comfortably on this Jetson at `MODE_30W`, and the worst observed deviation is
~1% of the watchdog's 0.5 s stall threshold. **The per-call costs from that run are not the real ones** —
they are the dry-run backend's 6-29 us, where a live `Move` adds an actual DDS write. The 50 ms budget has
room for it, but the live figure is still unmeasured.

**8.3 Foot force is the only usable standing signal, and it drifts. [LIMIT]**
*Measured October 14, 2026.* Readings across the afternoon, with physical state known because the robot was
stood and sat by hand with the remote:

| state | lightest foot | total |
| --- | --- | --- |
| lying | 0–17 | 24–49 |
| standing | 16–30 | 117–141 |

**The standing reading decays, and a fresh stand does not reset it.** The lightest foot went 29, 30, 28, 25,
22, 19, 16 and the total 141 down to 117 over roughly an hour, while `body_height` held at exactly 0.320
throughout. Load that falls while the geometry does not move is not a stance change, so this is very probably
foot-force **sensor drift**, plausibly thermal — the IMU read 79 °C at the survey. Re-standing was tried and
the decay continued, which rules out stance relaxation.

Consequences, and the current design:

- **The lightest foot cannot tell the states apart.** Worst standing 16 against worst lying 17: they overlap.
  So the per-foot floor is set to **8**, deliberately *below* every lying reading, and demoted to the only
  job it can still do — catching a foot bearing nothing at all.
- **The total still separates them cleanly**, 117 standing against 49 lying. It is therefore the whole
  standing decision, floored at **90**: 1.8× above the worst lying reading, and leaving about 27 counts of
  room for further drift.
- Both floors are operator-overridable (`--min-foot-force`, `--min-foot-force-sum`) precisely because their
  basis is known to move, and both are recorded in every log.

Three earlier attempts at this threshold were wrong, and the sequence is worth remembering:

- A guessed **50 per foot** refused a genuinely standing robot — standing forces here are far below a naive
  weight-over-four-feet estimate.
- **Half the lightest standing foot**, derived from a signature, gave 14 — *below* a lying reading, so it
  would have passed a lying robot.
- **Deriving a per-foot floor from a stance snapshot at all** was the wrong question: fitted to a good stance
  it refuses an adequate one, and a lifted leg reads near zero whatever the stance.

*Closes with:* understanding the drift. The proper fix is relative rather than absolute — compare a standing
total against a lying baseline measured minutes earlier, since both drift together and the ~70–100 count
*difference* is what actually carries the information. Deferred deliberately: the absolute floor of 90 is
adequate for bring-up, and chasing the number further would narrow the band toward lying. **If a standing
total below ~100 appears, fix the basis rather than lowering the floor again.**

**8.9 `StopMove` returning 0 is an acknowledgement, not a stop. [LIMIT]**
*Measured October 14, 2026* (run `20261014T172046_078_yaw`). `StopMove` returned code 0 in 1.4 ms, and the
robot kept moving for a further **0.82 s**, peaking at 0.25 rad/s of body yaw after the stop had been
acknowledged. At the forward stage's 0.15 m/s that is roughly 12 cm of travel past the point where the log
says the robot was told to stop.

Caveat on the number: the robot was mid-gait-cycle, so part of the 0.82 s is the trot finishing rather than
deceleration. A second run measured **0.483 s** after `StopMove` (settle 0.485 s from the last command), from
a longer 1.05 s of commanded rotation. So the figure varies with where in the gait cycle the stop lands, and
both numbers are from interrupted runs rather than from steady motion. Treat 0.8 s as the planning figure
until it is measured properly.

Consequences: clearance for a motion stage must allow for post-stop travel rather than commanded travel, and
no log should be read as "stopped at the time of the `StopMove` record". The `settle_time_s` and
`stop_translation_m` fields exist for exactly this and are measured from the last command, not from the
acknowledgement.

**8.10 `error_code` is a status field, not only a fault field. [LIMIT]**
*Measured over two runs, October 14, 2026.* `error_code` goes `0 -> 2` within 5 ms of the first `Move`,
exactly as `mode` goes `1 -> 3` and the robot enters a trot, and it **stays at 2 for as long as the robot
keeps walking** -- 829 consecutive samples with no break in run `20261014T173400_387_yaw` -- clearing about
0.6 s after `StopMove`, once the robot settles. So `2` means "walking".

Two wrong fixes were tried before that was understood, and the sequence is the useful part:

- Aborting on **any change from baseline** fired 5 ms into the run, before the ramp reached a tenth of
  target. No motion stage could complete.
- Adding a **1.0 s dwell** looked right, because the first run showed `2` clearing at 0.656 s. But it had
  cleared only because that run aborted at 0.052 s and the robot stopped. With the robot actually walking,
  `2` persists for the whole stage, so the dwell merely moved the spurious abort to 1.0 s. Run
  `20261014T173400_387_yaw` aborted that way after 22 ticks.

Code `2` now never aborts, however long it persists; it is recorded in `notes.error_codes_seen` and listed in
`BENIGN_ERROR_CODES`. An **undocumented** code still aborts on the dwell, which is the case the predicate
exists for.

*Residual:* `error_code` has no enumeration, so `2` is explained by correlation with `mode 3` alone. No code
other than 0 and 2 has ever been observed on this robot.

**8.11 The robot delivers about half the commanded rotation. [RISK]**
*Measured twice, with an external check, October 14, 2026.*

| run | ticks | commanded | achieved (odometry) | ratio |
| --- | --- | --- | --- | --- |
| `20261014T173400_387_yaw` | 23 | +0.248 rad (14.2°) | +0.129 rad (7.4°) | 0.52 |
| `20261014T174008_123_yaw` | 41 | +0.449 rad (25.7°) | +0.246 rad (14.1°) | **0.55** |

**The odometry is not the problem.** On the completed run the operator independently eyeballed **~15°**
against the odometry's 14.1°, agreeing to within about 6%. So the shortfall is real tracking behaviour, not a
measurement artefact -- which is what made this worth upgrading from a suspicion to a finding, and is why the
external reading was asked for.

Meanwhile the peak yaw *rate* **overshoots**: 0.370 rad/s against a commanded maximum of 0.30, in both runs.
So the robot exceeds the commanded rate instantaneously while falling short of the commanded displacement,
which is consistent with a trot whose body oscillates while its heading advances slowly.

*Why it matters:* a planner that treats a commanded bounded action as the motion executed will be wrong by
about a factor of two on rotation, and in the optimistic direction. A commanded rotation is a request.
*Closes with:* repeating this across several runs and both signs, and establishing whether the ratio is
stable enough to correct for, or whether rotation has to be closed-loop on measured yaw.

**8.12 `Move(0, 0, 0)` is not a no-op: it starts the gait. [LIMIT]**
*Measured October 14, 2026.* Tick 0 of every stage commands exactly zero, and the robot responds to it:
`error_code` goes to 2 and `mode` to 3 within 5 ms, and observable motion begins **23 ms** after that zero
command -- 27 ms *before* the first non-zero tick is even sent. So `Move` with any argument, including zero,
puts the robot into velocity-tracking locomotion and it begins trotting in place.

Two consequences:

- **Motion onset must be timed from the first `Move` of any magnitude.** Timing it from the first non-zero
  tick skipped the command the robot actually reacted to and reported ~1.4 ms, which is not a response
  latency at all. Corrected, onset is **24.5, 22.7 and 22.9 ms** across the three runs -- consistent, and
  the first real actuation latency measured on this robot.
- **`Move(0,0,0)` is a poor stop on its own.** The stop sequence sends it before `StopMove()` for its
  deterministic cost, but it sustains locomotion rather than ending it. `StopMove` is what leaves velocity
  tracking, and it must always follow immediately.

**8.4 `rclpy` and `unitree_sdk2py` cannot share a process. [LIMIT]**
*Measured.* Both claim CycloneDDS domain 0 and the second `dds_create_domain` fails with
`PRECONDITION_NOT_MET`. So `motion_check` is SDK-only and takes state from `ChannelSubscriber`, and
`perception.capture` and `robot.motion_check` must never be combined. The SDK also passes its own inline XML
to `dds_create_domain`, so the `CYCLONEDDS_URI` that pins `eno1` for ROS is **ignored** there — the interface
must be named explicitly or the SDK may bind the wireless NIC and silently never reach the robot.

**8.5 Commanded travel is an upper bound, not a prediction. [LIMIT]**
The pre-send bound integrates the commanded profile assuming perfect instantaneous tracking. That is what a
travel cap wants and is not what the robot will do. The geometric abort caps are checked against measured
odometry instead, precisely because they must not trust the profile.

**8.6 The odometry used to measure a stage is itself unvalidated. [LIMIT]**
*Measured.* `position` and `velocity` come from the same onboard estimator, which reports an all-zero
covariance (section 2.8), so their agreement proves little. Each motion stage takes `--tape-translation` and
reports odometry, tape and their ratio: at a bring-up the tape measure is ground truth and the odometry is
under test.

**8.7 Most of `motion_check.py` is still hardware-only. [LIMIT]**
168 tests cover the ceilings, the clamp, the profile, the readiness gate, the abort predicates, the log
format, the CLI, the dry-run backend and the signature handling — a deliberate improvement on `capture.py`,
which is untestable because its risky logic sits inside a ROS node (section 4). What remains untestable:
participant creation and interface binding, `SportClient`/`MotionSwitcherClient` behaviour, whether the robot
moves at all, whether `+vx`/`+wz` are forward and left, whether the remote overrides us, whether anything
robot-side stops a move after SIGKILL, and whether `Move` is gated by the Go2's own obstacle avoidance. Those
are the findings the stages exist to produce.

**8.8 `/tmp/cdds.LOG` cannot be turned off. [LIMIT]**
*Measured.* The SDK's inline config sets CycloneDDS tracing to a fixed path whenever an interface is named.
It reached 13 KB in a single connected dry run. `/` has 16 GB free, so this wants watching across many runs
but is not otherwise harmful. The SDK also prints `[Reader] take sample error` on teardown.

