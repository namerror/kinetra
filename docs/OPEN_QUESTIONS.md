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

**1.2 The motion path has never run on hardware — by anyone. [BLOCKING]**
*Untested.* `/home/jetson/repos/robot_control`'s own README states the real `unitree_sdk2py` calls and the
RealSense capture were only exercised in `--dry-run` on a laptop. The first commanded move will be that path's
first real test, so it must be treated as a bring-up, not a routine step.
*Closes with:* a single short move in a cleared area with a spotter, directions confirmed (+vx forward,
+vy left, +wz left), and a measured stop distance.

**1.3 Obstacle avoidance is switched on, but its main sensor is dead. [BLOCKING]**
*Measured.* `/multiplestate` (1 Hz) reports `obstaclesAvoidSwitch: true`, while the L1 lidar produces no point
clouds at all (§2.1). This is more dangerous than avoidance being off: the flag invites the belief that
something is watching for obstacles when the sensor feeding it is silent. It is also unknown whether the
feature gates `SportClient.Move` at all, or only the app's own joystick path.
*Closes with:* deciding explicitly not to rely on it, and separately testing whether a commanded move toward a
real obstacle is modified. Until then, assume **nothing** stops the robot but the operator.

**1.4 Kinetra has no agreed motion limits. [BLOCKING]**
*Open decision.* The bridge's defaults (0.4 m/s forward, 0.1 back, 0.05 lateral, 0.6 rad/s yaw, 0.5 s stale
timeout) are another project's settings, not ours. Maximum move length, safety perimeter, and speed for the
demo area are undecided.
*Closes with:* a written limit set in `docs/PROJECT_SPEC.md` before the first trial.

**1.5 `mode 7` in `/sportmodestate` is undecoded. [UNKNOWN]**
*Measured but not interpreted.* Foot forces were 9–17 (essentially unloaded), so the robot was not standing on
its feet. We cannot currently tell "standing and ready" from "lying down" or "damping" in a log.
*Closes with:* reading the SDK's mode enumeration, or observing the field across known physical states.

**1.6 Arbitration with the robot's own controllers is unknown. [UNKNOWN]**
*Measured.* `/api/sport/request` already has **12 publishers**, all internal Go2 services. Nothing is known
about priority, last-writer-wins, or whether our commands can be overridden mid-move.
*Closes with:* observing the topic while the robot is commanded, and reading the motion-switcher behaviour.

**1.7 No receiver-side watchdog is confirmed. [UNKNOWN]**
*Untested.* The bridge stops on its own *sender* side when commands go stale. Whether anything on the **robot**
stops if the Jetson process is killed mid-move, or the network drops, has not been shown.
*Closes with:* killing the sender mid-move with a spotter present and measuring what happens.

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
