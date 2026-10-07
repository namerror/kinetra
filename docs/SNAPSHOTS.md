# Snapshots

A **snapshot** is one stopped RGB-D observation plus the provenance needed to trust it. It is the unit of
observation the rest of Kinetra reasons about, and it is a directory on disk rather than a live stream handle —
so a recorded scene and a live robot look identical to every stage downstream, and a reviewer with no Go2 can
still exercise the system.

```
<out>/20261007T174445_529_lying-target-2m/
    color.png       bgr8, 640x480
    depth.png       16UC1, 640x480, aligned to colour. 0 means NO MEASUREMENT, not zero range
    snapshot.json   intrinsics, pose, clock report, pairing report, depth stats
```

## Capturing

Append to `PYTHONPATH` rather than replacing it — overwriting it removes ROS 2 and `import rclpy` fails.
The ROS-free modules and the tests do not care.

```bash
# Robot stationary. This node only subscribes; it cannot move the robot.
PYTHONPATH=src:$PYTHONPATH python3 -m kinetra.perception.capture --label lying-target-2m --count 5 --interval 2
PYTHONPATH=src:$PYTHONPATH python3 -m kinetra.perception.capture --dry-run   # report, write nothing
```

Snapshots default to `/mnt/data/kinetra/snapshots`. Keep them there: `/` had 16 GB free at the October 7
survey, and identifiable images must stay out of the public repository unless release is approved
(`docs/HACKATHON.md`). `.gitignore` also blocks them from being committed by accident.

Useful flags: `--label` tags a batch and drives the visibility trials; `--require-pose` refuses to save an
observation whose pose is missing or badly paired; `--max-range` discards depth past the D455's usable range;
`--camera` selects the namespace if the working camera ever changes.

## Reading them back, off-robot

`kinetra.perception.snapshot` and `.clock` import no ROS. Only `.capture` does.

```python
from kinetra.perception import load_snapshot

s = load_snapshot("/mnt/data/kinetra/snapshots/20261007T174445_529_lying-target-2m")
s.color_rgb                 # RGB, which is what image models expect
s.depth_m()                 # metres; dropouts and out-of-range pixels are NaN, never 0.0
s.depth_m_at(320, 240)      # patch median, or None if nothing valid nearby
s.point_at(320, 240)        # 3-D point in the camera optical frame, or None
s.depth_stats()             # valid fraction, min/median/p95 — is this frame worth sending?
```

## Why the pairing works the way it does

Two rules, both forced by what the [survey](ROBOT_INVENTORY.md) measured. Neither is cosmetic.

**Colour and aligned depth are paired by header stamp.** One RealSense driver stamps both on the Jetson's
clock, and their stamps were identical to within 0.0 ms across 178 consecutive frames. So this pairing is
exact. Every snapshot records the gap it actually achieved, and a pair further apart than `--pair-tolerance`
(one frame interval) is refused rather than quietly used.

**Robot state is converted into the local timebase first.** The Go2 stamps its messages about **2628.6 s —
44 minutes — behind** the Jetson. Pairing a frame with a pose by raw `header.stamp` would attach a pose from
44 minutes earlier and report no error at all, which is the worst available failure for a system whose output
is "the person is here".

The offset is stable, though: it varied by 4–6 ms across repeated 6 s windows. So rather than discarding stamps
and pairing by arrival time — which would inherit every scheduling and queueing delay — `clock.py` measures the
offset and converts. The estimator takes the **minimum** of `arrival − stamp` over a window, the same one-way
delay trick NTP uses: transport and scheduling can only ever add delay, so the minimum is the closest sample to
the true offset. The spread between minimum and maximum is reported too, because a growing spread is the signal
that the clocks are drifting and a single offset no longer describes them.

With both rules in place, odometry at 245 Hz is buffered and the pose **nearest the frame** is selected rather
than whichever arrived most recently. Measured pairing error: **1–6 ms**, down from ~28 ms when simply taking
the newest pose.

Every snapshot carries all of this in `snapshot.json`, so a doubtful observation can be audited after the fact:

```json
"pose": { "stamp_robot_clock": 1791406856.92, "stamp_local_clock": 1791409485.52,
          "capture_offset_s": 0.0055 },
"pairing": { "color_depth_stamp_gap_s": 0.0, "pose_stamp_converted": true,
             "pose_candidates": 256, "pose_offset_limit_s": 0.1 },
"clock": { "robot_vs_local": { "offset_s": 2628.6013, "spread_s": 0.0049, "trustworthy": true } }
```

A snapshot with a missing, stale, or untrustworthy pose is **still saved** — discarding evidence is worse — but
it is flagged in `pairing.pose_warning` so no later stage can mistake it for a located observation. Use
`--require-pose` when a batch is only useful with reliable poses.

## Known limits

These are the limits that affect anyone *using* snapshots. The full project-wide register, including the risks
to the demo premise, is in [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md).

- **`capture.py` is not covered by automated tests.** It needs ROS 2 and a live camera, so only the ROS-free
  half is tested. It has been exercised by hand against the camera, failure paths included, but nothing guards
  it against regression.
- **Camera header stamps are not a wall-clock reference.** They run ahead of local arrival, and the gap steps
  between driver re-syncs — ~84 ms early at one point in the survey, ~174 ms twenty-five minutes later. This is
  harmless here only because `captured_at` uses local arrival time and colour/depth are paired against each
  other, where their stamps agree exactly. Do not compare a camera header stamp to wall-clock time.
- **The clock offset drifts, measurably.** The Go2's offset moved +27 ms over 5 minutes — about 90 ppm, or
  +0.32 s per hour — so it is re-measured continuously over a ~1 s rolling window rather than stored.
  **Never hardcode 2628.6, and never cache it across a mission.**
- **`valid_fraction` is rounded to four decimals,** so a frame with a few bad pixels reports `1.0`. Not a
  strict "no dropouts" guarantee.

- **Depth scale is 1 mm per count by assumption.** `depth_module.depth_units` is unset on this robot, so
  librealsense's default applies, and the observed median of 4044 counts at room scale agrees. It is recorded
  in every snapshot as `depth_scale_m`, so a configuration change cannot silently rescale stored observations.
  Override with `--depth-scale` if that default ever changes.
- **Distortion is not undone** in `deproject`. The colour stream is nearly rectified (|k1| ≈ 0.055) and depth
  is used for coarse "how far, roughly which way" judgements well inside the pose uncertainty it is combined
  with. Revisit if depth is ever used for anything fine-grained.
- **The pose is in `odom`, and nothing relates the camera to the body.** `/tf_static` carries only
  intra-camera frames; there is no camera-to-`base_link` transform anywhere on this robot. So `point_at`
  returns a point in the *camera optical frame*, and it cannot yet be placed in the robot or odometry frame.
  Measuring the D455 mount pose and publishing that static transform is an open task
  ([OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) §2.2).
- **Odometry covariance is all zeros**, so the Go2 reports no uncertainty. Drift over a short move has not been
  measured. Treat positions as relative and short-range only.
