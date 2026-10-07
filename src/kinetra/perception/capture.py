"""ROS 2 node that turns the live D455 streams into saved RGB-D snapshots.

Subscribe-only by construction: this module creates no publisher and no service
client, so it cannot move the robot or alter its configuration. Motion belongs to
a separate robot-side controller (`docs/ARCHITECTURE.md`).

    python3 -m kinetra.perception.capture --label lying-target-2m --count 5

Two pairing rules, both forced by what the October 7 survey measured
(`docs/ROBOT_INVENTORY.md`):

* **Colour and aligned depth are paired by header stamp.** Both are stamped by the
  same RealSense driver on the Jetson's clock, and the survey found their stamps
  identical to within 0.0 ms across 178 frames, so this pairing is exact rather
  than approximate.
* **Robot state is converted into the local timebase first.** The Go2 stamps its
  messages ~2628.6 s behind the Jetson. Pairing by raw stamp would silently attach
  a pose from 44 minutes earlier, so every pose goes through the measured offset in
  `clock` and arrives with an explicit age.

A snapshot whose pose is missing, stale, or backed by an untrustworthy clock
estimate is still saved -- discarding evidence is worse -- but it is flagged in
its own metadata so that no later stage can mistake it for a located observation.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from kinetra.perception.clock import ClockOffsetEstimator
from kinetra.perception.snapshot import (
    DEFAULT_DEPTH_SCALE_M,
    DEFAULT_MAX_RANGE_M,
    CameraIntrinsics,
    RobotPose,
    Snapshot,
)

DEFAULT_CAMERA = "D455_2"
DEFAULT_ODOM_TOPIC = "/utlidar/robot_odom"
# Snapshots go to the NVMe, never the repository: `/` had 16 GB free at survey
# time, and identifiable images must stay out of the public tree.
DEFAULT_OUTPUT_ROOT = "/mnt/data/kinetra/snapshots"

# Colour and depth stamps were identical in the survey; one frame interval at
# 30 Hz is a generous ceiling that still rejects a genuinely mismatched pair.
DEFAULT_PAIR_TOLERANCE_S = 0.034
# Beyond this the robot may have moved between being photographed and being
# located, so the pose no longer describes where the snapshot was taken. Odometry
# arrives at 245 Hz, so a healthy pairing error is single-digit milliseconds and
# this limit only ever trips on a real stall.
DEFAULT_MAX_POSE_OFFSET_S = 0.1


def _slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def _image_to_array(msg: Image, channels: int, dtype: np.dtype) -> np.ndarray:
    """Convert a `sensor_msgs/Image` without depending on `cv_bridge`.

    `step` is honoured rather than assumed equal to ``width * itemsize`` so that a
    row-padded image is not silently sheared diagonally.
    """
    if msg.is_bigendian:
        raise ValueError("big-endian image data is not supported")
    itemsize = np.dtype(dtype).itemsize
    expected_row = msg.width * channels * itemsize
    if msg.step < expected_row:
        raise ValueError(f"step {msg.step} is too small for {msg.width}px x {channels}ch")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    tight = rows[:, :expected_row].copy()  # copy: the message buffer is read-only
    array = tight.view(dtype).reshape(msg.height, msg.width, channels)
    return array if channels > 1 else array[:, :, 0]


def _stamp_seconds(msg: Any) -> float:
    stamp = msg.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


class SnapshotCapture(Node):
    """Keeps the newest frames and robot state, and pairs them on demand."""

    def __init__(self, camera: str = DEFAULT_CAMERA, odom_topic: str = DEFAULT_ODOM_TOPIC,
                 buffer_frames: int = 15, odom_buffer: int = 256) -> None:
        super().__init__("kinetra_snapshot_capture")

        self._color: deque[tuple[float, float, Image]] = deque(maxlen=buffer_frames)
        self._depth: deque[tuple[float, float, Image]] = deque(maxlen=buffer_frames)
        # Buffered rather than kept as a single latest value: odometry arrives at
        # 245 Hz, so a short history lets a frame be paired with the pose nearest
        # it in time instead of whichever pose happened to be newest at capture.
        self._odom: deque[tuple[float, float, Odometry]] = deque(maxlen=odom_buffer)
        self._intrinsics: CameraIntrinsics | None = None

        # Separate estimators: the camera's offset is a driver artefact (~-84 ms),
        # the robot's is a genuine clock disagreement (~+2628.6 s). Both recorded.
        self.camera_clock = ClockOffsetEstimator()
        self.robot_clock = ClockOffsetEstimator()

        # RELIABLE to match the publishers found by the survey. A BEST_EFFORT
        # subscription would be QoS-incompatible and receive nothing at all.
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        info_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)

        self.color_topic = f"/camera/{camera}/color/image_raw"
        self.depth_topic = f"/camera/{camera}/aligned_depth_to_color/image_raw"
        self.info_topic = f"/camera/{camera}/color/camera_info"
        self.odom_topic = odom_topic

        self.create_subscription(Image, self.color_topic, self._on_color, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, info_qos)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, sensor_qos)

    # -- subscriptions ------------------------------------------------------

    def _on_color(self, msg: Image) -> None:
        arrival, stamp = time.time(), _stamp_seconds(msg)
        self.camera_clock.observe(arrival, stamp)
        self._color.append((stamp, arrival, msg))

    def _on_depth(self, msg: Image) -> None:
        self._depth.append((_stamp_seconds(msg), time.time(), msg))

    def _on_info(self, msg: CameraInfo) -> None:
        self._intrinsics = CameraIntrinsics.from_camera_info(
            k=list(msg.k), d=list(msg.d), width=msg.width, height=msg.height,
            frame_id=msg.header.frame_id, distortion_model=msg.distortion_model,
        )

    def _on_odom(self, msg: Odometry) -> None:
        arrival, stamp = time.time(), _stamp_seconds(msg)
        self.robot_clock.observe(arrival, stamp)
        self._odom.append((stamp, arrival, msg))

    # -- readiness ----------------------------------------------------------

    def missing(self) -> list[str]:
        """Which inputs are still absent, for an actionable error message."""
        gaps = []
        if not self._color:
            gaps.append(self.color_topic)
        if not self._depth:
            gaps.append(self.depth_topic)
        if self._intrinsics is None:
            gaps.append(self.info_topic)
        return gaps

    def wait_until_ready(self, timeout_s: float = 10.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.missing():
                return True
        return False

    def collect(self, duration_s: float) -> None:
        """Spin for a while, so the clock estimators have samples to work with."""
        deadline = time.time() + duration_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    # -- capture ------------------------------------------------------------

    def _pose(self, captured_at: float, max_offset_s: float) -> tuple[RobotPose | None, dict]:
        """Pick the odometry sample nearest the frame, in the local timebase."""
        if not self._odom:
            return None, {"pose_source": None, "pose_warning": f"no message on {self.odom_topic}"}

        # Convert each candidate into local time before comparing. Falling back to
        # arrival time while the offset is still being measured keeps the pairing
        # pessimistic rather than confidently wrong.
        trusted = self.robot_clock.trustworthy

        def local_time(stamp: float, arrival: float) -> float:
            converted = self.robot_clock.to_local(stamp)
            return converted if (converted is not None and trusted) else arrival

        stamp, arrival, msg = min(
            self._odom, key=lambda e: abs(local_time(e[0], e[1]) - captured_at))
        effective = local_time(stamp, arrival)
        offset = captured_at - effective

        warning = None
        if not trusted:
            warning = ("clock offset not yet trustworthy; pairing used arrival time "
                       f"({self.robot_clock.report()})")
        elif abs(offset) > max_offset_s:
            warning = (f"nearest pose is {abs(offset):.3f}s from the frame, above the "
                       f"{max_offset_s:.3f}s limit; the robot may have moved between them")

        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        pose = RobotPose(
            position=(p.x, p.y, p.z),
            orientation_xyzw=(o.x, o.y, o.z, o.w),
            frame_id=msg.header.frame_id,
            child_frame_id=msg.child_frame_id,
            stamp_robot_clock=stamp,
            stamp_local_clock=self.robot_clock.to_local(stamp),
            capture_offset_s=offset,
        )
        info: dict[str, Any] = {
            "pose_source": self.odom_topic,
            "pose_stamp_converted": trusted,
            "pose_candidates": len(self._odom),
            "pose_offset_limit_s": max_offset_s,
        }
        if warning:
            info["pose_warning"] = warning
        return pose, info

    def capture(self, label: str = "", pair_tolerance_s: float = DEFAULT_PAIR_TOLERANCE_S,
                max_pose_offset_s: float = DEFAULT_MAX_POSE_OFFSET_S,
                depth_scale_m: float = DEFAULT_DEPTH_SCALE_M,
                max_range_m: float = DEFAULT_MAX_RANGE_M) -> Snapshot:
        """Pair the newest colour frame with its depth frame and the robot's pose."""
        gaps = self.missing()
        if gaps:
            raise RuntimeError("no data yet on: " + ", ".join(gaps))

        color_stamp, color_arrival, color_msg = self._color[-1]
        depth_stamp, _, depth_msg = min(self._depth, key=lambda e: abs(e[0] - color_stamp))
        gap = abs(depth_stamp - color_stamp)
        if gap > pair_tolerance_s:
            raise RuntimeError(
                f"closest depth frame is {gap * 1e3:.1f} ms from the colour frame, "
                f"above the {pair_tolerance_s * 1e3:.1f} ms tolerance"
            )

        assert self._intrinsics is not None  # guaranteed by missing()
        pose, pose_info = self._pose(color_arrival, max_pose_offset_s)

        taken = datetime.fromtimestamp(color_arrival)
        suffix = f"_{_slug(label)}" if label else ""
        snapshot_id = f"{taken.strftime('%Y%m%dT%H%M%S')}_{taken.microsecond // 1000:03d}{suffix}"

        return Snapshot(
            snapshot_id=snapshot_id,
            captured_at=color_arrival,
            color_bgr=_image_to_array(color_msg, 3, np.dtype(np.uint8)),
            depth_raw=_image_to_array(depth_msg, 1, np.dtype(np.uint16)),
            intrinsics=self._intrinsics,
            pose=pose,
            depth_scale_m=depth_scale_m,
            max_range_m=max_range_m,
            label=label,
            clock={
                "local_epoch_at_capture": color_arrival,
                "color_header_stamp": color_stamp,
                "camera_vs_local": self.camera_clock.report(),
                "robot_vs_local": self.robot_clock.report(),
            },
            pairing={
                "color_depth_stamp_gap_s": round(gap, 6),
                "pair_tolerance_s": pair_tolerance_s,
                "rule": "colour/depth by header stamp (one driver, one clock); "
                        "robot state by measured clock offset",
                **pose_info,
            },
            notes={
                "color_topic": self.color_topic,
                "depth_topic": self.depth_topic,
                "odom_topic": self.odom_topic,
                "color_encoding": color_msg.encoding,
                "depth_encoding": depth_msg.encoding,
            },
        )


def _describe(snapshot: Snapshot, saved_to: Path | None) -> str:
    stats = snapshot.depth_stats()
    pose = snapshot.pose
    if pose is None:
        pose_text = "pose MISSING"
    else:
        x, y, _ = pose.position
        pose_text = (f"pose ({x:+.2f}, {y:+.2f}) yaw {pose.yaw_rad:+.2f} rad, "
                     f"pair err {abs(pose.capture_offset_s) * 1e3:.0f} ms"
                     if pose.capture_offset_s is not None else "pose timing unknown")
    warning = snapshot.pairing.get("pose_warning")
    return "  ".join(filter(None, [
        snapshot.snapshot_id,
        f"depth {stats['valid_fraction'] * 100:.1f}% valid, median {stats['median_m']} m",
        pose_text,
        f"pair gap {snapshot.pairing['color_depth_stamp_gap_s'] * 1e3:.1f} ms",
        f"-> {saved_to}" if saved_to else "(not written)",
        f"\n    WARNING: {warning}" if warning else "",
    ]))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--camera", default=DEFAULT_CAMERA,
                   help="camera namespace that is actually streaming (default: %(default)s)")
    p.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    p.add_argument("--out", default=DEFAULT_OUTPUT_ROOT,
                   help="output root, keep off the repo tree (default: %(default)s)")
    p.add_argument("--label", default="", help='tag for these snapshots, e.g. "lying-target-2m"')
    p.add_argument("--count", type=int, default=1)
    p.add_argument("--interval", type=float, default=2.0, help="seconds between captures")
    p.add_argument("--warmup", type=float, default=2.0,
                   help="seconds spent measuring the clock offset before the first capture")
    p.add_argument("--pair-tolerance", type=float, default=DEFAULT_PAIR_TOLERANCE_S)
    p.add_argument("--max-pose-offset", type=float, default=DEFAULT_MAX_POSE_OFFSET_S,
                   help="reject a pose further than this from the frame (default: %(default)s s)")
    p.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE_M,
                   help="depth beyond this is discarded as noise (default: %(default)s m)")
    p.add_argument("--depth-scale", type=float, default=DEFAULT_DEPTH_SCALE_M)
    p.add_argument("--require-pose", action="store_true",
                   help="fail instead of saving a snapshot with a missing or stale pose")
    p.add_argument("--dry-run", action="store_true", help="capture and report, write nothing")
    p.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for first data")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    rclpy.init()
    node = SnapshotCapture(camera=args.camera, odom_topic=args.odom_topic)
    try:
        if not node.wait_until_ready(args.timeout):
            print(f"No data within {args.timeout:.0f}s on: {', '.join(node.missing())}",
                  file=sys.stderr)
            print("Is the camera launch running, and is the ROS environment sourced?",
                  file=sys.stderr)
            return 1

        # The clock offset needs samples before any pose pairing means anything.
        node.collect(args.warmup)

        root = Path(args.out)
        written = 0
        for i in range(args.count):
            if i:
                node.collect(args.interval)
            else:
                node.collect(0.1)  # take the freshest frame, not the warmup's last

            snapshot = node.capture(
                label=args.label,
                pair_tolerance_s=args.pair_tolerance,
                max_pose_offset_s=args.max_pose_offset,
                depth_scale_m=args.depth_scale,
                max_range_m=args.max_range,
            )

            if args.require_pose and (snapshot.pose is None or "pose_warning" in snapshot.pairing):
                reason = snapshot.pairing.get("pose_warning", "no pose available")
                print(f"{snapshot.snapshot_id}: refusing to save, {reason}", file=sys.stderr)
                return 2

            saved = None if args.dry_run else snapshot.save(root / snapshot.snapshot_id)
            written += saved is not None
            print(_describe(snapshot, saved))

        if not args.dry_run:
            print(f"\n{written} snapshot(s) under {root}")
        print(f"robot clock offset: {node.robot_clock.report()}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
