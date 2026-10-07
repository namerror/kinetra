"""Stopped RGB-D snapshots: the unit of observation Kinetra reasons about.

A snapshot is deliberately a directory on disk rather than a live stream handle.
Every consumer downstream -- target-visibility checks, hosted model calls, mission
logs, the evaluation harness, the fixed-order baseline -- reads snapshots, so a
recorded scene and a live robot look identical to the rest of the system. That is
also what lets this repository be exercised by a reviewer with no Go2.

Nothing here imports ROS. See `capture` for the ROS 2 node that produces these.

Provenance is part of the data, not metadata to be added later. A snapshot records
which clock each stamp came from, how stale the pose was, and how far apart the
colour and depth frames were, because a candidate sighting is only as trustworthy
as the pose attached to it.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

# The RealSense driver publishes aligned depth as 16UC1. `depth_module.depth_units`
# is unset on this robot, so librealsense's default of 1 mm per count applies; the
# survey's observed median of 4044 counts at room scale is consistent with that.
# Recorded per snapshot so a future configuration change cannot silently rescale
# every stored observation.
DEFAULT_DEPTH_SCALE_M = 0.001

# The D455's usable range. Depth is reported well past this (the survey saw 62 m),
# but those counts are noise, not geometry, and must not reach the planner.
DEFAULT_MAX_RANGE_M = 10.0

COLOR_FILENAME = "color.png"
DEPTH_FILENAME = "depth.png"
META_FILENAME = "snapshot.json"


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the colour stream, which aligned depth shares."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str
    distortion_model: str = ""
    distortion: tuple[float, ...] = ()

    @classmethod
    def from_camera_info(cls, k: list[float], d: list[float], width: int, height: int,
                         frame_id: str, distortion_model: str = "") -> CameraIntrinsics:
        """Build from the row-major 3x3 `k` and `d` of a `sensor_msgs/CameraInfo`."""
        if len(k) != 9:
            raise ValueError(f"camera_info k must have 9 entries, got {len(k)}")
        return cls(
            width=width, height=height,
            fx=k[0], fy=k[4], cx=k[2], cy=k[5],
            frame_id=frame_id,
            distortion_model=distortion_model,
            distortion=tuple(float(x) for x in d),
        )

    def deproject(self, u: float, v: float, depth_m: float) -> tuple[float, float, float]:
        """Pixel plus range -> a point in the camera optical frame (x right, y down, z forward).

        Distortion is not undone. The D455's colour stream is nearly rectified
        (|k1| ~ 0.055) and Kinetra uses depth for coarse "how far, roughly which
        way" judgements, so the error is far below the pose uncertainty it is
        combined with. Revisit this if depth is ever used for fine manipulation.
        """
        return (
            (u - self.cx) * depth_m / self.fx,
            (v - self.cy) * depth_m / self.fy,
            depth_m,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "width": self.width, "height": self.height,
            "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
            "frame_id": self.frame_id,
            "distortion_model": self.distortion_model,
            "distortion": list(self.distortion),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> CameraIntrinsics:
        return cls(
            width=int(d["width"]), height=int(d["height"]),
            fx=float(d["fx"]), fy=float(d["fy"]),
            cx=float(d["cx"]), cy=float(d["cy"]),
            frame_id=str(d["frame_id"]),
            distortion_model=str(d.get("distortion_model", "")),
            distortion=tuple(float(x) for x in d.get("distortion", ())),
        )


@dataclass(frozen=True)
class RobotPose:
    """An odometry pose, carried with enough provenance to decide whether to trust it.

    Two stamps are kept on purpose. `stamp_robot_clock` is what the Go2 published;
    `stamp_local_clock` is that value converted with the measured clock offset.
    Only the latter is comparable to a camera stamp. See `clock`.
    """

    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    frame_id: str
    child_frame_id: str
    stamp_robot_clock: float
    stamp_local_clock: float | None
    capture_offset_s: float | None
    """Signed seconds between this pose and the frame, in the local timebase.

    Positive means the pose predates the frame, negative means it follows it.
    What matters is the magnitude: it is the pairing error, and it bounds how far
    the robot could have moved between being photographed and being located.
    """

    @property
    def yaw_rad(self) -> float:
        """Heading about the odom z axis."""
        x, y, z, w = self.orientation_xyzw
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": list(self.position),
            "orientation_xyzw": list(self.orientation_xyzw),
            "yaw_rad": self.yaw_rad,
            "frame_id": self.frame_id,
            "child_frame_id": self.child_frame_id,
            "stamp_robot_clock": self.stamp_robot_clock,
            "stamp_local_clock": self.stamp_local_clock,
            "capture_offset_s": self.capture_offset_s,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> RobotPose:
        return cls(
            position=tuple(float(x) for x in d["position"]),               # type: ignore[arg-type]
            orientation_xyzw=tuple(float(x) for x in d["orientation_xyzw"]),  # type: ignore[arg-type]
            frame_id=str(d["frame_id"]),
            child_frame_id=str(d["child_frame_id"]),
            stamp_robot_clock=float(d["stamp_robot_clock"]),
            stamp_local_clock=None if d.get("stamp_local_clock") is None else float(d["stamp_local_clock"]),
            capture_offset_s=(None if d.get("capture_offset_s") is None
                              else float(d["capture_offset_s"])),
        )


@dataclass
class Snapshot:
    """One stopped RGB-D observation, with its provenance."""

    snapshot_id: str
    captured_at: float
    """Local (Jetson) wall clock at capture."""
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    """Aligned depth as published, uint16. Multiply by `depth_scale_m` for metres."""
    intrinsics: CameraIntrinsics
    pose: RobotPose | None = None
    depth_scale_m: float = DEFAULT_DEPTH_SCALE_M
    max_range_m: float = DEFAULT_MAX_RANGE_M
    label: str = ""
    """Operator-supplied tag, e.g. "lying-target-2m". Drives the visibility trials."""
    clock: dict[str, Any] = field(default_factory=dict)
    pairing: dict[str, Any] = field(default_factory=dict)
    notes: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.color_bgr.shape[:2] != self.depth_raw.shape[:2]:
            raise ValueError(
                f"colour {self.color_bgr.shape[:2]} and depth {self.depth_raw.shape[:2]} "
                "differ in size; aligned depth was expected"
            )

    @property
    def color_rgb(self) -> np.ndarray:
        """RGB view, which is what image models and `PIL` expect."""
        return self.color_bgr[:, :, ::-1]

    def depth_m(self) -> np.ndarray:
        """Depth in metres, with invalid and out-of-range pixels as NaN.

        Zero means "no measurement" in the RealSense encoding, and must not be
        read as "something is touching the camera".
        """
        d = self.depth_raw.astype(np.float32) * self.depth_scale_m
        d[self.depth_raw == 0] = np.nan
        d[d > self.max_range_m] = np.nan
        return d

    def depth_m_at(self, u: int, v: int, patch: int = 5) -> float | None:
        """Median valid depth in metres over a patch around (u, v), or None.

        A patch rather than a single pixel because depth around an object's edge
        is speckled with dropouts, and a lying person is mostly edges.
        """
        half = patch // 2
        v0, v1 = max(0, v - half), min(self.depth_raw.shape[0], v + half + 1)
        u0, u1 = max(0, u - half), min(self.depth_raw.shape[1], u + half + 1)
        window = self.depth_m()[v0:v1, u0:u1]
        valid = window[np.isfinite(window)]
        return float(np.median(valid)) if valid.size else None

    def point_at(self, u: int, v: int, patch: int = 5) -> tuple[float, float, float] | None:
        """A 3-D point in the camera optical frame for a pixel, or None if no depth."""
        depth = self.depth_m_at(u, v, patch)
        return None if depth is None else self.intrinsics.deproject(u, v, depth)

    def depth_stats(self) -> dict[str, Any]:
        """Summary used to judge whether a snapshot is worth sending to a model."""
        d = self.depth_m()
        valid = d[np.isfinite(d)]
        total = int(d.size)
        if valid.size == 0:
            return {"valid_fraction": 0.0, "pixels": total,
                    "min_m": None, "median_m": None, "p95_m": None}
        return {
            "valid_fraction": round(float(valid.size) / total, 4),
            "pixels": total,
            "min_m": round(float(valid.min()), 3),
            "median_m": round(float(np.median(valid)), 3),
            "p95_m": round(float(np.percentile(valid, 95)), 3),
        }

    def to_metadata(self) -> dict[str, Any]:
        """Everything except the two image arrays."""
        return {
            "snapshot_id": self.snapshot_id,
            "captured_at": self.captured_at,
            "label": self.label,
            "depth_scale_m": self.depth_scale_m,
            "max_range_m": self.max_range_m,
            "color": {"file": COLOR_FILENAME, "encoding": "bgr8"},
            "depth": {"file": DEPTH_FILENAME, "encoding": "16UC1",
                      "note": "0 means no measurement, not zero range"},
            "intrinsics": self.intrinsics.to_dict(),
            "pose": None if self.pose is None else self.pose.to_dict(),
            "clock": self.clock,
            "pairing": self.pairing,
            "depth_stats": self.depth_stats(),
            "notes": self.notes,
        }

    def save(self, directory: str | Path) -> Path:
        """Write the snapshot to its own directory and return that path.

        Keep these off the repository tree: `/` has little free space and
        identifiable images must stay out of the public repository unless release
        is approved (`docs/HACKATHON.md`).
        """
        out = Path(directory)
        out.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(out / COLOR_FILENAME), self.color_bgr):
            raise OSError(f"could not write {out / COLOR_FILENAME}")
        if not cv2.imwrite(str(out / DEPTH_FILENAME), self.depth_raw):
            raise OSError(f"could not write {out / DEPTH_FILENAME}")
        (out / META_FILENAME).write_text(json.dumps(self.to_metadata(), indent=2) + "\n")
        return out


def load_snapshot(directory: str | Path) -> Snapshot:
    """Read back a saved snapshot. The off-robot path for tests and recorded runs."""
    src = Path(directory)
    meta = json.loads((src / META_FILENAME).read_text())

    color = cv2.imread(str(src / meta["color"]["file"]), cv2.IMREAD_COLOR)
    if color is None:
        raise OSError(f"could not read colour image in {src}")
    # IMREAD_UNCHANGED is required: the default would truncate 16-bit depth to 8.
    depth = cv2.imread(str(src / meta["depth"]["file"]), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise OSError(f"could not read depth image in {src}")
    if depth.dtype != np.uint16:
        raise ValueError(f"depth in {src} is {depth.dtype}, expected uint16")

    return Snapshot(
        snapshot_id=meta["snapshot_id"],
        captured_at=float(meta["captured_at"]),
        color_bgr=color,
        depth_raw=depth,
        intrinsics=CameraIntrinsics.from_dict(meta["intrinsics"]),
        pose=None if meta.get("pose") is None else RobotPose.from_dict(meta["pose"]),
        depth_scale_m=float(meta.get("depth_scale_m", DEFAULT_DEPTH_SCALE_M)),
        max_range_m=float(meta.get("max_range_m", DEFAULT_MAX_RANGE_M)),
        label=meta.get("label", ""),
        clock=meta.get("clock", {}),
        pairing=meta.get("pairing", {}),
        notes=meta.get("notes", {}),
    )
