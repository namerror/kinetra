"""Snapshots are the hand-off between the robot and everything else, so the
on-disk format has to survive a round trip exactly -- particularly 16-bit depth,
which a default image read would silently truncate to 8 bits."""

import json

import numpy as np
import pytest

from kinetra.perception import CameraIntrinsics, RobotPose, Snapshot, load_snapshot

# The real values measured from /camera/D455_2/color/camera_info on 2026-10-07.
INTRINSICS = CameraIntrinsics(
    width=640, height=480,
    fx=387.55224609375, fy=387.1597595214844,
    cx=327.8048400878906, cy=240.05801391601562,
    frame_id="D455_2_color_optical_frame",
    distortion_model="plumb_bob",
    distortion=(-0.0553, 0.0624, -0.0011, 0.0012, -0.0200),
)


def make_snapshot(**overrides) -> Snapshot:
    color = np.zeros((480, 640, 3), dtype=np.uint8)
    color[:, :, 2] = 200  # a red image in BGR, to catch channel-order mistakes
    depth = np.full((480, 640), 4044, dtype=np.uint16)  # ~4 m, the observed median
    depth[0, 0] = 0        # a dropout
    depth[0, 1] = 62148    # beyond the usable range
    kwargs = dict(
        snapshot_id="20261007T174445_529_test",
        captured_at=1791409485.5,
        color_bgr=color,
        depth_raw=depth,
        intrinsics=INTRINSICS,
        label="lying-target-2m",
    )
    kwargs.update(overrides)
    return Snapshot(**kwargs)


def test_roundtrip_preserves_images_and_provenance(tmp_path):
    pose = RobotPose(
        position=(1.16, -0.04, 0.04),
        orientation_xyzw=(-0.0224, -0.0544, -0.3771, 0.9243),
        frame_id="odom", child_frame_id="base_link",
        stamp_robot_clock=1791406856.92,
        stamp_local_clock=1791409485.52,
        capture_offset_s=0.0055,
    )
    original = make_snapshot(pose=pose)
    loaded = load_snapshot(original.save(tmp_path / original.snapshot_id))

    assert np.array_equal(loaded.color_bgr, original.color_bgr)
    assert np.array_equal(loaded.depth_raw, original.depth_raw)
    assert loaded.depth_raw.dtype == np.uint16
    assert loaded.label == "lying-target-2m"
    assert loaded.intrinsics == original.intrinsics
    assert loaded.pose is not None
    assert loaded.pose.stamp_robot_clock == pytest.approx(1791406856.92)
    assert loaded.pose.capture_offset_s == pytest.approx(0.0055)


def test_depth_survives_as_sixteen_bit(tmp_path):
    # An 8-bit read would clamp 62148 to 255 and quietly turn far geometry into
    # near geometry, which is the one failure that must never pass silently.
    loaded = load_snapshot(make_snapshot().save(tmp_path / "s"))
    assert loaded.depth_raw.max() == 62148


def test_roundtrip_without_a_pose_is_allowed(tmp_path):
    loaded = load_snapshot(make_snapshot(pose=None).save(tmp_path / "s"))
    assert loaded.pose is None


def test_dropouts_and_out_of_range_become_nan_not_distance():
    d = make_snapshot().depth_m()
    assert np.isnan(d[0, 0]), "a zero count means no measurement, not zero range"
    assert np.isnan(d[0, 1]), "62 m is past the D455's usable range and is noise"
    assert d[10, 10] == pytest.approx(4.044)


def test_depth_stats_counts_only_valid_pixels():
    # A quarter of the frame dropped out, which is the scale of invalidity that
    # actually decides whether a snapshot is usable (the live camera reports ~82%).
    depth = np.full((480, 640), 4044, dtype=np.uint16)
    depth[:120, :] = 0
    stats = make_snapshot(depth_raw=depth).depth_stats()
    assert stats["pixels"] == 640 * 480
    assert stats["median_m"] == pytest.approx(4.044)
    assert stats["valid_fraction"] == pytest.approx(0.75)


def test_depth_stats_handles_a_fully_invalid_frame():
    blind = make_snapshot(depth_raw=np.zeros((480, 640), dtype=np.uint16))
    stats = blind.depth_stats()
    assert stats["valid_fraction"] == 0.0
    assert stats["median_m"] is None


def test_depth_patch_ignores_surrounding_dropouts():
    s = make_snapshot()
    # The patch around (0, 0) is mostly dropout; the median of what remains is used.
    assert s.depth_m_at(0, 0, patch=5) == pytest.approx(4.044)
    assert s.point_at(0, 0) is not None


def test_patch_with_no_valid_depth_returns_none():
    blind = make_snapshot(depth_raw=np.zeros((480, 640), dtype=np.uint16))
    assert blind.depth_m_at(320, 240) is None
    assert blind.point_at(320, 240) is None


def test_deprojection_puts_the_principal_point_straight_ahead():
    x, y, z = INTRINSICS.deproject(INTRINSICS.cx, INTRINSICS.cy, 3.0)
    assert (x, y, z) == pytest.approx((0.0, 0.0, 3.0))


def test_deprojection_signs_follow_the_optical_frame():
    # Optical frame: +x right, +y down, +z forward.
    right_down = INTRINSICS.deproject(INTRINSICS.cx + 100, INTRINSICS.cy + 100, 2.0)
    assert right_down[0] > 0 and right_down[1] > 0
    scale = 2.0 / INTRINSICS.fx
    assert right_down[0] == pytest.approx(100 * scale)


def test_yaw_matches_the_quaternion():
    import math

    pose = RobotPose(
        position=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)),
        frame_id="odom", child_frame_id="base_link",
        stamp_robot_clock=0.0, stamp_local_clock=None, capture_offset_s=None,
    )
    assert pose.yaw_rad == pytest.approx(math.pi / 4)


def test_mismatched_colour_and_depth_sizes_are_rejected():
    with pytest.raises(ValueError, match="aligned depth"):
        make_snapshot(depth_raw=np.zeros((240, 320), dtype=np.uint16))


def test_intrinsics_from_camera_info_reads_the_right_matrix_entries():
    k = [387.55224609375, 0.0, 327.8048400878906,
         0.0, 387.1597595214844, 240.05801391601562,
         0.0, 0.0, 1.0]
    i = CameraIntrinsics.from_camera_info(k=k, d=[0.1], width=640, height=480,
                                          frame_id="f", distortion_model="plumb_bob")
    assert (i.fx, i.fy, i.cx, i.cy) == pytest.approx(
        (387.55224609375, 387.1597595214844, 327.8048400878906, 240.05801391601562))


def test_malformed_camera_info_is_rejected():
    with pytest.raises(ValueError, match="9 entries"):
        CameraIntrinsics.from_camera_info(k=[1.0, 2.0], d=[], width=1, height=1, frame_id="f")


def test_metadata_is_json_serialisable_and_records_the_depth_encoding():
    meta = json.loads(json.dumps(make_snapshot().to_metadata()))
    assert meta["depth"]["encoding"] == "16UC1"
    assert meta["depth_scale_m"] == 0.001
    assert "no measurement" in meta["depth"]["note"]


def test_color_rgb_is_a_channel_flip():
    s = make_snapshot()
    assert s.color_rgb[0, 0, 0] == s.color_bgr[0, 0, 2] == 200
