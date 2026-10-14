"""These numbers decide how fast a 15 kg robot moves in a room with people in it.

Every ceiling and every refusal is pinned down here rather than discovered on
hardware, because the first commanded move on this robot is also the first test
of the SDK path and there is no second chance to get the envelope right.
"""

import math

import pytest

from kinetra.robot.limits import (
    DEFAULT_MIN_FOOT_FORCE,
    DWELL_FEET_S,
    DWELL_NO_ONSET_S,
    FOOT_FORCE_LYING_OBSERVED,
    GATE_MIN_RATE_HZ,
    GATE_MIN_SAMPLES,
    HARD_MAX_FORWARD_MPS,
    HARD_MAX_TRANSLATION_M,
    HARD_MAX_WINDOW_S,
    HARD_MAX_YAW_RPS,
    MIN_COMMAND_RATE_HZ,
    AbortCriteria,
    AbortMonitor,
    CeilingViolation,
    MotionLimits,
    StateSignature,
    StreamHealth,
    Velocity,
    assess_readiness,
    build_profile,
    clamp_or_refuse,
    clamp_velocity,
    describe_return_code,
    displacement_between,
    integrate_profile,
    motion_onset,
    profile_at,
    settle_time,
    wrap_angle,
)


def make_limits(**overrides):
    return MotionLimits(**overrides)


def make_state(**overrides) -> StateSignature:
    """A healthy standing sample. Overrides make one thing wrong at a time."""
    base = dict(
        received_at=1792000000.0,
        received_monotonic=1000.0,
        stamp_robot_clock=1791997312.7,
        stamp_local_clock=1792000000.0,
        mode=7,
        gait_type=1,
        error_code=0,
        progress=0.0,
        body_height_m=0.32,
        position=(0.0, 0.0, 0.32),
        velocity=(0.0, 0.0, 0.0),
        yaw_speed_rps=0.0,
        rpy_rad=(0.0, 0.0, 0.0),
        foot_force=(170, 165, 172, 168),
        range_obstacle=(2.0, 2.0, 2.0, 2.0),
    )
    base.update(overrides)
    return StateSignature(**base)


def make_stream(**overrides) -> StreamHealth:
    base = dict(samples=600, observed_rate_hz=499.0, stamp_advancing=True)
    base.update(overrides)
    return StreamHealth(**base)


# -- ceilings ---------------------------------------------------------------

def test_a_limit_above_the_hard_ceiling_is_refused_not_quietly_clamped():
    with pytest.raises(ValueError, match="exceeds the hard ceiling"):
        make_limits(max_forward_mps=5.0)
    with pytest.raises(ValueError, match="exceeds the hard ceiling"):
        make_limits(max_yaw_rps=HARD_MAX_YAW_RPS + 0.01)
    with pytest.raises(ValueError, match="exceeds the hard ceiling"):
        make_limits(max_window_s=HARD_MAX_WINDOW_S + 0.5)


def test_a_negative_or_non_finite_limit_is_refused():
    with pytest.raises(ValueError, match="must not be negative"):
        make_limits(max_forward_mps=-0.1)
    with pytest.raises(ValueError, match="must be finite"):
        make_limits(max_forward_mps=float("nan"))
    with pytest.raises(ValueError, match="must be finite and positive"):
        make_limits(accel_mps2=0.0)


def test_a_command_rate_below_the_floor_is_refused():
    with pytest.raises(ValueError, match="below"):
        make_limits(command_rate_hz=MIN_COMMAND_RATE_HZ - 1.0)


def test_the_hard_ceilings_are_the_values_project_spec_records():
    # If this fails, PROJECT_SPEC.md and limits.py have diverged, and the
    # normative document no longer describes the code that enforces it.
    assert (HARD_MAX_FORWARD_MPS, HARD_MAX_YAW_RPS) == (0.15, 0.30)
    assert (HARD_MAX_WINDOW_S, HARD_MAX_TRANSLATION_M) == (2.0, 0.35)


# -- clamping ---------------------------------------------------------------

def test_clamping_reduces_a_request_and_reports_what_it_changed():
    clamped, changes = clamp_velocity(Velocity(vx=0.9, wz=-0.9), make_limits())
    assert clamped == Velocity(vx=HARD_MAX_FORWARD_MPS, wz=-HARD_MAX_YAW_RPS)
    assert len(changes) == 2
    assert any("forward ceiling" in c for c in changes)


def test_a_non_finite_velocity_is_refused_rather_than_zeroed():
    # Zeroing a NaN would send a stop while the log reported a move.
    with pytest.raises(ValueError, match="must be finite"):
        Velocity(vx=float("nan"))
    with pytest.raises(ValueError, match="must be finite"):
        Velocity(wz=float("inf"))


def test_backward_and_lateral_are_zero_so_any_such_request_is_refused():
    limits = make_limits()
    assert limits.max_backward_mps == 0.0
    assert limits.max_lateral_mps == 0.0
    _, changes = clamp_velocity(Velocity(vx=-0.05, vy=0.05), limits)
    assert any("reverse is refused" in c for c in changes)
    assert any("lateral is refused" in c for c in changes)


def test_the_send_chokepoint_raises_rather_than_correcting_a_bad_command():
    limits = make_limits()
    assert clamp_or_refuse(Velocity(wz=0.1), limits) == Velocity(wz=0.1)
    with pytest.raises(CeilingViolation):
        clamp_or_refuse(Velocity(vx=0.5), limits)


# -- the velocity profile ---------------------------------------------------

def test_the_profile_starts_and_ends_at_exactly_zero():
    limits, target = make_limits(), Velocity(wz=0.30)
    assert profile_at(0.0, target, 2.0, limits) == Velocity()
    assert profile_at(2.0, target, 2.0, limits) == Velocity()
    assert profile_at(99.0, target, 2.0, limits) == Velocity()


def test_the_profile_reaches_the_target_exactly_rather_than_asymptoting():
    limits, target = make_limits(), Velocity(wz=0.30)
    assert profile_at(1.0, target, 2.0, limits) == target


def test_every_profile_ends_at_exactly_zero_velocity():
    limits = make_limits()
    for target in (Velocity(wz=0.30), Velocity(vx=0.15), Velocity()):
        profile = build_profile(target, 2.0, limits)
        assert profile[-1] == Velocity()


def test_the_profile_never_exceeds_the_acceleration_limit_between_two_ticks():
    limits = make_limits()
    profile = build_profile(Velocity(vx=0.15), 2.0, limits)
    allowed = limits.accel_mps2 * limits.tick_s + 1e-9
    for a, b in zip(profile, profile[1:]):
        assert abs(b.vx - a.vx) <= allowed


def test_the_profile_terminates_within_a_bounded_number_of_ticks():
    limits = make_limits()
    profile = build_profile(Velocity(wz=0.30), 2.0, limits)
    assert len(profile) == int(math.ceil(2.0 / limits.tick_s)) + 1


def test_integrated_travel_is_an_upper_bound_not_a_prediction():
    # Trapezoid: the ramps each contribute half of what a full-speed hold would,
    # so the bound is strictly below target * window and strictly above zero.
    limits = make_limits()
    profile = build_profile(Velocity(vx=0.15), 2.0, limits)
    translation, lateral, rotation = integrate_profile(profile, limits.tick_s)
    assert 0.0 < translation < 0.15 * 2.0
    assert translation == pytest.approx(0.225, abs=1e-6)
    assert (lateral, rotation) == (0.0, 0.0)


def test_the_approved_stage_profiles_stay_inside_the_travel_caps():
    limits = make_limits()
    forward, _, _ = integrate_profile(build_profile(Velocity(vx=0.15), 2.0, limits),
                                      limits.tick_s)
    _, _, yaw = integrate_profile(build_profile(Velocity(wz=0.30), 2.0, limits),
                                  limits.tick_s)
    assert forward <= HARD_MAX_TRANSLATION_M
    assert yaw == pytest.approx(0.45, abs=1e-6)


# -- the readiness gate -----------------------------------------------------

def test_unloaded_feet_are_not_ready_however_promising_the_mode():
    # The real lying-down reading. mode 7 and body_height 0.32 both look like a
    # standing robot, and the feet are what give it away.
    verdict = assess_readiness(make_state(foot_force=FOOT_FORCE_LYING_OBSERVED),
                               make_stream(), 1000.0, make_limits())
    assert not verdict.ready
    assert any("not standing" in r for r in verdict.reasons)


def test_one_unloaded_foot_is_enough_to_refuse():
    verdict = assess_readiness(make_state(foot_force=(170, 165, 172, 3)),
                               make_stream(), 1000.0, make_limits())
    assert not verdict.ready
    assert any("[3]" in r for r in verdict.reasons)


def test_a_lopsided_stance_is_refused_even_with_every_foot_above_the_floor():
    verdict = assess_readiness(make_state(foot_force=(300, 300, 300, 55)),
                               make_stream(), 1000.0, make_limits())
    assert not verdict.ready
    assert any("lopsided" in r for r in verdict.reasons)


def test_mode_is_recorded_and_never_asserted():
    # An absurd mode must still pass: there is no sport-mode enumeration
    # anywhere, so comparing mode to a literal would be inventing a fact.
    verdict = assess_readiness(make_state(mode=99, gait_type=42), make_stream(),
                               1000.0, make_limits())
    assert verdict.ready
    assert verdict.observations["mode"] == 99


def test_body_height_is_recorded_and_never_gated_on():
    # It read 0.32 while the robot lay with unloaded feet, so it cannot
    # discriminate standing and must not be allowed to look like it can.
    verdict = assess_readiness(make_state(body_height_m=0.0), make_stream(),
                               1000.0, make_limits())
    assert verdict.ready
    assert verdict.observations["body_height_m"] == 0.0


def test_a_nonzero_error_code_blocks_readiness():
    verdict = assess_readiness(make_state(error_code=6), make_stream(), 1000.0,
                               make_limits())
    assert not verdict.ready
    assert any("error_code is 6" in r for r in verdict.reasons)


def test_a_stale_state_message_is_not_ready():
    verdict = assess_readiness(make_state(), make_stream(), 1000.5, make_limits())
    assert not verdict.ready
    assert any("ms old" in r for r in verdict.reasons)


def test_a_missing_state_message_is_not_ready_and_says_so_plainly():
    verdict = assess_readiness(None, make_stream(), 1000.0, make_limits())
    assert not verdict.ready
    assert any("rt/sportmodestate" in r for r in verdict.reasons)


def test_a_frozen_stream_is_refused_although_samples_keep_arriving():
    verdict = assess_readiness(make_state(), make_stream(stamp_advancing=False),
                               1000.0, make_limits())
    assert not verdict.ready
    assert any("frozen" in r for r in verdict.reasons)


def test_a_thin_or_slow_stream_is_refused():
    thin = assess_readiness(make_state(), make_stream(samples=GATE_MIN_SAMPLES - 1),
                            1000.0, make_limits())
    slow = assess_readiness(make_state(), make_stream(observed_rate_hz=GATE_MIN_RATE_HZ - 1),
                            1000.0, make_limits())
    assert not thin.ready and not slow.ready


def test_a_robot_already_moving_is_not_ready():
    verdict = assess_readiness(make_state(velocity=(0.2, 0.0, 0.0)), make_stream(),
                               1000.0, make_limits())
    assert not verdict.ready
    assert any("already moving" in r for r in verdict.reasons)


def test_a_tilted_robot_is_not_ready():
    verdict = assess_readiness(make_state(rpy_rad=(0.4, 0.0, 0.0)), make_stream(),
                               1000.0, make_limits())
    assert not verdict.ready
    assert any("tilt" in r for r in verdict.reasons)


def test_every_readiness_failure_is_reported_not_just_the_first():
    # An operator who fixes one blocker and re-runs only to hit the next has
    # been told half of what the script already knew.
    verdict = assess_readiness(
        make_state(error_code=6, foot_force=FOOT_FORCE_LYING_OBSERVED,
                   rpy_rad=(0.5, 0.0, 0.0), velocity=(0.3, 0.0, 0.0)),
        make_stream(stamp_advancing=False), 1000.0, make_limits())
    assert not verdict.ready
    assert len(verdict.reasons) >= 4


def test_the_gate_records_the_uncalibrated_foot_force_threshold_it_used():
    verdict = assess_readiness(make_state(), make_stream(), 1000.0, make_limits())
    assert verdict.observations["min_foot_force_required"] == DEFAULT_MIN_FOOT_FORCE
    assert "UNCALIBRATED" in verdict.observations["min_foot_force_note"]


def test_the_handshake_gate_can_skip_the_standing_checks():
    # handshake must be able to report what a lying robot looks like.
    verdict = assess_readiness(make_state(foot_force=FOOT_FORCE_LYING_OBSERVED),
                               make_stream(), 1000.0, make_limits(),
                               require_motion_ready=False)
    assert verdict.ready
    assert verdict.observations["loaded_feet"] == 0


# -- abort predicates -------------------------------------------------------

def monitor(**criteria):
    return AbortMonitor(make_state(), AbortCriteria(**criteria))


def test_abort_returns_none_while_everything_is_nominal():
    assert monitor().observe(make_state(), 1000.0, Velocity(wz=0.3)) is None


def test_abort_trips_when_state_stops_arriving_entirely():
    verdict = monitor().observe(None, 1000.0, Velocity(wz=0.3))
    assert verdict is not None and verdict.predicate == "state_missing"


def test_abort_trips_on_a_stale_sample():
    verdict = monitor().observe(make_state(), 1002.0, Velocity(wz=0.3))
    assert verdict is not None and verdict.predicate == "state_stale"


def test_abort_trips_on_an_error_code_change_not_merely_a_nonzero_value():
    # A nonzero baseline is plausible here with the lidar dead, so demanding
    # zero would be code that gets edited to ignore the field.
    started_faulty = AbortMonitor(make_state(error_code=6), AbortCriteria())
    assert started_faulty.observe(make_state(error_code=6), 1000.0, Velocity()) is None
    verdict = started_faulty.observe(make_state(error_code=7), 1000.0, Velocity())
    assert verdict is not None and verdict.predicate == "error_code_change"


def test_abort_trips_on_travel_beyond_the_cap():
    verdict = monitor(max_translation_m=0.15).observe(
        make_state(position=(0.4, 0.0, 0.32)), 1000.0, Velocity(wz=0.3))
    assert verdict is not None and verdict.predicate == "displacement"


def test_abort_trips_on_rotation_beyond_the_cap():
    verdict = monitor(max_rotation_rad=0.70).observe(
        make_state(rpy_rad=(0.0, 0.0, 1.2)), 1000.0, Velocity(wz=0.3))
    assert verdict is not None and verdict.predicate == "yaw_excursion"


def test_abort_trips_on_body_tilt_after_its_dwell():
    m = monitor()
    assert m.observe(make_state(rpy_rad=(0.3, 0.0, 0.0)), 1000.0,
                     Velocity(wz=0.3)) is None
    verdict = m.observe(make_state(received_monotonic=1000.2, rpy_rad=(0.3, 0.0, 0.0)),
                        1000.2, Velocity(wz=0.3))
    assert verdict is not None and verdict.predicate == "attitude"


def test_a_fall_escalates_to_damp_where_an_ordinary_abort_does_not():
    m = monitor()
    m.observe(make_state(rpy_rad=(0.9, 0.0, 0.0)), 1000.0, Velocity(wz=0.3))
    verdict = m.observe(make_state(received_monotonic=1000.25, rpy_rad=(0.9, 0.0, 0.0)),
                        1000.25, Velocity(wz=0.3))
    assert verdict is not None and verdict.predicate == "fall"
    assert verdict.escalate_to_damp


def test_abort_trips_on_a_speed_faster_than_anything_commanded():
    m = monitor()
    m.observe(make_state(velocity=(0.9, 0.0, 0.0)), 1000.0, Velocity(vx=0.1))
    verdict = m.observe(make_state(received_monotonic=1000.2, velocity=(0.9, 0.0, 0.0)),
                        1000.2, Velocity(vx=0.1))
    assert verdict is not None and verdict.predicate == "overspeed"


def test_all_four_feet_must_be_unloaded_before_that_predicate_trips():
    # Individual feet unload every gait cycle, so a per-foot predicate would
    # false-trigger continuously while the robot is walking normally.
    later = 1000.0 + DWELL_FEET_S + 0.01
    m = monitor()
    assert m.observe(make_state(foot_force=(170, 165, 172, 2)), 1000.0,
                     Velocity(wz=0.3)) is None
    assert m.observe(make_state(received_monotonic=later, foot_force=(170, 165, 172, 2)),
                     later, Velocity(wz=0.3)) is None

    m2 = monitor()
    m2.observe(make_state(foot_force=(1, 2, 1, 2)), 1000.0, Velocity(wz=0.3))
    verdict = m2.observe(make_state(received_monotonic=later, foot_force=(1, 2, 1, 2)),
                         later, Velocity(wz=0.3))
    assert verdict is not None and verdict.predicate == "feet_unloaded"


def test_no_onset_catches_a_robot_that_ignored_a_command_that_returned_zero():
    # The only predicate that detects the SDK's central ambiguity: Move()
    # returning 0 is a local DDS write, not an acknowledgement.
    m = monitor()
    still = make_state()
    verdict = m.observe(still, 1000.0, Velocity(wz=0.3),
                        commanded_since_s=DWELL_NO_ONSET_S + 0.1)
    assert verdict is not None and verdict.predicate == "no_onset"


def test_no_onset_does_not_trip_while_the_robot_is_actually_moving():
    m = monitor()
    moving = make_state(yaw_speed_rps=0.28)
    assert m.observe(moving, 1000.0, Velocity(wz=0.3),
                     commanded_since_s=1.0) is None


def test_a_nonzero_return_code_aborts_and_an_unknown_one_is_named_as_unsafe():
    verdict = monitor().observe(make_state(), 1000.0, Velocity(), return_code=3102)
    assert verdict is not None and verdict.predicate == "rpc_error"
    assert "local DDS write failed" in verdict.reason
    unknown = monitor().observe(make_state(), 1000.0, Velocity(), return_code=7777)
    assert unknown is not None and "UNDOCUMENTED" in unknown.reason


def test_a_disarmed_predicate_does_not_trip():
    # yaw-estop depends on this: armed, no_onset would make us stop the robot
    # during the operator's interrupt and we could never tell who stopped it.
    m = AbortMonitor(make_state(), AbortCriteria(disarmed=("no_onset", "overspeed")))
    assert m.observe(make_state(), 1000.0, Velocity(wz=0.3),
                     commanded_since_s=5.0) is None


def test_the_first_tripped_abort_reason_is_the_one_reported():
    m = monitor(max_translation_m=0.15)
    first = m.observe(make_state(position=(0.5, 0.0, 0.32)), 1000.0, Velocity())
    again = m.observe(None, 1001.0, Velocity())
    assert first is not None and again == first
    assert m.verdict == first


def test_describe_return_code_never_invents_a_meaning():
    assert "UNDOCUMENTED" in describe_return_code(9999)
    assert describe_return_code(None).startswith("no code")
    assert describe_return_code(0) == "ok"


# -- measurement ------------------------------------------------------------

def test_rotation_is_wrapped_so_a_pi_crossing_is_not_a_full_turn():
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    start = make_state(rpy_rad=(0.0, 0.0, math.pi - 0.05))
    end = make_state(rpy_rad=(0.0, 0.0, -math.pi + 0.05))
    _, rotation = displacement_between(start, end)
    assert rotation == pytest.approx(0.1, abs=1e-6)


def test_displacement_ignores_body_height_because_crouching_is_not_travel():
    start = make_state(position=(1.0, 1.0, 0.32))
    end = make_state(position=(1.0, 1.0, 0.10))
    distance, rotation = displacement_between(start, end)
    assert (distance, rotation) == (0.0, 0.0)


def test_motion_onset_is_none_when_the_robot_never_moved():
    samples = [make_state(received_monotonic=1000.0 + i * 0.05) for i in range(20)]
    assert motion_onset(samples, 1000.0) is None


def test_motion_onset_is_measured_from_the_first_command_and_ignores_earlier_noise():
    before = make_state(received_monotonic=999.0, yaw_speed_rps=0.9)
    samples = [before]
    samples += [make_state(received_monotonic=1000.0 + i * 0.05) for i in range(4)]
    samples.append(make_state(received_monotonic=1000.2, yaw_speed_rps=0.2))
    onset = motion_onset(samples, 1000.0)
    assert onset == pytest.approx(0.2, abs=1e-9)


def test_settle_time_requires_the_robot_to_stay_at_rest():
    moving = [make_state(received_monotonic=1000.0 + i * 0.05, yaw_speed_rps=0.3)
              for i in range(4)]
    resting = [make_state(received_monotonic=1000.2 + i * 0.05) for i in range(10)]
    assert settle_time(moving + resting, 1000.0) == pytest.approx(0.2, abs=1e-9)
    assert settle_time(moving, 1000.0) is None


def test_limits_and_state_dicts_are_json_serialisable():
    import json
    json.dumps(make_limits().to_dict())
    json.dumps(make_state().to_dict())
    json.dumps(assess_readiness(make_state(), make_stream(), 1000.0,
                                make_limits()).to_dict())


def test_a_state_signature_survives_a_json_round_trip():
    import json
    original = make_state()
    restored = StateSignature.from_dict(json.loads(json.dumps(original.to_dict())))
    assert restored == original
