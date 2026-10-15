"""A stage is a promise about what will be sent, made before anything is sent.

These tests check the catalogue cannot promise something the ceilings forbid,
and that the briefing an operator reads actually contains the facts the run
depends on -- including the ones that are uncomfortable, like the sign
convention being unverified and nothing but the operator being able to stop the
robot.
"""

import sys

import pytest

from kinetra.robot.limits import (
    HARD_MAX_ROTATION_RAD,
    HARD_MAX_TRANSLATION_M,
    ONSET_YAW_RPS,
    MotionLimits,
    Velocity,
    ramp_duration_s,
)
from kinetra.robot.stages import (
    STAGE_NAMES,
    STAGES,
    build_schedule,
    describe_schedule,
    refusals,
    schedule_bounds,
)


def make_limits(**overrides):
    return MotionLimits(**overrides)


def test_every_catalogued_stage_builds_a_schedule_inside_every_hard_ceiling():
    limits = make_limits()
    for name, stage in STAGES.items():
        assert refusals(stage, limits) == (), f"{name} is refused by its own ceilings"


def test_every_catalogued_stage_declares_its_own_travel_and_rotation_caps():
    for name, stage in STAGES.items():
        assert stage.abort.max_translation_m > 0.0, name
        assert stage.abort.max_rotation_rad > 0.0, name
        assert stage.abort.max_translation_m <= HARD_MAX_TRANSLATION_M, name


def test_the_stage_names_are_sorted_and_cover_the_catalogue():
    assert STAGE_NAMES == tuple(sorted(STAGES))
    assert set(STAGE_NAMES) == {"handshake", "yaw", "yaw-estop", "forward",
                                "watchdog-quiet", "watchdog-kill"}


def test_the_handshake_stage_sends_no_motion_command_at_all():
    stage = STAGES["handshake"]
    assert not stage.sends_motion
    assert build_schedule(stage, make_limits()) == ()
    assert stage.target == Velocity()


def test_the_handshake_stage_uses_only_calls_whose_codes_are_robot_side():
    # Its whole purpose is evidence, and Move() cannot provide any.
    stage = STAGES["handshake"]
    assert "SportClient.Move" not in stage.blocking_calls
    assert "SportClient.GetServerApiVersion" in stage.blocking_calls
    assert "MotionSwitcherClient.CheckMode" in stage.blocking_calls
    assert "SportClient.StopMove" in stage.blocking_calls


def test_the_handshake_stage_admits_that_stop_move_is_still_a_command():
    assert any("is not nothing" in note for note in STAGES["handshake"].notes)


def test_no_stage_commands_a_posture_change():
    # Standing the robot up is motion, and it is the operator's job with the
    # remote. Each of these methods is one convenient line from being added.
    forbidden = {"StandUp", "StandDown", "BalanceStand", "Sit", "RiseSit",
                 "RecoveryStand", "Damp", "SelectMode", "ReleaseMode"}
    for name, stage in STAGES.items():
        for call in stage.blocking_calls:
            assert call.split(".")[-1] not in forbidden, f"{name} calls {call}"


def test_only_the_handshake_stage_runs_without_a_standing_signature():
    for name, stage in STAGES.items():
        if stage.sends_motion:
            assert stage.requires_signature, f"{name} would run on a guessed gate"
    assert not STAGES["handshake"].requires_signature


def test_the_yaw_stage_commands_rotation_and_no_translation():
    stage = STAGES["yaw"]
    bounds = schedule_bounds(stage, build_schedule(stage, make_limits()), make_limits())
    assert bounds["commanded_rotation_rad"] == pytest.approx(0.45, abs=1e-6)
    assert bounds["commanded_translation_m"] == 0.0


def test_the_forward_stage_commands_translation_and_no_rotation():
    stage = STAGES["forward"]
    bounds = schedule_bounds(stage, build_schedule(stage, make_limits()), make_limits())
    assert bounds["commanded_translation_m"] == pytest.approx(0.225, abs=1e-6)
    assert bounds["commanded_rotation_rad"] == 0.0


def test_the_yaw_stage_demands_more_than_a_footprint_of_clearance():
    # In-place rotation drifts; "it stays on its own footprint" is an assumption.
    assert "1.5 m" in STAGES["yaw"].clearance_note


def test_the_estop_stage_disarms_the_predicates_that_would_stop_the_robot_for_us():
    # Armed, these fire the instant the remote takes over, and we could then
    # never distinguish "the remote stopped it" from "we stopped ourselves".
    disarmed = STAGES["yaw-estop"].abort.disarmed
    assert "no_onset" in disarmed
    assert "overspeed" in disarmed
    assert "operator_interrupt" in disarmed


def test_the_estop_stage_keeps_logging_long_enough_to_see_the_robot_resume():
    assert STAGES["yaw-estop"].post_log_s >= 3.0


def test_the_estop_stage_rotates_long_enough_to_actually_be_interrupted():
    # The operator has to see the rotation, reach for the remote and interrupt
    # it, which a 2.0 s window at 0.30 rad/s does not leave room for: it holds
    # a steady rate for about 1.5 s. The stage is slower and longer instead.
    stage = STAGES["yaw-estop"]
    limits = MotionLimits()
    ramp = ramp_duration_s(stage.target, stage.window_s, limits)
    steady_s = stage.window_s - 2.0 * ramp
    assert stage.window_s >= 4.0
    assert steady_s >= 3.0
    assert abs(stage.target.wz) < abs(STAGES["yaw"].target.wz)
    # Still unambiguously moving, not creeping below the onset threshold.
    assert abs(stage.target.wz) >= 2.0 * ONSET_YAW_RPS


def test_the_longer_estop_window_buys_no_extra_rotation():
    # The whole safety case for the longer window: the geometric cap is
    # unchanged, so the time is bought by going slower, not by travelling more.
    limits = MotionLimits()
    estop = STAGES["yaw-estop"]
    yaw = STAGES["yaw"]
    bounds = schedule_bounds(estop, build_schedule(estop, limits), limits)
    yaw_bounds = schedule_bounds(yaw, build_schedule(yaw, limits), limits)
    assert bounds["commanded_rotation_rad"] <= yaw_bounds["rotation_cap_rad"]
    assert bounds["commanded_rotation_rad"] <= HARD_MAX_ROTATION_RAD
    assert bounds["commanded_translation_m"] == 0.0
    assert refusals(estop, limits) == ()


def test_a_long_window_at_the_full_yaw_rate_is_refused():
    # The window ceiling allows 6.0 s; the rotation cap is what stops a 6.0 s
    # run at 0.30 rad/s, and it must still be the thing that bites.
    limits = MotionLimits()
    greedy = type(STAGES["yaw-estop"])(**{**STAGES["yaw-estop"].__dict__,
                                          "target": Velocity(wz=0.30),
                                          "window_s": 6.0})
    reasons = refusals(greedy, limits)
    assert any("rotation" in r for r in reasons), reasons


def test_the_interrupt_stages_tell_the_operator_not_to_let_go():
    # Measured 2026-10-15 (OPEN_QUESTIONS 8.13): the remote overrides a live
    # command stream only while it is held, and releasing it hands the robot
    # back. An operator briefing that omits this is actively misleading.
    for name in ("yaw-estop", "watchdog-kill"):
        actions = " ".join(STAGES[name].operator_actions).lower()
        assert "let go" in actions or "releasing" in actions, name
        assert "sit it down" in actions, name


def test_the_quiet_watchdog_stage_stops_publishing_inside_its_window():
    stage = STAGES["watchdog-quiet"]
    assert stage.quiet_from_s is not None
    assert 0.0 < stage.quiet_from_s < stage.window_s
    assert stage.self_kill_at_s is None


def test_the_kill_watchdog_stage_kills_itself_inside_its_window():
    stage = STAGES["watchdog-kill"]
    assert stage.self_kill_at_s is not None
    assert 0.0 < stage.self_kill_at_s < stage.window_s


def test_the_kill_watchdog_stage_rotates_in_place_so_its_travel_stays_bounded():
    # If there is no receiver-side watchdog, the last Move persists. A
    # persisting rotation is bounded in displacement; a persisting walk is not.
    stage = STAGES["watchdog-kill"]
    assert stage.target.vx == 0.0 and stage.target.vy == 0.0
    assert stage.target.wz != 0.0


def test_only_the_kill_watchdog_stage_demands_evidence_the_remote_works():
    assert STAGES["watchdog-kill"].requires_estop_evidence
    assert not STAGES["yaw"].requires_estop_evidence


def test_an_offset_outside_the_window_is_refused():
    stage = STAGES["watchdog-kill"]
    broken = type(stage)(**{**stage.__dict__, "self_kill_at_s": 99.0})
    assert any("self_kill_at_s" in r for r in refusals(broken, make_limits()))


def test_a_stage_whose_target_exceeds_the_envelope_is_refused():
    stage = STAGES["yaw"]
    broken = type(stage)(**{**stage.__dict__, "target": Velocity(vx=-0.3, vy=0.4)})
    reasons = refusals(broken, make_limits())
    assert any("outside the envelope" in r for r in reasons)


def test_a_stage_whose_window_exceeds_the_ceiling_is_refused():
    stage = STAGES["yaw"]
    broken = type(stage)(**{**stage.__dict__, "window_s": 30.0})
    assert any("window" in r for r in refusals(broken, make_limits()))


def test_a_stage_whose_integrated_travel_exceeds_its_cap_is_refused():
    from kinetra.robot.limits import AbortCriteria
    stage = STAGES["forward"]
    broken = type(stage)(**{**stage.__dict__,
                            "abort": AbortCriteria(max_translation_m=0.05,
                                                   max_rotation_rad=0.35)})
    assert any("commanded translation" in r for r in refusals(broken, make_limits()))


def test_the_schedule_is_evenly_spaced_and_monotonic_in_time():
    limits = make_limits()
    schedule = build_schedule(STAGES["yaw"], limits)
    offsets = [c.offset_s for c in schedule]
    assert offsets == sorted(offsets)
    gaps = {round(b - a, 9) for a, b in zip(offsets, offsets[1:])}
    assert gaps == {round(limits.tick_s, 9)}


def test_the_schedule_phases_run_up_hold_down_stop_in_order():
    phases = [c.phase for c in build_schedule(STAGES["yaw"], make_limits())]
    assert phases[0] == "ramp-up"
    assert phases[-1] == "stop"
    first_seen = [p for i, p in enumerate(phases) if i == 0 or phases[i - 1] != p]
    assert first_seen == ["ramp-up", "hold", "ramp-down", "stop"]


def test_the_briefing_names_the_stage_the_limits_and_the_integrated_bound():
    limits = make_limits()
    stage = STAGES["yaw"]
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "stage yaw" in text
    assert "wz <= 0.30 rad/s" in text
    assert "0.450 rad" in text
    assert "upper bound, not a prediction" in text


def test_the_briefing_says_a_dry_run_sends_nothing_and_proves_only_the_script():
    stage = STAGES["yaw"]
    limits = make_limits()
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "DRY RUN" in text
    assert "proves the script, not the robot" in text


def test_a_live_motion_briefing_says_the_robot_will_move():
    stage = STAGES["yaw"]
    limits = make_limits()
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=False)
    assert "THE ROBOT WILL MOVE" in text


def test_the_briefing_warns_that_the_sign_convention_is_unverified():
    stage = STAGES["yaw"]
    limits = make_limits()
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "UNVERIFIED" in text


def test_the_briefing_warns_that_only_the_operator_can_stop_the_robot():
    stage = STAGES["yaw"]
    limits = make_limits()
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "nothing stops this robot but the operator" in text


def test_the_briefing_says_mode_and_body_height_are_never_asserted():
    stage = STAGES["yaw"]
    limits = make_limits()
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "never asserted" in text


def test_the_briefing_lists_the_predicates_a_stage_has_disarmed():
    stage = STAGES["yaw-estop"]
    limits = make_limits()
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "DISARMED" in text and "no_onset" in text


def test_the_briefing_elides_the_long_run_of_identical_hold_ticks():
    stage = STAGES["yaw"]
    limits = make_limits()
    text = describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "identical hold ticks omitted" in text


def test_building_a_briefing_imports_neither_ros_nor_the_unitree_sdk():
    # The pure half of this package must stay importable off-robot, and must
    # never pull in a DDS participant as a side effect.
    limits = make_limits()
    for stage in STAGES.values():
        describe_schedule(stage, limits, build_schedule(stage, limits), dry_run=True)
    assert "rclpy" not in sys.modules
    assert "unitree_sdk2py" not in sys.modules


def test_every_stage_serialises_to_json():
    import json
    for stage in STAGES.values():
        json.dumps(stage.to_dict())
