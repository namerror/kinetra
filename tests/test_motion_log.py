"""The log is the only account of a move that nobody can replay.

Its one real failure mode is overstatement: `SportClient.Move` is
fire-and-forget, so a log reporting "32 commands sent, all returned 0" reads as
evidence the robot moved when it is only evidence that 32 local DDS writes
succeeded. These tests exist to keep that distinction in the file itself, where
a later reader will find it, rather than in anyone's memory.
"""

import json
from datetime import datetime

import pytest

from kinetra.robot.limits import (
    AbortCriteria,
    AbortVerdict,
    MotionLimits,
    ReadinessVerdict,
    StateSignature,
    Velocity,
)
from kinetra.robot.motion_log import (
    DEFAULT_OUTPUT_ROOT,
    RETURN_CODE_NOTES,
    CommandRecord,
    Measurement,
    MotionLog,
    StageResult,
    load_motion_log,
    run_id,
)
from kinetra.robot.stages import STAGES, build_schedule, schedule_bounds


def make_state(**overrides) -> StateSignature:
    base = dict(
        received_at=1792000000.0, received_monotonic=1000.0,
        stamp_robot_clock=1791997312.7, stamp_local_clock=1792000000.0,
        mode=7, gait_type=1, error_code=0, progress=0.0, body_height_m=0.32,
        position=(0.0, 0.0, 0.32), velocity=(0.0, 0.0, 0.0), yaw_speed_rps=0.0,
        rpy_rad=(0.0, 0.0, 0.0), foot_force=(170, 165, 172, 168),
        range_obstacle=(2.0, 2.0, 2.0, 2.0))
    base.update(overrides)
    return StateSignature(**base)


def make_command(**overrides) -> CommandRecord:
    base = dict(index=0, call="SportClient.Move", phase="hold",
                sent_at=1792000001.0, sent_monotonic=1001.0,
                returned_monotonic=1001.0002, velocity=Velocity(wz=0.3),
                return_code=0, blocking=False)
    base.update(overrides)
    return CommandRecord(**base)


def make_log(stage_name: str = "yaw", **overrides) -> MotionLog:
    stage = STAGES[stage_name]
    limits = MotionLimits()
    schedule = build_schedule(stage, limits)
    base = dict(
        run_id=run_id(stage_name, datetime(2026, 10, 14, 15, 31, 18, 902000)),
        stage=stage, limits=limits, dry_run=False, started_at=1792000000.0,
        schedule=schedule, schedule_bounds=schedule_bounds(stage, schedule, limits),
        readiness=ReadinessVerdict(True, (), {"mode": 7}),
        state_before=make_state(), state_after=make_state(),
        commands=[make_command()],
        result=StageResult("completed", commands_sent=1, moves_written=1),
        clock={"robot_vs_local": {"offset_s": 2687.41, "trustworthy": True}})
    base.update(overrides)
    return MotionLog(**base)


def test_the_run_id_carries_a_millisecond_timestamp_and_the_stage_name():
    rid = run_id("yaw", datetime(2026, 10, 14, 15, 31, 18, 902000))
    assert rid == "20261014T153118_902_yaw"
    assert run_id("yaw", datetime(2026, 10, 14, 15, 31, 18, 902000),
                  "sign-check").endswith("_yaw_sign-check")


def test_a_log_roundtrips_through_json_with_every_command_preserved():
    log = make_log(commands=[make_command(index=i) for i in range(5)])
    payload = json.loads(json.dumps(log.to_metadata()))
    assert len(payload["commands"]) == 5
    assert payload["run_id"] == log.run_id
    assert payload["stage"]["name"] == "yaw"


def test_a_move_record_states_that_a_zero_return_code_proves_only_a_local_write():
    record = make_command(call="SportClient.Move", return_code=0)
    assert "NOT evidence" in record.proves
    assert "local DDS write" in record.proves
    assert "local DDS write" in record.to_dict()["proves"]


def test_a_blocking_call_record_is_marked_as_robot_side():
    record = make_command(call="SportClient.StopMove", blocking=True, velocity=None)
    assert "robot" in record.proves
    assert record.to_dict()["blocking"] is True


def test_every_call_the_stages_make_has_a_proves_note():
    # A call with no note would be logged with no claim attached, and the next
    # reader would supply their own.
    for stage in STAGES.values():
        for call in stage.blocking_calls:
            assert call in RETURN_CODE_NOTES, call
    assert "SportClient.Move" in RETURN_CODE_NOTES


def test_an_unknown_nonzero_return_code_is_recorded_verbatim_with_no_invented_meaning():
    record = make_command(return_code=7777)
    assert record.to_dict()["return_code"] == 7777
    assert "UNDOCUMENTED" in record.to_dict()["return_code_meaning"]


def test_a_documented_nonzero_return_code_is_named():
    assert "Init()" in make_command(return_code=3103).to_dict()["return_code_meaning"]


def test_a_raised_call_records_the_error_and_a_null_return_code():
    record = make_command(return_code=None, error="DDSException: participant closed")
    payload = record.to_dict()
    assert payload["return_code"] is None
    assert "participant closed" in payload["error"]
    assert payload["return_code_meaning"].startswith("no code")


def test_a_command_record_measures_how_long_the_call_blocked():
    # Move() waits for a matched subscriber before writing, so an unreachable
    # robot turns a cheap call into a long one. This is where that shows up.
    record = make_command(sent_monotonic=1001.0, returned_monotonic=1001.9)
    assert record.call_duration_s == pytest.approx(0.9)


def test_the_log_records_both_timebases_and_the_measured_clock_report():
    payload = make_log().to_metadata()
    before = payload["state_before"]
    assert before["stamp_robot_clock"] != before["received_at"]
    assert before["stamp_local_clock"] is not None
    assert payload["clock"]["robot_vs_local"]["trustworthy"] is True


def test_the_log_never_contains_a_hardcoded_clock_offset():
    # The Go2 offset drifts ~90 ppm and was 2628.60 on October 7 and 2687.29 a
    # week later. A log carrying the old constant would be silently wrong.
    text = json.dumps(make_log().to_metadata())
    assert "2628.6" not in text


def test_a_dry_run_log_contains_no_command_records():
    log = make_log(dry_run=True, commands=[], result=StageResult("dry-run"))
    payload = log.to_metadata()
    assert payload["dry_run"] is True
    assert payload["commands"] == []
    assert payload["result"]["outcome"] == "dry-run"


def test_a_refused_run_still_records_the_state_and_the_refusals():
    # A refusal is evidence, so it belongs on disk like any other outcome.
    log = make_log(commands=[],
                   readiness=ReadinessVerdict(False, ("feet [0, 1, 2, 3] are below 50 force",),
                                              {"foot_force": [7, 14, 14, 7]}),
                   result=StageResult("refused", refusals=("the robot is not standing",)))
    payload = log.to_metadata()
    assert payload["result"]["outcome"] == "refused"
    assert payload["result"]["refusals"] == ["the robot is not standing"]
    assert payload["readiness"]["ready"] is False
    assert payload["state_before"] is not None


def test_the_log_names_the_abort_reason_and_whether_it_escalated():
    log = make_log(result=StageResult(
        "aborted", abort=AbortVerdict("travelled 0.40 m, cap 0.15 m", "displacement")))
    payload = make_log().to_metadata()
    assert payload["result"]["abort"] is None
    aborted = log.to_metadata()["result"]["abort"]
    assert aborted["predicate"] == "displacement"
    assert aborted["escalate_to_damp"] is False


def test_moves_written_is_labelled_as_writes_rather_than_acknowledgements():
    payload = make_log().to_metadata()
    assert "not acknowledgements" in payload["result"]["moves_written_note"]


def test_motion_onset_appears_in_the_result_so_a_silent_robot_is_visible():
    quiet = make_log(result=StageResult("completed", commands_sent=41, moves_written=41,
                                        measurement=Measurement(motion_onset_s=None)))
    measurement = quiet.to_metadata()["result"]["measurement"]
    assert measurement["motion_onset_s"] is None
    assert "never moved" in measurement["motion_onset_note"]


def test_the_measurement_reports_odometry_against_the_tape_measure():
    # At a first bring-up the tape is ground truth and the odometry is under
    # test, so a lone odometry figure is a claim rather than a measurement.
    measurement = Measurement(translation_m=0.24, tape_translation_m=0.20)
    payload = measurement.to_dict()
    assert payload["odometry_over_tape"] == pytest.approx(1.2)
    assert "under test" in payload["ground_truth_note"]


def test_rotation_is_compared_against_the_operators_own_measurement():
    # The yaw stages turn on this comparison, so a stored rotation reading that
    # was never compared to anything would be a field that earns nothing.
    import math
    measurement = Measurement(rotation_rad=math.radians(30.0), tape_rotation_deg=25.0)
    payload = measurement.to_dict()
    assert payload["rotation_deg"] == pytest.approx(30.0)
    assert payload["rotation_odometry_over_tape"] == pytest.approx(1.2)


def test_the_measurement_tolerates_a_missing_tape_reading():
    payload = Measurement(translation_m=0.24, rotation_rad=0.45).to_dict()
    assert payload["odometry_over_tape"] is None
    assert payload["rotation_odometry_over_tape"] is None


def test_stop_distance_is_recorded_separately_from_stage_displacement():
    measurement = Measurement(translation_m=0.22, stop_translation_m=0.03,
                              settle_time_s=0.18)
    payload = measurement.to_dict()
    assert payload["translation_m"] == 0.22
    assert payload["stop_translation_m"] == 0.03
    assert payload["settle_time_s"] == 0.18


def test_the_default_output_root_is_off_the_repository_tree():
    # `.gitignore` does not cover motion.json, so the default location is the
    # only thing keeping a run out of a public repository.
    assert DEFAULT_OUTPUT_ROOT.startswith("/mnt/data")


def test_saving_writes_a_readable_log_and_returns_the_directory(tmp_path):
    out = make_log().save(tmp_path / "run")
    assert out == tmp_path / "run"
    assert (out / "motion.json").read_text().endswith("\n")
    assert load_motion_log(out)["run_id"].endswith("_yaw")


def test_saving_twice_overwrites_in_place_so_a_killed_run_keeps_its_partial_log():
    # The watchdog-kill stage is killed with SIGKILL, which runs no handler, so
    # a log written only at the end is the log that stage cannot produce.
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        log = make_log("watchdog-kill", finished_at=None,
                       result=StageResult("awaiting-kill", commands_sent=20))
        log.save(tmp)
        log.commands.append(make_command(index=1))
        log.save(tmp)
        payload = load_motion_log(tmp)
        assert len(payload["commands"]) == 2
        assert payload["finished_at"] is None
        assert payload["result"]["outcome"] == "awaiting-kill"


def test_loading_something_that_is_not_a_motion_log_is_refused(tmp_path):
    (tmp_path / "motion.json").write_text('{"hello": "world"}\n')
    with pytest.raises(ValueError, match="does not look like"):
        load_motion_log(tmp_path)


def test_loading_malformed_json_is_refused(tmp_path):
    (tmp_path / "motion.json").write_text("{not json")
    with pytest.raises(ValueError, match="not valid JSON"):
        load_motion_log(tmp_path)


def test_loading_a_missing_log_is_refused(tmp_path):
    with pytest.raises(OSError, match="cannot read a motion log"):
        load_motion_log(tmp_path / "nope")


def test_the_readiness_observations_survive_the_round_trip_including_mode():
    log = make_log(readiness=ReadinessVerdict(
        True, (), {"mode": 7, "foot_force": [170, 165, 172, 168], "loaded_feet": 4}))
    payload = json.loads(json.dumps(log.to_metadata()))
    assert payload["readiness"]["observations"]["mode"] == 7
    assert payload["readiness"]["observations"]["loaded_feet"] == 4


def test_the_log_says_where_the_foot_force_threshold_came_from():
    # The threshold is robot-specific and in unknown units, so a log that
    # recorded only the number would not be auditable later.
    note = make_log().to_metadata()["limits"]["min_foot_force_note"]
    assert "measured 2026-10-14" in note
    assert "drifts" in note
    assert "Units unknown" in note


def test_the_log_records_the_stage_schedule_and_its_open_loop_bound():
    payload = make_log().to_metadata()
    assert len(payload["schedule"]) == 41
    assert payload["schedule_bounds"]["commanded_rotation_rad"] == pytest.approx(0.45,
                                                                                abs=1e-6)
