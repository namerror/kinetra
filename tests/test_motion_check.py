"""The parts of the command shell that can be tested without a robot.

`motion_check` imports `unitree_sdk2py` lazily, inside the functions that need
it, so importing the module costs nothing and its CLI, its dry-run backend and
its signature handling are all reachable here. That matters: the alternative is
the state `capture.py` is in, where the whole module is hardware-only and
nothing guards it against regression.

What remains genuinely hardware-only is listed in `docs/MOTION.md`.
"""

import json
import sys

import pytest

from kinetra.robot.limits import MotionLimits, StateSignature, Velocity
from kinetra.robot.motion_check import (
    EXIT_ABORTED,
    EXIT_INCONCLUSIVE,
    EXIT_NO_DATA,
    EXIT_OK,
    EXIT_REFUSED,
    HANDSHAKE_TIMEOUT_S,
    LOOP_TIMEOUT_S,
    SHUTDOWN_TIMEOUT_S,
    STATE_QUEUE_LEN,
    DryRunBackend,
    MotionBackend,
    build_parser,
    main,
    read_signature,
    write_signature,
)


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


def test_importing_the_command_shell_pulls_in_neither_ros_nor_the_sdk():
    # The SDK imports are deliberately inside the functions that use them, so
    # that this module stays importable -- and testable -- off the robot.
    assert "rclpy" not in sys.modules
    assert "unitree_sdk2py" not in sys.modules


# -- the CLI ----------------------------------------------------------------

def test_a_stage_must_be_named_and_cannot_be_defaulted():
    # A defaulted stage is a move nobody chose.
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_naming_a_stage_and_listing_them_are_mutually_exclusive():
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--stage", "yaw", "--list-stages"])


def test_an_unknown_stage_is_rejected_by_the_parser():
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--stage", "backflip"])


def test_the_parser_offers_no_flag_for_reverse_or_lateral_speed():
    # Refused structurally: there is nothing to validate and nothing to raise.
    text = build_parser().format_help()
    assert "--max-backward" not in text
    assert "--max-lateral" not in text


def test_the_parser_defaults_to_sending_nothing_and_to_the_go2_interface():
    args = build_parser().parse_args(["--stage", "handshake"])
    assert args.dry_run is False
    assert args.network_interface == "eno1"
    assert args.out.startswith("/mnt/data")


def test_there_is_no_yes_flag_that_skips_the_typed_confirmation():
    assert "--yes" not in build_parser().format_help()


def test_listing_the_stages_sends_nothing_and_succeeds(capsys):
    assert main(["--list-stages"]) == EXIT_OK
    out = capsys.readouterr().out
    assert "stage yaw" in out
    assert "proves the script, not the robot" in out


def test_a_command_rate_above_the_ceiling_is_refused_before_anything_connects(capsys):
    assert main(["--stage", "yaw", "--command-rate", "500"]) == EXIT_REFUSED
    assert "exceeds the hard ceiling" in capsys.readouterr().err


def test_a_motion_stage_without_a_standing_signature_is_refused(tmp_path, capsys):
    assert main(["--stage", "yaw", "--dry-run", "--offline",
                 "--out", str(tmp_path)]) == EXIT_REFUSED
    assert "no standing signature" in capsys.readouterr().err


def test_the_kill_stage_is_refused_without_evidence_the_remote_can_stop_us(tmp_path,
                                                                          capsys):
    write_signature(tmp_path / "standing-signature.json", make_state(), {})
    assert main(["--stage", "watchdog-kill", "--dry-run", "--offline",
                 "--out", str(tmp_path)]) == EXIT_REFUSED
    assert "--estop-evidence is required" in capsys.readouterr().err


def test_a_refused_run_is_still_written_to_disk(tmp_path):
    # A refusal is evidence and belongs on disk like any other outcome.
    main(["--stage", "yaw", "--dry-run", "--offline", "--out", str(tmp_path)])
    logs = list(tmp_path.glob("*_yaw/motion.json"))
    assert len(logs) == 1
    payload = json.loads(logs[0].read_text())
    assert payload["result"]["outcome"] == "refused"
    assert payload["result"]["refusals"]


def test_a_live_motion_run_needs_an_operator_and_a_spotter(tmp_path, capsys):
    write_signature(tmp_path / "standing-signature.json", make_state(), {})
    assert main(["--stage", "yaw", "--offline", "--out", str(tmp_path)]) == EXIT_REFUSED
    assert "--operator and --spotter" in capsys.readouterr().err


def test_the_forward_stage_needs_a_measured_clearance_for_a_live_run(tmp_path, capsys):
    write_signature(tmp_path / "standing-signature.json", make_state(), {})
    assert main(["--stage", "forward", "--offline", "--out", str(tmp_path),
                 "--operator", "a", "--spotter", "b"]) == EXIT_REFUSED
    assert "--clearance is required" in capsys.readouterr().err


# -- per-phase timeouts -----------------------------------------------------

def test_the_loop_timeout_is_far_shorter_than_the_handshake_timeout():
    # Move() waits for a matched subscriber before writing, so a 1.0 s timeout
    # would turn a 20 Hz loop into a 1 Hz one exactly when the robot goes away.
    assert LOOP_TIMEOUT_S < SHUTDOWN_TIMEOUT_S < HANDSHAKE_TIMEOUT_S
    assert LOOP_TIMEOUT_S <= 0.2


def test_the_state_queue_is_short_because_the_reader_drops_the_newest_sample():
    # BQueue.Put discards the NEW sample when full, so a long queue on a 500 Hz
    # topic accumulates stale backlog and throws away current state.
    assert STATE_QUEUE_LEN <= 8


# -- the dry-run backend ----------------------------------------------------

def test_the_dry_run_backend_mirrors_the_real_signatures_including_tuples():
    # The lab bridge's stub answered every attribute with a bare 0, so the real
    # GetServerApiVersion tuple unpacking never ran in dry-run and would have
    # raised live -- a crash introduced by the de-risking mechanism.
    backend = DryRunBackend()
    code, version = backend.server_api_version()
    assert (code, version) == (0, "1.0.0.1")
    mode_code, mode = backend.check_mode()
    assert mode_code == 0 and isinstance(mode, dict)


def test_the_dry_run_backend_refuses_an_attribute_the_real_one_lacks():
    # A typo must fail identically in both, not be fine in dry-run and an
    # AttributeError mid-move.
    with pytest.raises(AttributeError):
        DryRunBackend().stop_move_now  # noqa: B018


def test_the_dry_run_backend_records_what_it_would_have_sent():
    backend = DryRunBackend()
    backend.move(Velocity(wz=0.3))
    backend.move(Velocity())
    assert backend.sent == [Velocity(wz=0.3), Velocity()]


def test_the_dry_run_backend_can_return_a_failure_so_that_path_runs_too():
    backend = DryRunBackend(move_code=3102, stop_code=3104, api_code=3001)
    assert backend.move(Velocity()) == 3102
    assert backend.stop_move() == 3104
    assert backend.server_api_version() == (3001, None)


def test_the_dry_run_backend_satisfies_the_backend_protocol():
    assert isinstance(DryRunBackend(), MotionBackend)


# -- the standing signature -------------------------------------------------

def test_a_signature_round_trips_and_records_the_lightest_foot(tmp_path):
    path = tmp_path / "sig.json"
    write_signature(path, make_state(foot_force=(170, 165, 172, 168)), {"offset_s": 2687.4})
    payload = read_signature(path)
    assert payload is not None
    assert payload["foot_force_min"] == 165
    assert payload["clock"]["offset_s"] == 2687.4


def test_a_signature_says_mode_and_body_height_are_reference_only(tmp_path):
    path = tmp_path / "sig.json"
    write_signature(path, make_state(), {})
    assert "lying down" in read_signature(path)["note"]


def test_a_missing_or_malformed_signature_reads_as_absent(tmp_path):
    assert read_signature(tmp_path / "nope.json") is None
    bad = tmp_path / "bad.json"
    bad.write_text("{not json")
    assert read_signature(bad) is None
    wrong = tmp_path / "wrong.json"
    wrong.write_text('{"hello": "world"}')
    assert read_signature(wrong) is None


def test_the_gate_threshold_is_not_derived_from_a_stance_snapshot(tmp_path):
    # Fitting the per-foot floor to one standing reading was wrong twice: half
    # the lightest foot lands below a lying reading, and 0.85 of a good stance
    # refuses a merely adequate one. The floor exists to catch a lifted leg,
    # which reads near zero whatever the stance, so it is a measured constant.
    write_signature(tmp_path / "standing-signature.json",
                    make_state(foot_force=(200, 180, 190, 184)), {})
    main(["--stage", "yaw", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "0.1"])
    payload = json.loads(next(tmp_path.glob("*_yaw/motion.json")).read_text())
    assert payload["limits"]["min_foot_force"] == MotionLimits().min_foot_force


def test_an_explicit_threshold_flag_still_overrides_the_default(tmp_path):
    write_signature(tmp_path / "standing-signature.json", make_state(), {})
    main(["--stage", "yaw", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "0.1", "--min-foot-force", "33"])
    payload = json.loads(next(tmp_path.glob("*_yaw/motion.json")).read_text())
    assert payload["limits"]["min_foot_force"] == pytest.approx(33.0)


def test_an_offline_dry_run_never_writes_a_standing_signature(tmp_path):
    # Synthetic foot forces are not a measurement, and the signature is what
    # unlocks every motion stage.
    main(["--stage", "handshake", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "0.1"])
    assert not (tmp_path / "standing-signature.json").exists()


# -- the log the run leaves behind ------------------------------------------

def test_an_offline_run_records_that_it_proves_less(tmp_path):
    main(["--stage", "handshake", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "0.1"])
    payload = json.loads(next(tmp_path.glob("*_handshake/motion.json")).read_text())
    assert payload["environment"]["offline"] is True
    assert payload["environment"]["transport"] == "synthetic"


def test_every_run_records_that_rclpy_was_never_imported(tmp_path):
    main(["--stage", "handshake", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "0.1"])
    payload = json.loads(next(tmp_path.glob("*_handshake/motion.json")).read_text())
    assert payload["environment"]["rclpy_imported"] is False
    assert "domain 0" in payload["environment"]["rclpy_note"]


def test_a_dry_run_outcome_is_tagged_so_it_cannot_be_read_as_a_real_run(tmp_path):
    # `dry_run: true` is already in the log, but a plain "completed" outcome
    # would survive a grep over outcomes and look like the robot did something.
    # Enough warm-up to satisfy the stream-health gate, which needs 500 samples.
    main(["--stage", "handshake", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "1.4"])
    payload = json.loads(next(tmp_path.glob("*_handshake/motion.json")).read_text())
    assert payload["result"]["outcome"] == "dry-run-completed"


def test_a_dry_run_is_not_aborted_for_the_robot_not_moving(tmp_path):
    # Nothing is sent in a dry run, so `no_onset` would fire on every one of
    # them at 0.6 s and hide the loop timing the rest of the window measures.
    write_signature(tmp_path / "standing-signature.json", make_state(), {})
    code = main(["--stage", "yaw", "--dry-run", "--offline", "--out", str(tmp_path),
                 "--warmup", "1.4"])
    payload = json.loads(next(tmp_path.glob("*_yaw/motion.json")).read_text())
    assert payload["result"]["abort"] is None, payload["result"]["abort"]
    # The loop sends while elapsed < window, so the tick at exactly 2.0 s is not
    # sent -- the stop sequence's Move(0,0,0) covers it. A margin below that
    # keeps the test from being flaky on a loaded Jetson.
    assert 36 <= payload["result"]["moves_written"] <= len(payload["schedule"])
    # It is still reported as inconclusive rather than as a success.
    assert code == EXIT_INCONCLUSIVE


# -- Init() must happen on every path -------------------------------------
#
# Regression tests for a real bug: `initialise()` was called only inside
# `handshake()` and in main's dry-run branch, so a LIVE motion stage built a
# client, never registered the api ids, and had every Move rejected locally with
# 3103 -- publishing nothing at all while the log filled with ordinary-looking
# error codes. The fix moved the call into the runner, which owns both paths.

def test_a_motion_stage_initialises_the_command_path_before_commanding(tmp_path):
    write_signature(tmp_path / "standing-signature.json", make_state(), {})
    main(["--stage", "yaw", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "1.4"])
    payload = json.loads(next(tmp_path.glob("*_yaw/motion.json")).read_text())
    calls = [c["call"] for c in payload["commands"]]
    assert "SportClient.Init" in calls
    assert calls.index("SportClient.Init") < calls.index("SportClient.Move")


def test_the_handshake_stage_also_records_its_init(tmp_path):
    main(["--stage", "handshake", "--dry-run", "--offline", "--out", str(tmp_path),
          "--warmup", "1.4"])
    payload = json.loads(next(tmp_path.glob("*_handshake/motion.json")).read_text())
    calls = [c["call"] for c in payload["commands"]]
    # Init goes AFTER the version probe, which works unregistered, so that
    # "unreachable" and "not registered" stay separable findings.
    assert calls.index("SportClient.GetServerApiVersion") < calls.index("SportClient.Init")
    assert calls.index("SportClient.Init") < calls.index("SportClient.StopMove")


def test_initialising_is_part_of_the_backend_contract():
    assert hasattr(MotionBackend, "initialise")
    backend = DryRunBackend()
    assert backend.initialised is False
    backend.initialise()
    assert backend.initialised is True


def test_the_log_says_that_init_reaches_nothing_but_gates_everything():
    from kinetra.robot.motion_log import RETURN_CODE_NOTES
    note = RETURN_CODE_NOTES["SportClient.Init"]
    assert "3103" in note
    assert "never reaches the robot" in note


def test_a_command_that_raises_aborts_the_stage(tmp_path):
    # A raised call records no return code, and `None` reads as "no code yet" to
    # the abort monitor, so without an explicit check an exception on every tick
    # would go unnoticed and the stage would run to completion looking fine.
    write_signature(tmp_path / "standing-signature.json", make_state(), {})
    code = main(["--stage", "yaw", "--dry-run", "--offline", "--out", str(tmp_path),
                 "--warmup", "1.4", "--sim-move-raise"])
    assert code == EXIT_ABORTED
    payload = json.loads(next(tmp_path.glob("*_yaw/motion.json")).read_text())
    abort = payload["result"]["abort"]
    assert abort["predicate"] == "move_raised"
    assert "injected failure" in abort["reason"]
    # And it stopped on the first tick rather than grinding through the window.
    assert payload["result"]["moves_written"] == 1


def test_a_missing_return_code_alone_does_not_abort_the_monitor():
    # Documents why the check above lives in the runner: `None` is also what the
    # monitor sees before any call has been made, so it cannot mean failure.
    from kinetra.robot.limits import AbortCriteria, AbortMonitor
    monitor = AbortMonitor(make_state(), AbortCriteria())
    assert monitor.observe(make_state(), 1000.0, Velocity(wz=0.3),
                           return_code=None) is None


def test_the_exit_codes_are_distinct():
    assert len({EXIT_OK, EXIT_NO_DATA, EXIT_REFUSED}) == 3
