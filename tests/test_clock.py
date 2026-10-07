"""The clock estimator is what keeps a snapshot from being paired with a pose
from 44 minutes earlier, so its edge cases are worth pinning down."""

from kinetra.perception.clock import ClockOffsetEstimator


def test_unmeasured_estimator_reports_nothing_rather_than_zero():
    e = ClockOffsetEstimator()
    assert e.offset_s is None
    assert e.spread_s is None
    assert e.to_local(123.0) is None
    assert not e.trustworthy


def test_offset_is_the_minimum_not_the_mean():
    # Added transport delay must not bias the estimate upward. The true offset
    # here is 10.0; every other sample is inflated by queueing.
    e = ClockOffsetEstimator()
    for extra in (0.0, 0.5, 1.0, 2.0, 0.3, 0.9, 0.4, 0.2):
        e.observe(arrival_s=100.0 + extra, stamp_s=90.0)
    assert e.offset_s == 10.0
    assert e.spread_s == 2.0


def test_to_local_applies_the_measured_offset():
    e = ClockOffsetEstimator()
    for _ in range(8):
        e.observe(arrival_s=2628.6, stamp_s=0.0)
    assert e.to_local(100.0) == 2728.6


def test_a_stable_offset_is_trustworthy_and_a_drifting_one_is_not():
    stable = ClockOffsetEstimator()
    for i in range(16):
        stable.observe(arrival_s=1000.0 + i + 0.002, stamp_s=float(i))
    assert stable.trustworthy

    drifting = ClockOffsetEstimator(max_spread_s=0.1)
    for i in range(16):
        drifting.observe(arrival_s=1000.0 + i + 0.5 * i, stamp_s=float(i))
    assert not drifting.trustworthy
    assert drifting.offset_s is not None  # still reported, just not trusted


def test_too_few_samples_is_not_trustworthy_however_clean():
    e = ClockOffsetEstimator()
    for _ in range(7):
        e.observe(arrival_s=5.0, stamp_s=0.0)
    assert e.spread_s == 0.0
    assert not e.trustworthy
    e.observe(arrival_s=5.0, stamp_s=0.0)
    assert e.trustworthy


def test_window_bounds_memory_and_forgets_stale_samples():
    e = ClockOffsetEstimator(window=4)
    e.observe(arrival_s=0.0, stamp_s=100.0)  # a very low outlier, later evicted
    for _ in range(4):
        e.observe(arrival_s=50.0, stamp_s=0.0)
    assert e.samples == 4
    assert e.offset_s == 50.0


def test_report_is_json_serialisable():
    import json

    e = ClockOffsetEstimator()
    e.observe(arrival_s=1.0, stamp_s=0.0)
    assert json.loads(json.dumps(e.report()))["samples"] == 1


def test_window_must_be_positive():
    import pytest

    with pytest.raises(ValueError):
        ClockOffsetEstimator(window=0)
