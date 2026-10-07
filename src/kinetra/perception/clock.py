"""Bridging the Go2's clock and the Jetson's.

The two timebases on this robot do not agree. Measured on October 7, 2026
(see `docs/ROBOT_INVENTORY.md`): message stamps originating on the Go2 run about
2628.6 s behind the Jetson's NTP-synchronised clock, while the RealSense driver
stamps frames on the Jetson's own clock. Pairing a camera frame with an odometry
pose by raw `header.stamp` therefore pairs observations 44 minutes apart and
reports no error at all, which is the worst possible failure mode for a system
whose output is "the person is here".

The offset is, however, stable: it varied by only 6 ms across a 6 s window. So the
honest fix is not to throw stamps away and pair by arrival time -- that would
inherit every scheduling and queueing delay -- but to *measure* the offset and
convert robot stamps into the local timebase.

The estimator below is the standard one-way-delay trick used by NTP: over a
window of samples, the **minimum** of ``arrival - stamp`` is the best estimate of
the clock offset, because transport, queueing and scheduling can only ever add
delay, never remove it. The spread between minimum and maximum is reported too,
because a spread that grows is the signal that the clocks are drifting apart and
the estimate can no longer be trusted.
"""

from __future__ import annotations

from collections import deque

# Beyond this spread the two clocks are not merely offset, they are drifting, and
# a single offset no longer describes them. Chosen well above the 6 ms observed on
# this robot and well below the pose staleness we would accept for a snapshot.
DEFAULT_MAX_SPREAD_S = 0.25


class ClockOffsetEstimator:
    """Estimates ``local_time - remote_stamp`` for one remote publisher."""

    def __init__(self, window: int = 256, max_spread_s: float = DEFAULT_MAX_SPREAD_S) -> None:
        if window < 1:
            raise ValueError("window must be at least 1")
        self._deltas: deque[float] = deque(maxlen=window)
        self._max_spread_s = max_spread_s

    def observe(self, arrival_s: float, stamp_s: float) -> None:
        """Record one (local arrival time, remote header stamp) pair."""
        self._deltas.append(arrival_s - stamp_s)

    @property
    def samples(self) -> int:
        return len(self._deltas)

    @property
    def offset_s(self) -> float | None:
        """Best estimate of ``local_time - remote_stamp``, or None if unmeasured.

        The minimum is used rather than the mean: added delay biases the mean
        upward, but cannot push any single sample below the true offset.
        """
        return min(self._deltas) if self._deltas else None

    @property
    def spread_s(self) -> float | None:
        """Observed range of the delta. Small means a clean constant offset."""
        if not self._deltas:
            return None
        return max(self._deltas) - min(self._deltas)

    @property
    def trustworthy(self) -> bool:
        """Whether the offset can be used to convert a stamp.

        Requires enough samples to have plausibly seen a low-delay one, and a
        spread small enough that a single offset still describes both clocks.
        """
        spread = self.spread_s
        return self.samples >= 8 and spread is not None and spread <= self._max_spread_s

    def to_local(self, stamp_s: float) -> float | None:
        """Convert a remote header stamp into local time, or None if unmeasured."""
        offset = self.offset_s
        return None if offset is None else stamp_s + offset

    def report(self) -> dict:
        """A JSON-serialisable record, stored with every snapshot for auditing."""
        return {
            "samples": self.samples,
            "offset_s": self.offset_s,
            "spread_s": self.spread_s,
            "trustworthy": self.trustworthy,
            "max_spread_s": self._max_spread_s,
        }
