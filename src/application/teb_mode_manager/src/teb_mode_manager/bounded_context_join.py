"""Bounded sequence/time cache join for asynchronous V2 context inputs."""

from dataclasses import dataclass
import math
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class JoinResult:
    valid: bool
    payload: Optional[Any]
    reason: str
    context_sequence: int
    geometry_sequence: Optional[int]
    sequence_delta: Optional[int]
    timestamp_delta_s: Optional[float]
    arrival_age_s: Optional[float]
    cache_size: int


@dataclass(frozen=True)
class _Entry:
    sequence: int
    source_stamp_s: float
    arrival_stamp_s: float
    payload: Any


def _finite(value, name):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(name))
    return number


class BoundedContextJoin:
    """Retain recent geometry and resolve context without an exact-tick race.

    Exact sequence matches are preferred. If the exact entry has already been
    displaced, the nearest non-future sequence may be used only when both the
    sequence and source-timestamp deltas remain inside frozen bounds.
    """

    def __init__(
        self,
        *,
        maximum_entries=32,
        maximum_arrival_age_s=1.0,
        maximum_sequence_delta=2,
        maximum_timestamp_delta_s=0.45,
    ):
        if not isinstance(maximum_entries, int) or maximum_entries <= 0:
            raise ValueError("maximum_entries must be a positive int")
        if not isinstance(maximum_sequence_delta, int) or maximum_sequence_delta < 0:
            raise ValueError("maximum_sequence_delta must be a non-negative int")
        self.maximum_entries = maximum_entries
        self.maximum_arrival_age_s = _finite(
            maximum_arrival_age_s, "maximum_arrival_age_s"
        )
        self.maximum_sequence_delta = maximum_sequence_delta
        self.maximum_timestamp_delta_s = _finite(
            maximum_timestamp_delta_s, "maximum_timestamp_delta_s"
        )
        if self.maximum_arrival_age_s <= 0.0 or self.maximum_timestamp_delta_s < 0.0:
            raise ValueError("join time bounds are invalid")
        self._entries: Dict[int, _Entry] = {}
        self._last_now_s = None

    @property
    def size(self):
        return len(self._entries)

    def reset(self):
        self._entries.clear()
        self._last_now_s = None

    def _advance(self, now_s):
        now = _finite(now_s, "now_s")
        if self._last_now_s is not None and now < self._last_now_s:
            self.reset()
        self._last_now_s = now
        stale = [
            sequence for sequence, entry in self._entries.items()
            if now - entry.arrival_stamp_s > self.maximum_arrival_age_s
        ]
        for sequence in stale:
            del self._entries[sequence]
        return now

    def add(self, sequence, source_stamp_s, arrival_stamp_s, payload):
        if not isinstance(sequence, int) or sequence < 0:
            raise ValueError("geometry sequence must be a non-negative int")
        source = _finite(source_stamp_s, "source_stamp_s")
        arrival = self._advance(arrival_stamp_s)
        self._entries[sequence] = _Entry(sequence, source, arrival, payload)
        while len(self._entries) > self.maximum_entries:
            oldest = min(
                self._entries.values(),
                key=lambda entry: (entry.arrival_stamp_s, entry.sequence),
            )
            del self._entries[oldest.sequence]

    def resolve(self, context_sequence, context_stamp_s, now_s):
        if not isinstance(context_sequence, int) or context_sequence < 0:
            raise ValueError("context sequence must be a non-negative int")
        context_stamp = _finite(context_stamp_s, "context_stamp_s")
        now = self._advance(now_s)
        if not self._entries:
            return JoinResult(
                False, None, "GEOMETRY_CACHE_EMPTY", context_sequence,
                None, None, None, None, 0,
            )
        candidates = [
            entry for entry in self._entries.values()
            if entry.sequence <= context_sequence
        ]
        bounded = []
        for entry in candidates:
            sequence_delta = context_sequence - entry.sequence
            timestamp_delta = context_stamp - entry.source_stamp_s
            if (
                0 <= sequence_delta <= self.maximum_sequence_delta
                and 0.0 <= timestamp_delta <= self.maximum_timestamp_delta_s
            ):
                bounded.append((sequence_delta, timestamp_delta, entry))
        if not bounded:
            return JoinResult(
                False, None, "NO_BOUNDED_GEOMETRY_MATCH", context_sequence,
                None, None, None, None, len(self._entries),
            )
        sequence_delta, timestamp_delta, entry = min(
            bounded, key=lambda row: (row[0], row[1], -row[2].sequence)
        )
        age = max(0.0, now - entry.arrival_stamp_s)
        return JoinResult(
            True,
            entry.payload,
            "EXACT_SEQUENCE_JOIN" if sequence_delta == 0 else "BOUNDED_SEQUENCE_TIME_JOIN",
            context_sequence,
            entry.sequence,
            sequence_delta,
            timestamp_delta,
            age,
            len(self._entries),
        )
