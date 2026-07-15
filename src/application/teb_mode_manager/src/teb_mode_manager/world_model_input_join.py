"""Atomic bounded cache join for the three world-model supervisor inputs."""

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional


STREAMS = ("geometry", "tracks", "health")


@dataclass(frozen=True)
class WorldModelJoinResult:
    valid: bool
    payloads: Optional[Mapping[str, Any]]
    reason: str
    world_model_sequence: Optional[int]
    newest_observed_sequence: Optional[int]
    sequence_lag: Optional[int]
    timestamp_spread_s: Optional[float]
    maximum_arrival_age_s: Optional[float]
    cache_sizes: Mapping[str, int]


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


class BoundedWorldModelInputJoin:
    """Join geometry, tracks and health without a latest-message race.

    The three messages are emitted from one world-model update and therefore
    must share one exact ``world_model_seq``. Caches absorb callback ordering;
    source-stamp spread, lag behind the newest observed sequence, arrival age
    and capacity are independently bounded. No cross-sequence synthesis is
    permitted because that would manufacture a world state that never existed.
    """

    def __init__(
        self,
        *,
        maximum_entries_per_stream=32,
        maximum_arrival_age_s=1.0,
        maximum_sequence_lag=2,
        maximum_timestamp_spread_s=0.05,
    ):
        if (
            not isinstance(maximum_entries_per_stream, int)
            or maximum_entries_per_stream <= 0
        ):
            raise ValueError("maximum_entries_per_stream must be a positive int")
        if not isinstance(maximum_sequence_lag, int) or maximum_sequence_lag < 0:
            raise ValueError("maximum_sequence_lag must be a non-negative int")
        self.maximum_entries_per_stream = maximum_entries_per_stream
        self.maximum_arrival_age_s = _finite(
            maximum_arrival_age_s, "maximum_arrival_age_s"
        )
        self.maximum_sequence_lag = maximum_sequence_lag
        self.maximum_timestamp_spread_s = _finite(
            maximum_timestamp_spread_s, "maximum_timestamp_spread_s"
        )
        if (
            self.maximum_arrival_age_s <= 0.0
            or self.maximum_timestamp_spread_s < 0.0
        ):
            raise ValueError("world-model join time bounds are invalid")
        self._entries: Dict[str, Dict[int, _Entry]] = {
            stream: {} for stream in STREAMS
        }
        self._last_now_s = None

    @property
    def cache_sizes(self):
        return {stream: len(entries) for stream, entries in self._entries.items()}

    def reset(self):
        for entries in self._entries.values():
            entries.clear()
        self._last_now_s = None

    def _advance(self, now_s):
        now = _finite(now_s, "now_s")
        if self._last_now_s is not None and now < self._last_now_s:
            self.reset()
        self._last_now_s = now
        for entries in self._entries.values():
            stale = [
                sequence for sequence, entry in entries.items()
                if now - entry.arrival_stamp_s > self.maximum_arrival_age_s
            ]
            for sequence in stale:
                del entries[sequence]
        return now

    def add(self, stream, sequence, source_stamp_s, arrival_stamp_s, payload):
        if stream not in self._entries:
            raise ValueError("unknown world-model input stream: {}".format(stream))
        if not isinstance(sequence, int) or sequence < 0:
            raise ValueError("world-model sequence must be a non-negative int")
        source = _finite(source_stamp_s, "source_stamp_s")
        arrival = self._advance(arrival_stamp_s)
        entries = self._entries[stream]
        entries[sequence] = _Entry(sequence, source, arrival, payload)
        while len(entries) > self.maximum_entries_per_stream:
            oldest = min(
                entries.values(),
                key=lambda entry: (entry.arrival_stamp_s, entry.sequence),
            )
            del entries[oldest.sequence]

    def _invalid(self, reason, **values):
        return WorldModelJoinResult(
            valid=False,
            payloads=None,
            reason=reason,
            world_model_sequence=values.get("world_model_sequence"),
            newest_observed_sequence=values.get("newest_observed_sequence"),
            sequence_lag=values.get("sequence_lag"),
            timestamp_spread_s=values.get("timestamp_spread_s"),
            maximum_arrival_age_s=values.get("maximum_arrival_age_s"),
            cache_sizes=self.cache_sizes,
        )

    def resolve(self, now_s):
        now = self._advance(now_s)
        if any(not entries for entries in self._entries.values()):
            return self._invalid("WORLD_MODEL_INPUT_STREAM_MISSING")
        common = set.intersection(*(
            set(self._entries[stream]) for stream in STREAMS
        ))
        newest = max(
            max(entries) for entries in self._entries.values() if entries
        )
        if not common:
            return self._invalid(
                "NO_COMPLETE_WORLD_MODEL_SEQUENCE",
                newest_observed_sequence=newest,
            )
        sequence = max(common)
        lag = newest - sequence
        if lag > self.maximum_sequence_lag:
            return self._invalid(
                "WORLD_MODEL_SEQUENCE_LAG_EXCEEDED",
                world_model_sequence=sequence,
                newest_observed_sequence=newest,
                sequence_lag=lag,
            )
        joined = {stream: self._entries[stream][sequence] for stream in STREAMS}
        source_stamps = [entry.source_stamp_s for entry in joined.values()]
        spread = max(source_stamps) - min(source_stamps)
        if spread > self.maximum_timestamp_spread_s:
            return self._invalid(
                "WORLD_MODEL_TIMESTAMP_SPREAD_EXCEEDED",
                world_model_sequence=sequence,
                newest_observed_sequence=newest,
                sequence_lag=lag,
                timestamp_spread_s=spread,
            )
        if max(source_stamps) > now:
            return self._invalid(
                "WORLD_MODEL_SOURCE_TIMESTAMP_IN_FUTURE",
                world_model_sequence=sequence,
                newest_observed_sequence=newest,
                sequence_lag=lag,
                timestamp_spread_s=spread,
            )
        maximum_age = max(
            max(0.0, now - entry.arrival_stamp_s) for entry in joined.values()
        )
        return WorldModelJoinResult(
            valid=True,
            payloads={stream: entry.payload for stream, entry in joined.items()},
            reason="ATOMIC_EXACT_SEQUENCE_JOIN",
            world_model_sequence=sequence,
            newest_observed_sequence=newest,
            sequence_lag=lag,
            timestamp_spread_s=spread,
            maximum_arrival_age_s=maximum_age,
            cache_sizes=self.cache_sizes,
        )
