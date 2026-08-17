"""Small deterministic multi-target tracker for stationary FOD objects."""

from dataclasses import dataclass
from math import hypot
from typing import List, Optional, Sequence, Tuple


@dataclass
class GroundObservation:
    class_id: int
    class_name: str
    confidence: float
    x: float
    y: float
    covariance_xx: float = 0.01
    covariance_xy: float = 0.0
    covariance_yy: float = 0.01


@dataclass
class Track:
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    x: float
    y: float
    covariance_xx: float
    covariance_xy: float
    covariance_yy: float
    first_observed: float
    last_observed: float
    hit_count: int = 1
    miss_count: int = 0

    def confirmed(self, min_hits: int) -> bool:
        return self.hit_count >= min_hits


class MultiTargetTracker:
    def __init__(
        self,
        association_distance: float = 0.6,
        alpha: float = 0.45,
        min_hits: int = 3,
        max_age: float = 0.8,
        max_misses: int = 10,
    ):
        if association_distance <= 0.0:
            raise ValueError("association_distance must be positive")
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        if min_hits <= 0 or max_age <= 0.0 or max_misses < 0:
            raise ValueError("invalid tracker lifetime parameters")
        self.association_distance = float(association_distance)
        self.alpha = float(alpha)
        self.min_hits = int(min_hits)
        self.max_age = float(max_age)
        self.max_misses = int(max_misses)
        self.tracks: List[Track] = []
        self._next_id = 1
        self._last_update: Optional[float] = None

    def _new_track(self, observation: GroundObservation, stamp: float) -> Track:
        track = Track(
            track_id=self._next_id,
            class_id=observation.class_id,
            class_name=observation.class_name,
            confidence=float(observation.confidence),
            x=float(observation.x),
            y=float(observation.y),
            covariance_xx=float(observation.covariance_xx),
            covariance_xy=float(observation.covariance_xy),
            covariance_yy=float(observation.covariance_yy),
            first_observed=float(stamp),
            last_observed=float(stamp),
        )
        self._next_id += 1
        return track

    def update(
        self, observations: Sequence[GroundObservation], stamp: float
    ) -> List[Track]:
        stamp = float(stamp)
        if self._last_update is not None and stamp < self._last_update:
            return list(self.tracks)
        self._last_update = stamp

        candidates: List[Tuple[float, int, int]] = []
        for track_index, track in enumerate(self.tracks):
            for observation_index, observation in enumerate(observations):
                if (
                    track.class_id != observation.class_id
                    and track.class_name != observation.class_name
                ):
                    continue
                distance = hypot(track.x - observation.x, track.y - observation.y)
                if distance <= self.association_distance:
                    candidates.append((distance, track_index, observation_index))
        candidates.sort(key=lambda item: item[0])

        matched_tracks = set()
        matched_observations = set()
        for _, track_index, observation_index in candidates:
            if track_index in matched_tracks or observation_index in matched_observations:
                continue
            track = self.tracks[track_index]
            observation = observations[observation_index]
            alpha = self.alpha
            residual_x = observation.x - track.x
            residual_y = observation.y - track.y
            track.x += alpha * residual_x
            track.y += alpha * residual_y
            track.confidence = (
                alpha * observation.confidence + (1.0 - alpha) * track.confidence
            )
            track.covariance_xx = max(
                1e-6,
                alpha * observation.covariance_xx
                + (1.0 - alpha) * track.covariance_xx
                + alpha * (1.0 - alpha) * residual_x * residual_x,
            )
            track.covariance_xy = (
                alpha * observation.covariance_xy
                + (1.0 - alpha) * track.covariance_xy
                + alpha * (1.0 - alpha) * residual_x * residual_y
            )
            track.covariance_yy = max(
                1e-6,
                alpha * observation.covariance_yy
                + (1.0 - alpha) * track.covariance_yy
                + alpha * (1.0 - alpha) * residual_y * residual_y,
            )
            track.last_observed = stamp
            track.hit_count += 1
            track.miss_count = 0
            matched_tracks.add(track_index)
            matched_observations.add(observation_index)

        for index, track in enumerate(self.tracks):
            if index not in matched_tracks:
                track.miss_count += 1
        for index, observation in enumerate(observations):
            if index not in matched_observations:
                self.tracks.append(self._new_track(observation, stamp))

        self.prune(stamp)
        return list(self.tracks)

    def prune(self, now: float) -> None:
        now = float(now)
        self.tracks = [
            track
            for track in self.tracks
            if now - track.last_observed <= self.max_age
            and (self.max_misses == 0 or track.miss_count <= self.max_misses)
        ]

    def confirmed_tracks(self, now: float) -> List[Track]:
        self.prune(now)
        return [
            track
            for track in self.tracks
            if track.confirmed(self.min_hits)
            and now - track.last_observed <= self.max_age
        ]

    def select_target(
        self, now: float, policy: str = "reject_ambiguous"
    ) -> Tuple[Optional[Track], str, int]:
        candidates = self.confirmed_tracks(now)
        count = len(candidates)
        if count == 0:
            return None, "NO_CONFIRMED_TARGET", 0
        if count == 1:
            return candidates[0], "TRACKING", 1
        if policy == "highest_confidence":
            return max(candidates, key=lambda track: track.confidence), "TRACKING", count
        if policy == "oldest":
            return min(candidates, key=lambda track: track.first_observed), "TRACKING", count
        return None, "AMBIGUOUS", count
