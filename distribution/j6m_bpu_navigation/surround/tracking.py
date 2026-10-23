"""Conservative cross-camera association and persistent experimental map targets."""
import copy
import itertools
import math
from collections import deque

import numpy as np

from . import MATERIALS


def distance(a, b):
    return float(np.linalg.norm(np.asarray(a["map_xyz"])[:2] - np.asarray(b["map_xyz"])[:2]))


def fuse(observations, overlap_pairs, max_dt_ns=100_000_000, max_distance=.4, ambiguity=.08):
    """Mutual unambiguous nearest neighbours, with at most one box per camera.

    Spatial thresholds alone cannot prove that two nearby pieces are one object.
    Ambiguous pairs stay separate. Material is a noisy cue, not a hard class.
    """
    obs = sorted(observations, key=lambda x: x["id"])
    if len({o["id"] for o in obs}) != len(obs):
        raise ValueError("Repeated observation identity")
    allowed = {frozenset(p) for p in overlap_pairs}
    groups = [{i} for i in range(len(obs))]
    candidates = {}
    for i, a in enumerate(obs):
        candidates[i] = []
        for j, b in enumerate(obs):
            if (i == j or frozenset((a["camera"], b["camera"])) not in allowed
                    or a["camera"] == b["camera"] or a["class_id"] != b["class_id"]
                    or a["map_id"] != b["map_id"]
                    or a["localization_session"] != b["localization_session"]
                    or abs(a["stamp_ns"] - b["stamp_ns"]) > max_dt_ns):
                continue
            d = distance(a, b)
            if d <= max_distance:
                candidates[i].append((d, j))
        candidates[i].sort()

    def nearest(i, camera):
        same = [(d, j) for d, j in candidates[i] if obs[j]["camera"] == camera]
        if not same or (len(same) > 1 and same[1][0] - same[0][0] < ambiguity):
            return None
        return same[0]

    edges = []
    for i, a in enumerate(obs):
        for camera in {obs[j]["camera"] for _, j in candidates[i]}:
            value = nearest(i, camera)
            if value:
                d, j = value
                reverse = nearest(j, a["camera"])
                if reverse and reverse[1] == i:
                    edges.append((d, i, j))
    for _, i, j in sorted(edges):
        left = next(g for g in groups if i in g)
        right = next(g for g in groups if j in g)
        if left is right:
            continue
        joint = left | right
        if len({obs[k]["camera"] for k in joint}) != len(joint):
            continue
        # Do not merge transitive chains whose endpoints fail the gates.
        if any(abs(obs[a]["stamp_ns"] - obs[b]["stamp_ns"]) > max_dt_ns
               or distance(obs[a], obs[b]) > max_distance
               or frozenset((obs[a]["camera"], obs[b]["camera"])) not in allowed
               for a, b in itertools.combinations(joint, 2)):
            continue
        left.update(right)
        groups.remove(right)
    result = []
    for group in groups:
        members = [obs[k] for k in sorted(group)]
        best = max(members, key=lambda o: o["detect_confidence"])
        item = copy.deepcopy(best)
        item["sources"] = [{k: m[k] for k in (
            "id", "camera", "frame_key", "stamp_ns", "raw_stamp", "clock_mapping_id",
            "clock_id", "calibration_id", "rectification_id", "bbox",
            "detect_confidence", "material", "classify_confidence")}
            for m in members]
        # Selecting one estimate avoids assuming correlated calibration errors
        # disappear when several cameras see the same object.
        item["last_observed_ns"] = max(m["stamp_ns"] for m in members)
        result.append(item)
    return result


class TargetStore:
    def __init__(self, session, confirm_hits=3, confirm_window_ns=2_000_000_000,
                 independent_ns=100_000_000, stale_ns=3_000_000_000, max_targets=1000):
        self.session = session
        self.identity = None
        self.targets = {}
        self.counter = 0
        self.confirm_hits, self.confirm_window_ns = confirm_hits, confirm_window_ns
        self.independent_ns, self.stale_ns = independent_ns, stale_ns
        self.max_targets = max_targets
        self.resets = 0

    def reset_if_changed(self, map_id, localization_session, calibration_id):
        identity = (map_id, localization_session, calibration_id)
        if self.identity != identity:
            self.targets.clear()
            self.identity = identity
            self.resets += 1

    def update(self, observations, now_ns, visibility=None):
        if observations:
            identities = {(o["map_id"], o["localization_session"], o["calibration_id"])
                          for o in observations}
            if len(identities) != 1:
                raise ValueError("Mixed map/localization/calibration epochs")
            self.reset_if_changed(*next(iter(identities)))
        matched, consumed = set(), set()
        edges = []
        for i, o in enumerate(observations):
            for key, target in self.targets.items():
                if target["class_id"] == o["class_id"] and distance(target, o) <= .4:
                    edges.append((distance(target, o), key, i))
        # One-to-one assignment, reject ambiguous alternatives rather than
        # collapsing adjacent pieces. A completed tombstone also participates.
        for d, key, i in sorted(edges):
            if key in matched or i in consumed:
                continue
            alternatives = [e[0] for e in edges if
                            (e[1] == key and e[2] != i) or (e[2] == i and e[1] != key)]
            if alternatives and min(alternatives) - d < .08:
                continue
            matched.add(key)
            consumed.add(i)
            self._observe(self.targets[key], observations[i])
        for i, o in enumerate(observations):
            if i in consumed:
                continue
            # Keep ambiguous observations as observations; do not manufacture
            # extra persistent targets next to an existing ambiguous track.
            if any(e[2] == i for e in edges):
                continue
            if len(self.targets) >= self.max_targets:
                raise ValueError("Target capacity reached; archive/reset explicitly")
            self.counter += 1
            key = "%s/%d" % (self.session, self.counter)
            target = copy.deepcopy(o)
            target.update(id=key, status="tentative", first_observed_ns=o["stamp_ns"],
                          last_observed_ns=o["stamp_ns"], hits=0, visible_misses=0,
                          vote=[0.] * 5, vote_weight=0., confirmation_times=[],
                          independent_stamp_ns=-10**18, last_visibility_ns=0)
            self.targets[key] = target
            self._observe(target, o)
            matched.add(key)
        for key, target in list(self.targets.items()):
            if key not in matched and visibility is not None:
                visible_stamp = visibility(target)
                if visible_stamp and visible_stamp > max(target["last_observed_ns"],
                                                          target["last_visibility_ns"]):
                    target["visible_misses"] += 1
                    target["last_visibility_ns"] = visible_stamp
            age = now_ns - target["last_observed_ns"]
            if target["status"] == "tentative" and age > self.confirm_window_ns:
                del self.targets[key]
            elif target["status"] in ("confirmed", "stale") and (
                    age > self.stale_ns or target["visible_misses"] >= 5):
                target["status"] = "stale"
        return self.snapshot(now_ns)

    def _observe(self, target, observation):
        stamp = observation["last_observed_ns"]
        if stamp < target["last_observed_ns"]:
            return
        target["last_observed_ns"] = stamp
        target["sources"] = observation["sources"]
        if target["status"] == "completed":
            return
        target["map_xyz"] = list(observation["map_xyz"])
        target["base_xyz"] = list(observation["base_xyz"])
        target["covariance_xy"] = copy.deepcopy(observation["covariance_xy"])
        target["visible_misses"] = 0
        target["detect_confidence"] = float(observation["detect_confidence"])
        if stamp - target["independent_stamp_ns"] >= self.independent_ns:
            target["independent_stamp_ns"] = stamp
            target["hits"] += 1
            times = [s for s in target["confirmation_times"]
                     if stamp - s <= self.confirm_window_ns]
            target["confirmation_times"] = (times + [stamp])[-16:]
            p = observation.get("probabilities")
            if p is not None:
                target["vote"] = (np.asarray(target["vote"]) * .8 + np.asarray(p)).tolist()
                target["vote_weight"] = target["vote_weight"] * .8 + 1
                average = np.asarray(target["vote"]) / target["vote_weight"]
                target["material"] = MATERIALS[int(average.argmax())]
                target["classify_confidence"] = float(average.max())
        if (len(target["confirmation_times"]) >= self.confirm_hits
                and target["status"] in ("tentative", "stale")):
            target["status"] = "confirmed"

    def set_status(self, target_id, status):
        if status not in ("claimed", "completed", "needs_recheck"):
            raise ValueError("Unknown task observation status")
        target = self.targets[target_id]
        if target["status"] == "tentative":
            raise ValueError("Tentative targets cannot enter a task")
        target["status"] = status

    def snapshot(self, now_ns):
        result = []
        for target in self.targets.values():
            item = {k: copy.deepcopy(v) for k, v in target.items()
                    if k not in ("vote", "vote_weight", "confirmation_times")}
            item["age_ms"] = max(0, now_ns - target["last_observed_ns"]) / 1e6
            item["confidence"] = target["detect_confidence"] * math.exp(
                -max(0, now_ns - target["last_observed_ns"]) / 5e9)
            item["production_ready"] = False
            item["motion_eligible"] = False
            result.append(item)
        return result
