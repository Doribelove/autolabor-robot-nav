"""Observation-only map correction gate. Never publishes TF or controls navigation."""
from collections import deque
import math
import numpy as np
from .map_localization import rigid_matrix


class LandmarkStability:
    def __init__(self, window_sec=6.0, translation_limit=.12, rotation_limit_deg=2.0):
        self.window_sec = window_sec
        self.translation_limit = translation_limit
        self.rotation_limit = math.radians(rotation_limit_deg)
        self.samples = deque(maxlen=512)
        self.generation = 0
        self.status = "WAITING_LOCALIZATION"
        self.matrix = None
        self.last_stamp = None

    def invalidate(self, reason):
        if self.matrix is not None or self.samples:
            self.generation += 1
        self.samples.clear()
        self.matrix = None
        self.status = reason

    def observe(self, stamp, correction):
        correction = rigid_matrix(correction).copy()
        if not math.isfinite(stamp) or stamp <= 0:
            self.invalidate("INVALID_STAMP")
            return
        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                self.invalidate("CLOCK_RESET")
                self.last_stamp = stamp
                return
            if stamp - self.last_stamp > .5:
                self.invalidate("OBSERVATION_GAP")
        self.last_stamp = stamp
        if self.samples:
            anchor = self.samples[0][1]
            rotation = correction[:3, :3] @ anchor[:3, :3].T
            angle = math.acos(float(np.clip((np.trace(rotation)-1)/2, -1, 1)))
            if (np.linalg.norm(correction[:3, 3]-anchor[:3, 3]) > self.translation_limit
                    or angle > self.rotation_limit):
                self.invalidate("MAP_CORRECTION_JUMP")
        self.samples.append((stamp, correction))
        # Keep the sample bracketing the full window. Six seconds spans several
        # 0.5 Hz ICP opportunities; detector frames are not independent ICP votes.
        while len(self.samples) > 1 and stamp-self.samples[1][0] >= self.window_sec:
            self.samples.popleft()
        if stamp-self.samples[0][0] < self.window_sec:
            self.matrix = None
            if self.status != "MAP_CORRECTION_JUMP":
                self.status = "STABILIZING"
            return
        matrices = np.stack([item[1] for item in self.samples])
        candidate = np.eye(4)
        candidate[:3, 3] = np.median(matrices[:, :3, 3], axis=0)
        u, _, vh = np.linalg.svd(np.mean(matrices[:, :3, :3], axis=0))
        candidate[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vh)]) @ vh
        self.matrix = rigid_matrix(candidate)
        self.status = "STABLE"
