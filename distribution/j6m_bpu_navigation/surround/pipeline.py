"""Camera -> perspective detector -> ground observations -> map targets."""
import time
import uuid

from . import SCHEMA
from .geometry import Rectifier, GroundProjector
from .tracking import TargetStore, fuse


class Pipeline:
    def __init__(self, calibration, engine, poses, config, session=None, *, offline_reference=False):
        self.calibration, self.engine, self.poses, self.config = calibration, engine, poses, config
        self.session = session or uuid.uuid4().hex
        self.rectifier = Rectifier(calibration, offline_reference=offline_reference)
        self.projector = GroundProjector(calibration, **config.get("projection", {}))
        self.store = TargetStore(self.session)
        self.pending = []
        self.latest_result = None
        self.camera_results = {}
        self.latest_images = {}
        self.latest_visibility = {}
        self.last_pose_identity = None
        self.fusion_ns = int(config.get("fusion_window_ms", 100) * 1e6)
        self.completed_observations = []

    def process(self, frame, now_ns=None):
        realtime = now_ns is None
        now_ns = time.time_ns() if now_ns is None else now_ns
        frame.validate(now_ns)
        if frame.synthetic != self.calibration.synthetic:
            raise ValueError("Synthetic/live calibration boundary violated")
        result = dict(schema=SCHEMA, session=self.session, source=frame.identity(),
                      produced_ns=now_ns, state="awaiting_intrinsics", detections=[],
                      observations=[], rejected=[], production_ready=False, motion_eligible=False)
        self.camera_results[frame.camera] = result
        def finish():
            result["produced_ns"] = time.time_ns() if realtime else now_ns
            result["source_age_ms"] = (result["produced_ns"] - frame.stamp_ns) / 1e6
            self.latest_result = result
            return result
        # Raw images remain available to the GUI before installation/calibration.
        self.latest_images[frame.camera] = (frame.stamp_ns, frame.image, False)
        try:
            image = self.rectifier.apply(frame)
        except ValueError as e:
            if not frame.rectified and not self.rectifier.offline_reference:
                result["state"] = "awaiting_gdc"
            result["rejected"].append(str(e))
            return finish()
        self.latest_images[frame.camera] = (frame.stamp_ns, image, True)
        result["inference_image"] = dict(space="rectified_perspective",
            rectification_backend="offline_reference" if self.rectifier.offline_reference else "gdc",
            rectification_id=self.calibration.cameras[frame.camera]["rectification_id"],
            width=image.shape[1], height=image.shape[0])
        objects, timing = self.engine.infer(image, frame.stamp_ns + 500_000_000)
        result.update(detections=objects, timing=timing, state="detections_only")
        try:
            frame.validate(time.time_ns() if realtime else now_ns)
            pose = self.poses.at(frame.stamp_ns)
            identity = (pose.map_id, pose.localization_session, self.calibration.id)
            if identity != self.last_pose_identity:
                self.pending.clear()
                self.latest_visibility.clear()
                self.store.reset_if_changed(*identity)
                self.last_pose_identity = identity
            self.calibration.ground_camera(frame.camera)
            self.latest_visibility[frame.camera] = pose
        except ValueError as e:
            result["rejected"].append(str(e))
            return finish()
        for index, obj in enumerate(objects):
            try:
                base, world, cov, pixel = self.projector.project(frame.camera, obj["bbox"], pose)
                observation = dict(obj, id=frame.key + "/%d" % index, frame_key=frame.key,
                                   camera=frame.camera, stamp_ns=frame.stamp_ns,
                                   raw_stamp=frame.raw_stamp, clock_mapping_id=frame.clock_mapping_id,
                                   clock_id=frame.clock_id,
                                   rectification_id=self.calibration.cameras[frame.camera]["rectification_id"],
                                   calibration_id=self.calibration.id, map_id=pose.map_id,
                                   localization_session=pose.localization_session,
                                   base_xyz=base.tolist(), map_xyz=world.tolist(),
                                   covariance_xy=cov.tolist(), contact_pixel=list(pixel),
                                   position_source="calibrated_ground_plane",
                                   depth_valid=False, synthetic=frame.synthetic)
                result["observations"].append(observation)
            except ValueError as e:
                result["rejected"].append("box %d: %s" % (index, e))
        if result["observations"]:
            self.pending.extend(result["observations"])
        result["state"] = "ground_observations"
        finish()
        self.tick(result["produced_ns"])
        return result

    def tick(self, now_ns, flush=False):
        latest_pose = self.poses.latest()
        if latest_pose is not None:
            identity = (latest_pose.map_id, latest_pose.localization_session, self.calibration.id)
            if identity != self.last_pose_identity:
                self.pending.clear()
                self.latest_visibility.clear()
                self.store.reset_if_changed(*identity)
                self.last_pose_identity = identity
            if not latest_pose.valid or not 0 <= now_ns - latest_pose.stamp_ns <= 500_000_000:
                self.latest_visibility.clear()
        if self.pending:
            oldest = min(o["stamp_ns"] for o in self.pending)
            if flush or now_ns - oldest >= self.fusion_ns:
                selected_frames = {}
                for o in sorted(self.pending, key=lambda v: v["stamp_ns"]):
                    if o["stamp_ns"] <= oldest + self.fusion_ns:
                        selected_frames.setdefault(o["camera"], o["frame_key"])
                batch = [o for o in self.pending if
                         selected_frames.get(o["camera"]) == o["frame_key"]]
                selected_ids = {o["id"] for o in batch}
                self.pending = [o for o in self.pending if o["id"] not in selected_ids]
                clusters = fuse(batch, self.config.get("overlap_pairs", []))
                self.completed_observations = clusters
                self.store.update(clusters, now_ns)
        def visible(target):
            stamps = [pose.stamp_ns for camera, pose in self.latest_visibility.items()
                      if 0 <= now_ns - pose.stamp_ns <= 500_000_000
                      and self.projector.visible(camera, target["map_xyz"], pose)]
            return max(stamps, default=0)
        self.store.update([], now_ns, visible)
        return self.snapshot(now_ns)

    def snapshot(self, now_ns):
        pose = max(self.latest_visibility.values(), key=lambda p: p.stamp_ns, default=None)
        latest_pose = self.poses.latest()
        localized = (latest_pose is not None and latest_pose.valid
                     and 0 <= now_ns - latest_pose.stamp_ns <= 500_000_000)
        return dict(schema=SCHEMA, session=self.session, produced_ns=now_ns,
                    calibration_id=self.calibration.id, synthetic=self.calibration.synthetic,
                    latest_result=self.latest_result, targets=self.store.snapshot(now_ns),
                    camera_results=self.camera_results.copy(),
                    map_identity=self.store.identity, map_resets=self.store.resets,
                    localization_valid=localized,
                    model_identity=getattr(self.engine, "identity", {}),
                    display_pose=None if pose is None else dict(
                        stamp_ns=pose.stamp_ns, T_map_base=pose.T_map_base.tolist()),
                    calibration_issues=self.calibration.issues(),
                    production_ready=False, motion_eligible=False)
