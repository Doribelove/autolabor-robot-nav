#!/usr/bin/env python3
"""Submit all configured latest-map regions as one coverage batch."""

import json
import os
import time
import uuid

import rospy
from autolabor_coverage.msg import CoverageRegion, CoverageStatus
from autolabor_coverage.srv import StartCoverageBatch
from geometry_msgs.msg import Point32


class BatchRunner:
    def __init__(self):
        self.status = None
        self.seen_active = False
        self.region_results = {}
        self.result_dir = os.path.abspath(rospy.get_param("~result_dir"))
        self.regions_file = os.path.abspath(rospy.get_param("~regions_file"))
        self.region_order = rospy.get_param("~region_order", ["A区", "C区"])
        self.maximum_wall_time = float(rospy.get_param("~maximum_wall_time", 900.0))
        os.makedirs(self.result_dir, exist_ok=True)
        rospy.Subscriber("/coverage/status", CoverageStatus, self._status_callback)

    def _status_callback(self, message):
        self.status = message
        self.seen_active = self.seen_active or bool(message.batch_active)
        if message.current_region_id:
            record = self.region_results.setdefault(
                message.current_region_id,
                {
                    "id": message.current_region_id,
                    "name": message.current_region_name,
                    "state": message.state,
                    "max_coverage_ratio": 0.0,
                    "max_segment": 0,
                    "total_segments": 0,
                },
            )
            record["name"] = message.current_region_name
            record["state"] = message.state
            record["max_coverage_ratio"] = max(
                record["max_coverage_ratio"], float(message.coverage_ratio)
            )
            record["max_segment"] = max(
                record["max_segment"], int(message.current_segment)
            )
            record["total_segments"] = max(
                record["total_segments"], int(message.total_segments)
            )
        if message.last_region_id:
            record = self.region_results.setdefault(
                message.last_region_id,
                {
                    "id": message.last_region_id,
                    "name": message.last_region_name,
                    "state": message.last_region_state,
                    "max_coverage_ratio": 0.0,
                    "max_segment": 0,
                    "total_segments": 0,
                },
            )
            record["name"] = message.last_region_name
            record["state"] = message.last_region_state

    def _load_regions(self):
        with open(self.regions_file, "r", encoding="utf-8") as stream:
            records = json.load(stream)["regions"]
        by_name = {record["name"]: record for record in records}
        ordered = []
        for name in self.region_order:
            if name not in by_name:
                raise RuntimeError("coverage region {!r} is absent".format(name))
            record = by_name[name]
            region = CoverageRegion()
            region.id = record["id"]
            region.name = record["name"]
            region.region.header.frame_id = "map"
            region.region.header.stamp = rospy.Time.now()
            region.region.polygon.points = [
                Point32(x=float(point["x"]), y=float(point["y"]), z=0.0)
                for point in record["polygon"]
            ]
            ordered.append(region)
        return ordered

    def run(self):
        deadline = time.monotonic() + 45.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.status is not None and self.status.map_ready and self.status.localized:
                break
            rospy.sleep(0.1)
        if self.status is None or not self.status.map_ready:
            raise RuntimeError("coverage manager did not receive the static map")
        rospy.wait_for_service("/coverage/start_batch", timeout=30.0)
        client = rospy.ServiceProxy("/coverage/start_batch", StartCoverageBatch)
        request_id = "coverage-batch-{}".format(uuid.uuid4().hex)
        response = client(
            client_request_id=request_id,
            regions=self._load_regions(),
            operation_width_m=float(rospy.get_param("~operation_width", 1.0)),
            overlap_ratio=float(rospy.get_param("~overlap_ratio", 0.15)),
            allow_reverse_transit=True,
            max_speed_mps=float(rospy.get_param("~maximum_speed", 0.80)),
            reverse_speed_mps=float(rospy.get_param("~reverse_speed", 0.30)),
            max_angular_speed_rps=float(rospy.get_param("~maximum_angular_speed", 0.60)),
            linear_accel_mps2=float(rospy.get_param("~linear_accel", 1.00)),
            angular_accel_rps2=float(rospy.get_param("~angular_accel", 0.50)),
            direction_change_penalty_sec=float(
                rospy.get_param("~direction_change_penalty", 1.0)
            ),
            segment_handoff_penalty_sec=float(
                rospy.get_param("~segment_handoff_penalty", 0.50)
            ),
            transit_replan_period_sec=1.0,
            map_digest=self.status.map_digest,
        )
        rospy.loginfo(
            "coverage batch response: accepted=%s id=%s message=%s",
            response.accepted, response.batch_id, response.message,
        )
        if not response.accepted:
            raise RuntimeError("coverage batch rejected: {}".format(response.message))

        started = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic() - started < self.maximum_wall_time:
            if self.seen_active and self.status is not None and not self.status.batch_active:
                ordered_results = sorted(
                    self.region_results.values(),
                    key=lambda record: self.region_order.index(record["name"])
                    if record["name"] in self.region_order else len(self.region_order),
                )
                ratios = [
                    record["max_coverage_ratio"] for record in ordered_results
                    if record["total_segments"] > 0
                ]
                payload = {
                    "state": self.status.state,
                    "detail": self.status.detail,
                    "batch_id": self.status.batch_id,
                    "completed": int(self.status.batch_completed_regions),
                    "partial": int(self.status.batch_partial_regions),
                    "skipped": int(self.status.batch_skipped_regions),
                    "coverage_ratio": (
                        sum(ratios) / len(ratios) if ratios else 0.0
                    ),
                    "terminal_reported_coverage_ratio": float(
                        self.status.coverage_ratio
                    ),
                    "regions": ordered_results,
                }
                with open(os.path.join(self.result_dir, "batch_result.json"), "w", encoding="utf-8") as stream:
                    json.dump(payload, stream, indent=2, ensure_ascii=False, sort_keys=True)
                    stream.write("\n")
                rospy.loginfo("coverage batch terminal result: %s", payload)
                rospy.signal_shutdown("coverage batch finished")
                return
            rospy.sleep(0.2)
        raise RuntimeError("coverage batch exceeded the wall-time budget")


if __name__ == "__main__":
    rospy.init_node("coverage_batch_runner")
    runner = BatchRunner()
    rospy.sleep(2.0)
    runner.run()
