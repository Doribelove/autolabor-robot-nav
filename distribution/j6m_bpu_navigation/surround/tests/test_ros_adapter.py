import copy
import io
import importlib.util
import unittest

from distribution.j6m_bpu_navigation.surround import SCHEMA
from distribution.j6m_bpu_navigation.surround.ros_adapter import messages, legacy_ground
from test_surround import observation, T0
from distribution.j6m_bpu_navigation.surround.tracking import fuse, TargetStore


@unittest.skipUnless(importlib.util.find_spec("autolabor_surround_vision"),
                     "Source isolated catkin workspace to test generated messages")
class RosAdapterTests(unittest.TestCase):
    def state(self):
        o = observation()
        o.update(position_source="calibrated_ground_plane", depth_valid=False)
        store = TargetStore("s")
        store.update(fuse([o], []), T0)
        return dict(schema=SCHEMA, session="s", synthetic=False, produced_ns=T0,
                    localization_valid=True,
                    model_identity=dict(detector_zed=dict(sha256="0" * 64),
                                        classifier=dict(sha256="1" * 64)),
                    map_identity=["map-v1", "loc-v1", "test"], calibration_id="test",
                    latest_result=dict(source=dict(stamp_ns=T0), observations=[o]),
                    targets=store.snapshot(T0))

    def test_source_time_ground_semantics_and_serialization(self):
        obs, targets = messages(self.state(), T0 + 10_000_000)
        self.assertFalse(obs.observations[0].depth_valid)
        self.assertEqual(obs.observations[0].source.exposure_stamp.to_nsec(), T0)
        self.assertEqual(targets.targets[0].last_observed.to_nsec(), T0)
        self.assertFalse(targets.targets[0].motion_eligible)
        for msg in (obs, targets, legacy_ground(obs, (640, 480))):
            out = io.BytesIO()
            msg.serialize(out)
            decoded = type(msg)().deserialize(out.getvalue())
            encoded_again = io.BytesIO()
            decoded.serialize(encoded_again)
            self.assertEqual(encoded_again.getvalue(), out.getvalue())
            self.assertEqual(decoded.header.stamp, msg.header.stamp)

    def test_fixture_replay_stopped_and_stale_never_reach_ros(self):
        for change in (dict(synthetic=True), dict(replay=True), dict(service_stopped=True),
                       dict(produced_ns=T0 - 600_000_000), dict(localization_valid=False)):
            state = self.state()
            state.update(change)
            with self.assertRaises(ValueError):
                messages(state, T0)

    def test_old_observations_not_retimestamped_as_fresh(self):
        state = self.state()
        state["produced_ns"] = T0 + 600_000_000
        obs, targets = messages(state, T0 + 600_000_000)
        self.assertEqual(len(obs.observations), 0)
        self.assertEqual(targets.targets[0].last_observed.to_nsec(), T0)


if __name__ == "__main__":
    unittest.main()
