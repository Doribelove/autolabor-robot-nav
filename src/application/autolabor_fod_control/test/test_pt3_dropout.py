#!/usr/bin/env python3
"""Replay the real field dropout which previously abandoned visible FOD."""
import json
from pathlib import Path
import unittest
import yaml
from autolabor_fod_control.visual_servo import PixelDetection, TargetMachineConfig, TargetPhaseMachine

class Pt3DropoutTest(unittest.TestCase):
    def test_recorded_target_recovers_without_track_wait_or_blind_commit(self):
        root=Path(__file__).resolve().parents[1]
        params=yaml.safe_load((root/'config/visual_camera_zed2.yaml').read_text())
        machine=TargetPhaseMachine(TargetMachineConfig(acquire_frames=3,select_nearest_by_depth=True,
            roi_top_fraction=.25,roi_bottom_fraction=.625,
            tracking_loss_grace_sec=params['tracking_loss_grace_sec'],
            recovery_loss_grace_sec=params['recovery_loss_grace_sec']))
        states=[]
        for stamp,boxes in json.loads((root/'test/pt3_dropout_20261014.json').read_text()):
            observations=[PixelDetection(0,d['name'],d['conf'],d['x'],d['y'],d['w'],d['h'],d['u'],d['v'],
                track_id=d['id'],depth_valid=d['valid'],depth_m=d['depth'],range_m=d['range']) for d in boxes]
            decision=machine.process_frame(observations,1280,720,634.918,0.,100+stamp)
            states.append(decision.state)
        self.assertEqual(states[-1],'APPROACH')
        self.assertNotIn('TRACK_WAIT',states)
        self.assertNotIn('BLIND_ADVANCE',states)
        self.assertEqual(machine.locked.track_id,1711)

if __name__=='__main__': unittest.main()
