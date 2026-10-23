"""Transport identity, allocation bounds, model substitution and CPU decoding."""
from pathlib import Path
import socket
import sys
import threading
import time
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import tensor_wire as wire
from tensor_transport import TensorClient, ssh_command, ROOT
from production_io import classifier_output, detector_output, validate_production_interface


class BridgeTests(unittest.TestCase):
    def test_framing_and_noncontiguous_arrays(self):
        array = np.arange(24, dtype=np.float32).reshape(2, 3, 4)[:, :, ::2]
        specs = [{'shape': list(array.shape), 'type': 7}]
        descriptions, payload = wire.pack_arrays([array])
        left, right = socket.socketpair()
        with left, right, left.makefile('rwb', buffering=0) as reader, right.makefile('rwb', buffering=0) as writer:
            wire.send(writer, {'tensors': descriptions}, payload)
            header, raw = wire.receive(reader)
            output = wire.unpack_arrays(header['tensors'], raw, specs)[0]
        np.testing.assert_array_equal(output, array)

    def test_oversized_prefix_rejected_without_reading_payload(self):
        for magic, size, payload in [(b'FAIL', 2, 0), (wire.MAGIC, wire.MAX_HEADER + 1, 0),
                                     (wire.MAGIC, 2, wire.MAX_PAYLOAD + 1)]:
            left, right = socket.socketpair()
            with left, right, left.makefile('rb', buffering=0) as reader:
                right.sendall(wire.PREFIX.pack(magic, size, payload))
                with self.assertRaises(ValueError):
                    wire.receive(reader, timeout=.1)

    def test_duplicate_json_and_nonfinite_headers(self):
        for data in [b'{"sequence":1,"sequence":2}', b'{"value":NaN}', b'[]']:
            left, right = socket.socketpair()
            with left, right, left.makefile('rb', buffering=0) as reader:
                right.sendall(wire.PREFIX.pack(wire.MAGIC, len(data), 0) + data)
                with self.assertRaises(ValueError):
                    wire.receive(reader, timeout=.1)

    def test_partial_frame_deadline_and_disconnect(self):
        left, right = socket.socketpair()
        with left, right, left.makefile('rb', buffering=0) as reader:
            right.sendall(wire.PREFIX.pack(wire.MAGIC, 2, 512) + b'{}')
            with self.assertRaises(TimeoutError):
                wire.receive(reader, timeout=.03)
            right.close()
            with self.assertRaises(EOFError):
                wire.receive(reader, timeout=.1)

    def test_tensor_contract_and_finite_values(self):
        spec = [{'shape': [1, 2], 'type': 7}]
        array = np.array([[.1, .2]], dtype=np.float32)
        descriptions, raw = wire.pack_arrays([array])
        for descriptor in [dict(descriptions[0], shape=[2, 1]), dict(descriptions[0], dtype='u1'),
                           dict(descriptions[0], shape=[True, 2]), dict(descriptions[0], bytes=True)]:
            with self.assertRaises(ValueError):
                wire.unpack_arrays([descriptor], raw, spec)
        with self.assertRaises(ValueError):
            wire.unpack_arrays(descriptions, raw + b'X', spec)
        with self.assertRaises(ValueError):
            wire.unpack_arrays(descriptions, np.array([[np.inf, 0]], dtype=np.float32).tobytes(), spec)
        with self.assertRaises(ValueError):
            wire.pack_arrays([array.astype(np.float64)])

    def test_source_identity_is_strict(self):
        valid = {'sequence': 1, 'stamp_ns': 123, 'frame_id': 'zed_left_optical_frame'}
        self.assertEqual(wire.identity(valid), valid)
        for changed in [dict(valid, sequence=True), dict(valid, sequence=0),
                        dict(valid, stamp_ns=-1), dict(valid, frame_id='')]:
            with self.assertRaises(ValueError):
                wire.identity(changed)

    def test_ssh_validation_precedes_process_creation(self):
        args = dict(target='root@192.168.10.100', relay=ROOT+'/bpu_production_20261010/tools/tensor_stdio.py',
                    library=ROOT+'/bpu_production_20261010/build_v3/libj6m_tensor.so',
                    model=ROOT+'/bpu_lab_20260905/models/fcos_nash_m/model.hbm',
                    model_name='fcos_efficientnetb3_mscoco', sha256='0'*64)
        self.assertIn('StrictHostKeyChecking=yes', ssh_command(**args))
        for changed in [dict(args, target='host;id'), dict(args, relay=ROOT+'/../evil/tools/tensor_stdio.py'),
                        dict(args, model_name='x$(id)'), dict(args, sha256='wrong')]:
            with mock.patch('tensor_transport.subprocess.Popen') as process:
                with self.assertRaises(ValueError):
                    TensorClient(**changed)
                process.assert_not_called()

    def test_wrong_response_identity_and_eligibility_close_worker(self):
        for changes in [{'sequence': 99}, {'model_sha256': '1'*64}, {'motion_eligible': True}]:
            left, right = socket.socketpair()
            client_stream = left.makefile('rwb', buffering=0)
            server_stream = right.makefile('rwb', buffering=0)
            process = mock.Mock(stdin=client_stream, stdout=client_stream)
            process.poll.return_value = None
            process.wait.return_value = 0
            client = TensorClient.__new__(TensorClient)
            client.process, client.timeout, client.sequence, client.sha256 = process, 1., 0, '0'*64
            client.metadata = {'inputs': [{'shape': [1, 2], 'type': 7}],
                               'outputs': [{'shape': [1, 5], 'type': 7}]}
            def reply():
                header, _ = wire.receive(server_stream)
                descriptors, payload = wire.pack_arrays([np.full((1, 5), .2, dtype=np.float32)])
                response = dict(wire.identity(header), ok=True, model_sha256='0'*64,
                                production_ready=False, motion_eligible=False, tensors=descriptors, timing={})
                response.update(changes)
                wire.send(server_stream, response, payload)
            thread = threading.Thread(target=reply)
            thread.start()
            try:
                with self.assertRaises(ValueError):
                    client.infer([np.zeros((1, 2), dtype=np.float32)], time.time_ns(), 'zed')
                self.assertIsNone(client.process)
                process.wait.assert_called()
            finally:
                thread.join(timeout=1)
                server_stream.close(); client_stream.close(); left.close(); right.close()

    def test_stale_input_never_sent(self):
        client = TensorClient.__new__(TensorClient)
        client.process, client.sequence = mock.Mock(), 0
        client.process.poll.return_value = None
        with mock.patch('tensor_transport.send') as send:
            with self.assertRaises(TimeoutError):
                client.infer([], time.time_ns()-1_000_000_000, 'zed')
            send.assert_not_called()

    def test_single_class_nms_and_original_geometry(self):
        raw = np.zeros((1, 5, 12096), dtype=np.float32)
        raw[0, :, 0] = [50, 40, 20, 20, .9]
        raw[0, :, 1] = [51, 40, 20, 20, .8]
        raw[0, :, 2] = [150, 100, 30, 10, .7]
        raw[0, :, 3] = [250, 100, 30, 10, .2]
        result = detector_output(raw, (2, 10, 4, 100, 200))
        self.assertEqual(result.shape, (2, 5))
        np.testing.assert_allclose(result[0], [15, 13, 25, 23, .9], atol=1e-6)
        with self.assertRaises(ValueError):
            detector_output(np.zeros((1, 84, 12096), dtype=np.float32), (2, 0, 0, 100, 200))

    def test_classifier_order_probabilities_and_output_ownership(self):
        raw = np.array([[.7, .1, .1, .05, .05]], dtype=np.float32)
        value = classifier_output(raw)
        np.testing.assert_array_equal(value, raw[0])
        raw[:] = .2
        self.assertAlmostEqual(float(value[0]), .7, places=6)
        for bad in [np.zeros((1, 80), dtype=np.float32), np.zeros((1, 5), dtype=np.float32),
                    np.full((1, 5), np.nan, dtype=np.float32)]:
            with self.assertRaises(ValueError):
                classifier_output(bad)

    def test_sigmoid_roundoff_preserves_valid_detections(self):
        raw = np.zeros((1, 5, 12096), dtype=np.float32)
        raw[0, :, 0] = [50, 40, 20, 20, .9]
        raw[0, 4, 1] = -5.9604645e-8
        result = detector_output(raw, (1, 0, 0, 576, 1024))
        self.assertEqual(result.shape, (1, 5))
        for invalid in [-.001, 1.001, np.nan]:
            raw[0, 4, 1] = invalid
            with self.assertRaises(ValueError):
                detector_output(raw, (1, 0, 0, 576, 1024))

    def test_coco_or_changed_featuremap_cannot_be_production_model(self):
        metadata = {'inputs': [{'name': 'images', 'shape': [1, 3, 576, 1024], 'type': 7, 'quant': 0}],
                    'outputs': [{'name': 'output0', 'shape': [1, 5, 12096]}]}
        validate_production_interface(metadata, 'detector_zed')
        metadata['inputs'][0]['type'] = 3
        with self.assertRaises(ValueError):
            validate_production_interface(metadata, 'detector_zed')
        metadata['inputs'][0]['type'] = 7
        metadata['outputs'][0]['shape'] = [1, 84, 12096]
        with self.assertRaises(ValueError):
            validate_production_interface(metadata, 'detector_zed')


if __name__ == '__main__':
    unittest.main()
