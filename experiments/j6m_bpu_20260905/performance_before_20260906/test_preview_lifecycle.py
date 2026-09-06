import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from preview_lifecycle import (BoundedLog, drain_stderr, request_stop,
                               session_identity, within_lifetime)


class LifecycleTest(unittest.TestCase):
    def test_continuous_has_no_client_or_worker_demo_deadline(self):
        for elapsed in (0, 599, 600, 660, 661, 86400 * 365):
            self.assertTrue(within_lifetime(100, 100 + elapsed, None))
        self.assertTrue(within_lifetime(100, 699.9, 600))
        self.assertFalse(within_lifetime(100, 700, 600))
        self.assertFalse(within_lifetime(100, 760, 660))

    def test_log_rotation_bounds_utf8_and_retains_newest(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'results.jsonl'
            with BoundedLog(path, max_bytes=100, backups=2) as log:
                for n in range(60):
                    log.write(json.dumps({'n': n, 'text': '检测结果'}, ensure_ascii=False))
            files = list(Path(temp).iterdir())
            self.assertEqual(len(files), 3)
            self.assertTrue(all(p.stat().st_size <= 100 for p in files))
            self.assertEqual(json.loads(path.read_text().splitlines()[-1])['n'], 59)
            with BoundedLog(path, max_bytes=100, backups=2) as log:
                log.write('reopened')
            self.assertIn('reopened', path.read_text())

    def test_large_record_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'events.log'
            with BoundedLog(path, max_bytes=32) as log:
                with self.assertRaises(ValueError):
                    log.write('X' * 32)
            alias = Path(temp) / 'alias.log'
            alias.symlink_to(path)
            with self.assertRaises(ValueError):
                BoundedLog(alias)

    def test_stderr_is_drained_and_bounded(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'remote.log'
            worker = SimpleNamespace(stderr=io.BytesIO(b'X' * 25000))
            with BoundedLog(path, max_bytes=8192) as log:
                drain_stderr(worker, log)
                worker.preview_stderr_thread.join(timeout=2)
                self.assertFalse(worker.preview_stderr_thread.is_alive())
                self.assertIsNone(worker.preview_stderr_error)
            self.assertLessEqual(sum(p.stat().st_size for p in Path(temp).iterdir()), 3 * 8192)

    def test_master_and_gui_identity_must_both_exist(self):
        master = SimpleNamespace(getPid=lambda _: [1, '', 123],
                                 lookupNode=lambda *_: [1, '', 'http://192.168.10.50:5555/'])
        self.assertEqual(session_identity(master), (123, 'http://192.168.10.50:5555/'))
        master.lookupNode = lambda *_: [-1, 'missing', 0]
        with self.assertRaises(RuntimeError):
            session_identity(master)

    def test_master_restart_changes_identity(self):
        master = SimpleNamespace(getPid=lambda _: [1, '', 123],
                                 lookupNode=lambda *_: [1, '', 'http://192.168.10.50:5555/'])
        before = session_identity(master)
        master.getPid = lambda _: [1, '', 124]
        self.assertNotEqual(before, session_identity(master))

    def test_stop_marker_is_confined_and_does_not_signal(self):
        with tempfile.TemporaryDirectory() as temp:
            lab = Path(temp).resolve()
            self.assertIn('No preview', request_stop(lab))
            directory = lab / 'live' / 'qt_20260905_223000'
            directory.mkdir(parents=True)
            status = dict(active=True, run_id=directory.name, directory=str(directory), pid=1)
            (lab / 'qt_bridge_current.json').write_text(json.dumps(status))
            self.assertIn('stop requested', request_stop(lab))
            self.assertTrue((directory / 'STOP').is_file())
            self.assertIn('stop requested', request_stop(lab))
            status['directory'] = '/tmp'
            (lab / 'qt_bridge_current.json').write_text(json.dumps(status))
            with self.assertRaises(ValueError):
                request_stop(lab)

    def test_continuous_preserves_control_and_resource_guards(self):
        bridge = Path(__file__).with_name('qt_bpu_bridge.py').read_text()
        worker = Path(__file__).with_name('bpu_preview_worker.py').read_text()
        launch = Path(__file__).with_name('start_qt_bridge.sh').read_text()
        for token in ('within_lifetime(started, time.monotonic(), seconds)',
                      'session_identity(master_client) != owned_session',
                      'ready.get("continuous") is not True', 'get_num_connections() == 0',
                      'next_frame = sent + 2.0', 'flock(', '512 * 1024 * 1024',
                      'len(worker_starts) >= 30', 'cleanup.callback(lambda: stop_worker(worker))'):
            self.assertIn(token, bridge)
        for token in ('seconds = None if args.continuous else 660',
                      'within_lifetime(started_session, time.monotonic(), seconds)',
                      'timeout=10', 'timeout=8', '512 * 1024 * 1024',
                      'flock(', 'BoundedLog(directory / "inference.log")',
                      'if path.resolve() != path:', 'motion_eligible'):
            self.assertIn(token, worker)
        self.assertIn('set -- --continuous', launch)
        self.assertIn('verify_master_owner.sh', launch)


if __name__ == '__main__':
    unittest.main()
