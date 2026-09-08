#!/usr/bin/env python3
import errno
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class IsolationTests(unittest.TestCase):
    def test_copied_optional_runtime_assets_are_present(self):
        for relative in ('runtime/fod_clip/python/clip/__init__.py',
                         'runtime/fod_clip/python/clip/bpe_simple_vocab_16e6.txt.gz',
                         'runtime/asr/venv/bin/python3',
                         'runtime/asr/models/small.pt', 'runtime/asr/models/medium.pt',
                         'runtime/asr/models/large-v3.pt'):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_original_file_cannot_be_opened_for_writing(self):
        # O_WRONLY without O_TRUNC cannot modify existing contents, even if a
        # mount is accidentally writable. Do not create sentinels in the original.
        for path in ('/home/slam/robot_j6m_ws/config/dual_host.env',
                     '/home/slam/robot_ws/AGENTS.md'):
            try:
                fd = os.open(path, os.O_WRONLY)
            except OSError as error:
                self.assertEqual(error.errno, errno.EROFS)
            else:
                os.close(fd)
                self.fail('Original file was writable: ' + path)

    def test_all_implicit_output_roots_are_inside_candidate(self):
        for key in ('ROS_HOME', 'ROS_LOG_DIR', 'XDG_CONFIG_HOME', 'XDG_CACHE_HOME',
                    'XDG_DATA_HOME', 'CUDA_CACHE_PATH', 'MPLCONFIGDIR', 'YOLO_CONFIG_DIR'):
            Path(os.environ[key]).resolve().relative_to(ROOT)
        self.assertEqual(os.environ.get('PYTHONDONTWRITEBYTECODE'), '1')
        self.assertEqual(os.environ.get('NO_ALBUMENTATIONS_UPDATE'), '1')

    def test_candidate_writable(self):
        with tempfile.TemporaryDirectory(dir=str(ROOT / 'validation')) as directory:
            sample = Path(directory) / 'sample'
            sample.write_text('isolated')
            self.assertEqual(sample.read_text(), 'isolated')

    def test_deployment_help_and_invalid_options_do_not_call_ssh(self):
        with tempfile.TemporaryDirectory(dir=str(ROOT / 'validation')) as directory:
            # A PATH-resolved fake ssh deliberately fails. Both argument-only
            # operations must return before any SSH/deployment command.
            marker = Path(directory) / 'ssh_called'
            fake = Path(directory) / 'ssh'
            fake.write_text('#!/bin/sh\ntouch "' + str(marker) + '"\nexit 87\n')
            fake.chmod(0o755)
            env = dict(os.environ, PATH=directory + ':' + os.environ['PATH'])
            for args, expected in ((['--help'], 0), (['--unknown'], 2), (['--help', 'extra'], 0)):
                result = subprocess.run([str(ROOT / 'scripts/deploy_j6m.sh')] + args,
                                        cwd=directory, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.assertEqual(result.returncode, expected, result.stderr.decode())
                self.assertFalse(marker.exists())

    def test_master_owner_guard_rejects_original_unknown_and_disconnected(self):
        with tempfile.TemporaryDirectory(dir=str(ROOT / 'validation')) as directory:
            fixtures = {
                'ssh': '#!/bin/bash\nexec bash -s -- "${@: -1}"\n',
                'ss': '#!/bin/sh\nprintf "%s\\n" "$MOCK_LISTENERS"\n',
                'readlink': '#!/bin/sh\nprintf "%s\\n" "$MOCK_PROC_ROOT"\n',
            }
            for name, content in fixtures.items():
                path = Path(directory) / name
                path.write_text(content)
                path.chmod(0o755)
            env = dict(os.environ, PATH=directory + ':' + os.environ['PATH'])
            listener = 'LISTEN 0 128 *:11311 *:* users:(("rosmaster",pid=123,fd=3))'
            cases = (
                ('', '', [], False),
                ('', '', ['--allow-absent'], True),
                (listener, '/map/robot_j6m_navigation_20260907/rootfs', [], True),
                (listener, '/map/robot_j6m_optimized_20260905/rootfs', [], False),
                (listener, '/map/autolabor_runtime/rootfs', [], False),
                (listener, '/map/autolabor_runtime/rootfs', ['--allow-absent'], False),
                ('LISTEN 0 128 *:11311 *:*', '', ['--allow-absent'], False),
                (listener, '/', [], False),
            )
            for listeners, root, args, success in cases:
                env.update(MOCK_LISTENERS=listeners, MOCK_PROC_ROOT=root)
                result = subprocess.run([str(ROOT / 'scripts/verify_master_owner.sh')] + args,
                                        env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.assertEqual(result.returncode == 0, success, (args, root, result.stderr))
            (Path(directory) / 'ssh').write_text('#!/bin/sh\nexit 255\n')
            self.assertNotEqual(subprocess.call([str(ROOT / 'scripts/verify_master_owner.sh'), '--allow-absent'], env=env), 0)

    def test_shutdown_checks_master_ownership_before_shared_graph_mutations(self):
        script = (ROOT / 'scripts/stop_dual_host.sh').read_text()
        self.assertLess(script.index('"$SCRIPT_DIR/verify_master_owner.sh"'),
                        script.index('rostopic pub -1 /move_base/cancel'))
        self.assertEqual(script.count('if [[ "$candidate_graph_owned" == true ]] &&'), 2)

    def test_unmount_rejects_live_pid_and_pid_start_ticks_records(self):
        script = (ROOT / 'deploy/j6m/unmount_chroot.sh').read_text()
        loop = script.split('for pid_file in ', 1)[1].split('\ntargets=(', 1)[0]
        with tempfile.TemporaryDirectory(dir=str(ROOT / 'validation')) as directory:
            record = Path(directory) / 'stack.pid'
            for value in (str(os.getpid()),
                          str(os.getpid()) + ' 987654321',
                          str(os.getpid()) + ':987654321'):
                record.write_text(value + '\n')
                code = 'set -euo pipefail\npid_files=("' + str(record) + '")\nfor pid_file in ' + loop
                result = subprocess.run(['bash'], input=code.encode(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.assertEqual(result.returncode, 3, result.stderr)
                self.assertIn(b'stop it before unmounting', result.stderr)

    def test_remote_lifecycle_rejects_original_runtime_override(self):
        env = dict(os.environ, J6M_RUNTIME_BASE='/map/autolabor_runtime',
                   J6M_ROOTFS='/map/autolabor_runtime/rootfs')
        with tempfile.TemporaryDirectory(dir=str(ROOT / 'validation')) as directory:
            staged = Path(directory)
            # Reproduce the installed helper layout (process_control is shared
            # source in scripts/, then deployed beside start/stop).
            (staged / 'process_control.sh').write_text((ROOT / 'scripts/process_control.sh').read_text())
            for name in ('stop.sh', 'health_check.sh', 'rollback.sh', 'unmount_chroot.sh'):
                (staged / name).write_text((ROOT / 'deploy/j6m' / name).read_text())
                result = subprocess.run(['bash', str(staged / name)],
                                        env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.assertEqual(result.returncode, 2, (name, result.stderr))

    def test_rollback_link_is_chroot_relative_and_invalid_release_keeps_current(self):
        script = (ROOT / 'deploy/j6m/rollback.sh').read_text()
        with tempfile.TemporaryDirectory(dir=str(ROOT / 'validation')) as directory:
            fixture = Path(directory)
            base = fixture / 'rootfs/opt/autolabor/dual_host'
            release = '20260905_120000'
            install = base / 'releases' / release / 'install'
            install.mkdir(parents=True)
            (install / 'setup.bash').touch()
            (base / 'current').symlink_to('/opt/autolabor/dual_host/releases/previous/install')
            fake_id = fixture / 'id'
            fake_id.write_text('#!/bin/sh\necho 0\n')
            fake_id.chmod(0o755)
            env = dict(os.environ, PATH=directory + ':' + os.environ['PATH'])
            env.pop('J6M_RUNTIME_BASE', None)
            env.pop('J6M_ROOTFS', None)
            # Redirect the entire fixture, including the strict path assertion.
            code = script.replace('/map/robot_j6m_navigation_20260907', directory)
            for requested, expected in ((release, 0), ('20260905_999999', 4)):
                result = subprocess.run(['bash', '-s', '--', requested], env=env,
                                        input=code.encode(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.assertEqual(result.returncode, expected, result.stderr)
                self.assertEqual(os.readlink(base / 'current'),
                                 '/opt/autolabor/dual_host/releases/' + release + '/install')
            self.assertEqual(list(base.glob('current.next.*')), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
