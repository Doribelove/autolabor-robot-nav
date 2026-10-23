"""One resident SSH tensor worker, explicit model pin, no automatic fallback."""
import re
import shlex
import subprocess
import time

from tensor_wire import send, receive, identity, pack_arrays, unpack_arrays

ROOT = '/map/robot_j6m_navigation_20260907'


def ssh_command(target, relay, library, model, model_name, sha256):
    if (not isinstance(target, str) or not re.fullmatch(
            r'(?:[A-Za-z_][A-Za-z0-9_-]*@)?[A-Za-z0-9][A-Za-z0-9.-]*', target)):
        raise ValueError('Expected a literal SSH host')
    for path in [relay, library, model]:
        if (not isinstance(path, str) or not path.startswith(ROOT + '/')
                or not re.fullmatch(r'/[A-Za-z0-9_./-]+', path) or '..' in path.split('/')):
            raise ValueError('Expected a private project path without shell syntax')
    if (not relay.endswith('/tools/tensor_stdio.py') or not library.endswith('/libj6m_tensor.so')
            or not model.endswith('.hbm') or not re.fullmatch('[0-9a-f]{64}', sha256)
            or not isinstance(model_name, str) or not re.fullmatch('[A-Za-z0-9_.-]+', model_name)):
        raise ValueError('Invalid relay/library/HBM/model identity')
    remote = shlex.join(['env', 'PYTHONDONTWRITEBYTECODE=1', 'python3', '-u', relay,
                         '--library', library, '--model', model, '--model-name', model_name,
                         '--sha256', sha256])
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'UpdateHostKeys=no', '-o', 'ControlMaster=no', '-o', 'ControlPath=none', target, remote]


class TensorClient:
    def __init__(self, target, relay, library, model, model_name, sha256, timeout=3.0):
        self.process = None
        if not isinstance(timeout, (int, float)) or not 0 < timeout <= 30:
            raise ValueError('Transport deadline must be positive and at most 30 s')
        command = ssh_command(target, relay, library, model, model_name, sha256)
        self.sha256, self.timeout, self.sequence = sha256, float(timeout), 0
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        try:
            ready, payload = receive(self.process.stdout, timeout=self.timeout)
            if (payload or ready.get('ready') is not True or ready.get('model_sha256') != sha256
                    or ready.get('production_ready') is not False or ready.get('motion_eligible') is not False
                    or ready.get('metadata', {}).get('model_name') != model_name):
                raise ValueError('Unexpected worker identity/readiness/eligibility')
            self.metadata = ready['metadata']
            self.load_ms = ready['load_ms']
        except Exception:
            self.close()
            raise

    def infer(self, tensors, stamp_ns, frame_id, max_age_ms=500):
        if not self.process or self.process.poll() is not None:
            raise RuntimeError('Tensor worker is closed')
        self.sequence += 1
        source = identity({'sequence': self.sequence, 'stamp_ns': stamp_ns, 'frame_id': frame_id})
        if type(max_age_ms) is not int or not 1 <= max_age_ms <= 8000:
            raise ValueError('Invalid source result deadline')
        def check_age():
            age = (time.time_ns() - stamp_ns) / 1e6
            if age < 0 or age > max_age_ms:
                raise TimeoutError('Source-stamped tensor result is stale or from the future')
        check_age()
        descriptors, payload = pack_arrays(tensors)
        # Validate the complete request against the ready HBM before writing.
        unpack_arrays(descriptors, payload, self.metadata['inputs'])
        try:
            started = time.monotonic()
            send(self.process.stdin, dict(source, op='infer', timeout_ms=max_age_ms,
                                          tensors=descriptors), payload, timeout=self.timeout)
            header, raw = receive(self.process.stdout, timeout=self.timeout - (time.monotonic() - started))
            if (identity(header) != source or header.get('ok') is not True
                    or header.get('model_sha256') != self.sha256
                    or header.get('motion_eligible') is not False or header.get('production_ready') is not False):
                raise ValueError('Rejected tensor response identity/eligibility')
            check_age()
            outputs = unpack_arrays(header.get('tensors'), raw, self.metadata['outputs'], outputs=True)
            return [value.copy() for value in outputs], header['timing']
        except Exception:
            self.close()
            raise

    def close(self):
        process, self.process = self.process, None
        if process is None:
            return
        if process.stdin:
            process.stdin.close()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)
        if process.stdout:
            process.stdout.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
