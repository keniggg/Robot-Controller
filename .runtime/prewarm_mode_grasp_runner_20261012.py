"""Import the unchanged audited runner before one exact selection is released.

READY means Python imports completed, not motion/plan permission. Until the
parent supplies the committed selection through its private pipe, the child
does not initialize ROS, create the runner, or call any runner method.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time


MAX_MESSAGE_BYTES = 4096


def _read_message(fd, timeout):
    deadline = time.monotonic() + timeout
    raw = bytearray()
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            raise TimeoutError('prewarmed runner handshake timed out')
        part = os.read(fd, min(1024, MAX_MESSAGE_BYTES + 1 - len(raw)))
        if not part:
            raise EOFError('prewarmed runner handshake closed without release')
        raw.extend(part)
        if len(raw) > MAX_MESSAGE_BYTES:
            raise ValueError('prewarmed runner message exceeds bound')
        if b'\n' in raw:
            if not raw.endswith(b'\n') or raw.count(b'\n') != 1:
                raise ValueError('prewarmed runner requires one message')
            return json.loads(raw)


def _write_message(fd, value):
    raw = (json.dumps(value, sort_keys=True, allow_nan=False) + '\n').encode()
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError('prewarmed runner message exceeds bound')
    # One bounded payload is below Linux PIPE_BUF and is never interleaved.
    if os.write(fd, raw) != len(raw):
        raise RuntimeError('prewarmed runner message incomplete')


def _selection(value):
    # This deliberately narrow operational helper is only for this one-round
    # unknown/observe_return wrapper. The original runner independently checks
    # the exact active selection before computation and before grasp start.
    if (not isinstance(value, dict)
            or set(value) != {'mode', 'strategy', 'generation', 'stamp_ns'}
            or value['mode'] != 'unknown' or value['strategy'] != 'observe_return'
            or type(value['generation']) is not int or value['generation'] <= 0
            or type(value['stamp_ns']) is not int or value['stamp_ns'] <= 0):
        raise ValueError('exact unknown observe_return selection required')
    return dict(value)


class PrewarmedModeRunner:
    def __init__(self, runner_path, *, ready_timeout=30.0, release_timeout=30.0):
        self.runner_path = Path(runner_path).resolve(strict=True)
        self.expected_sha256 = hashlib.sha256(self.runner_path.read_bytes()).hexdigest()
        self.released = False
        self.ready = None
        self.process = None
        self._release_fd = None
        ready_read, ready_write = os.pipe()
        release_read, release_write = os.pipe()
        try:
            env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
            self.process = subprocess.Popen([
                sys.executable, '-u', str(Path(__file__).resolve()),
                '--runner-path', str(self.runner_path),
                '--ready-fd', str(ready_write), '--release-fd', str(release_read),
                '--release-timeout', str(release_timeout),
            ], pass_fds=(ready_write, release_read), env=env)
            os.close(ready_write); ready_write = None
            os.close(release_read); release_read = None
            self._release_fd = release_write; release_write = None
            ready = _read_message(ready_read, ready_timeout)
            if (not isinstance(ready, dict)
                    or set(ready) != {'version', 'state', 'pid', 'runner_path', 'runner_sha256'}
                    or type(ready['version']) is not int or ready['version'] != 1
                    or ready['state'] != 'imports_ready'
                    or ready['pid'] != self.process.pid
                    or ready['runner_path'] != str(self.runner_path)
                    or ready['runner_sha256'] != self.expected_sha256
                    or self.process.poll() is not None):
                raise RuntimeError('prewarmed runner READY does not match child/source')
            self.ready = ready
        except BaseException:
            self.close()
            raise
        finally:
            for fd in (ready_read, ready_write, release_read, release_write):
                if fd is not None:
                    os.close(fd)

    def run_once(self, selected):
        if self.released or self._release_fd is None:
            raise RuntimeError('prewarmed runner selection already released or closed')
        selection = _selection(selected)
        if self.process.poll() is not None:
            raise RuntimeError('prewarmed runner exited before selection release')
        if hashlib.sha256(self.runner_path.read_bytes()).hexdigest() != self.expected_sha256:
            raise RuntimeError('runner source changed after prewarming')
        # Set before pipe write: after any partial release, never kill or retry
        # a potentially executing runner. Its existing stop/finish path owns it.
        self.released = True
        try:
            _write_message(self._release_fd, dict(version=1,
                operation='run_exact_selection_once', selection=selection))
        finally:
            os.close(self._release_fd)
            self._release_fd = None
        return self.process.wait()

    def close(self):
        if self._release_fd is not None:
            os.close(self._release_fd)
            self._release_fd = None
        if self.process is not None and not self.released:
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                # No selection was sent: imports-only child has no motion
                # authority. A released runner is never terminated here.
                self.process.terminate()
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
        elif self.process is not None:
            # Reap an already-exited released child, but never stop one that
            # may be executing the original runner after a caller exception.
            self.process.poll()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


def child_main(runner_path, ready_fd, release_fd, release_timeout):
    path = Path(runner_path).resolve(strict=True)
    source_sha = hashlib.sha256(path.read_bytes()).hexdigest()
    spec = importlib.util.spec_from_file_location('_prewarmed_audited_mode_runner', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hashlib.sha256(path.read_bytes()).hexdigest() != source_sha:
        raise RuntimeError('runner source changed during prewarming')
    _write_message(ready_fd, dict(version=1, state='imports_ready', pid=os.getpid(),
        runner_path=str(path), runner_sha256=source_sha))
    os.close(ready_fd)
    try:
        request = _read_message(release_fd, release_timeout)
    finally:
        os.close(release_fd)
    if (not isinstance(request, dict)
            or set(request) != {'version', 'operation', 'selection'}
            or type(request['version']) is not int or request['version'] != 1
            or request['operation'] != 'run_exact_selection_once'):
        raise ValueError('invalid prewarmed runner release')
    selected = _selection(request['selection'])
    if hashlib.sha256(path.read_bytes()).hexdigest() != source_sha:
        raise RuntimeError('runner source changed before selection release')
    runner = module.ModeAwareGraspRunner('unknown', 'observe_return', selected)
    try:
        return runner.run()
    except Exception as exc:
        # Match the original entrypoint's cleanup and truthful exit status.
        try:
            runner.stop_candidate_computation()
        except Exception as stop_error:
            print('CANDIDATE_COMPUTATION_STOP_EXCEPTION %s' % stop_error, flush=True)
        print('RUNNER_EXCEPTION %s execution_response_unknown=%s' % (
            exc, runner.execution_inflight), flush=True)
        return 10


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner-path', required=True)
    parser.add_argument('--ready-fd', type=int, required=True)
    parser.add_argument('--release-fd', type=int, required=True)
    parser.add_argument('--release-timeout', type=float, default=30.0)
    args = parser.parse_args()
    try:
        sys.exit(child_main(args.runner_path, args.ready_fd, args.release_fd, args.release_timeout))
    except Exception as exc:
        print('PREWARMED_RUNNER_HANDSHAKE_FAILURE %s: %s' % (type(exc).__name__, exc), flush=True)
        sys.exit(75)
