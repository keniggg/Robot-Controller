"""Read-only, stdout-only offline tests when the experiment disk is absent.

This is a source helper, not an experiment archive. Tests requiring filesystem
output cannot run here; the normal external-disk suite must cover them later.
Python audit hooks reject writes, hardware opens, network and child processes.
"""
import os
import sys

sys.dont_write_bytecode = True
os.environ.update(PYTHONDONTWRITEBYTECODE='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
                  ROS_MASTER_URI='http://127.0.0.1:1',
                  OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')


def read_only(event, args):
    if event == 'open':
        path, mode, flags = args
        if ((isinstance(mode, str) and any(c in mode for c in 'wax+'))
                or (isinstance(flags, int) and flags & (
                    os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND))
                or (isinstance(path, (str, bytes))
                    and os.fsdecode(path).startswith(('/dev/tty', '/dev/serial')))):
            raise PermissionError('offline stdout-only test forbids writes or hardware access')
    if event in {'os.mkdir', 'os.remove', 'os.rmdir', 'os.rename', 'os.link', 'os.symlink',
                 'os.chmod', 'os.chown', 'os.truncate', 'os.system', 'os.posix_spawn',
                 'subprocess.Popen', 'socket.connect', 'socket.bind'}:
        raise PermissionError('offline stdout-only test forbids ' + event)


sys.addaudithook(read_only)
import pytest

raise SystemExit(pytest.main(['-q', '-p', 'no:cacheprovider', '--capture=sys', *sys.argv[1:]]))
