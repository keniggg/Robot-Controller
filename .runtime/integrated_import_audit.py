"""Bind actual Python execution/import paths to this run's frozen sources."""
import atexit
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import types

_installed = False
_observed = {}
_violations = []
_busy = threading.local()
_audit_lock = threading.RLock()
_external_inputs = {}


def _inside(path, root):
    return path == root or root in path.parents


def _capture(path, label):
    if getattr(_busy, 'active', False) or not path or not isinstance(path, (str, bytes)) or os.fsdecode(path).startswith('<'):
        return
    _audit_lock.acquire()
    _busy.active = True
    try:
        resolved = Path(os.fsdecode(path)).resolve()
        key = str(resolved)
        if key in _observed or not resolved.is_file():
            return
        # Third-party/system modules are not mislabeled as project sources.
        generated = _inside(resolved, _output) or any(_inside(resolved, root) for root in _generated_roots)
        if not (_inside(resolved, _project) or generated):
            return
        digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
        category = 'generated_fixture' if generated else 'project'
        expected = _expected.get(key)
        if key in _bootstrap:
            category, expected = 'runner_bootstrap', _bootstrap[key]
        row = dict(path=key, sha256=digest, category=category, label=label,
                   pid=os.getpid(), expected_sha256=expected)
        _observed[key] = row
        with _events.open('a') as stream:
            stream.write(json.dumps(row, sort_keys=True)+'\n')
        if category != 'generated_fixture' and digest != expected:
            _violations.append(row)
            raise RuntimeError('executed project source is outside frozen manifest: '+key)
    finally:
        _busy.active = False
        _audit_lock.release()


def _record_input(path):
    if getattr(_busy, 'active', False) or not isinstance(path, (str, bytes)):
        return
    with _audit_lock:
        _busy.active = True
        try:
            resolved = Path(os.fsdecode(path)).resolve()
            if not any(_inside(resolved, root) for root in _experiments) or _inside(resolved, _output) or not resolved.is_file():
                return
            stat = resolved.stat()
            row = dict(path=str(resolved), size=stat.st_size, mtime_ns=stat.st_mtime_ns,
                       device=stat.st_dev, inode=stat.st_ino)
            previous = _external_inputs.get(str(resolved))
            if previous is not None:
                if previous != row:
                    _violations.append(dict(external_input_changed=dict(before=previous, after=row)))
                return
            _external_inputs[str(resolved)] = row
            with _input_events.open('a') as stream:
                stream.write(json.dumps(row, sort_keys=True)+'\n')
        finally:
            _busy.active = False


def _audit(event, args):
    if event == 'exec' and args:
        _capture(getattr(args[0], 'co_filename', None), 'exec')
    elif event == 'import' and len(args) > 1:
        _capture(args[1], 'import:'+str(args[0]))
    elif event == 'open' and args:
        _record_input(args[0])


def _scan_modules():
    pending = list(sys.modules.items())
    visited = set()
    while pending:
        label, module = pending.pop()
        if not isinstance(module, types.ModuleType) or id(module) in visited:
            continue
        visited.add(id(module))
        path = getattr(module, '__file__', None)
        _capture(path, label)
        if isinstance(path, (str, bytes)) and _inside(Path(path).resolve(), _project):
            # exec_module may leave modules only in a test's globals.
            pending.extend(('holder:'+label+'.'+name, value)
                           for name, value in list(vars(module).items())
                           if isinstance(value, types.ModuleType))


def _finish():
    if not _installed:
        return
    try:
        _scan_modules()
    except BaseException as exc:
        _violations.append(dict(audit_error=repr(exc)))
    path = _output/('imports.%d.json'%os.getpid())
    path.write_text(json.dumps(dict(schema=1, pid=os.getpid(), complete=True,
        sources=list(_observed.values()), violations=_violations), indent=2)+'\n')


def install():
    global _installed, _project, _output, _expected, _bootstrap, _events, _experiments, _input_events, _generated_roots
    if _installed:
        return
    workspace = Path(os.environ['INTEGRATED_WORKSPACE_ROOT']).resolve()
    _project = Path(os.environ['INTEGRATED_PROJECT_ROOT']).resolve()
    _output = Path(os.environ['INTEGRATED_WORKER_DIR']).resolve()
    _generated_roots = tuple(Path(p).resolve() for p in json.loads(os.environ.get('INTEGRATED_GENERATED_ROOTS','[]')))
    assert all(str(p) in ('/tmp','/var/tmp') for p in _generated_roots)
    roots = json.loads(os.environ.get('INTEGRATED_EXPERIMENT_ROOTS',
                      json.dumps([os.environ['INTEGRATED_EXPERIMENT_ROOT']])))
    assert type(roots) is list and roots and all(type(p) is str and Path(p).is_absolute() for p in roots)
    _experiments = tuple(Path(p).resolve() for p in roots)
    manifest = json.loads(Path(os.environ['INTEGRATED_SOURCE_MANIFEST']).read_bytes())
    # Resolve the complete manifest once in the parent. Repeating thousands
    # of path walks in short-lived children would delay their actual startup.
    _expected = manifest['canonical_source_sha256']
    _bootstrap = json.loads(Path(os.environ['INTEGRATED_BOOTSTRAP_MANIFEST']).read_bytes())
    _events = _output/('source_events.%d.jsonl'%os.getpid())
    _input_events = _output/('input_events.%d.jsonl'%os.getpid())
    _installed = True
    sys.addaudithook(_audit)
    atexit.register(_finish)
    _capture(__file__, 'audit_plugin')
    _capture(str(_output/'bootstrap/sitecustomize.py'), 'offline_guard')


def pytest_sessionfinish(session, exitstatus):
    _finish()
