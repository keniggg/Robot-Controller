#!/usr/bin/env python3
"""Select one experiment disk; never mount, upload, clean up, or fall back."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import stat

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / '.runtime/experiment_storage.json'
BACKEND_ENV = 'ALICIA_STORAGE_BACKEND'
IDENTITY_ENV = 'ALICIA_STORAGE_IDENTITY'


class StorageError(RuntimeError):
    pass


def require(ok, message):
    if not ok:
        raise StorageError('EXPERIMENT_STORAGE_UNAVAILABLE: '+message+'; no fallback')


def absolute(value):
    require(type(value) is str and Path(value).is_absolute(), 'absolute storage path required')
    path = Path(os.path.normpath(value))
    require('..' not in Path(value).parts, 'parent traversal in storage path')
    return path


def read_config(path=None):
    path = Path(path or CONFIG_PATH)
    try:
        raw = path.read_bytes()
        config = json.loads(raw)
        require(config.get('schema') == 2, 'storage configuration schema must be 2')
        require(config.get('default_backend') in ('ssd', 'sanstand'), 'invalid default backend')
        require(set(config['backends']) == {'ssd', 'sanstand'}, 'both storage backends required')
        for name, value in config['backends'].items():
            mount = absolute(value['mount_point'])
            base = absolute(value['experiment_root'])
            require(mount in base.parents, name+' experiment root escapes mount')
            for key in ('archive_root', 'ros_log_root', 'maintenance_root'):
                require(base in absolute(value[key]).parents, name+' '+key+' escapes experiment root')
            require(type(value['filesystem_uuid']) is str
                    and re.fullmatch('[a-zA-Z0-9-]+', value['filesystem_uuid']) is not None,
                    name+' filesystem UUID required')
            require(type(value['filesystem_type']) is str and bool(value['filesystem_type']),
                    name+' filesystem type required')
        require(config['backends']['ssd']['mount_point'] != config['backends']['sanstand']['mount_point'],
                'storage backends must be distinct')
    except (OSError, KeyError, ValueError, TypeError) as exc:
        raise StorageError('invalid experiment storage configuration: '+str(exc)) from exc
    return config, hashlib.sha256(raw).hexdigest(), path.resolve()


def backend_for_path(path, config=None):
    config = config or read_config()[0]
    path = absolute(str(Path(path).expanduser().absolute()))
    matches = [name for name, value in config['backends'].items()
               if path == Path(value['mount_point']) or Path(value['mount_point']) in path.parents]
    require(len(matches) <= 1, 'ambiguous storage backend')
    return matches[0] if matches else None


def select(backend=None, *, path=None, config_path=None):
    config, digest, source = read_config(config_path)
    pinned = backend or os.environ.get(BACKEND_ENV)
    implied = backend_for_path(path, config) if path is not None else None
    require(not pinned or pinned in config['backends'], 'unknown storage backend: '+str(pinned))
    require(not (pinned and implied and pinned != implied), 'requested path conflicts with selected backend')
    name = pinned or implied or config['default_backend']
    return Storage(name, config['backends'][name], source, digest)


def _unescape_mount(value):
    return re.sub(r'\\([0-7]{3})', lambda m: chr(int(m[1], 8)), value)


def mounted_identity(mount, expected_uuid, expected_type):
    """Read kernel mount identity and UUID link without opening the block device."""
    require(mount.is_mount() and mount.resolve() == mount, 'selected mount is not mounted or is a symlink: '+str(mount))
    rows = []
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        left, right = line.split(' - ', 1); fields, kind = left.split(), right.split()
        if _unescape_mount(fields[4]) == str(mount):
            rows.append((fields, kind))
    require(len(rows) == 1, 'selected mount identity is ambiguous')
    fields, kind = rows[0]
    require(fields[3] == '/' and kind[0] == expected_type, 'mounted filesystem type/root differs')
    require('rw' in fields[5].split(',') and 'rw' in kind[2].split(',')
            and not os.statvfs(mount).f_flag & os.ST_RDONLY, 'selected disk is read only')
    major, minor = (int(value) for value in fields[2].split(':'))
    device = os.makedev(major, minor)
    require(mount.stat().st_dev == device, 'mount device changed')
    try:
        uuid_stat = (Path('/dev/disk/by-uuid')/expected_uuid).stat()
    except OSError as exc:
        raise StorageError('EXPERIMENT_STORAGE_UNAVAILABLE: expected filesystem UUID missing; no fallback') from exc
    require(stat.S_ISBLK(uuid_stat.st_mode) and uuid_stat.st_rdev == device, 'mounted filesystem UUID differs')
    return dict(mount_id=int(fields[0]), device=fields[2], filesystem_uuid=expected_uuid,
                filesystem_type=kind[0], mount_point=str(mount))


class Storage:
    def __init__(self, backend, value, source, digest):
        self.backend, self.value = backend, dict(value)
        self.config_path, self.config_sha256 = source, digest
        self.mount = Path(value['mount_point'])
        self.base = Path(value['experiment_root'])
        self.archive_root = Path(value['archive_root'])
        self.ros_log_root = Path(value['ros_log_root'])
        self.maintenance_root = Path(value['maintenance_root'])

    def validate(self, path=None, *, subtree=None, expected=None):
        # Re-read the configuration so a running invocation cannot silently
        # switch media after construction.
        require(read_config(self.config_path)[1] == self.config_sha256, 'storage configuration changed')
        identity = mounted_identity(self.mount, self.value['filesystem_uuid'], self.value['filesystem_type'])
        for root in (self.base, self.archive_root, self.ros_log_root, self.maintenance_root):
            require(root.resolve() == root, 'symlink in configured storage root')
        selected = absolute(str(path)) if path is not None else self.base
        root = Path(subtree) if subtree is not None else self.base
        require(selected == root or root in selected.parents, 'output escapes selected storage root')
        require(selected.resolve() == selected, 'symlink in experiment output path')
        ancestor = selected
        while not ancestor.exists():
            ancestor = ancestor.parent
        require(ancestor.is_dir() if ancestor == selected and not selected.is_file() else True,
                'output ancestor is not a directory')
        require(ancestor.stat().st_dev == self.mount.stat().st_dev, 'output is on a different filesystem')
        writable = ancestor.parent if ancestor.is_file() else ancestor
        require(os.access(writable, os.W_OK | os.X_OK), 'selected output is not writable')
        binding = dict(backend=self.backend, experiment_root=str(self.base), archive_root=str(self.archive_root),
                       ros_log_root=str(self.ros_log_root), config_path=str(self.config_path),
                       config_sha256=self.config_sha256, **identity)
        inherited = os.environ.get(IDENTITY_ENV)
        if inherited:
            try:
                previous = json.loads(inherited)
            except ValueError as exc:
                raise StorageError('invalid inherited storage identity') from exc
            require(previous == binding, 'inherited storage identity changed')
        if expected is not None:
            require(expected == binding, 'frozen storage identity changed')
        return binding

    def environment(self, ros_log_dir, *, session=None):
        identity = self.validate(ros_log_dir)
        if session is not None:
            self.validate(session)
        return {BACKEND_ENV: self.backend, IDENTITY_ENV: json.dumps(identity, sort_keys=True, separators=(',', ':')),
                'ALICIA_EXPERIMENT_STORAGE_BASE': str(self.base), 'GRASP_RECORD_ROOT': str(self.archive_root),
                'ALICIA_SESSION_LOG_DIR': str(session or ros_log_dir), 'ROS_LOG_DIR': str(ros_log_dir)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('ssd', 'sanstand'))
    parser.add_argument('--path', type=Path)
    parser.add_argument('--shell-environment', action='store_true')
    args = parser.parse_args(argv)
    selected = select(args.backend, path=args.path)
    if args.shell_environment:
        for key, value in selected.environment(args.path or selected.ros_log_root,
                                               session=os.environ.get('ALICIA_SESSION_LOG_DIR')).items():
            print('export '+key+'='+shlex.quote(value))
    else:
        print(json.dumps(selected.validate(args.path), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
