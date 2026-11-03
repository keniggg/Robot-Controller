import importlib.util
import os
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('active_mirror', Path(__file__).parents[1] / 'mirror_active_ros_logs.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


@pytest.fixture
def setup(tmp_path):
    source = tmp_path / 'source'; source.mkdir()
    mount = tmp_path / 'mount'; mount.mkdir()
    destination = mount / 'ros_log'
    mirror = m.ActiveLogMirror(source, destination, mount, mount_check=lambda _: True)
    yield source, destination, mount, mirror
    mirror.close()


def record(stream, path):
    stream.flush()
    return {'path': str(path), 'reader': '/proc/self/fd/' + str(stream.fileno()),
            'identity': m.identity(os.fstat(stream.fileno()))}


def test_append_restart_and_final_close(setup):
    source, destination, mount, mirror = setup
    path = source / 'node.log'
    with path.open('wb') as writer:
        writer.write(b'first\n')
        assert mirror.run_once([record(writer, path)])['bytes_copied'] == 6
        writer.write(b'second\n')
        assert mirror.run_once([record(writer, path)])['bytes_copied'] == 7
        mirror.close()
        mirror = m.ActiveLogMirror(source, destination, mount, mount_check=lambda _: True)
        writer.write(b'final\n'); writer.flush()
    try:
        assert mirror.run_once([])['bytes_copied'] == 6
        assert (destination / 'node.log').read_bytes() == path.read_bytes()
        assert mirror.state['files']['node.log']['closed'] is True
    finally:
        mirror.close()


def test_existing_rsync_copy_adopted_only_if_exact_prefix(setup):
    source, destination, _, mirror = setup
    path = source / 'node.log'; target = destination / 'node.log'
    target.write_bytes(b'prefix')
    with path.open('wb') as writer:
        writer.write(b'prefix growth')
        assert mirror.run_once([record(writer, path)])['bytes_copied'] == 7
    assert target.read_bytes() == b'prefix growth'


def test_conflicting_existing_target_is_untouched(setup):
    source, destination, _, mirror = setup
    target = destination / 'node.log'; target.write_bytes(b'external')
    with (source / 'node.log').open('wb') as writer:
        writer.write(b'source bytes')
        with pytest.raises(m.MirrorError, match='differs'):
            mirror.run_once([record(writer, source / 'node.log')])
    assert target.read_bytes() == b'external'


def test_rename_rotation_and_closed_tail_are_preserved(setup):
    source, destination, _, mirror = setup
    path = source / 'node.log'; rotated = source / 'node.log.1'
    with path.open('wb') as old:
        old.write(b'old')
        mirror.run_once([record(old, path)])
        path.rename(rotated)
        old.write(b' tail'); old.flush()
    with path.open('wb') as new:
        new.write(b'new')
        mirror.run_once([record(new, path)])
        assert (destination / 'node.log.1').read_bytes() == b'old tail'
        assert (destination / 'node.log').read_bytes() == b'new'
        assert mirror.run_once([record(new, path)])['bytes_copied'] == 0
    assert rotated.read_bytes() == b'old tail'


def test_copytruncate_keeps_previous_generation(setup):
    source, destination, _, mirror = setup
    path = source / 'node.log'
    with path.open('wb') as writer:
        writer.write(b'old content')
        mirror.run_once([record(writer, path)])
        writer.seek(0); writer.truncate(); writer.write(b'new')
        mirror.run_once([record(writer, path)])
    archived = destination / mirror.state['rotations'][0]['archive']
    assert archived.read_bytes() == b'old content'
    assert (destination / 'node.log').read_bytes() == b'new'
    assert path.read_bytes() == b'new'


def test_unlinked_open_log_finished_without_replacing_new_generation(setup):
    source, destination, _, mirror = setup
    path = source / 'node.log'
    with path.open('wb') as old:
        old.write(b'old')
        mirror.run_once([record(old, path)])
        path.unlink(); old.write(b' tail')
        old_record = record(old, path); old_record['path'] += ' (deleted)'
        with path.open('wb') as new:
            new.write(b'new')
            mirror.run_once([record(new, path), old_record])
            mirror.run_once([record(new, path), old_record])
            assert (destination / 'node.log').read_bytes() == b'new'
            unlinked = list(destination.glob('node.log.unlinked.*.log'))
            assert len(unlinked) == 1 and unlinked[0].read_bytes() == b'old tail'


def test_external_destination_write_or_open_is_rejected(setup):
    source, destination, _, mirror = setup
    path = source / 'node.log'
    with path.open('wb') as writer:
        writer.write(b'one')
        mirror.run_once([record(writer, path)])
        target = destination / 'node.log'
        with target.open('ab') as external:
            with pytest.raises(m.MirrorError, match='currently being written'):
                mirror.run_once([record(writer, path), record(external, target)])
            external.write(b' external'); external.flush()
        with pytest.raises(m.MirrorError, match='changed externally'):
            mirror.run_once([record(writer, path)])
        assert target.read_bytes() == b'one external'


def test_excludes_and_nonlogs_are_never_copied(setup):
    source, destination, mount, original = setup
    original.close()
    mirror = m.ActiveLogMirror(source, destination, mount, excludes=['live'], mount_check=lambda _: True)
    (source / 'live').mkdir()
    writers = []
    try:
        for relative in ['live/supervision.log', 'data.bag', 'data.bag.active', 'code.py', 'config.yaml', 'file.partial']:
            path = source / relative
            stream = path.open('wb'); stream.write(b'bytes'); writers.append((stream, path))
        assert mirror.run_once([record(stream, path) for stream, path in writers])['files'] == 0
        assert not list(destination.rglob('*.log'))
    finally:
        for stream, _ in writers: stream.close()
        mirror.close()


def test_unmounted_storage_never_creates_destination(tmp_path):
    source = tmp_path / 'source'; source.mkdir()
    mount = tmp_path / 'unmounted'; destination = mount / 'logs'
    with pytest.raises(m.MirrorError, match='not mounted'):
        m.ActiveLogMirror(source, destination, mount, mount_check=lambda _: False)
    assert not mount.exists()


def test_proc_scanner_ignores_readers_and_other_file_types(tmp_path):
    proc = tmp_path / 'proc'; pid = proc / '123'; (pid / 'fd').mkdir(parents=True)
    (pid / 'fdinfo').mkdir()
    target = tmp_path / 'node.log'; target.write_bytes(b'log')
    for number, flags in [('1', '0100001'), ('2', '0100002'), ('3', '0100000')]:
        (pid / 'fd' / number).symlink_to(target)
        (pid / 'fdinfo' / number).write_text('flags:\t' + flags + '\n')
    assert len(m.writable_fds(proc)) == 2


@pytest.mark.parametrize('existing', [b'older copy', b'older copy' * 10])
def test_rotation_preserves_conflicting_unowned_old_rsync_destination(setup, existing):
    source, destination, _, mirror = setup
    path = source / 'node.log'; rotated = source / 'node.log.1'
    (destination / 'node.log.1').write_bytes(existing)
    with path.open('wb') as old:
        old.write(b'old current generation')
        mirror.run_once([record(old, path)])
        path.rename(rotated)
        with path.open('wb') as new:
            new.write(b'new generation')
            mirror.run_once([record(old, rotated), record(new, path)])
            mirror.run_once([record(old, rotated), record(new, path)])
    assert (destination / 'node.log.1').read_bytes() == existing
    assert (destination / 'node.log').read_bytes() == b'new generation'
    aliases = list(destination.glob('node.log.1.generation.*.log'))
    assert len(aliases) == 1 and aliases[0].read_bytes() == b'old current generation'


def test_identical_destination_replacement_rebound_after_full_comparison(setup):
    source, destination, _, mirror = setup
    path = source / 'node.log'
    with path.open('wb') as writer:
        writer.write(b'old prefix'); mirror.run_once([record(writer, path)])
        target = destination / 'node.log'
        replacement = destination / 'replacement'
        replacement.write_bytes(target.read_bytes()); replacement.replace(target)
        writer.write(b' tail')
        assert mirror.run_once([record(writer, path)])['bytes_copied'] == 5
    assert target.read_bytes() == b'old prefix tail'
    assert len(mirror.state['destination_rebindings']) == 1


def test_changed_destination_prefix_still_rejected_with_unchanged_tail(setup):
    source, destination, _, mirror = setup
    path = source / 'node.log'
    with path.open('wb') as writer:
        writer.write(b'a' + b'z' * (m.TAIL + 5)); mirror.run_once([record(writer, path)])
        target = destination / 'node.log'
        replacement = destination / 'replacement'
        replacement.write_bytes(b'b' + b'z' * (m.TAIL + 5)); replacement.replace(target)
        with pytest.raises(m.MirrorError, match='changed externally'):
            mirror.run_once([record(writer, path)])
    assert target.read_bytes().startswith(b'b')


@pytest.mark.parametrize('corruption', [None, 'hash', 'source_identity', 'source_path', 'destination_path', 'size_bytes', 'state', 'busy'])
def test_missing_closed_source_requires_matching_migration_receipt(setup, corruption):
    import hashlib
    import json
    source, destination, mount, mirror = setup
    path = source / 'node.log'
    with path.open('wb') as writer:
        writer.write(b'final bytes'); mirror.run_once([record(writer, path)])
    previous = dict(mirror.state['files']['node.log'])
    mirror.close()
    receipt = {'source_path': str(path), 'destination_path': str(destination / 'node.log'),
               'state': 'removed', 'source_identity': previous['source_identity'],
               'size_bytes': previous['offset'],
               'sha256': hashlib.sha256(b'final bytes').hexdigest()}
    if corruption == 'hash':
        receipt['sha256'] = '0' * 64
    elif corruption == 'source_identity':
        receipt['source_identity'] = [0, 1]
    elif corruption in ('source_path', 'destination_path'):
        receipt[corruption] = '/wrong/path'
    elif corruption == 'size_bytes':
        receipt['size_bytes'] += 1
    elif corruption == 'state':
        receipt['state'] = 'mismatch'
    (destination.parent / 'system_disk_migration_cleanup.json').write_text(json.dumps({'files': {str(path): receipt}}))
    path.unlink()
    restored = m.ActiveLogMirror(source, destination, mount, mount_check=lambda _: True)
    try:
        if corruption is None:
            assert restored.run_once([])['files'] == 0
            assert restored.state['files']['node.log']['closed'] is True
            assert restored.state['files']['node.log']['recovered_closed_source']['sha256'] == receipt['sha256']
        else:
            with pytest.raises(m.MirrorError, match='source unavailable'):
                if corruption == 'busy':
                    with (destination / 'node.log').open('ab') as active:
                        restored.run_once([record(active, destination / 'node.log')])
                else:
                    restored.run_once([])
            assert restored.state['files']['node.log']['closed'] is False
        assert (destination / 'node.log').read_bytes() == b'final bytes'
    finally:
        restored.close()
