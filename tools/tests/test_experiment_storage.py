"""Disk selection and fail-closed bindings; no ROS, device IO, or network."""
import json
import os
import stat
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import experiment_storage as storage
from storage_test_support import fake_storage
REAL_MOUNTED_IDENTITY = storage.mounted_identity


@pytest.fixture
def selected(tmp_path, monkeypatch):
    return fake_storage(tmp_path, monkeypatch)


def test_default_ssd_and_explicit_sanstand_are_distinct(selected):
    assert storage.select().backend == 'ssd'
    other = storage.select('sanstand')
    assert other.backend == 'sanstand' and other.mount != selected.mount
    assert storage.select(path=other.archive_root/'record').backend == 'sanstand'
    assert selected.validate()['filesystem_uuid'] == 'ssd-uuid'


def test_environment_pins_every_recording_output(selected, monkeypatch):
    record = selected.archive_root/'one'
    env = selected.environment(record/'ros', session=record)
    for key, value in env.items(): monkeypatch.setenv(key, value)
    assert env['GRASP_RECORD_ROOT'] == str(selected.archive_root)
    assert env['ALICIA_EXPERIMENT_STORAGE_BASE'] == str(selected.base)
    assert storage.select().validate(record) == json.loads(env[storage.IDENTITY_ENV])
    with pytest.raises(storage.StorageError, match='conflicts'):
        storage.select(path=Path(storage.read_config()[0]['backends']['sanstand']['archive_root']))


@pytest.mark.parametrize('fault', ['unmounted','readonly','wrong_device','unwritable','symlink','nested_mount',
                                 'config_changed','identity_changed','outside','unknown_backend'])
def test_storage_failure_never_creates_or_redirects_output(selected, monkeypatch, tmp_path, fault):
    output = selected.ros_log_root/'new'
    original = selected.validate(output)
    if fault == 'unmounted': monkeypatch.setattr(Path,'is_mount',lambda path:False)
    elif fault == 'readonly': monkeypatch.setattr(storage.os,'statvfs',lambda path:type('FS',(),{'f_flag':os.ST_RDONLY})())
    elif fault in ('wrong_device','nested_mount'):
        selected.base.mkdir()
        real = Path.stat
        def different(path, *args, **kwargs):
            value = real(path,*args,**kwargs)
            return os.stat_result([value.st_mode,value.st_ino,value.st_dev+1,*list(value)[3:]]) if path == selected.base else value
        monkeypatch.setattr(Path,'stat',different)
    elif fault == 'unwritable': monkeypatch.setattr(storage.os,'access',lambda *args:False)
    elif fault == 'symlink':
        selected.base.symlink_to(tmp_path, target_is_directory=True)
    elif fault == 'config_changed': selected.config_path.write_text(selected.config_path.read_text()+' ')
    elif fault == 'identity_changed': original['mount_id'] += 1
    elif fault == 'outside': output = tmp_path/'outside'/'new'
    elif fault == 'unknown_backend':
        with pytest.raises(storage.StorageError): storage.select('usb')
        return
    with pytest.raises(storage.StorageError): selected.validate(output, expected=original)
    assert not output.exists()


def test_missing_configuration_is_not_a_system_disk_fallback(tmp_path, monkeypatch):
    monkeypatch.setattr(storage,'CONFIG_PATH',tmp_path/'absent.json')
    with pytest.raises(storage.StorageError): storage.select()
    assert not (tmp_path/'absent.json').exists()


def test_inherited_remount_binding_rejected(selected, monkeypatch):
    binding = selected.validate(); binding['mount_id'] += 1
    monkeypatch.setenv(storage.IDENTITY_ENV,json.dumps(binding))
    with pytest.raises(storage.StorageError,match='identity changed'): selected.validate()


def test_explicit_backend_does_not_admit_other_disk_path(selected):
    other = storage.select('sanstand')
    with pytest.raises(storage.StorageError,match='conflicts'):
        storage.select('ssd',path=other.archive_root)


@pytest.mark.parametrize('fault', ['none','uuid_missing','uuid_wrong_device','uuid_not_block','wrong_type',
                                 'readonly_mount','readonly_superblock','duplicate_mount','bind_subroot'])
def test_kernel_mount_uuid_identity_is_checked_without_opening_device(tmp_path, monkeypatch, fault):
    mount = tmp_path/'disk'; mount.mkdir()
    dev = mount.stat().st_dev
    fields = '17 1 %d:%d / %s rw - ext4 /dev/fake rw' % (os.major(dev),os.minor(dev),mount)
    if fault == 'wrong_type': fields = fields.replace(' - ext4 ', ' - exfat ')
    if fault == 'readonly_mount': fields = fields.replace(' rw - ', ' ro - ')
    if fault == 'readonly_superblock': fields = fields[:-2]+'ro'
    if fault == 'bind_subroot': fields = fields.replace(' / '+str(mount), ' /sub '+str(mount))
    if fault == 'duplicate_mount': fields += '\n'+fields
    monkeypatch.setattr(Path,'is_mount',lambda path:path==mount)
    read_text, original_stat = Path.read_text, Path.stat
    monkeypatch.setattr(Path,'read_text',lambda path,*a,**kw:fields if path==Path('/proc/self/mountinfo') else read_text(path,*a,**kw))
    uuid = Path('/dev/disk/by-uuid/frozen-uuid')
    def fake_stat(path, *a, **kw):
        if path != uuid: return original_stat(path,*a,**kw)
        if fault == 'uuid_missing': raise FileNotFoundError(str(uuid))
        return type('Block',(),dict(st_mode=stat.S_IFREG if fault=='uuid_not_block' else stat.S_IFBLK,
                    st_rdev=dev+1 if fault=='uuid_wrong_device' else dev))()
    monkeypatch.setattr(Path,'stat',fake_stat)
    if fault == 'none':
        assert REAL_MOUNTED_IDENTITY(mount,'frozen-uuid','ext4')['mount_id'] == 17
    else:
        with pytest.raises(storage.StorageError): REAL_MOUNTED_IDENTITY(mount,'frozen-uuid','ext4')
