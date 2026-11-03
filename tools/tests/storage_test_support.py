"""Isolated fake disks for offline storage integration tests."""
import json
from pathlib import Path

import experiment_storage as storage


def fake_storage(tmp_path, monkeypatch):
    backends = {}
    for name in ('ssd', 'sanstand'):
        mount = tmp_path/name; mount.mkdir(exist_ok=True)
        base = mount/'experiments'
        backends[name] = dict(mount_point=str(mount), experiment_root=str(base),
            archive_root=str(base/'grasp_records'), ros_log_root=str(base/'ros_log'),
            maintenance_root=str(base/'maintenance'), filesystem_uuid=name+'-uuid', filesystem_type='ext4')
    config = tmp_path/'storage.json'
    config.write_text(json.dumps(dict(schema=2, default_backend='ssd', backends=backends)))
    monkeypatch.setattr(storage, 'CONFIG_PATH', config)
    monkeypatch.delenv(storage.BACKEND_ENV, raising=False)
    monkeypatch.delenv(storage.IDENTITY_ENV, raising=False)
    monkeypatch.delenv('GRASP_RECORD_ROOT', raising=False)
    mounts = {Path(row['mount_point']) for row in backends.values()}
    monkeypatch.setattr(Path, 'is_mount', lambda path: path in mounts)
    def identity(mount, uuid, kind):
        storage.require(mount.is_mount(), 'selected mount is not mounted')
        storage.require(not storage.os.statvfs(mount).f_flag & storage.os.ST_RDONLY, 'selected disk is read only')
        return dict(mount_id=17 if mount.name == 'ssd' else 19, device='fake-'+mount.name,
                    filesystem_uuid=uuid, filesystem_type=kind, mount_point=str(mount))
    monkeypatch.setattr(storage, 'mounted_identity', identity)
    return storage.select()
