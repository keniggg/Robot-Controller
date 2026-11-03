"""Offline OS adapter: fake only disk identity; exercise real storage and files.

The isolated runner places the pytest temporary tree on its writable SSD worker
directory. No block device is visible in that namespace. Each test therefore
declares two synthetic mounts/UUIDs and substitutes only mounted_identity().
Configuration parsing/hash binding, path/subtree/symlink/device/permission checks,
file bytes, gzip, locks and serialization remain the production implementation.
No deployment identity or robot authorization is added to the environment.
"""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from alicia_flexible_grasp.robot.experiment_storage_binding import storage_module


@pytest.fixture(autouse=True)
def storage_os(tmp_path_factory, monkeypatch):
    sandbox = tmp_path_factory.mktemp('storage_os')
    helper = storage_module()
    backends, states = {}, {}
    for index, name in enumerate(('ssd', 'sanstand')):
        mount = sandbox/name; base = mount/'experiments'; base.mkdir(parents=True)
        backends[name] = dict(mount_point=str(mount), experiment_root=str(base),
            archive_root=str(base/'grasp_records'), ros_log_root=str(base/'ros_log'),
            maintenance_root=str(base/'maintenance'), filesystem_uuid='fixture-'+name,
            filesystem_type='ext4' if name == 'ssd' else 'exfat')
        states[name] = dict(mounted=True,mount_id=101+index,uuid='fixture-'+name)
    config = sandbox/'storage.json'
    config.write_text(json.dumps(dict(schema=2,default_backend='ssd',backends=backends)))
    monkeypatch.setattr(helper, 'CONFIG_PATH', config)
    monkeypatch.delenv(helper.BACKEND_ENV, raising=False)
    monkeypatch.delenv(helper.IDENTITY_ENV, raising=False)
    monkeypatch.delenv('GRASP_RECORD_ROOT', raising=False)
    def mounted_identity(mount, expected_uuid, expected_type):
        name = next((n for n, row in backends.items() if Path(row['mount_point']) == mount), None)
        helper.require(name is not None, 'undeclared test mount')
        state, row = states[name], backends[name]
        helper.require(state['mounted'] is True, 'selected mount is not mounted')
        helper.require(expected_uuid == state['uuid'], 'mounted filesystem UUID differs')
        helper.require(expected_type == row['filesystem_type'], 'mounted filesystem type differs')
        helper.require(not os.statvfs(mount).f_flag & os.ST_RDONLY, 'selected disk is read only')
        device = mount.stat().st_dev
        return dict(mount_id=state['mount_id'], device='%d:%d'%(os.major(device),os.minor(device)),
                    filesystem_uuid=expected_uuid, filesystem_type=expected_type,mount_point=str(mount))
    monkeypatch.setattr(helper, 'mounted_identity', mounted_identity)
    selected = helper.select()
    monkeypatch.setenv('ROS_LOG_DIR',str(selected.base/'ros'))
    return SimpleNamespace(helper=helper,selected=selected,config=config,states=states,backends=backends,
                           sandbox=sandbox,select=helper.select)


@pytest.fixture
def tmp_path(storage_os):
    """Keep existing tests' relative trees inside the selected real base path."""
    return storage_os.selected.base
