"""Bounded RGB acquisition evidence, written only after the RPC action ends."""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

import numpy as np
from alicia_flexible_grasp.robot.experiment_storage_binding import select_storage


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError('unsupported evidence value: '+type(value).__name__)


def require_storage():
    selected = select_storage()
    selected.validate(selected.ros_log_root)
    return selected.ros_log_root


def save_artifacts(sources, outcome, *, audit=None, trajectory=None, robot_description=''):
    """At most initial/postplan/proof/submission frames; raw arrays remain uncompressed.

    Never called by a subscriber, proof callback, or before-send hook. This
    function does not authorize a move and cannot renew a source timestamp.
    """
    root = require_storage()
    selected = select_storage(path=root)
    binding = selected.validate(root, subtree=selected.ros_log_root)
    out = root / ('rgb_acquisition_gateway_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                  + '_' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True, exist_ok=False)
    selected.validate(out, expected=binding)
    report = dict(version=1, outcome=outcome, audit=audit, frames={}, storage=binding)
    if trajectory is not None:
        import io
        stream = io.BytesIO(); trajectory.serialize(stream)
        payload = stream.getvalue()
        selected.validate(out, expected=binding)
        (out/'trajectory.rosmsg').write_bytes(payload)
        report['trajectory_file'] = str(out/'trajectory.rosmsg')
        report['trajectory_sha256'] = hashlib.sha256(payload).hexdigest()
    if robot_description:
        selected.validate(out, expected=binding)
        (out/'robot_description.urdf').write_text(robot_description)
        report['robot_description_sha256'] = hashlib.sha256(robot_description.encode()).hexdigest()
    for label in ('initial', 'postplan', 'proof', 'submission'):
        source = sources.get(label)
        if not isinstance(source, dict):
            continue
        volume = source['volume']; context = source['source_context']
        camera = context['camera']
        path = out/(label+'_source.npz')
        selected.validate(out, expected=binding)
        np.savez(path, color_bgr=source['rgb'], depth_raw=source['depth_raw'],
            depth_scale=np.float64(source['depth_scale']),
            region_mask=source['mask'], source_stamp_ns=np.int64(source['source_stamp_ns']),
            K=np.array([camera[k] for k in ('fx','fy','cx','cy')]),
            T_base_camera=np.asarray(volume.T_base_camera), T_tool_camera=np.asarray(context['tool_camera']),
            q_source=np.asarray(source['q_source']))
        report['frames'][label] = dict(source_file=str(path),
            source_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            source_stamp_ns=str(source['source_stamp_ns']), source_id=source['source_id'],
            source_rgb_sha256=source['source_rgb_sha256'], source_depth_sha256=source['source_depth_sha256'],
            source_context=context, interest=source['interest'], volume=asdict(volume),
            accepted_before=source['accepted_before'], accepted_after=source['accepted_after'],
            scene=(source['proof_scene'].scope() if source.get('proof_scene') is not None else None))
    selected.validate(out, expected=binding)
    (out/'result.json').write_text(json.dumps(report, indent=2, sort_keys=True,allow_nan=False,default=_json_value)+'\n')
    return str(out)
