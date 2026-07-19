#!/usr/bin/env python3
from xml.etree import ElementTree
import http.client
import importlib.util
import json
import math
import os
import pathlib
import struct
import threading
import types
import urllib.request

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'mujoco_digital_twin_server.py'
CLIENT_SCRIPT = (
    ROOT
    / 'src'
    / 'alicia_flexible_grasp_supervisor'
    / 'src'
    / 'alicia_flexible_grasp'
    / 'vision'
    / 'mujoco_digital_twin_client.py'
)
MODEL_XML = (
    ROOT
    / 'src'
    / 'arm-mujoco'
    / 'synriard'
    / 'mjcf'
    / 'Alicia_D_v5_6'
    / 'Alicia_D_v5_6_gripper_50mm.xml'
)
spec = importlib.util.spec_from_file_location('mujoco_digital_twin_server', str(SCRIPT))
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)
client_spec = importlib.util.spec_from_file_location('mujoco_digital_twin_client_contract', str(CLIENT_SCRIPT))
client_module = importlib.util.module_from_spec(client_spec)
client_spec.loader.exec_module(client_module)


PERFORMANCE_FIELDS = (
    'server_receive_sec',
    'server_send_sec',
    'preprocess_ms',
    'inference_ms',
    'postprocess_ms',
    'server_total_ms',
    'gpu_allocated_mb',
    'gpu_reserved_mb',
    'gpu_peak_allocated_mb',
)


BASE_XML = '''
<mujoco model="test_gripper">
  <asset/>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.125" contype="1" conaffinity="1"/>
    <body name="Link6">
      <body name="Link7"><joint name="left_finger" type="slide" range="-0.025 0"/></body>
      <body name="Link8"><joint name="right_finger" type="slide" range="0 0.025"/></body>
    </body>
  </worldbody>
</mujoco>
'''


class FakeContact:
    def __init__(self, first_body, second_body, distance_m=0.0):
        self.first_body = first_body
        self.second_body = second_body
        self.distance_m = distance_m


def valid_payload(now_sec=100.0):
    trajectory = []
    for _name, position in (
        ('pregrasp', [0.25, 0.0, 0.20]),
        ('approach', [0.28, 0.0, 0.16]),
        ('grasp', [0.30, 0.0, 0.12]),
        ('lift', [0.30, 0.0, 0.20]),
    ):
        trajectory.append(
            {
                'position_m': position,
                'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            }
        )
    return {
        'schema_version': 2,
        'plan_id': 'plan-carton-001',
        'snapshot_stamp_sec': now_sec - 0.5,
        'model_choice': 'carton_seg',
        'joint_names': ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
        'joint_positions': [0.0, 0.1, -0.2, 0.3, -0.1, 0.2],
        'trajectory': trajectory,
        'candidate_width_m': 0.040,
        'required_open_width_m': 0.045,
        'gripper': {
            'model_name': 'Alicia_D_v5_6_gripper_50mm',
            'max_inner_gap_m': 0.050,
            'finger_size_xyz_m': [0.0434, 0.0286, 0.0600],
            'palm_size_xyz_m': [0.1175, 0.1550, 0.0774],
        },
        'object_model': {
            'type': 'carton_box',
            'pose_base': {
                'position_m': [0.30, 0.0, 0.04],
                'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            },
            'size_xyz_m': [0.20, 0.10, 0.08],
            'mass_kg': 0.08,
            'friction': [1.2, 0.08, 0.02],
        },
        'support_plane': {
            'normal_base': [0.0, 0.0, 1.0],
            'offset_m': 0.0,
        },
    }


def _pose(position, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return types.SimpleNamespace(
        position=types.SimpleNamespace(
            x=position[0],
            y=position[1],
            z=position[2],
        ),
        orientation=types.SimpleNamespace(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )


def test_task10_builder_output_is_accepted_by_server_schema_validator():
    now_sec = server_module.time.time()
    geometry = types.SimpleNamespace(
        valid=True,
        pose_base=_pose([0.30, 0.0, 0.04]),
        size_xyz_m=types.SimpleNamespace(x=0.20, y=0.10, z=0.08),
        support_normal_base=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
        support_offset_m=0.0,
    )
    plan = types.SimpleNamespace(
        valid=True,
        plan_id='task10-builder-contract',
        model_choice='carton_seg',
        score=0.9,
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_sec=lambda: now_sec - 0.5),
        ),
        poses=[
            _pose([0.25, 0.0, 0.20]),
            _pose([0.28, 0.0, 0.16]),
            _pose([0.30, 0.0, 0.12]),
            _pose([0.30, 0.0, 0.20]),
        ],
        candidate_width_m=0.040,
        required_open_width_m=0.045,
        object_geometry=geometry,
    )

    payload = client_module.build_mujoco_payload(
        plan,
        ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
        [0.0, 0.1, -0.2, 0.3, -0.1, 0.2],
    )

    assert set(payload) == set(valid_payload())
    validated = server_module._validate_v2_payload(payload, now_sec=now_sec)
    assert validated['plan_id'] == plan.plan_id


def _invalid_code(payload, now_sec=100.0):
    with pytest.raises(server_module.ProtocolValidationError) as caught:
        server_module._validate_v2_payload(payload, now_sec=now_sec)
    return caught.value.code


def test_validate_v2_payload_normalizes_pose_and_support_quaternions():
    payload = valid_payload()
    payload['trajectory'][0]['quaternion_xyzw'] = [0.0, 0.0, 0.0, 2.0]
    payload['support_plane']['normal_base'] = [0.0, 0.0, 4.0]
    payload['support_plane']['offset_m'] = 0.8

    validated = server_module._validate_v2_payload(payload, now_sec=100.0)

    assert math.sqrt(sum(value * value for value in validated['trajectory'][0]['quaternion_xyzw'])) == pytest.approx(1.0)
    assert validated['support_plane']['normal_base'] == pytest.approx([0.0, 0.0, 1.0])
    assert validated['support_plane']['offset_m'] == pytest.approx(0.2)


def test_snapshot_age_boundary_is_accepted_but_zero_and_future_are_stale():
    boundary = valid_payload()
    boundary['snapshot_stamp_sec'] = 98.0
    assert server_module._validate_v2_payload(boundary, now_sec=100.0)['plan_id'] == boundary['plan_id']

    for stamp in (0.0, 100.001):
        payload = valid_payload()
        payload['snapshot_stamp_sec'] = stamp
        assert _invalid_code(payload) == 'PLAN_STALE'


def test_plan_shape_failure_precedes_other_validation_failures():
    payload = valid_payload()
    payload['trajectory'] = payload['trajectory'][:3]
    payload['joint_positions'][0] = float('nan')
    payload['required_open_width_m'] = 0.2
    assert _invalid_code(payload) == 'PLAN_INVALID'


@pytest.mark.parametrize(
    ('mutation', 'expected_code'),
    [
        (lambda p: p.update(schema_version=1), 'PLAN_INVALID'),
        (lambda p: p.update(plan_id=''), 'PLAN_INVALID'),
        (lambda p: p.update(model_choice=''), 'PLAN_INVALID'),
        (lambda p: p.update(snapshot_stamp_sec=float('nan')), 'PLAN_INVALID'),
        (lambda p: p.update(snapshot_stamp_sec=97.0), 'PLAN_STALE'),
        (lambda p: p.update(joint_names=[]), 'JOINT_STATE_INVALID'),
        (lambda p: p.update(joint_names=['Joint1'] * 6), 'JOINT_STATE_INVALID'),
        (lambda p: p.update(joint_names=['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'left_finger']), 'JOINT_STATE_INVALID'),
        (lambda p: p.update(joint_positions=p['joint_positions'][:-1]), 'JOINT_STATE_INVALID'),
        (lambda p: p['joint_positions'].__setitem__(0, float('inf')), 'JOINT_STATE_INVALID'),
        (lambda p: p.update(trajectory=p['trajectory'][:3]), 'PLAN_INVALID'),
        (lambda p: p['trajectory'][0].update(position_m=[0.0, float('nan'), 0.0]), 'PLAN_INVALID'),
        (lambda p: p['trajectory'][0].update(quaternion_xyzw=[0.0, 0.0, 0.0, 0.0]), 'PLAN_INVALID'),
        (lambda p: p.update(candidate_width_m=float('nan')), 'PLAN_INVALID'),
        (lambda p: p.update(required_open_width_m=0.0), 'GRIPPER_TOO_NARROW'),
        (lambda p: p.update(required_open_width_m=0.05001), 'GRIPPER_TOO_NARROW'),
        (lambda p: p['gripper'].update(model_name='wrong_gripper'), 'GRIPPER_MODEL_MISMATCH'),
        (lambda p: p['gripper'].update(max_inner_gap_m=0.049), 'GRIPPER_MODEL_MISMATCH'),
        (lambda p: p['gripper'].update(finger_size_xyz_m=[0.0434, 0.0286, 0.059]), 'GRIPPER_MODEL_MISMATCH'),
        (lambda p: p['gripper'].update(palm_size_xyz_m=[0.1175, 0.1550, 0.0784]), 'GRIPPER_MODEL_MISMATCH'),
        (lambda p: p['support_plane'].update(normal_base=[0.0, 1.0]), 'SUPPORT_PLANE_INVALID'),
        (lambda p: p['support_plane'].update(normal_base=[0.0, 0.0, 0.0]), 'SUPPORT_PLANE_INVALID'),
        (lambda p: p['support_plane'].update(offset_m=float('inf')), 'SUPPORT_PLANE_INVALID'),
        (lambda p: p['object_model'].update(size_xyz_m=[0.2, -0.1, 0.08]), 'OBB_INVALID'),
        (lambda p: p['object_model'].update(size_xyz_m=[0.2, float('nan'), 0.08]), 'OBB_INVALID'),
        (lambda p: p['object_model'].update(type='mouse_compound'), 'OBB_INVALID'),
        (lambda p: p['object_model'].update(size_xyz_m=[0.6001, 0.1, 0.08]), 'OBB_INVALID'),
        (lambda p: p['object_model'].update(size_xyz_m=[0.2, 0.1, 0.5001]), 'OBB_INVALID'),
        (lambda p: p['object_model']['pose_base'].update(position_m=[0.3, 0.0, float('inf')]), 'OBB_INVALID'),
        (lambda p: p['object_model']['pose_base'].update(quaternion_xyzw=[0.0, 0.0, 0.0, 0.0]), 'OBB_INVALID'),
        (lambda p: p['object_model'].update(mass_kg=0.0), 'OBB_INVALID'),
        (lambda p: p['object_model'].update(friction=[1.2, -0.1, 0.02]), 'OBB_INVALID'),
    ],
)
def test_invalid_v2_payloads_fail_closed_with_stable_code(mutation, expected_code):
    payload = valid_payload()
    mutation(payload)
    assert _invalid_code(payload) == expected_code


def test_carton_size_is_quantized_to_one_millimetre_for_cache_key():
    first = valid_payload()
    second = valid_payload()
    first['object_model']['size_xyz_m'] = [0.2004, 0.1004, 0.0804]
    second['object_model']['size_xyz_m'] = [0.20049, 0.10049, 0.08049]

    assert server_module._model_cache_key(first) == server_module._model_cache_key(second)


def test_same_cache_key_injects_the_same_canonical_quantized_carton_size():
    first = valid_payload()['object_model']
    second = copy_mapping(first)
    first['size_xyz_m'] = [0.2004, 0.1004, 0.0804]
    second['size_xyz_m'] = [0.20049, 0.10049, 0.08049]

    first_carton = ElementTree.fromstring(
        server_module._inject_dynamic_scene(BASE_XML, first)
    ).find(".//geom[@name='target_carton']")
    second_carton = ElementTree.fromstring(
        server_module._inject_dynamic_scene(BASE_XML, second)
    ).find(".//geom[@name='target_carton']")
    assert first_carton.attrib['size'] == second_carton.attrib['size']
    assert first_carton.attrib['size'] == '0.10000 0.05000 0.04000'


def copy_mapping(value):
    return json.loads(json.dumps(value))


@pytest.mark.parametrize('field', ['mass_kg', 'friction'])
def test_model_cache_key_includes_dynamic_carton_material(field):
    first = valid_payload()
    second = valid_payload()
    if field == 'mass_kg':
        second['object_model'][field] = 0.09
    else:
        second['object_model'][field] = [1.1, 0.08, 0.02]
    assert server_module._model_cache_key(first) != server_module._model_cache_key(second)


def test_model_cache_reuses_geometry_but_dynamic_scene_state_is_per_request():
    first = valid_payload()
    second = valid_payload()
    second['object_model']['pose_base']['position_m'] = [0.42, -0.10, 0.07]
    second['support_plane'] = {'normal_base': [1.0, 0.0, 0.0], 'offset_m': -0.25}

    assert server_module._model_cache_key(first) == server_module._model_cache_key(second)
    assert server_module._dynamic_scene_state(first) != server_module._dynamic_scene_state(second)


def test_dynamic_scene_state_converts_object_and_support_poses_to_mujoco_wxyz():
    payload = valid_payload()
    root_half = math.sqrt(0.5)
    payload['object_model']['pose_base'] = {
        'position_m': [0.42, -0.10, 0.07],
        'quaternion_xyzw': [0.0, 0.0, root_half, root_half],
    }
    payload['support_plane'] = {
        'normal_base': [2.0, 0.0, 0.0],
        'offset_m': -0.4,
    }

    state = server_module._dynamic_scene_state(payload)

    assert state['object_position_m'] == pytest.approx([0.42, -0.10, 0.07])
    assert state['object_quaternion_wxyz'] == pytest.approx([root_half, 0.0, 0.0, root_half])
    assert state['support_position_m'] == pytest.approx([0.2, 0.0, 0.0])
    assert state['support_quaternion_wxyz'] == pytest.approx([root_half, 0.0, root_half, 0.0])


def test_dynamic_scene_state_is_written_to_freejoint_and_mocap_for_each_request():
    class FakeModel:
        jnt_qposadr = server_module.np.asarray([2], dtype=int)

    class FakeData:
        def __init__(self):
            self.qpos = server_module.np.zeros(12, dtype=float)
            self.mocap_pos = server_module.np.zeros((1, 3), dtype=float)
            self.mocap_quat = server_module.np.zeros((1, 4), dtype=float)

    meta = {'object_joint': 0, 'support_mocap_id': 0}
    first = valid_payload()
    second = valid_payload()
    second['object_model']['pose_base']['position_m'] = [0.50, 0.02, 0.09]
    second['support_plane'] = {'normal_base': [1.0, 0.0, 0.0], 'offset_m': -0.3}
    first_data = FakeData()
    second_data = FakeData()

    server_module._apply_dynamic_scene_state(
        FakeModel(), first_data, meta, server_module._dynamic_scene_state(first)
    )
    server_module._apply_dynamic_scene_state(
        FakeModel(), second_data, meta, server_module._dynamic_scene_state(second)
    )

    assert first_data.qpos[2:5] == pytest.approx([0.30, 0.0, 0.04])
    assert first_data.qpos[5:9] == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert second_data.qpos[2:5] == pytest.approx([0.50, 0.02, 0.09])
    assert second_data.mocap_pos[0] == pytest.approx([0.3, 0.0, 0.0])
    assert second_data.mocap_quat[0] == pytest.approx(
        server_module._support_plane_pose([1.0, 0.0, 0.0], -0.3)[1]
    )


def test_invalid_payload_is_rejected_before_mujoco_import_or_model_loading(monkeypatch):
    backend = server_module.MujocoDigitalTwinBackend(model_xml=MODEL_XML)
    calls = []

    def forbidden_import():
        calls.append('import')
        raise AssertionError('MuJoCo import happened before schema validation')

    monkeypatch.setattr(backend, '_import_mujoco', forbidden_import)
    payload = valid_payload(now_sec=server_module.time.time())
    payload['trajectory'] = payload['trajectory'][:3]

    response = backend.simulate_grasp(payload)

    assert calls == []
    assert response['plan_id'] == payload['plan_id']
    assert response['failure_code'] == 'PLAN_INVALID'
    assert response['simulation_ok'] is False


def test_compiled_model_path_invokes_runtime_gripper_contract_validator(monkeypatch):
    fake_model = object()

    class FakeMjModelFactory:
        @staticmethod
        def from_xml_string(_xml):
            return fake_model

    class FakeMujoco:
        MjModel = FakeMjModelFactory

        @staticmethod
        def MjData(model):
            assert model is fake_model
            return object()

    backend = server_module.MujocoDigitalTwinBackend(model_xml=MODEL_XML)
    backend._mujoco = FakeMujoco()
    meta = {'compiled': True}
    calls = []
    monkeypatch.setattr(backend, '_model_meta', lambda model: meta)
    monkeypatch.setattr(
        backend,
        '_validate_runtime_gripper_contract',
        lambda model, data, model_meta: calls.append((model, data, model_meta)),
        raising=False,
    )

    model, _data, returned_meta = backend._model_for_payload(valid_payload())

    assert model is fake_model
    assert returned_meta is meta
    assert len(calls) == 1
    assert calls[0][0] is fake_model
    assert calls[0][2] is meta


def test_injected_carton_uses_half_extents_and_dynamic_support_body():
    xml = server_module._inject_dynamic_scene(BASE_XML, valid_payload()['object_model'])
    root = ElementTree.fromstring(xml)
    carton = root.find(".//geom[@name='target_carton']")
    assert carton is not None
    assert carton.attrib['size'] == '0.10000 0.05000 0.04000'
    assert carton.attrib['mass'] == '0.08000'
    assert carton.attrib['friction'] == '1.20000 0.08000 0.02000'
    assert root.find(".//body[@name='target_object']/freejoint[@name='target_object_joint']") is not None
    assert root.find(".//body[@name='detected_support']").attrib['mocap'] == 'true'
    assert root.find(".//body[@name='detected_support']/geom").attrib['type'] == 'plane'


def test_schema_v2_dynamic_scene_disables_legacy_floor_collision_authority():
    root = ElementTree.fromstring(
        server_module._inject_dynamic_scene(BASE_XML, valid_payload()['object_model'])
    )
    floor = root.find(".//geom[@name='floor']")
    assert floor.attrib['contype'] == '0'
    assert floor.attrib['conaffinity'] == '0'


def test_support_plane_pose_uses_point_minus_offset_times_normal():
    position, quaternion_wxyz = server_module._support_plane_pose([0.0, 0.0, 1.0], 0.12)
    assert position == pytest.approx([0.0, 0.0, -0.12])
    assert quaternion_wxyz == pytest.approx([1.0, 0.0, 0.0, 0.0])


def test_support_plane_quaternion_rotates_local_z_onto_detected_normal():
    _, quaternion_wxyz = server_module._support_plane_pose([1.0, 0.0, 0.0], -0.2)
    rotated = server_module._rotate_vector_by_quaternion_wxyz(quaternion_wxyz, [0.0, 0.0, 1.0])
    assert rotated == pytest.approx([1.0, 0.0, 0.0], abs=1e-7)


def test_support_plane_quaternion_handles_opposite_z_without_nan():
    position, quaternion_wxyz = server_module._support_plane_pose([0.0, 0.0, -2.0], 0.4)
    rotated = server_module._rotate_vector_by_quaternion_wxyz(quaternion_wxyz, [0.0, 0.0, 1.0])
    assert position == pytest.approx([0.0, 0.0, 0.2])
    assert rotated == pytest.approx([0.0, 0.0, -1.0], abs=1e-7)
    assert all(math.isfinite(value) for value in quaternion_wxyz)


@pytest.fixture
def current_gripper_contract():
    root = ElementTree.parse(str(MODEL_XML)).getroot()
    left_range = [float(v) for v in root.find(".//joint[@name='left_finger']").attrib['range'].split()]
    right_range = [float(v) for v in root.find(".//joint[@name='right_finger']").attrib['range'].split()]
    mesh_dir = (MODEL_XML.parent / root.find('compiler').attrib['meshdir']).resolve()
    mesh_files = {
        mesh.attrib['name']: mesh_dir / mesh.attrib['file']
        for mesh in root.findall('./asset/mesh')
    }

    facing_surfaces = {}
    for body_name in ('Link7', 'Link8'):
        body = root.find(".//body[@name='%s']" % body_name)
        collision_geom = next(
            geom
            for geom in body.findall('geom')
            if geom.attrib.get('mesh') and geom.attrib.get('contype', '1') != '0'
        )
        vertices = _binary_stl_vertices(mesh_files[collision_geom.attrib['mesh']])
        rotation = _rotation_matrix_wxyz([float(v) for v in body.attrib['quat'].split()])
        position = server_module.np.asarray(
            [float(v) for v in body.attrib['pos'].split()],
            dtype=float,
        )
        transformed = vertices @ rotation.T + position
        facing_surfaces[body_name] = (
            float(transformed[:, 1].max())
            if body_name == 'Link7'
            else float(transformed[:, 1].min())
        )
    facing_surface_gap_m = facing_surfaces['Link8'] - facing_surfaces['Link7']
    return left_range, right_range, facing_surfaces, facing_surface_gap_m


def _binary_stl_vertices(path):
    data = path.read_bytes()
    triangle_count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + triangle_count * 50
    vertices = []
    for index in range(triangle_count):
        record = struct.unpack_from('<12fH', data, 84 + index * 50)
        vertices.extend((record[3:6], record[6:9], record[9:12]))
    return server_module.np.asarray(vertices, dtype=float)


def _rotation_matrix_wxyz(quaternion):
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return server_module.np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def test_current_v56_gripper_joint_ranges_and_facing_gap_satisfy_contract(current_gripper_contract):
    left_range, right_range, facing_surfaces, gap_m = current_gripper_contract
    assert facing_surfaces['Link7'] == pytest.approx(-0.02496875, abs=1e-7)
    assert facing_surfaces['Link8'] == pytest.approx(0.02496875, abs=1e-7)
    assert gap_m == pytest.approx(0.04993750, abs=1e-7)
    server_module._validate_gripper_contract_values(left_range, right_range, gap_m)


@pytest.mark.parametrize(
    ('left_range', 'right_range', 'gap_m'),
    [
        ([-0.024, 0.0], [0.0, 0.025], 0.0499375),
        ([-0.025, 0.0], [0.0, 0.024], 0.0499375),
        ([-0.025, 0.0], [0.0, 0.025], 0.0494),
        ([-0.025, 0.0], [0.0, 0.025], 0.0506),
    ],
)
def test_gripper_contract_mismatch_fails_closed(left_range, right_range, gap_m):
    with pytest.raises(server_module.ProtocolValidationError) as caught:
        server_module._validate_gripper_contract_values(left_range, right_range, gap_m)
    assert caught.value.code == 'GRIPPER_MODEL_MISMATCH'


def test_requested_inner_gap_maps_open_to_zero_joint_travel_and_closed_to_limits():
    assert server_module._finger_qpos_for_inner_gap(0.050, max_gap_m=0.050) == pytest.approx((0.0, 0.0))
    assert server_module._finger_qpos_for_inner_gap(0.000, max_gap_m=0.050) == pytest.approx((-0.025, 0.025))


@pytest.mark.parametrize('width_m', [-0.001, 0.0501, float('nan')])
def test_requested_inner_gap_mapping_rejects_out_of_range_instead_of_clamping(width_m):
    with pytest.raises(server_module.ProtocolValidationError) as caught:
        server_module._finger_qpos_for_inner_gap(width_m, max_gap_m=0.050)
    assert caught.value.code == 'GRIPPER_TOO_NARROW'


def test_closure_widths_are_monotonic_and_have_at_least_35_increments():
    widths = server_module._closure_widths(0.050, 0.0)
    assert len(widths) >= 36
    assert widths[0] == pytest.approx(0.050)
    assert widths[-1] == pytest.approx(0.0)
    assert all(next_width < width for width, next_width in zip(widths, widths[1:]))


def test_arm_and_gripper_targets_hold_qpos_qvel_and_position_actuator_ctrl():
    class FakeMujoco:
        forward_calls = 0

        @classmethod
        def mj_forward(cls, _model, _data):
            cls.forward_calls += 1

    class FakeModel:
        jnt_qposadr = server_module.np.asarray([0, 1], dtype=int)
        jnt_dofadr = server_module.np.asarray([0, 1], dtype=int)

    class FakeData:
        qpos = server_module.np.zeros(2, dtype=float)
        qvel = server_module.np.ones(2, dtype=float)
        ctrl = server_module.np.zeros(4, dtype=float)

    backend = server_module.MujocoDigitalTwinBackend(model_xml=MODEL_XML)
    backend._mujoco = FakeMujoco
    data = FakeData()
    backend._set_arm_qpos(
        FakeModel(),
        data,
        joints=[0, 1],
        values=[0.2, -0.3],
        actuator_ids=[2, 3],
    )
    assert data.qpos == pytest.approx([0.2, -0.3])
    assert data.qvel == pytest.approx([0.0, 0.0])
    assert data.ctrl == pytest.approx([0.0, 0.0, 0.2, -0.3])

    data.qvel[:] = 1.0
    meta = {
        'left_finger_joint': 0,
        'right_finger_joint': 1,
        'left_finger_actuator': 0,
        'right_finger_actuator': 1,
    }
    backend._apply_gripper_inner_gap(FakeModel(), data, 0.031, meta)
    assert data.qpos == pytest.approx([-0.0095, 0.0095])
    assert data.qvel == pytest.approx([0.0, 0.0])
    assert data.ctrl[:2] == pytest.approx([-0.0095, 0.0095])


def test_single_finger_contact_never_passes():
    contacts = [FakeContact('Link7', 'target_object')]
    result = server_module._classify_close_contacts(contacts, left_body='Link7', right_body='Link8')
    assert result.left_contact
    assert not result.right_contact
    assert not result.two_sided


def test_contact_body_pair_order_is_symmetric():
    result = server_module._classify_close_contacts(
        [
            FakeContact('target_object', 'Link7'),
            FakeContact('Link8', 'target_object'),
        ],
        left_body='Link7',
        right_body='Link8',
    )
    assert result.two_sided


def test_palm_first_contact_is_disallowed():
    result = server_module._classify_close_contacts(
        [FakeContact('Link6', 'target_object')],
        left_body='Link7',
        right_body='Link8',
    )
    assert result.palm_contact
    assert result.disallowed_collision


def test_object_support_penetration_is_disallowed():
    result = server_module._classify_close_contacts(
        [FakeContact('target_object', 'detected_support', distance_m=-0.002)],
        left_body='Link7',
        right_body='Link8',
    )
    assert result.object_support_penetration
    assert result.disallowed_collision


def test_normal_object_support_contact_is_allowed_but_robot_support_is_not():
    support_contact = server_module._classify_close_contacts(
        [FakeContact('target_object', 'detected_support', distance_m=-0.0001)],
        left_body='Link7',
        right_body='Link8',
    )
    assert not support_contact.object_support_penetration
    assert not support_contact.disallowed_collision

    robot_contact = server_module._classify_close_contacts(
        [FakeContact('Link3', 'detected_support')],
        left_body='Link7',
        right_body='Link8',
    )
    assert robot_contact.robot_support_collision
    assert robot_contact.disallowed_collision


def test_first_simultaneous_two_sided_contact_width_is_retained():
    samples = [
        {'width_m': 0.040, 'contacts': [], 'object_position_m': [0.0, 0.0, 0.0]},
        {
            'width_m': 0.035,
            'contacts': [FakeContact('Link7', 'target_object')],
            'object_position_m': [0.0002, 0.0, 0.0],
        },
        {
            'width_m': 0.031,
            'contacts': [
                FakeContact('Link7', 'target_object'),
                FakeContact('Link8', 'target_object'),
            ],
            'object_position_m': [0.0003, 0.0, 0.0],
        },
        {
            'width_m': 0.020,
            'contacts': [
                FakeContact('Link7', 'target_object'),
                FakeContact('Link8', 'target_object'),
            ],
            'object_position_m': [0.0003, 0.0, 0.0],
        },
    ]
    result = server_module._evaluate_close_contact_samples(
        samples,
        left_body='Link7',
        right_body='Link8',
    )
    assert result.success
    assert result.contact_width_m == pytest.approx(0.031)


def test_single_finger_only_close_sequence_fails():
    result = server_module._evaluate_close_contact_samples(
        [
            {'width_m': 0.040, 'contacts': [], 'object_position_m': [0.0, 0.0, 0.0]},
            {
                'width_m': 0.030,
                'contacts': [FakeContact('Link7', 'target_object')],
                'object_position_m': [0.0002, 0.0, 0.0],
            },
            {
                'width_m': 0.020,
                'contacts': [FakeContact('Link7', 'target_object')],
                'object_position_m': [0.0003, 0.0, 0.0],
            },
        ],
        left_body='Link7',
        right_body='Link8',
    )
    assert not result.success
    assert result.failure_code == 'MUJOCO_CONTACT_FAILED'


def test_single_finger_push_beyond_stability_threshold_fails_immediately():
    result = server_module._evaluate_close_contact_samples(
        [
            {'width_m': 0.040, 'contacts': [], 'object_position_m': [0.0, 0.0, 0.0]},
            {
                'width_m': 0.035,
                'contacts': [FakeContact('Link7', 'target_object')],
                'object_position_m': [0.003, 0.0, 0.0],
            },
        ],
        left_body='Link7',
        right_body='Link8',
        max_single_finger_object_motion_m=0.002,
    )
    assert not result.success
    assert result.failure_code == 'MUJOCO_CONTACT_FAILED'
    assert 'unstable' in result.failure_reason


def test_lift_holds_retained_contact_width_and_checks_every_step(monkeypatch):
    class FakeData:
        def __init__(self):
            self.xpos = server_module.np.asarray([[0.0, 0.0, 0.04]], dtype=float)

    class FakeMujoco:
        @staticmethod
        def mj_forward(_model, _data):
            return None

        @staticmethod
        def mj_step(_model, data):
            data.xpos[0, 2] += 0.0015

    backend = server_module.MujocoDigitalTwinBackend(model_xml=MODEL_XML)
    backend._mujoco = FakeMujoco
    retained_data = FakeData()
    widths = []
    classifications = []
    safe_contact = None

    def classify_every_step(_model, _data, _meta):
        nonlocal safe_contact
        if safe_contact is None:
            safe_contact = server_module._classify_close_contacts(
                [
                    FakeContact('Link7', 'target_object'),
                    FakeContact('Link8', 'target_object'),
                ],
                left_body='Link7',
                right_body='Link8',
            )
        classifications.append(safe_contact)
        return safe_contact

    monkeypatch.setattr(backend, '_copy_data', lambda _model, data: data)
    monkeypatch.setattr(backend, '_set_arm_qpos', lambda *args, **kwargs: None)
    monkeypatch.setattr(
        backend,
        '_apply_gripper_inner_gap',
        lambda _model, _data, width_m, _meta: widths.append(width_m),
        raising=False,
    )
    monkeypatch.setattr(
        backend,
        '_classify_current_contacts',
        classify_every_step,
        raising=False,
    )

    result = backend._simulate_lift(
        model=object(),
        retained_data=retained_data,
        meta={'arm_joints': [0], 'arm_actuators': [0], 'object_body': 0},
        grasp_q=[0.0],
        lift_q=[1.0],
        payload=valid_payload(),
        contact_width_m=0.031,
        commanded_lift_m=0.080,
    )

    assert result.collision_free
    assert result.contact_retained
    assert result.lift_success
    assert len(widths) >= 35
    assert len(classifications) == len(widths)
    assert all(width == pytest.approx(0.031) for width in widths)
    assert all(item.two_sided and not item.disallowed_collision for item in classifications)


@pytest.mark.parametrize(
    ('classification', 'step_delta_m', 'expected'),
    [
        (
            server_module.ContactClassification(
                left_contact=True,
                right_contact=True,
                two_sided=True,
                disallowed_collision=True,
            ),
            0.002,
            (False, True, False, 'MUJOCO_COLLISION'),
        ),
        (
            server_module.ContactClassification(),
            0.002,
            (True, False, False, 'MUJOCO_CONTACT_FAILED'),
        ),
        (
            server_module.ContactClassification(
                left_contact=True,
                right_contact=True,
                two_sided=True,
            ),
            0.0005,
            (True, True, False, 'MUJOCO_LIFT_FAILED'),
        ),
    ],
    ids=('collision', 'contact-lost', 'insufficient-displacement'),
)
def test_lift_result_preserves_failure_component_and_backend_threshold(
    monkeypatch,
    classification,
    step_delta_m,
    expected,
):
    class FakeData:
        def __init__(self):
            self.xpos = server_module.np.asarray([[0.0, 0.0, 0.04]], dtype=float)

    class FakeMujoco:
        @staticmethod
        def mj_step(_model, data):
            data.xpos[0, 2] += step_delta_m

    backend = server_module.MujocoDigitalTwinBackend(
        model_xml=MODEL_XML,
        min_lift_success_m=0.030,
    )
    backend._mujoco = FakeMujoco
    monkeypatch.setattr(backend, '_copy_data', lambda _model, data: data)
    monkeypatch.setattr(backend, '_set_arm_qpos', lambda *args, **kwargs: None)
    monkeypatch.setattr(backend, '_apply_gripper_inner_gap', lambda *args, **kwargs: None)
    monkeypatch.setattr(
        backend,
        '_classify_current_contacts',
        lambda *_args, **_kwargs: classification,
    )
    payload = valid_payload()
    # This unknown extension is deliberately adversarial: only the backend
    # constructor may define the safety threshold.
    payload['min_lift_success_m'] = 0.001

    result = backend._simulate_lift(
        model=object(),
        retained_data=FakeData(),
        meta={'arm_joints': [0], 'arm_actuators': [0], 'object_body': 0},
        grasp_q=[0.0],
        lift_q=[1.0],
        payload=payload,
        contact_width_m=0.031,
        commanded_lift_m=0.080,
    )

    assert (
        result.collision_free,
        result.contact_retained,
        result.lift_success,
        result.failure_code,
    ) == expected


@pytest.mark.parametrize(
    ('lift_flags', 'expected_code', 'expected_components'),
    [
        ((False, True, False), 'MUJOCO_COLLISION', (False, True, False)),
        ((True, False, False), 'MUJOCO_CONTACT_FAILED', (True, False, False)),
        ((True, True, False), 'MUJOCO_LIFT_FAILED', (True, True, False)),
    ],
    ids=('collision', 'contact-lost', 'lift-short'),
)
def test_validated_simulation_maps_lift_failure_to_the_exact_component(
    monkeypatch,
    lift_flags,
    expected_code,
    expected_components,
):
    class FakeMujoco:
        @staticmethod
        def mj_forward(_model, _data):
            return None

    backend = server_module.MujocoDigitalTwinBackend(model_xml=MODEL_XML)
    backend._mujoco = FakeMujoco
    model = object()
    data = object()
    meta = {'arm_joints': [], 'arm_actuators': []}
    monkeypatch.setattr(backend, '_import_mujoco', lambda: FakeMujoco)
    monkeypatch.setattr(backend, '_model_for_payload', lambda _payload: (model, data, meta))
    monkeypatch.setattr(backend, '_apply_joint_state', lambda *args, **kwargs: None)
    monkeypatch.setattr(backend, '_apply_gripper_inner_gap', lambda *args, **kwargs: None)
    monkeypatch.setattr(server_module, '_apply_dynamic_scene_state', lambda *args, **kwargs: None)
    monkeypatch.setattr(backend, '_copy_data', lambda _model, value: value)
    monkeypatch.setattr(
        backend,
        '_solve_ik',
        lambda *args, **kwargs: {
            'success': True,
            'joint_positions': [0.0] * 6,
            'position_error_m': 0.0,
            'orientation_error': 0.0,
            'iterations': 1,
        },
    )
    monkeypatch.setattr(backend, '_set_arm_qpos', lambda *args, **kwargs: None)
    monkeypatch.setattr(
        backend,
        '_check_trajectory_collisions',
        lambda *args, **kwargs: (True, ['trajectory clear']),
    )
    monkeypatch.setattr(
        backend,
        '_simulate_close_contact',
        lambda *args, **kwargs: (
            True,
            True,
            0.031,
            data,
            ['two-sided contact'],
        ),
    )
    collision_free, contact_retained, lift_success = lift_flags
    monkeypatch.setattr(
        backend,
        '_simulate_lift',
        lambda *args, **kwargs: server_module.LiftResult(
            collision_free=collision_free,
            contact_retained=contact_retained,
            lift_success=lift_success,
            failure_code=expected_code,
            failure_reason='synthetic lift classification',
            diagnosis=('synthetic lift classification',),
        ),
    )
    now_sec = server_module.time.time()
    payload = server_module._validate_v2_payload(
        valid_payload(now_sec=now_sec),
        now_sec=now_sec,
    )

    response = backend._simulate_validated_grasp(payload, payload['plan_id'])

    assert response['failure_code'] == expected_code
    assert (
        response['collision_free'],
        response['contact_success'],
        response['lift_success'],
    ) == expected_components
    assert response['simulation_ok'] is False


def test_lift_failure_when_object_does_not_follow_or_contact_is_lost():
    assert not server_module._lift_succeeded(
        object_delta_m=0.005,
        commanded_delta_m=0.080,
        contact_retained=True,
        collision_free=True,
        min_lift_m=0.015,
    )
    assert not server_module._lift_succeeded(
        object_delta_m=float('nan'),
        commanded_delta_m=0.080,
        contact_retained=True,
        collision_free=True,
        min_lift_m=0.015,
    )
    assert not server_module._lift_succeeded(
        object_delta_m=0.080,
        commanded_delta_m=0.080,
        contact_retained=True,
        collision_free=False,
        min_lift_m=0.015,
    )
    assert not server_module._lift_succeeded(
        object_delta_m=0.080,
        commanded_delta_m=0.080,
        contact_retained=False,
        collision_free=True,
        min_lift_m=0.015,
    )


def test_component_response_echoes_plan_and_requires_all_components():
    passing = server_module._build_component_response(
        plan_id='plan-carton-001',
        pass_score=80,
        score=90.0,
        ik_success=True,
        collision_free=True,
        contact_success=True,
        lift_success=True,
    )
    assert passing['plan_id'] == 'plan-carton-001'
    assert passing['simulation_ok'] is True
    assert passing['failure_code'] == ''
    assert passing['failure_reason'] == ''
    assert passing['simulation_ok'] == all(
        passing[key]
        for key in ('ik_success', 'collision_free', 'contact_success', 'lift_success')
    )

    blocked = server_module._build_component_response(
        plan_id='plan-carton-001',
        pass_score=80,
        score=100.0,
        ik_success=True,
        collision_free=True,
        contact_success=False,
        lift_success=True,
        failure_code='MUJOCO_CONTACT_FAILED',
        failure_reason='two-sided contact was not retained',
    )
    assert blocked['simulation_ok'] is False
    assert blocked['failure_code'] == 'MUJOCO_CONTACT_FAILED'


def test_component_response_rejects_nonfinite_or_below_threshold_score():
    nonfinite = server_module._build_component_response(
        plan_id='plan-carton-001',
        pass_score=80,
        score=float('nan'),
        ik_success=True,
        collision_free=True,
        contact_success=True,
        lift_success=True,
    )
    assert nonfinite['simulation_ok'] is False
    assert math.isfinite(nonfinite['score'])

    below = server_module._build_component_response(
        plan_id='plan-carton-001',
        pass_score=80,
        score=79.999,
        ik_success=True,
        collision_free=True,
        contact_success=True,
        lift_success=True,
    )
    assert below['simulation_ok'] is False
    assert below['failure_code'] == 'MUJOCO_SCORE_BELOW_THRESHOLD'


@pytest.mark.parametrize('invalid_plan_id', ['', None, 123])
def test_mock_backend_invalid_plan_id_echoes_empty_string_and_fails_closed(invalid_plan_id):
    payload = valid_payload(now_sec=server_module.time.time())
    if invalid_plan_id is None:
        payload.pop('plan_id')
    else:
        payload['plan_id'] = invalid_plan_id
    response = server_module.MockDigitalTwinBackend().simulate_grasp(payload)
    assert response['plan_id'] == ''
    assert response['failure_code'] == 'PLAN_INVALID'
    assert response['simulation_ok'] is False


def test_mock_backend_rejects_malformed_payload_and_echoes_available_plan_id():
    payload = valid_payload(now_sec=server_module.time.time())
    payload['trajectory'] = payload['trajectory'][:3]
    response = server_module.MockDigitalTwinBackend().simulate_grasp(payload)
    assert response['plan_id'] == payload['plan_id']
    assert response['failure_code'] == 'PLAN_INVALID'
    assert response['simulation_ok'] is False
    assert all(type(response[key]) is bool for key in (
        'simulation_ok',
        'ik_success',
        'collision_free',
        'contact_success',
        'lift_success',
    ))


def test_mock_backend_never_authorizes_physical_execution_for_valid_payload():
    response = server_module.MockDigitalTwinBackend().simulate_grasp(
        valid_payload(now_sec=server_module.time.time())
    )

    assert response['plan_id'] == 'plan-carton-001'
    assert response['failure_code'] == 'MUJOCO_INTERNAL_ERROR'
    assert 'mock' in response['failure_reason'].lower()
    assert 'physical' in response['failure_reason'].lower()
    assert response['score'] == 0.0
    assert all(response[key] is False for key in (
        'simulation_ok',
        'ik_success',
        'collision_free',
        'contact_success',
        'lift_success',
    ))


class PassingBackend(server_module.MockDigitalTwinBackend):
    name = 'test_passing_mujoco'

    def simulate_grasp(self, payload):
        return server_module._build_component_response(
            plan_id=payload.get('plan_id', ''),
            pass_score=80,
            score=90.0,
            ik_success=True,
            collision_free=True,
            contact_success=True,
            lift_success=True,
            backend=self.name,
        )


class DiagnosticGraspBackend:
    name = 'diagnostic_graspnet'

    def __init__(self):
        self.diagnostics = {}

    def health(self):
        return {
            'ok': True,
            'backend': self.name,
            'loaded': True,
            'protocol_version': server_module.GRASP6D_PROTOCOL_VERSION,
            'candidate_fields': list(server_module.CANDIDATE_FIELDS),
        }

    def predict_batch(self, payload):
        self.diagnostics = {
            'raw_candidates': 280,
            'after_nms': 41,
            'after_collision': 17,
            'returned': 12,
            'stages': {'width_rejected': 5},
        }
        return server_module.PredictionBatch(
            request_id=payload['request_id'],
            snapshot_stamp_sec=payload['snapshot_stamp_sec'],
            candidates=({'score': 0.99},),
            diagnostics=self.diagnostics,
            performance={
                'server_receive_sec': 200.0,
                'server_send_sec': 200.25,
                'preprocess_ms': 1.0,
                'inference_ms': 2.0,
                'postprocess_ms': 3.0,
                'server_total_ms': 6.0,
                'gpu_allocated_mb': 100.0,
                'gpu_reserved_mb': 120.0,
                'gpu_peak_allocated_mb': 110.0,
            },
        )


def _direct_predict(grasp_backend, payload=None):
    handler = types.SimpleNamespace(
        server=types.SimpleNamespace(grasp_backend=grasp_backend)
    )
    return server_module.MujocoDigitalTwinHTTPHandler._handle_predict(
        handler,
        (
            {'request_id': 41, 'snapshot_stamp_sec': 123.25}
            if payload is None
            else payload
        ),
    )


def test_unified_predict_contract_snapshots_complete_grasp_diagnostics():
    backend = DiagnosticGraspBackend()
    response = _direct_predict(backend)

    assert response['ok'] is True
    assert response['backend'] == backend.name
    assert response['protocol_version'] == 3
    assert response['candidate_fields'] == list(server_module.CANDIDATE_FIELDS)
    assert response['request_id'] == 41
    assert response['snapshot_stamp_sec'] == 123.25
    assert response['candidates'] == [{'score': 0.99}]
    assert response['diagnostics'] == {
        'raw_candidates': 280,
        'after_nms': 41,
        'after_collision': 17,
        'returned': 12,
        'stages': {'width_rejected': 5},
    }

    assert response['server_total_ms'] == 6.0
    assert response['gpu_reserved_mb'] == 120.0

    backend.diagnostics['raw_candidates'] = 0
    backend.diagnostics['stages']['width_rejected'] = 0
    assert response['diagnostics']['raw_candidates'] == 280
    assert response['diagnostics']['stages']['width_rejected'] == 5


@pytest.mark.parametrize(
    'unsafe_diagnostics',
    [
        None,
        ['raw_candidates', 280],
        {'raw_candidates': float('nan')},
        {'raw_candidates': object()},
    ],
    ids=('none', 'non-dict', 'nan', 'non-json-value'),
)
def test_unified_predict_normalizes_unsafe_diagnostics_to_empty_dict(unsafe_diagnostics):
    class UnsafeDiagnosticsBackend(DiagnosticGraspBackend):
        def predict_batch(self, payload):
            batch = super().predict_batch(payload)
            return server_module.PredictionBatch(
                request_id=batch.request_id,
                snapshot_stamp_sec=batch.snapshot_stamp_sec,
                candidates=(),
                diagnostics=unsafe_diagnostics,
                performance=batch.performance,
            )

    response = _direct_predict(UnsafeDiagnosticsBackend())

    assert response['ok'] is True
    assert response['candidates'] == []
    assert response['diagnostics'] == {}
    json.dumps(response, allow_nan=False)


def test_unified_predict_normalizes_diagnostics_property_exception():
    class ExplodingBatch:
        request_id = 41
        snapshot_stamp_sec = 123.25
        candidates = ()
        performance = {
            field: 0.0
            for field in PERFORMANCE_FIELDS
        }

        @property
        def diagnostics(self):
            raise RuntimeError('diagnostics unavailable')

    class ExplodingDiagnosticsBackend:
        name = 'exploding_diagnostics'

        def predict_batch(self, _payload):
            return ExplodingBatch()

    response = _direct_predict(ExplodingDiagnosticsBackend())

    assert response['ok'] is True
    assert response['diagnostics'] == {}


def test_unified_predict_backend_exception_returns_complete_fail_closed_contract():
    class ExplodingGraspBackend:
        name = 'exploding_graspnet'

        def predict_batch(self, _payload):
            raise RuntimeError('synthetic inference failure')

    response = _direct_predict(ExplodingGraspBackend())

    assert response['ok'] is False
    assert response['backend'] == 'exploding_graspnet'
    assert response['protocol_version'] == 3
    assert response['candidate_fields'] == list(server_module.CANDIDATE_FIELDS)
    assert response['request_id'] == 41
    assert response['snapshot_stamp_sec'] == 123.25
    assert response['candidates'] == []
    assert response['diagnostics'] == {}
    assert all(response[field] == 0.0 for field in PERFORMANCE_FIELDS)
    assert response['error'] == 'synthetic inference failure'


def test_unified_http_predict_transports_grasp_diagnostics_contract():
    grasp_backend = DiagnosticGraspBackend()
    http_server = server_module.make_server(
        '127.0.0.1',
        0,
        grasp_backend=grasp_backend,
        sim_backend=PassingBackend(),
    )
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()
    try:
        response = _json_post(
            'http://127.0.0.1:%d/predict' % http_server.server_port,
            {
                'encoding': 'contract-test',
                'request_id': 41,
                'snapshot_stamp_sec': 123.25,
            },
        )
    finally:
        http_server.shutdown()
        http_server.server_close()
        thread.join(timeout=2.0)

    assert response['ok'] is True
    assert response['backend'] == grasp_backend.name
    assert response['protocol_version'] == 3
    assert response['candidate_fields'] == list(server_module.CANDIDATE_FIELDS)
    assert response['request_id'] == 41
    assert response['snapshot_stamp_sec'] == 123.25
    assert response['candidates'] == [{'score': 0.99}]
    assert response['diagnostics']['raw_candidates'] == 280
    assert response['diagnostics']['after_nms'] == 41
    assert response['diagnostics']['after_collision'] == 17
    assert response['diagnostics']['returned'] == 12
    assert response['inference_ms'] == 2.0
    assert response['gpu_peak_allocated_mb'] == 110.0


@pytest.mark.parametrize(
    ('body', 'content_length'),
    [
        (b'{"encoding":"broken",', None),
        (b'{}', 'not-an-integer'),
    ],
    ids=('malformed-json', 'invalid-content-length'),
)
def test_unified_http_predict_parse_failures_return_complete_fail_closed_contract(
    body,
    content_length,
):
    grasp_backend = DiagnosticGraspBackend()
    http_server = server_module.make_server(
        '127.0.0.1',
        0,
        grasp_backend=grasp_backend,
        sim_backend=PassingBackend(),
    )
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()
    try:
        raw, response = _raw_post(
            http_server.server_port,
            '/predict',
            body,
            content_length=content_length,
        )
    finally:
        http_server.shutdown()
        http_server.server_close()
        thread.join(timeout=2.0)

    assert b'NaN' not in raw and b'Infinity' not in raw
    assert response['ok'] is False
    assert response['backend'] == grasp_backend.name
    assert response['protocol_version'] == server_module.GRASP6D_PROTOCOL_VERSION
    assert response['candidate_fields'] == list(server_module.CANDIDATE_FIELDS)
    assert response['request_id'] is None
    assert response['snapshot_stamp_sec'] is None
    assert response['candidates'] == []
    assert response['diagnostics'] == {}
    assert all(response[field] == 0.0 for field in PERFORMANCE_FIELDS)
    assert isinstance(response['error'], str) and response['error']


def test_server_serves_schema_v2_health_sync_predict_and_simulate():
    http_server = server_module.make_server(
        '127.0.0.1',
        0,
        grasp_backend=server_module.MockGraspNetBackend(),
        sim_backend=PassingBackend(),
    )
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()
    try:
        base_url = 'http://127.0.0.1:%d' % http_server.server_port
        health = _json_get(base_url + '/health')
        assert health['ok'] is True
        assert health['protocol_version'] == 3
        assert health['candidate_fields'] == list(server_module.CANDIDATE_FIELDS)
        assert health['grasp_backend']['backend'] == 'mock'
        assert health['digital_twin']['backend'] == 'test_passing_mujoco'
        assert health['digital_twin']['joint_state_age_sec'] is None

        sync = _json_post(
            base_url + '/sync_joint_state',
            {'joint_names': ['Joint1'], 'joint_positions': [0.1]},
        )
        assert sync['ok'] is True

        predict = _json_post(
            base_url + '/predict',
            {
                'encoding': 'mock',
                'request_id': 7,
                'snapshot_stamp_sec': 123.25,
                'max_candidates': 1,
            },
        )
        assert predict['ok'] is True
        assert predict['backend'] == 'mock'
        assert predict['protocol_version'] == 3
        assert predict['candidate_fields'] == list(server_module.CANDIDATE_FIELDS)
        assert predict['request_id'] == 7
        assert predict['snapshot_stamp_sec'] == 123.25
        assert len(predict['candidates']) == 1
        assert predict['diagnostics']['returned'] == 1
        assert all(predict[field] >= 0.0 for field in PERFORMANCE_FIELDS)

        sim = _json_post(
            base_url + '/simulate_grasp',
            valid_payload(now_sec=server_module.time.time()),
        )
    finally:
        http_server.shutdown()
        http_server.server_close()
        thread.join(timeout=2.0)

    assert sim['ok'] is True
    assert sim['plan_id'] == 'plan-carton-001'
    assert sim['simulation_ok'] is True
    assert all(sim[key] is True for key in (
        'ik_success',
        'collision_free',
        'contact_success',
        'lift_success',
    ))


@pytest.mark.parametrize(
    ('body', 'content_length'),
    [
        (b'{"plan_id":"unrecoverable",', None),
        (b'{}', 'not-an-integer'),
    ],
    ids=('malformed-json', 'invalid-content-length'),
)
def test_simulate_grasp_parse_failures_are_component_complete(body, content_length):
    http_server = server_module.make_server(
        '127.0.0.1',
        0,
        grasp_backend=server_module.MockGraspNetBackend(),
        sim_backend=PassingBackend(),
    )
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()
    try:
        raw, response = _raw_post(
            http_server.server_port,
            '/simulate_grasp',
            body,
            content_length=content_length,
        )
    finally:
        http_server.shutdown()
        http_server.server_close()
        thread.join(timeout=2.0)

    assert b'NaN' not in raw and b'Infinity' not in raw
    _assert_complete_failure(response, plan_id='')
    assert response['failure_code'] == 'MUJOCO_INTERNAL_ERROR'


@pytest.mark.parametrize(
    'unsafe_result',
    [
        None,
        {'simulation_ok': True, 'score': 100.0},
        {
            'plan_id': 'plan-carton-001',
            'simulation_ok': True,
            'score': float('nan'),
            'ik_success': True,
            'collision_free': True,
            'contact_success': True,
            'lift_success': True,
        },
        {
            'plan_id': 'different-plan',
            'simulation_ok': True,
            'score': 100.0,
            'ik_success': True,
            'collision_free': True,
            'contact_success': True,
            'lift_success': True,
        },
        {
            'plan_id': 'plan-carton-001',
            'simulation_ok': 1,
            'score': 100.0,
            'ik_success': True,
            'collision_free': True,
            'contact_success': True,
            'lift_success': True,
        },
    ],
    ids=('none', 'incomplete', 'nan-score', 'wrong-plan', 'non-bool'),
)
def test_http_boundary_rejects_unsafe_backend_simulation_result(unsafe_result):
    class UnsafeBackend(PassingBackend):
        def simulate_grasp(self, _payload):
            return unsafe_result

    http_server = server_module.make_server(
        '127.0.0.1',
        0,
        grasp_backend=server_module.MockGraspNetBackend(),
        sim_backend=UnsafeBackend(),
    )
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()
    try:
        body = json.dumps(
            valid_payload(now_sec=server_module.time.time()),
            allow_nan=False,
        ).encode('utf-8')
        raw, response = _raw_post(
            http_server.server_port,
            '/simulate_grasp',
            body,
        )
    finally:
        http_server.shutdown()
        http_server.server_close()
        thread.join(timeout=2.0)

    assert b'NaN' not in raw and b'Infinity' not in raw
    _assert_complete_failure(response, plan_id='plan-carton-001')
    assert response['failure_code'] == 'MUJOCO_INTERNAL_ERROR'


def test_health_with_empty_joint_cache_is_strict_json_null_not_infinity():
    http_server = server_module.make_server(
        '127.0.0.1',
        0,
        grasp_backend=server_module.MockGraspNetBackend(),
        sim_backend=server_module.MockDigitalTwinBackend(),
    )
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()
    try:
        connection = http.client.HTTPConnection(
            '127.0.0.1',
            http_server.server_port,
            timeout=2.0,
        )
        connection.request('GET', '/health')
        http_response = connection.getresponse()
        raw = http_response.read()
        connection.close()
    finally:
        http_server.shutdown()
        http_server.server_close()
        thread.join(timeout=2.0)

    assert b'Infinity' not in raw and b'NaN' not in raw
    response = json.loads(raw.decode('utf-8'))
    assert response['digital_twin']['joint_state_age_sec'] is None


def test_http_backend_exception_is_plan_correlated_and_component_complete():
    class ExplodingBackend(server_module.MockDigitalTwinBackend):
        def simulate_grasp(self, payload):
            raise RuntimeError('synthetic MuJoCo failure')

    http_server = server_module.make_server(
        '127.0.0.1',
        0,
        grasp_backend=server_module.MockGraspNetBackend(),
        sim_backend=ExplodingBackend(),
    )
    thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    thread.start()
    try:
        response = _json_post(
            'http://127.0.0.1:%d/simulate_grasp' % http_server.server_port,
            valid_payload(now_sec=server_module.time.time()),
        )
    finally:
        http_server.shutdown()
        http_server.server_close()
        thread.join(timeout=2.0)

    assert response['plan_id'] == 'plan-carton-001'
    assert response['failure_code'] == 'MUJOCO_INTERNAL_ERROR'
    assert 'synthetic MuJoCo failure' in response['failure_reason']
    assert response['score'] == 0.0
    assert all(response[key] is False for key in (
        'simulation_ok',
        'ik_success',
        'collision_free',
        'contact_success',
        'lift_success',
    ))


@pytest.mark.skipif(
    os.environ.get('MUJOCO_SMOKE') != '1',
    reason='set MUJOCO_SMOKE=1 on the MuJoCo/mesh-equipped WSL host',
)
def test_optional_mujoco_smoke_compiles_real_dynamic_carton_model():
    import mujoco

    now_sec = server_module.time.time()
    payload = server_module._validate_v2_payload(
        valid_payload(now_sec=now_sec),
        now_sec=now_sec,
    )
    backend = server_module.MujocoDigitalTwinBackend(model_xml=MODEL_XML)
    backend._mujoco = mujoco

    model, data, meta = backend._model_for_payload(payload)
    server_module._apply_dynamic_scene_state(
        model,
        data,
        meta,
        server_module._dynamic_scene_state(payload),
    )
    mujoco.mj_forward(model, data)

    assert model.nq > 0
    assert meta['object_body'] >= 0
    assert meta['support_mocap_id'] >= 0
    assert all(math.isfinite(value) for value in data.qpos)


def _json_get(url):
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return json.loads(response.read().decode('utf-8'))


def _json_post(url, payload):
    data = json.dumps(payload).encode('utf-8')
    request = urllib.request.Request(
        url,
        data=data,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        return json.loads(response.read().decode('utf-8'))


def _raw_post(port, path, body, content_length=None):
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=2.0)
    connection.putrequest('POST', path)
    connection.putheader('Content-Type', 'application/json')
    connection.putheader(
        'Content-Length',
        str(len(body)) if content_length is None else str(content_length),
    )
    connection.endheaders()
    connection.send(body)
    response = connection.getresponse()
    raw = response.read()
    connection.close()
    assert response.status == 200
    return raw, json.loads(raw.decode('utf-8'))


def _assert_complete_failure(response, plan_id):
    assert response['plan_id'] == plan_id
    assert math.isfinite(response['score'])
    assert isinstance(response['failure_code'], str) and response['failure_code']
    assert isinstance(response['failure_reason'], str) and response['failure_reason']
    for key in (
        'simulation_ok',
        'ik_success',
        'collision_free',
        'contact_success',
        'lift_success',
    ):
        assert type(response[key]) is bool
        assert response[key] is False
