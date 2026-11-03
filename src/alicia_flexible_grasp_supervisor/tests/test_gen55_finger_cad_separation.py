"""Independent frozen CAD support evidence; no ROS, devices or motion I/O."""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
import hashlib
from itertools import product
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from alicia_flexible_grasp.robot import finger_cad_separation as cad
from alicia_flexible_grasp.robot import observation_path_guard as guard
from alicia_flexible_grasp.grasp import gripper_geometry as gg
from types import SimpleNamespace as NS


ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).parent/'fixtures/gen55_finger_cad_actual_receipts.json'
DATA = json.loads(FIXTURE.read_text())
XML = (ROOT/DATA['xml_relative_path']).read_text()
assert hashlib.sha256(XML.encode()).hexdigest() == DATA['xml_sha256']


@pytest.fixture
def model():
    value = cad.load_finger_cad_model(XML)
    assert isinstance(value, cad.FingerCadModel)
    return value


def original_box(body, opening):
    sign = 1. if body == 'left_finger' else -1.
    return (np.array([.0004, .0003+sign*(opening+.0286)/2., -.0302]),
            np.array([.0434, .0286, .0600])+.0012)


def oracle_gap(body, transform, scene, displacement=0.):
    """Independent padded convex hull was built and all-source-vertex checked.

    This oracle projects its explicit padded vertices in the world frame;
    production instead projects unpadded complete meshes in relative axes.
    """
    T = np.asarray(transform)
    R = np.asarray(scene['rotation'])
    vertices = np.asarray(DATA['hulls'][body]) @ T[:3, :3].T+T[:3, 3]
    axes = list(T[:3, :3].T)+list(R.T)
    axes += [np.cross(u, v) for u in T[:3, :3].T for v in R.T]
    values = []
    for axis in axes:
        length = np.linalg.norm(axis)
        if length <= 1e-12:
            continue
        axis = axis/length
        projected = vertices @ axis
        extra = displacement*np.abs(T[:3, :3].T @ axis).sum()
        center = np.dot(scene['center'], axis)
        radius = np.abs(R.T @ axis) @ (np.asarray(scene['size'])/2.)
        values.append(max(center-radius-projected.max()-extra,
                          projected.min()-extra-center-radius))
    return max(values)


@pytest.mark.parametrize('index', range(8))
@pytest.mark.parametrize('reference', ['nominal_T', 'command_T'])
def test_original_eight_receipts_match_independent_complete_padded_hull(model, index, reference):
    case = DATA['cases'][index]
    before = deepcopy(case)
    for body in ('left_finger', 'right_finger'):
        scene = case['scene']
        proof = model.separation(body, case['actual_opening_m'], case[reference],
            scene['center'], scene['rotation'], scene['size'])
        gap = oracle_gap(body, case[reference], scene)
        assert proof['separated'] is bool(gap > 1e-9)
        assert proof['gap_m'] == pytest.approx(max(0., gap), abs=1e-12)
        assert proof['padding_each_tool_axis_m'] == .0006
        assert proof['execution_authorized'] is False
    assert case == before


@pytest.mark.parametrize('index', [2, 4, 7])
def test_actual_padded_false_positive_has_positive_finite_cad_support_gap(model, index):
    case = DATA['cases'][index]
    s = case['scene']
    proof = model.separation('right_finger', case['actual_opening_m'], case['command_T'],
        s['center'], s['rotation'], s['size'])
    # Fifteen sufficient axes may give a smaller gap than full hull SAT.
    assert proof['separated'] and proof['gap_m'] > 1e-9


@pytest.mark.parametrize('index', [0, 1, 3, 5, 6])
def test_original_remaining_command_collision_does_not_acquire_separation(model, index):
    case = DATA['cases'][index]
    s = case['scene']
    proof = model.separation('right_finger', case['actual_opening_m'], case['command_T'],
        s['center'], s['rotation'], s['size'])
    assert proof['separated'] is False
    assert proof['status'] == 'no_sufficient_axis'


@pytest.mark.parametrize('displacement', [0., .0001, .0006, .001, .01])
def test_interval_displacement_is_added_on_each_tool_axis_and_never_shrinks_padding(model, displacement):
    case = DATA['cases'][4];s = case['scene'];T = case['command_T']
    proof = model.separation('right_finger', DATA['opening_m'], T,
        s['center'], s['rotation'], s['size'], displacement_m=displacement)
    gap = oracle_gap('right_finger', T, s, displacement)
    assert proof['separated'] is bool(gap > 1e-9)
    assert proof['gap_m'] == pytest.approx(max(0., gap), abs=1e-12)
    assert proof['gap_m'] <= oracle_gap('right_finger', T, s)+1e-12


@pytest.mark.parametrize('opening', [0., .0001, .023, .0498, .05])
@pytest.mark.parametrize('body', ['left_finger', 'right_finger'])
def test_broadphase_contains_old_box_and_both_complete_meshes_with_padding(model, body, opening):
    center, size = original_box(body, opening)
    c, s = model.box(body, opening, center, size)
    assert np.all(c-s/2. <= center-size/2.+1e-15)
    assert np.all(c+s/2. >= center+size/2.-1e-15)
    for part in model._body_components(body):
        vertices = part.vertices_at_zero+opening*part.velocity_per_opening
        for corner in product((-.0006, .0006), repeat=3):
            assert np.all(vertices+corner >= c-s/2.-1e-15)
            assert np.all(vertices+corner <= c+s/2.+1e-15)


def test_active_public_opening_is_not_halved_and_old_box_cannot_clear_it(model):
    opening = .0498;body = 'right_finger'
    active, reference = model._body_components(body)
    active_points = active.vertices_at_zero+opening*active.velocity_per_opening
    reference_points = reference.vertices_at_zero+opening*reference.velocity_per_opening
    assert active_points[:, 1].min() == pytest.approx(-.0780541186163675, abs=1e-14)
    assert reference_points[:, 1].min() == pytest.approx(-.05306884182494747, abs=1e-14)
    target = active_points[int(np.argmin(active_points[:, 1]))]
    center, size = original_box(body, opening)
    assert target[1] < center[1]-size[1]/2.-.01
    c, s = model.box(body, opening, center, size)
    assert np.all(np.abs(target-c) < s/2.)
    proof = model.separation(body, opening, np.eye(4), target, np.eye(3), [.0004]*3)
    assert proof['separated'] is False


def test_model_is_frozen_and_provenance_cannot_change_cached_identity(model):
    assert cad.load_finger_cad_model(XML) is model
    with pytest.raises(FrozenInstanceError):
        model._components = ()
    for part in model._body_components('left_finger'):
        with pytest.raises(ValueError):
            part.vertices_at_zero.setflags(write=True)
        with pytest.raises(ValueError):
            part.velocity_per_opening[0] = 1.
    provenance = model.provenance
    provenance['assets'].clear()
    assert len(model.provenance['assets']) == 6


@pytest.mark.parametrize('path,attribute,value', [
    ("./joint[@name='Grasp2tool']/origin", 'xyz', '0 0 0'),
    ("./joint[@name='Joint6_to_gripper_base']/origin", 'rpy', '0 .01 0'),
    ("./joint[@name='left_finger']/origin", 'rpy', '-1.57079632679 0 0'),
    ("./joint[@name='left_finger']/axis", 'xyz', '0 0 1'),
    ("./joint[@name='left_finger']/mimic", 'multiplier', '-.5'),
    ("./joint[@name='left_finger']/mimic", 'offset', '.001'),
    ("./joint[@name='right_finger']/limit", 'upper', '.025'),
    ("./joint[@name='right_finger']/parent", 'link', 'Link6'),
    ("./link[@name='Link7']/collision/origin", 'xyz', '.001 0 0'),
    ("./link[@name='Link8']/collision/geometry/mesh", 'scale', '.99 1 1'),
    ("./link[@name='Link8']/collision/geometry/mesh", 'filename', 'other.STL'),
])
def test_changed_finger_or_fixed_tool_contract_falls_back_to_original_box(path, attribute, value):
    robot = ET.fromstring(XML)
    robot.find(path).set(attribute, value)
    assert cad.load_finger_cad_model(ET.tostring(robot, encoding='unicode')) is None


@pytest.mark.parametrize('mutation', ['extra_collision', 'missing_collision', 'duplicate_joint', 'extra_attachment'])
def test_complete_subtree_not_just_one_matching_mesh_is_required(mutation):
    robot = ET.fromstring(XML)
    link = robot.find("./link[@name='Link7']")
    if mutation == 'extra_collision':
        link.append(deepcopy(link.find('collision')))
    elif mutation == 'missing_collision':
        link.remove(link.find('collision'))
    elif mutation == 'duplicate_joint':
        robot.append(deepcopy(robot.find("./joint[@name='left_finger']")))
    else:
        robot.append(ET.fromstring('<joint name="extra" type="fixed"><parent link="Link7"/><child link="added"/></joint>'))
    assert cad.load_finger_cad_model(ET.tostring(robot, encoding='unicode')) is None


@pytest.mark.parametrize('xml', ['', 'not xml', '<robot><joint/></robot>', '<!DOCTYPE robot><robot/>', '<robot/>', None])
def test_missing_or_invalid_models_cannot_supply_a_safe_result(xml):
    assert cad.load_finger_cad_model(xml) is None


@pytest.mark.parametrize('asset_index', range(6))
def test_mesh_identity_mismatch_fails_closed_before_building_model(tmp_path, monkeypatch, asset_index):
    for rel, _digest in cad._ASSETS:
        path = tmp_path/rel
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/rel, path)
    path = tmp_path/cad._ASSETS[asset_index][0]
    data = bytearray(path.read_bytes());data[-1] ^= 1;path.write_bytes(data)
    cad._validated_assets.cache_clear();cad._load_frozen_model.cache_clear()
    monkeypatch.setattr(cad, '_ROOT', tmp_path)
    try:
        assert cad.load_finger_cad_model(XML) is None
    finally:
        cad._validated_assets.cache_clear();cad._load_frozen_model.cache_clear()


def test_upstream_fk_change_is_not_confused_with_a_changed_finger_shape():
    for index in range(8):
        robot = ET.fromstring(XML)
        robot.find("./joint[@name='Joint1']/origin").set('xyz', '0 0 %s' % (.01*index))
        model = cad.load_finger_cad_model(ET.tostring(robot, encoding='unicode'))
        assert model is not None
    assert cad._load_frozen_model.cache_info().currsize <= 4


@pytest.mark.parametrize('mutation', ['opening_nan', 'opening_high', 'opening_negative', 'target_nan',
    'target_size_zero', 'target_rotation_reflection', 'transform_nan', 'transform_last_row',
    'displacement_negative', 'displacement_nan', 'body'])
def test_invalid_inputs_never_become_a_separation_proof(model, mutation):
    args = dict(body='left_finger', opening=.0498, T_base_tool=np.eye(4), target_center=np.array([1., 0., 0.]),
                target_rotation=np.eye(3), target_size=np.array([.02]*3), displacement_m=0.)
    if mutation == 'opening_nan': args['opening'] = float('nan')
    elif mutation == 'opening_high': args['opening'] = .050001
    elif mutation == 'opening_negative': args['opening'] = -.001
    elif mutation == 'target_nan': args['target_center'][0] = float('nan')
    elif mutation == 'target_size_zero': args['target_size'][0] = 0.
    elif mutation == 'target_rotation_reflection': args['target_rotation'][0, 0] = -1.
    elif mutation == 'transform_nan': args['T_base_tool'][0, 0] = float('nan')
    elif mutation == 'transform_last_row': args['T_base_tool'][3, 0] = .1
    elif mutation == 'displacement_negative': args['displacement_m'] = -.1
    elif mutation == 'displacement_nan': args['displacement_m'] = float('nan')
    elif mutation == 'body': args['body'] = 'palm'
    result = model.separation(**args)
    assert result['separated'] is False and result['gap_m'] == 0.
    assert result['status'] == 'invalid_input'


def test_box_invalid_inputs_raise_instead_of_returning_a_clear_envelope(model):
    center, size = original_box('left_finger', .05)
    with pytest.raises(ValueError): model.box('left_finger', float('nan'), center, size)
    with pytest.raises(ValueError): model.box('palm', .05, center, size)
    with pytest.raises(ValueError): model.box('left_finger', .05, center, [0., 1., 1.])


def test_far_target_has_a_positive_proof_without_claiming_full_path_or_execution(model):
    result = model.separation('left_finger', .05, np.eye(4), [1., 0., 0.], np.eye(3), [.02]*3)
    assert result['separated'] and result['gap_m'] > .9
    assert result['execution_authorized'] is False
    assert result['scope'] == 'sufficient_CAD_union_separation'


def guard_false_positive_inputs():
    case = DATA['cases'][4]
    fk = guard.SerialUrdfFk(XML, ['Joint%d' % i for i in range(1, 7)])
    g = gg.GripperGeometry(.05, .002, np.array([.0434, .0286, .0600]),
                           np.array([.1175, .155, .0774]), .003)
    scene = NS(**{k: np.asarray(v) for k, v in case['scene'].items()})
    transform = np.asarray(case['command_T'])
    body, center, size = next(row for row in gg._stage_boxes(transform, g,
        case['actual_opening_m'], 'y', 'z') if row[0] == 'right_finger')
    assert gg._obb_overlap(center, transform[:3, :3], size,
                           scene.center, scene.rotation, scene.size)
    return [fk, g, case['actual_opening_m'], body, transform, center, size, scene]


def test_fk_snapshot_retains_verified_immutable_model_and_source_binding():
    args = guard_false_positive_inputs()
    fk = args[0]
    copied_fk = deepcopy(fk)
    assert copied_fk.finger_cad_model is fk.finger_cad_model
    assert cad.validated_model_matches(copied_fk.finger_cad_model, copied_fk.model_sha256)
    args[0] = copied_fk
    overlap, proof = guard.gripper_target_overlap(*args)
    assert overlap is False and proof['separated'] is True


@pytest.mark.parametrize('mutation', ['foreign_source', 'missing_source', 'constructed_model',
    'replaced_components', 'subclass', 'wrong_base', 'wrong_tool', 'unsupported_gripper'])
def test_guard_never_uses_unverified_or_inconsistent_model_to_clear_old_collision(mutation):
    args = guard_false_positive_inputs();fk = args[0];model = fk.finger_cad_model
    if mutation == 'foreign_source':
        robot = ET.fromstring(XML)
        robot.find("./joint[@name='Joint1']/origin").set('xyz', '0 0 .01')
        fk.finger_cad_model = cad.load_finger_cad_model(ET.tostring(robot, encoding='unicode'))
        assert fk.finger_cad_model is not None
    elif mutation == 'missing_source': fk.model_sha256 = None
    elif mutation == 'constructed_model':
        fk.finger_cad_model = cad.FingerCadModel(model._components,
            model._provenance_json, model._robot_description_sha256)
    elif mutation == 'replaced_components':
        fk.finger_cad_model = replace(model, _components=((), ()))
    elif mutation == 'subclass':
        class OtherModel(cad.FingerCadModel):
            pass
        fk.finger_cad_model = OtherModel(model._components,
            model._provenance_json, model._robot_description_sha256)
    elif mutation == 'wrong_base': fk.base_frame = 'Link1'
    elif mutation == 'wrong_tool': fk.tool_frame = 'Link6'
    elif mutation == 'unsupported_gripper':
        args[1] = replace(args[1], finger_size_xyz_m=np.array([.0434, .0287, .0600]))
    overlap, proof = guard.gripper_target_overlap(*args)
    assert overlap is True and proof is None


@pytest.mark.parametrize('base,tool,names', [
    ('base_link', 'Link6', ['Joint%d' % i for i in range(1, 7)]),
    ('Link1', 'tool0', ['Joint%d' % i for i in range(2, 7)]),
])
def test_noncanonical_fk_frames_do_not_load_tool0_cad(base, tool, names):
    fk = guard.SerialUrdfFk(XML, names, base=base, tool=tool)
    assert fk.finger_cad_model is None


@pytest.mark.parametrize('mutation', ['missing_fields', 'gap_nan', 'gap_boolean',
    'status', 'scope', 'execution', 'body', 'opening', 'padding', 'displacement', 'source'])
def test_malformed_cad_proof_preserves_rejection(monkeypatch, mutation):
    args = guard_false_positive_inputs();model = args[0].finger_cad_model
    overlap, proof = guard.gripper_target_overlap(*args)
    assert overlap is False
    if mutation == 'missing_fields': proof = dict(separated=True, gap_m=1.)
    elif mutation == 'gap_nan': proof['gap_m'] = float('nan')
    elif mutation == 'gap_boolean': proof['gap_m'] = True
    elif mutation == 'status': proof['status'] = 'invalid_input'
    elif mutation == 'scope': proof['scope'] = 'endpoint_only'
    elif mutation == 'execution': proof['execution_authorized'] = True
    elif mutation == 'body': proof['body'] = 'left_finger'
    elif mutation == 'opening': proof['opening_m'] = .05
    elif mutation == 'padding': proof['padding_each_tool_axis_m'] = 0.
    elif mutation == 'displacement': proof['displacement_each_tool_axis_m'] = .001
    elif mutation == 'source': proof['provenance']['robot_description_sha256'] = 'other'
    monkeypatch.setattr(cad.FingerCadModel, 'separation', lambda *a, **kw: proof)
    assert guard.gripper_target_overlap(*args)[0] is True
