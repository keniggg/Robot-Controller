"""Sufficient separation proofs for the validated Alicia finger CAD union.

This is not a replacement collision detector. A positive support-projection
gap proves separation; failure to find one leaves the original conservative
verdict intact. The complete active and reference meshes are both enclosed,
with the existing 0.6 mm allowance on each tool axis on each side.
"""
from collections import OrderedDict
from dataclasses import dataclass
from functools import lru_cache
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import struct
from threading import RLock
from weakref import WeakValueDictionary
import xml.etree.ElementTree as ET

import numpy as np


_ROOT = Path(__file__).resolve().parents[5]
_ACTIVE_URDF = 'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf'
_REFERENCE_URDF = 'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf'
_ASSETS = (
    (_ACTIVE_URDF, '890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7'),
    (_REFERENCE_URDF, '7e08a01f1a6cf3b45ed78a26d3a9f5939e2bb6bc33f6bc01032db097882edaeb'),
    ('src/real-arm/alicia_d_descriptions/meshes/Link7.STL', 'ff006151147d25c431d868fc6a0f46b96275690d30526d13c741ffe51ac6fcea'),
    ('src/real-arm/alicia_d_descriptions/meshes/Link8.STL', 'fd49b1a4c9ecf39854d0de1ee3645239aae2427cca6ce7a24b195bf041c6098e'),
    ('src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link7.STL', 'd546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d'),
    ('src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link8.STL', '5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b'),
)
_BODY_NAMES = ('left_finger', 'right_finger')
_PADDING_M = 0.0006
_MAX_OPENING_M = 0.05
_PROOF_EPSILON_M = 1e-9
_VERIFIED_MODELS = WeakValueDictionary()
_FULL_AXIS_CACHE_MAXSIZE = 16
_FULL_AXIS_CACHE = OrderedDict()
_FULL_AXIS_CACHE_LOCK = RLock()


def _hash(value):
    return hashlib.sha256(value).hexdigest()


def _frozen(value):
    array = np.asarray(value, dtype='<f8')
    return np.frombuffer(array.tobytes(), dtype='<f8').reshape(array.shape)


def _vector(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError('invalid '+name)
    return array


def _rotation(value, name):
    rotation = _vector(value, (3, 3), name)
    if (np.max(np.abs(rotation.T @ rotation-np.eye(3))) > 1e-8
            or abs(float(np.linalg.det(rotation))-1.) > 1e-8):
        raise ValueError('invalid '+name)
    return rotation


def _opening(value):
    number = float(value)
    if not math.isfinite(number) or not 0. <= number <= _MAX_OPENING_M:
        raise ValueError('opening outside validated joint mappings')
    return number


def _parse_xml(xml):
    if not isinstance(xml, str) or not 0 < len(xml) <= 1000000:
        raise ValueError('invalid robot description')
    if '<!DOCTYPE' in xml.upper() or '<!ENTITY' in xml.upper():
        raise ValueError('robot description entities are unsupported')
    root = ET.fromstring(xml)
    if root.tag != 'robot':
        raise ValueError('robot root required')
    for tag in ('joint', 'link'):
        names = [node.get('name') for node in root.findall(tag)]
        if None in names or len(names) != len(set(names)):
            raise ValueError('ambiguous '+tag+' identity')
    return root


def _canonical(node):
    return (node.tag, tuple(sorted(node.attrib.items())),
            tuple(_canonical(child) for child in node))


def _finger_contract(root):
    joints = ('Joint6_to_gripper_base', 'Grasp2tool', 'left_finger', 'right_finger')
    links = ('Grasp_base', 'tool0', 'Link7', 'Link8')
    signature = []
    for tag, names in (('joint', joints), ('link', links)):
        for name in names:
            found = root.findall("./%s[@name='%s']" % (tag, name))
            if len(found) != 1:
                raise ValueError('missing or duplicate finger contract element')
            signature.append(_canonical(found[0]))
    # Reject extra parents or attachments on the fixed tool/finger subtree.
    # Unrelated upstream arm changes remain the FK owner's responsibility.
    connected = []
    for joint in root.findall('joint'):
        parent, child = joint.find('parent'), joint.find('child')
        if parent is None or child is None:
            raise ValueError('incomplete robot joint')
        if parent.get('link') in links or child.get('link') in links:
            connected.append(joint.get('name'))
    if set(connected) != set(joints) or len(connected) != len(joints):
        raise ValueError('extra finger/tool attachment')
    return tuple(signature)


@lru_cache(maxsize=1)
def _validated_assets():
    assets = {}
    for relative, expected in _ASSETS:
        data = (_ROOT/relative).read_bytes()
        if _hash(data) != expected:
            raise ValueError('CAD asset identity mismatch: '+relative)
        assets[relative] = data
    return assets


def _stl_vertices(data):
    if len(data) < 84:
        raise ValueError('truncated CAD mesh')
    count = struct.unpack_from('<I', data, 80)[0]
    if count <= 0 or len(data) != 84+50*count:
        raise ValueError('invalid CAD mesh length')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                      ('attribute', '<u2')])
    vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    if not np.all(np.isfinite(vertices)):
        raise ValueError('non-finite CAD vertex')
    # Exact duplicate removal only; every triangle vertex is represented.
    # The support of these vertices equals the support of their full hull.
    return np.unique(vertices.astype(float), axis=0)


def _xyz(element, key):
    if element is None:
        raise ValueError('missing CAD origin/axis')
    return _vector([float(x) for x in element.attrib[key].split()], (3,), key)


def _rpy(element):
    roll, pitch, yaw = _xyz(element, 'rpy')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


@dataclass(frozen=True)
class _Component:
    vertices_at_zero: np.ndarray
    velocity_per_opening: np.ndarray
    lower_at_zero: np.ndarray
    upper_at_zero: np.ndarray


def _component(robot, urdf_path, joint_name, q_zero, q_slope, assets):
    joint = robot.find("./joint[@name='%s']" % joint_name)
    link = robot.find("./link[@name='%s']" % joint.find('child').get('link'))
    collisions = link.findall('collision')
    if len(collisions) != 1:
        raise ValueError('unrecognized complete collision subtree')
    collision = collisions[0]
    mesh = collision.find('geometry/mesh')
    if mesh is None or len(collision.find('geometry')) != 1:
        raise ValueError('unrecognized collision mesh')
    filename = mesh.attrib['filename']
    if filename.startswith('package://alicia_d_descriptions/'):
        relative = 'src/real-arm/alicia_d_descriptions/'+filename.split('package://alicia_d_descriptions/', 1)[1]
    else:
        relative = str(((_ROOT/urdf_path).parent/filename).resolve().relative_to(_ROOT))
    if relative not in assets:
        raise ValueError('unvalidated CAD mesh reference')
    vertices = _stl_vertices(assets[relative])
    scale = _vector([float(x) for x in mesh.get('scale', '1 1 1').split()], (3,), 'mesh scale')
    if not np.array_equal(scale, np.ones(3)):
        raise ValueError('unrecognized mesh scale')
    jr, cr = _rpy(joint.find('origin')), _rpy(collision.find('origin'))
    jo, co = _xyz(joint.find('origin'), 'xyz'), _xyz(collision.find('origin'), 'xyz')
    axis = _xyz(joint.find('axis'), 'xyz')
    tool = robot.find("./joint[@name='Grasp2tool']/origin")
    tr, to = _rpy(tool), _xyz(tool, 'xyz')
    p = ((vertices*scale @ cr.T+co+q_zero*axis) @ jr.T+jo-to) @ tr
    velocity = tr.T @ jr @ (q_slope*axis)
    return _Component(_frozen(p), _frozen(velocity), _frozen(p.min(axis=0)), _frozen(p.max(axis=0)))


def _unit_axes(values):
    """Normalize without angular quantization; exact duplicate removal only."""
    values = np.asarray(values, dtype=float).reshape(-1, 3)
    if not np.all(np.isfinite(values)):
        raise ValueError('non-finite CAD candidate axis')
    lengths = np.linalg.norm(values, axis=1)
    values = values[lengths > 1e-12]/lengths[lengths > 1e-12, None]
    if not len(values):
        return np.empty((0, 3))
    leading = values[np.arange(len(values)), np.argmax(np.abs(values), axis=1)]
    values[leading < 0.] *= -1.
    values[values == 0.] = 0.
    return np.unique(values, axis=0)


def _full_axis_features(model, body, opening):
    """Bounded immutable shape cache, with no pose, target or verdict caching."""
    if not validated_model_matches(model, model._robot_description_sha256):
        raise ValueError('unverified CAD model cannot supply hull features')
    key = (model._robot_description_sha256, body, opening, _PADDING_M)
    with _FULL_AXIS_CACHE_LOCK:
        if key in _FULL_AXIS_CACHE:
            value = _FULL_AXIS_CACHE.pop(key)
            _FULL_AXIS_CACHE[key] = value
            return value
    # Optional and lazy: the original positive fifteen-axis path needs no Qhull.
    from scipy.spatial import ConvexHull
    parts = model._body_components(body)
    vertices = np.unique(np.concatenate([
        part.vertices_at_zero+opening*part.velocity_per_opening
        for part in parts]), axis=0)
    hull = ConvexHull(vertices)
    corners = _PADDING_M*np.asarray(list(product((-1., 1.), repeat=3)))
    padded = np.unique((vertices[:, None, :]+corners[None, :, :]).reshape(-1, 3), axis=0)
    padded_hull = ConvexHull(padded)
    pairs = np.concatenate((padded_hull.simplices[:, [0, 1]],
                            padded_hull.simplices[:, [1, 2]],
                            padded_hull.simplices[:, [0, 2]]))
    edges = padded[pairs[:, 1]]-padded[pairs[:, 0]]
    # A subset of original vertices may screen proposed axes. It can never
    # supply a positive proof: that always reprojects all original components.
    value = (_frozen(vertices[hull.vertices]),
             _frozen(_unit_axes(padded_hull.equations[:, :3])),
             _frozen(_unit_axes(edges)))
    with _FULL_AXIS_CACHE_LOCK:
        _FULL_AXIS_CACHE[key] = value
        _FULL_AXIS_CACHE.move_to_end(key)
        while len(_FULL_AXIS_CACHE) > _FULL_AXIS_CACHE_MAXSIZE:
            _FULL_AXIS_CACHE.popitem(last=False)
    return value


def _full_axis_separation(model, body, opening, transform, center, target_r,
                          size, displacement, minimum_roundoff):
    """Propose full padded-hull axes, then prove using every original vertex.

    The fixed-padding hull is only an axis generator. Any additional interval
    displacement is included in every support projection, with no cached
    displacement or verdict and no promise to resolve every inconclusive case.
    """
    screen, face_axes, edge_axes = _full_axis_features(model, body, opening)
    rotation = transform[:3, :3]
    axes = _unit_axes(np.concatenate((
        face_axes @ rotation.T, target_r.T,
        np.cross((edge_axes @ rotation.T)[:, None, :],
                 target_r.T[None, :, :]).reshape(-1, 3))))
    if not len(axes):
        return None
    tool_axes = axes @ rotation
    radius = (_PADDING_M+displacement)*np.abs(tool_axes).sum(axis=1)
    target_mid = axes @ (center-transform[:3, 3])
    target_radius = np.abs(axes @ target_r) @ (size/2.)
    target_lower, target_upper = target_mid-target_radius, target_mid+target_radius
    proposed_gap = np.empty(len(axes))
    for start in range(0, len(axes), 128):
        stop = min(start+128, len(axes))
        values = screen @ tool_axes[start:stop].T
        lower, upper = values.min(axis=0)-radius[start:stop], values.max(axis=0)+radius[start:stop]
        proposed_gap[start:stop] = np.maximum(target_lower[start:stop]-upper,
                                             lower-target_upper[start:stop])
    if not np.all(np.isfinite(proposed_gap)):
        raise ValueError('non-finite hull axis screening')
    candidates = np.flatnonzero(proposed_gap > _PROOF_EPSILON_M)
    candidates = candidates[np.argsort(proposed_gap[candidates])[::-1]]
    parts = model._body_components(body)
    for index in candidates:
        axis = tool_axes[index]
        lower, upper = float('inf'), -float('inf')
        for part in parts:
            projected = part.vertices_at_zero @ axis
            shift = opening*float(axis @ part.velocity_per_opening)
            lower = min(lower, float(projected.min())+shift)
            upper = max(upper, float(projected.max())+shift)
        lower, upper = lower-float(radius[index]), upper+float(radius[index])
        raw_gap = max(float(target_lower[index])-upper, lower-float(target_upper[index]))
        # Exactly the original roundoff formula over the old axes plus this
        # fully reprojected axis; never reduce the original numerical reserve.
        roundoff = max(minimum_roundoff, float(64.*np.finfo(float).eps*(1.+float(max(
            np.max(np.abs(center)), np.max(np.abs(transform[:3, 3])),
            np.max(size), abs(lower), abs(upper))))))
        gap = max(0., raw_gap-roundoff)
        if math.isfinite(gap) and gap > _PROOF_EPSILON_M:
            return dict(separated=True, gap_m=gap, status='separated',
                axis_count=len(axes), axis_base=axes[index].tolist(),
                projection_origin_base=transform[:3, 3].tolist(),
                finger_projection_m=[lower, upper],
                target_projection_m=[float(target_lower[index]), float(target_upper[index])],
                numerical_gap_reserve_m=roundoff,
                axis_policy='padded_hull_axes_full_original_vertex_recheck',
                full_axis_candidate_count=len(axes))
    return None


@dataclass(frozen=True)
class FingerCadModel:
    """Immutable source-validated support model; no execution authority."""
    _components: tuple
    _provenance_json: str
    _robot_description_sha256: str = ''

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        # Every component array is backed by immutable bytes. FK snapshots
        # must retain this verified object, not writable numpy deep copies.
        memo[id(self)] = self
        return self

    @property
    def provenance(self):
        return json.loads(self._provenance_json)

    def _body_components(self, body):
        if body not in _BODY_NAMES:
            raise ValueError('body is not a validated finger')
        return self._components[_BODY_NAMES.index(body)]

    def box(self, body, opening, original_center_tool, original_size):
        """Enclose the old padded box AND the padded active/reference union.

        The original box is already padded. Padding is added only to CAD.
        Invalid input raises; callers must retain their fail-closed behavior.
        """
        opening = _opening(opening)
        center = _vector(original_center_tool, (3,), 'original center')
        size = _vector(original_size, (3,), 'original size')
        if np.any(size <= 0.):
            raise ValueError('original box size must be positive')
        lower, upper = center-size/2., center+size/2.
        for part in self._body_components(body):
            shift = opening*part.velocity_per_opening
            lower = np.minimum(lower, part.lower_at_zero+shift-_PADDING_M)
            upper = np.maximum(upper, part.upper_at_zero+shift+_PADDING_M)
        return _frozen((lower+upper)/2.), _frozen(upper-lower)

    def separation(self, body, opening, T_base_tool, target_center,
                   target_rotation, target_size, displacement_m=0.):
        """Try the original 15 axes, then a lazy full-hull axis fallback.

        Projection includes every vertex of both CAD meshes for this body.
        Tool-axis L-infinity displacement enlarges the original per-axis
        padding; no Euclidean-only substitute or target shrink is used.
        A missing axis proof is UNKNOWN, not proof of a physical collision.
        """
        result = dict(separated=False, gap_m=0., scope='sufficient_CAD_union_separation',
                      execution_authorized=False, provenance=self.provenance)
        try:
            parts = self._body_components(body)
            opening = _opening(opening)
            transform = _vector(T_base_tool, (4, 4), 'tool transform')
            if np.max(np.abs(transform[3]-[0., 0., 0., 1.])) > 1e-12:
                raise ValueError('invalid homogeneous tool transform')
            rotation = _rotation(transform[:3, :3], 'tool rotation')
            center = _vector(target_center, (3,), 'target center')
            target_r = _rotation(target_rotation, 'target rotation')
            size = _vector(target_size, (3,), 'target size')
            displacement = float(displacement_m)
            if np.any(size <= 0.) or not math.isfinite(displacement) or displacement < 0.:
                raise ValueError('invalid target size or displacement')
            axes = np.concatenate((rotation.T, target_r.T,
                np.cross(rotation.T[:, None, :], target_r.T[None, :, :]).reshape(-1, 3)))
            norms = np.linalg.norm(axes, axis=1)
            axes = axes[norms > 1e-12]/norms[norms > 1e-12, None]
            tool_axes = axes @ rotation
            lower = np.full(len(axes), np.inf)
            upper = np.full(len(axes), -np.inf)
            for part in parts:
                projected = part.vertices_at_zero @ tool_axes.T
                shift = opening*(tool_axes @ part.velocity_per_opening)
                lower = np.minimum(lower, projected.min(axis=0)+shift)
                upper = np.maximum(upper, projected.max(axis=0)+shift)
            radius = (_PADDING_M+displacement)*np.abs(tool_axes).sum(axis=1)
            lower, upper = lower-radius, upper+radius
            target_mid = axes @ (center-transform[:3, 3])
            target_radius = np.abs(axes @ target_r) @ (size/2.)
            target_lower, target_upper = target_mid-target_radius, target_mid+target_radius
            gaps = np.maximum(target_lower-upper, lower-target_upper)
            if not np.all(np.isfinite(gaps)):
                raise ValueError('non-finite support projection')
            index = int(np.argmax(gaps))
            # A roundoff allowance only reduces a claimed gap. Scaling by the
            # coordinates also protects the relative-center subtraction.
            roundoff = float(64.*np.finfo(float).eps*(1.+float(max(np.max(np.abs(center)),
                np.max(np.abs(transform[:3, 3])), np.max(size), np.max(np.abs(lower)), np.max(np.abs(upper))))))
            gap = max(0., float(gaps[index])-roundoff)
            result.update(separated=gap > _PROOF_EPSILON_M, gap_m=gap,
                status='separated' if gap > _PROOF_EPSILON_M else 'no_sufficient_axis',
                body=body, opening_m=opening, axis_count=len(axes),
                axis_base=axes[index].tolist(), projection_origin_base=transform[:3, 3].tolist(),
                finger_projection_m=[float(lower[index]), float(upper[index])],
                target_projection_m=[float(target_lower[index]), float(target_upper[index])],
                padding_each_tool_axis_m=_PADDING_M, displacement_each_tool_axis_m=displacement,
                numerical_gap_reserve_m=roundoff)
            if not result['separated']:
                try:
                    extra = _full_axis_separation(self, body, opening, transform,
                        center, target_r, size, displacement, roundoff)
                    if extra is not None:
                        result.update(extra)
                        result['fast_axis_count'] = len(axes)
                except (ImportError, ValueError, TypeError, OverflowError,
                        RuntimeError, MemoryError, np.linalg.LinAlgError) as exc:
                    # Missing optional hull support or any inconclusive input
                    # retains the original conservative result.
                    result['full_axis_fallback_unavailable'] = type(exc).__name__
        except (ValueError, TypeError, KeyError, OverflowError, IndexError, np.linalg.LinAlgError) as exc:
            result.update(status='invalid_input', reason=str(exc))
        return result


@lru_cache(maxsize=4)
def _load_frozen_model(xml):
    try:
        active = _parse_xml(xml)
        assets = _validated_assets()
        expected = _parse_xml(assets[_ACTIVE_URDF].decode('utf-8'))
        contract = _finger_contract(active)
        if contract != _finger_contract(expected):
            return None
        reference = _parse_xml(assets[_REFERENCE_URDF].decode('utf-8'))
        # Active public travel is used unchanged. Reference joints encode
        # half-travel from a 50 mm opening and have reversed body names.
        components = (
            (_component(active, _ACTIVE_URDF, 'left_finger', 0., -1., assets),
             _component(reference, _REFERENCE_URDF, 'right_finger', .025, -.5, assets)),
            (_component(active, _ACTIVE_URDF, 'right_finger', 0., 1., assets),
             _component(reference, _REFERENCE_URDF, 'left_finger', -.025, .5, assets)),
        )
        provenance = dict(schema=1, policy='complete_active_reference_finger_mesh_union',
            robot_description_sha256=_hash(xml.encode('utf-8')),
            collision_subtree_sha256=_hash(repr(contract).encode('utf-8')),
            assets=dict(_ASSETS), complete_mesh_vertices_used=True,
            vertex_reduction='exact_duplicate_removal_only',
            support_equals_complete_convex_hull=True, fixed_padding_each_tool_axis_m=_PADDING_M,
            active_opening_mapping='public_value_unchanged; left_mimic_minus_one',
            reference_opening_mapping='left_q=-(.05-opening)/2; right_q=(.05-opening)/2; reversed_body_names',
            component_vertex_counts=[[len(p.vertices_at_zero) for p in body] for body in components],
            component_zero_vertex_sha256=[[_hash(p.vertices_at_zero.tobytes()) for p in body] for body in components],
            execution_authorized=False)
        provenance['model_sha256'] = _hash(json.dumps(provenance, sort_keys=True).encode('utf-8'))
        model = FingerCadModel(components, json.dumps(provenance, sort_keys=True),
                               provenance['robot_description_sha256'])
        _VERIFIED_MODELS[id(model)] = model
        return model
    except (OSError, ValueError, TypeError, KeyError, OverflowError, AttributeError, ET.ParseError):
        return None


def load_finger_cad_model(xml):
    """Return a frozen validated model, or None for the original-box fallback.

    Cache size is bounded. Models retain immutable verified mesh bytes through
    their vertices; cached geometry never follows later filesystem changes.
    """
    if not isinstance(xml, str) or not 0 < len(xml) <= 1000000:
        return None
    return _load_frozen_model(xml)


def validated_model_matches(model, robot_description_sha256):
    """Accept only a loader-verified immutable model for this exact FK source.

    A manually constructed, copied-with-replaced-fields, or subclassed model
    cannot turn an old-box rejection into a separation proof. Weak references
    do not extend model lifetimes beyond the bounded loader cache/FK owners.
    """
    return (type(model) is FingerCadModel
            and _VERIFIED_MODELS.get(id(model)) is model
            and isinstance(robot_description_sha256, str)
            and model._robot_description_sha256 == robot_description_sha256)
