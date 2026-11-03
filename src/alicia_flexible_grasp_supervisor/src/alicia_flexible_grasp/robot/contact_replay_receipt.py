"""Lossless diagnostic inputs; never a plan, permission, or execution cache.

This module imports no ROS or device interfaces. Captured messages are copied
to plain data. Storage is restricted to the selected mounted experiment disk.
No controller, planner, or authority consumer should read these receipts.
"""
from collections.abc import Mapping
import fcntl
import gzip
import hashlib
import io
import json
import math
from numbers import Integral, Real
import os
from pathlib import Path
import tempfile
import threading
from alicia_flexible_grasp.robot.experiment_storage_binding import select_storage


ARM_NAMES = tuple('Joint%d' % i for i in range(1, 7))
_FLAGS = dict(diagnostic_only=True, execution_authorized=False,
              future_following_verified=False, real_grasp_success=False)
_KINDS = ('contact_curve_failure', 'frozen_contact_surface')


class ReplayDataError(ValueError):
    pass


class ReplayStorageUnavailable(OSError):
    pass


def _integer(value, label, minimum=0):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ReplayDataError('%s must be an integer >= %d' % (label, minimum))
    return int(value)


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ReplayDataError('%s must be a finite number' % label)
    return float(value)


def _text(value, label, empty=False):
    if (not isinstance(value, str) or value != value.strip()
            or (not value and not empty) or '\x00' in value):
        raise ReplayDataError('%s must be a canonical string' % label)
    return value


def _digest(value, label):
    if (not isinstance(value, str) or len(value) != 64
            or any(c not in '0123456789abcdef' for c in value)):
        raise ReplayDataError('%s must be a lowercase SHA256' % label)
    return value


def _sequence(value, label):
    if hasattr(value, 'tolist'):
        value = value.tolist()
    if not isinstance(value, (tuple, list)):
        raise ReplayDataError('%s must be an array' % label)
    return value


def _vector(value, count, label, empty=False):
    value = _sequence(value, label)
    if len(value) != count and not (empty and not value):
        raise ReplayDataError('%s must have %d values%s' % (
            label, count, ' or be empty' if empty else ''))
    return [_number(v, label) for v in value]


def _matrix(value, rows, columns, label):
    value = _sequence(value, label)
    if len(value) != rows:
        raise ReplayDataError('%s must have %d rows' % (label, rows))
    return [_vector(v, columns, label) for v in value]


def _points(value, label):
    value = _sequence(value, label)
    if not value:
        raise ReplayDataError('%s must not be empty' % label)
    return [_vector(row, 3, label) for row in value]


def plain_data(value, label='metadata'):
    """Copy JSON-compatible metadata, rejecting nonfinite/opaque objects."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        return _number(value, label)
    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            _text(key, label+' key')
            result[key] = plain_data(item, label+'.'+key)
        return result
    return [plain_data(item, label) for item in _sequence(value, label)]


def _ns(value, label):
    if isinstance(value, Integral):
        return _integer(value, label)
    if hasattr(value, 'to_nsec'):
        return _integer(value.to_nsec(), label)
    seconds = _integer(getattr(value, 'secs', None), label+' seconds')
    nanos = _integer(getattr(value, 'nsecs', None), label+' nanoseconds')
    if nanos >= 1000000000:
        raise ReplayDataError(label+' nanoseconds are not normalized')
    return seconds*1000000000+nanos


def _header(value):
    return dict(seq=_integer(getattr(value, 'seq', None), 'header seq'),
                stamp_ns=_ns(getattr(value, 'stamp', None), 'header stamp'),
                frame_id=_text(getattr(value, 'frame_id', None), 'header frame', True))


def _point(value, *, timed):
    result = {name: _vector(getattr(value, name, ()), 6, name,
                           empty=(timed and name != 'positions'))
              for name in ('positions', 'velocities', 'accelerations')}
    result['effort'] = _vector(getattr(value, 'effort', ()), 6, 'effort', empty=True)
    if timed:
        result['time_from_start_ns'] = _ns(value.time_from_start, 'trajectory time')
    return result


def trajectory_data(value):
    """Copy the actual checked, retimed RobotTrajectory/JointTrajectory.

    Empty velocity/acceleration arrays are retained: their presence determines
    interpolation. Times use integer nanoseconds, including the original t=0.
    """
    if getattr(getattr(value, 'multi_dof_joint_trajectory', None), 'points', ()):
        raise ReplayDataError('multi-DOF contact trajectory is unsupported')
    tr = getattr(value, 'joint_trajectory', value)
    if tuple(tr.joint_names) != ARM_NAMES:
        raise ReplayDataError('trajectory joint names/order are not canonical')
    points = [_point(point, timed=True) for point in tr.points]
    if not 2 <= len(points) <= 4096:
        raise ReplayDataError('trajectory must contain 2..4096 points')
    times = [point['time_from_start_ns'] for point in points]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ReplayDataError('trajectory times must strictly increase')
    wire_sha = None
    if callable(getattr(value, 'serialize', None)):
        stream = io.BytesIO()
        value.serialize(stream)
        wire_sha = hashlib.sha256(stream.getvalue()).hexdigest()
    return dict(header=_header(tr.header), joint_names=list(ARM_NAMES), points=points,
                wire_sha256=wire_sha)


def controller_hold_data(value):
    """Keep all hold derivatives; this function does not certify a hold."""
    result = _point(value, timed=False)
    if hasattr(value, 'time_from_start'):
        result['time_from_start_ns'] = _ns(value.time_from_start, 'controller hold time')
    return result


def _identity(value):
    if isinstance(value, Mapping):
        epoch, track = value.get('epoch'), value.get('track_id')
    else:
        epoch, track = value.epoch, value.track_id
    return dict(epoch=_integer(epoch, 'target epoch'),
                track_id=_text(track, 'target track'))


def _scene(value):
    get = value.get if isinstance(value, Mapping) else lambda key: getattr(value, key)
    return dict(plan_id=_text(get('plan_id'), 'scene plan id'),
                digest=_digest(get('digest'), 'committed plan digest'),
                source_ns=_integer(get('source_ns'), 'scene source', 1),
                track_id=_text(get('track_id'), 'scene target track'),
                geometry_digest=_digest(get('geometry_digest'), 'geometry digest'),
                normal=_vector(get('normal'), 3, 'support normal'),
                offset=_number(get('offset'), 'support offset'),
                center=_vector(get('center'), 3, 'OBB center'),
                rotation=_matrix(get('rotation'), 3, 3, 'OBB rotation'),
                size=_vector(get('size'), 3, 'OBB size'))


def _binding(value):
    result = plain_data(value, 'binding')
    if not isinstance(result, dict):
        raise ReplayDataError('binding must be a mapping')
    for name in ('plan_id', 'target_track_id'):
        _text(result.get(name), 'binding '+name)
    _digest(result.get('plan_sha256'), 'binding plan digest')
    for name in ('source_stamp_ns', 'control_epoch_ns'):
        _integer(result.get(name), 'binding '+name, 1)
    if not isinstance(result.get('selection'), dict) or not result['selection']:
        raise ReplayDataError('binding selection must be present')
    return result


def _first_sample(value):
    result = plain_data(value, 'first failure sample')
    if not isinstance(result, dict) or type(result.get('available')) is not bool:
        raise ReplayDataError('first failure sample needs explicit availability')
    if not result['available']:
        _text(result.get('reason'), 'missing sample reason')
        return result
    for name in ('stage', 'curve', 'label'):
        _text(result.get(name), 'first failure '+name)
    _vector(result.get('joint_positions_rad'), 6, 'failed sample joints')
    _matrix(result.get('T_base_tool0'), 4, 4, 'failed sample FK')
    if 'displacement_bound_m' in result:
        if _number(result['displacement_bound_m'], 'displacement bound') < 0.:
            raise ReplayDataError('displacement bound must be nonnegative')
    return result


def contact_failure_data(*, binding, checked_curve, controller_desired, scene,
                         model_xml, opening_m, opening_source, following_sample,
                         config, first_failed_sample, failure, plan_data=None,
                         other_curves=None):
    """Build a diagnostic from already captured inputs, without live lookups."""
    if not isinstance(model_xml, str) or not model_xml:
        raise ReplayDataError('exact model XML is required')
    model_sha = hashlib.sha256(model_xml.encode('utf-8')).hexdigest()
    result = dict(schema_version=1, kind='contact_curve_failure', **_FLAGS)
    result.update(binding=_binding(binding), checked_curve=trajectory_data(checked_curve),
        controller_desired=controller_hold_data(controller_desired), scene=_scene(scene),
        model=dict(xml=model_xml, sha256=model_sha), opening_m=_number(opening_m, 'opening'),
        opening_source=_text(opening_source, 'opening source'),
        following_sample=plain_data(following_sample, 'following'),
        config=plain_data(config, 'config'), first_failed_sample=_first_sample(first_failed_sample),
        failure=plain_data(failure, 'failure'), plan_data=plain_data(plan_data, 'plan data'),
        other_curves={_text(name, 'curve name'): trajectory_data(curve)
                      for name, curve in (other_curves or {}).items()})
    validate_data(result)
    return result


def surface_data(surface, reference, *, snapshot_stamp_ns, coordinate_frame,
                 snapshot_to_reference, request_binding=None):
    """Freeze the actual fused object AND its exact bound reference view.

    Reference points are retained too, so restoring a SurfaceView requires no
    invented placeholder points or inverse-transform approximation.
    """
    identity = _identity(surface.identity)
    if identity != _identity(reference.identity):
        raise ReplayDataError('surface/reference target identity mismatch')
    points = _points(surface.points_base, 'surface points')
    indices = [_integer(v, 'view index') for v in surface.view_indices]
    stamps = [_integer(v, 'view stamp', 1) for v in surface.view_stamps_ns]
    sources = [[_integer(v, 'voxel source') for v in row] for row in surface.view_sources]
    result = dict(schema_version=1, kind='frozen_contact_surface', **_FLAGS)
    result.update(identity=identity, points_base=points, view_indices=indices,
        view_stamps_ns=stamps, view_sources=sources,
        reference=dict(identity=identity.copy(), stamp_ns=_integer(reference.stamp_ns, 'reference stamp', 1),
            points_base=_points(reference.points_base, 'reference points'),
            support_normal_base=_vector(reference.support_normal_base, 3, 'reference normal'),
            support_offset_m=_number(reference.support_offset_m, 'reference offset'),
            edge_clearance_px=_integer(reference.edge_clearance_px, 'reference edge clearance')),
        snapshot_stamp_ns=_integer(snapshot_stamp_ns, 'snapshot source', 1),
        coordinate_frame=_text(coordinate_frame, 'surface coordinate frame'),
        snapshot_to_reference=_matrix(snapshot_to_reference, 4, 4, 'registration transform'),
        request_binding=plain_data(request_binding, 'request binding'))
    validate_data(result)
    return result


def _validate_point_data(point, timed):
    if not isinstance(point, dict):
        raise ReplayDataError('trajectory point must be a mapping')
    for name in ('positions', 'velocities', 'accelerations'):
        _vector(point.get(name), 6, name, empty=timed and name != 'positions')
    _vector(point.get('effort'), 6, 'effort', empty=True)
    if timed:
        _integer(point.get('time_from_start_ns'), 'trajectory time')


def _validate_curve_data(curve):
    if not isinstance(curve, dict) or curve.get('joint_names') != list(ARM_NAMES):
        raise ReplayDataError('stored curve has noncanonical joint names')
    header = curve.get('header')
    if not isinstance(header, dict):
        raise ReplayDataError('stored curve header is missing')
    _integer(header.get('seq'), 'header seq')
    _integer(header.get('stamp_ns'), 'header stamp')
    _text(header.get('frame_id'), 'header frame', True)
    if curve.get('wire_sha256') is not None:
        _digest(curve['wire_sha256'], 'trajectory wire SHA')
    points = _sequence(curve.get('points'), 'trajectory points')
    if not 2 <= len(points) <= 4096:
        raise ReplayDataError('stored curve needs 2..4096 points')
    for point in points:
        _validate_point_data(point, True)
    if any(b['time_from_start_ns'] <= a['time_from_start_ns']
           for a, b in zip(points, points[1:])):
        raise ReplayDataError('stored trajectory times must strictly increase')


def validate_data(value):
    """Validate structure on write AND read. Nothing here grants authority."""
    plain_data(value)
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
            or value.get('schema_version') != 1):
        raise ReplayDataError('unsupported replay schema')
    if value.get('kind') not in _KINDS:
        raise ReplayDataError('unsupported replay kind')
    if any(value.get(key) is not expected for key, expected in _FLAGS.items()):
        raise ReplayDataError('replay data must remain explicitly diagnostic only')
    if value['kind'] == 'contact_curve_failure':
        binding = _binding(value.get('binding'))
        _validate_curve_data(value.get('checked_curve'))
        _validate_point_data(value.get('controller_desired'), False)
        if 'time_from_start_ns' in value['controller_desired']:
            _integer(value['controller_desired']['time_from_start_ns'], 'controller hold time')
        scene = _scene(value.get('scene'))
        if (scene['plan_id'] != binding['plan_id'] or scene['digest'] != binding['plan_sha256']
                or scene['source_ns'] != binding['source_stamp_ns']
                or scene['track_id'] != binding['target_track_id']):
            raise ReplayDataError('frozen scene does not match plan binding')
        model = value.get('model')
        if (not isinstance(model, dict) or not isinstance(model.get('xml'), str)
                or not model['xml'] or _digest(model.get('sha256'), 'model SHA')
                != hashlib.sha256(model['xml'].encode('utf-8')).hexdigest()):
            raise ReplayDataError('model bytes/hash mismatch')
        following = value.get('following_sample')
        if not isinstance(following, dict) or following.get('model_sha256') != model['sha256']:
            raise ReplayDataError('following sample model mismatch')
        if tuple(following.get('joint_names', ())) != ARM_NAMES:
            raise ReplayDataError('following joint names are not canonical')
        for name in ('sdk_positions_rad', 'accepted_positions_rad'):
            _vector(following.get(name), 6, name)
        for name in ('sdk_stamp_ns', 'accepted_stamp_ns'):
            _integer(following.get(name), name, 1)
        if not 0. <= _number(value.get('opening_m'), 'opening') <= .05:
            raise ReplayDataError('diagnostic opening is outside physical range')
        _text(value.get('opening_source'), 'opening source')
        if not isinstance(value.get('config'), dict):
            raise ReplayDataError('frozen config must be a mapping')
        failure = value.get('failure')
        if not isinstance(failure, dict):
            raise ReplayDataError('original failure must be a mapping')
        for name in ('code', 'message', 'stage'):
            _text(failure.get(name), 'original failure '+name)
        _first_sample(value.get('first_failed_sample'))
        for name, curve in value.get('other_curves', {}).items():
            _text(name, 'other curve name')
            _validate_curve_data(curve)
    else:
        identity = _identity(value.get('identity'))
        points = _points(value.get('points_base'), 'surface points')
        indices = [_integer(v, 'view index') for v in _sequence(value.get('view_indices'), 'view indices')]
        stamps = [_integer(v, 'view stamp', 1) for v in _sequence(value.get('view_stamps_ns'), 'view stamps')]
        sources = _sequence(value.get('view_sources'), 'view sources')
        if (len(indices) != len(points) or len(sources) != len(points) or not stamps
                or any(b <= a for a, b in zip(stamps, stamps[1:]))):
            raise ReplayDataError('surface provenance lengths/times mismatch')
        for index, row in zip(indices, sources):
            row = [_integer(v, 'voxel source') for v in _sequence(row, 'voxel sources')]
            if (not row or index >= len(stamps) or index not in row
                    or row[-1] >= len(stamps) or any(b <= a for a, b in zip(row, row[1:]))):
                raise ReplayDataError('surface voxel provenance mismatch')
        ref = value.get('reference')
        if not isinstance(ref, dict) or _identity(ref.get('identity')) != identity:
            raise ReplayDataError('surface/reference identity mismatch')
        _integer(ref.get('stamp_ns'), 'reference stamp', 1)
        _points(ref.get('points_base'), 'reference points')
        _vector(ref.get('support_normal_base'), 3, 'reference normal')
        _number(ref.get('support_offset_m'), 'reference offset')
        _integer(ref.get('edge_clearance_px'), 'edge clearance')
        _integer(value.get('snapshot_stamp_ns'), 'snapshot source', 1)
        _text(value.get('coordinate_frame'), 'surface coordinate frame')
        _matrix(value.get('snapshot_to_reference'), 4, 4, 'registration transform')
    return value


def canonical_bytes(data):
    data = plain_data(data)
    validate_data(data)
    return json.dumps(data, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False).encode('utf-8')


class DiagnosticStore:
    """Immutable gzip objects, named by the SHA256 of canonical plain bytes."""
    def __init__(self, root=None):
        self.root = Path(root).absolute() if root is not None else None
        self._storage = None
        self._storage_binding = None
        self._storage_lock = threading.RLock()

    def _ready(self):
        with self._storage_lock:
            try:
                selected = self._storage or select_storage(path=self.root)
                root = self.root if self.root is not None else selected.archive_root/'replay_objects'
                binding = selected.validate(root, expected=self._storage_binding)
            except (OSError, ValueError, RuntimeError) as exc:
                raise ReplayStorageUnavailable(str(exc)) from exc
            if root == selected.base:
                raise ReplayStorageUnavailable('replay store must have its own archive subdirectory')
            self.root, self._storage, self._storage_binding = root, selected, binding
            return root

    @staticmethod
    def _relative(digest):
        return Path('sha256')/digest[:2]/(digest+'.json.gz')

    def put(self, data):
        # Invalid payloads create neither a directory, lock, nor partial file.
        raw = canonical_bytes(data)
        digest = hashlib.sha256(raw).hexdigest()
        packed = gzip.compress(raw, compresslevel=9, mtime=0)
        root = self._ready()
        relative = self._relative(digest)
        destination = root/relative
        if destination.resolve() != destination:
            raise ReplayStorageUnavailable('replay object path must not follow symlinks')
        destination.parent.mkdir(parents=True, exist_ok=True)
        self._ready()
        # Advisory locks work across threads/processes; publication is rename
        # atomic. An existing corrupt object is rejected, never overwritten.
        lock_path = destination.with_suffix(destination.suffix+'.lock')
        with lock_path.open('a+b') as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            temporary = None
            try:
                if destination.exists():
                    existing = destination.read_bytes()
                    try:
                        recovered = gzip.decompress(existing)
                    except (OSError, EOFError) as exc:
                        raise ReplayDataError('existing gzip object is corrupt') from exc
                    if recovered != raw:
                        raise ReplayDataError('existing content-addressed object differs')
                    # The logical content address is independent of gzip/zlib
                    # versions. Keep the original verified compressed bytes.
                    packed = existing
                else:
                    fd, temporary = tempfile.mkstemp(prefix='.'+digest+'-', dir=destination.parent)
                    with os.fdopen(fd, 'wb') as stream:
                        stream.write(packed)
                        stream.flush()
                        os.fsync(stream.fileno())
                    self._ready()
                    os.replace(temporary, destination)
                    temporary = None
                    directory = os.open(str(destination.parent), os.O_RDONLY|os.O_DIRECTORY)
                    try:
                        os.fsync(directory)
                    finally:
                        os.close(directory)
                if destination.read_bytes() != packed:
                    raise ReplayDataError('replay object readback mismatch')
            finally:
                if temporary is not None:
                    os.unlink(temporary)
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        return dict(schema_version=1, kind=data['kind'], diagnostic_only=True,
                    execution_authorized=False, store_root=str(root),
                    relative_path=str(relative), object_sha256=digest,
                    gzip_sha256=hashlib.sha256(packed).hexdigest(),
                    uncompressed_bytes=len(raw), gzip_bytes=len(packed), codec='gzip-json-v1')

    def get(self, reference):
        root = self._ready()
        digest = _digest(reference.get('object_sha256'), 'object SHA')
        relative = self._relative(digest)
        if (reference.get('store_root') != str(root)
                or reference.get('relative_path') != str(relative)
                or reference.get('diagnostic_only') is not True
                or reference.get('execution_authorized') is not False
                or reference.get('codec') != 'gzip-json-v1'):
            raise ReplayDataError('invalid diagnostic object reference')
        path = (root/relative).resolve()
        if path != root/relative:
            raise ReplayDataError('replay object path must not follow symlinks')
        packed = path.read_bytes()
        if (len(packed) != reference.get('gzip_bytes')
                or hashlib.sha256(packed).hexdigest() != reference.get('gzip_sha256')):
            raise ReplayDataError('gzip object bytes/hash mismatch')
        raw = gzip.decompress(packed)
        if (len(raw) != reference.get('uncompressed_bytes')
                or hashlib.sha256(raw).hexdigest() != digest):
            raise ReplayDataError('uncompressed replay bytes/hash mismatch')
        data = json.loads(raw.decode('utf-8'))
        if canonical_bytes(data) != raw or data['kind'] != reference.get('kind'):
            raise ReplayDataError('noncanonical or mismatched replay object')
        self._ready()
        return data


def best_effort_failure_receipt(original_error, build_data, store):
    """Call in an existing except block, then re-raise its ORIGINAL exception.

    Ordinary diagnostic/storage errors are returned as unavailable metadata.
    BaseException (shutdown, interruption, strict offline boundary guard) is
    deliberately not swallowed. The original error is never replaced/mutated.
    """
    try:
        reference = store.put(build_data())
        return dict(status='stored', reference=reference, diagnostic_only=True,
                    execution_authorized=False)
    except Exception as exc:
        return dict(status='unavailable', diagnostic_only=True, execution_authorized=False,
                    original_error_type=type(original_error).__name__,
                    original_error_message=str(original_error),
                    diagnostic_error_type=type(exc).__name__, diagnostic_error_message=str(exc))
