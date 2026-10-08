"""Pure recorder contract tests; never initialize or contact ROS."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
import zipfile
import zlib

path = Path(__file__).with_name('record_support_gap_15s.py')
spec = importlib.util.spec_from_file_location('support_gap_recorder', path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Stamp:
    def __init__(self, ns):
        self.ns = ns

    def to_nsec(self):
        return self.ns


class Message:
    _type, _md5sum = 'sensor_msgs/Image', 'test-original-md5'

    def __init__(self, ns, data=b'raw-pixels', seq=7):
        self.header = SimpleNamespace(stamp=Stamp(ns), seq=seq, frame_id='camera_link')
        self.data = data

    def serialize(self, out):
        out.write(str(self.header.stamp.to_nsec()).encode()+b'\0'+self.data)


class CacheTests(unittest.TestCase):
    def test_only_exact_integer_source_matches(self):
        cache = m.ExactPairCache()
        cache.add('color', Message(10), 100, 1.)
        cache.add('depth', Message(11), 101, 1.1)
        self.assertIsNone(cache.exact(10, 1.2))
        cache.add('depth', Message(10), 102, 1.3)
        color, depth = cache.exact(10, 1.4)
        self.assertEqual((color['receipt_ros_ns'], depth['receipt_ros_ns']), (100, 102))
        self.assertEqual(color['source_seq'], 7)
        self.assertIsNone(cache.exact(11, 1.4))

    def test_two_second_expiry_does_not_relabel(self):
        cache = m.ExactPairCache()
        for name in ('color', 'depth'):
            cache.add(name, Message(10), 100, 1.)
        self.assertIsNotNone(cache.exact(10, 3.))
        self.assertIsNone(cache.exact(10, 3.000001))
        self.assertEqual(cache.bytes, 0)

    def test_memory_cap_eviction_is_explicit(self):
        cache = m.ExactPairCache(maximum_bytes=5)
        cache.add('color', Message(1, b'123'), 1, 1.)
        cache.add('depth', Message(1, b'456'), 1, 1.1)
        self.assertLessEqual(cache.bytes, 5)
        self.assertIsNone(cache.exact(1, 1.2))
        self.assertEqual(cache.counts['memory_bound_evictions'], 1)

    def test_reference_fields_keep_original_origin(self):
        status = dict(source_stamp_ns=20, first_loss=dict(source_stamp_ns=21,
            last_good_source_stamp_ns=18, last_consumed_source_stamp_ns=19))
        self.assertEqual(m.referenced_stamps(status), [('source_stamp_ns', 20),
            ('first_loss.source_stamp_ns', 21), ('first_loss.last_good_source_stamp_ns', 18),
            ('first_loss.last_consumed_source_stamp_ns', 19)])
        self.assertEqual(m.referenced_stamps(dict(source_stamp_ns='20')), [])
        self.assertEqual(m.referenced_stamps(dict(source_stamp_ns=True)), [])


class ArchiveTests(unittest.TestCase):
    def test_pair_admission_atomic_and_priority_reserve(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'out.zip'
            archive = m.BoundedArchive(path, maximum_bytes=5000, ready_bytes=1300, status_reserve=1000)
            pair = [('color', b'a'*100), ('depth', b'b'*100)]
            self.assertTrue(archive.put_many(pair, 'ready'))
            self.assertFalse(archive.put_many([('color2', b'x'*200), ('depth2', b'y'*200)], 'ready'))
            self.assertTrue(archive.put_many([('priority', b'p'*1000)], 'priority'))
            self.assertFalse(archive.put_many([('too_big', b'z'*4000)], 'priority'))
            self.assertTrue(archive.put_many([('status', b's'*500)]))
            archive.close()
            self.assertLess(path.stat().st_size, 5000)
            with zipfile.ZipFile(path) as reader:
                self.assertEqual(reader.namelist(), ['color', 'depth', 'priority', 'status'])


class MetadataTests(unittest.TestCase):
    def test_read_diagnostic_json_envelope_is_saved_without_decoding_or_coercion(self):
        envelope = dict(schema=1, encoding='json',
            payload_json='{"source_stamp_ns":1791284753317145300,"failed_stage":null}')
        def get_param(key):
            return envelope if key == '/camera_read_diagnostics' else {'original':key}
        values, errors, defaults = m.read_metadata_parameters(SimpleNamespace(get_param=get_param))
        self.assertEqual(errors,{})
        self.assertEqual(defaults,{})
        self.assertEqual(values['/camera_read_diagnostics'],envelope)
        self.assertIsInstance(values['/camera_read_diagnostics']['payload_json'],str)
        self.assertEqual(values['/camera'],{'original':'/camera'})

    def test_absent_read_diagnostics_is_explicit_optional_evidence(self):
        def get_param(key):
            if key == '/camera_read_diagnostics':raise KeyError(key)
            return {'original':key}
        values, errors, defaults = m.read_metadata_parameters(SimpleNamespace(get_param=get_param))
        self.assertEqual(errors,{})
        self.assertEqual(values['/camera_read_diagnostics'],{})
        self.assertFalse(defaults['/camera_read_diagnostics']['parameter_present'])
        self.assertIn('no substitution',defaults['/camera_read_diagnostics']['semantics'])
        self.assertEqual(values['/camera'],{'original':'/camera'})

    def test_only_optional_absent_namespace_uses_recorded_producer_default(self):
        def get_param(key):
            if key == '/unknown_perception':
                raise KeyError(key)
            return {'original': key}
        values, errors, defaults = m.read_metadata_parameters(SimpleNamespace(get_param=get_param))
        self.assertEqual(errors, {})
        self.assertEqual(values['/unknown_perception'], {})
        self.assertEqual(defaults['/unknown_perception']['parameter_present'], False)
        self.assertEqual(defaults['/unknown_perception']['effective_default'], {})
        self.assertEqual(values['/camera'], {'original':'/camera'})

    def test_missing_required_or_transport_failure_is_not_defaulted(self):
        for absent, exception in (('/camera', KeyError), ('/unknown_perception', OSError)):
            def get_param(key):
                if key == absent:
                    raise exception('missing or transport error')
                return {'original': key}
            values, errors, defaults = m.read_metadata_parameters(SimpleNamespace(get_param=get_param))
            self.assertIn(absent, errors)
            self.assertNotIn(absent, values)
            self.assertEqual(defaults, {})


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.output = Path(self.temporary.name)
        self.recorder = m.Recorder(self.output, 15., SimpleNamespace(Time=SimpleNamespace(now=lambda:Stamp(1000))))
        self.recorder.ready = {'ready_monotonic': time.monotonic()-1.}
        self.recorder.initial_generation = 16
        self.recorder.initial_selection_ns = 100

    def tearDown(self):
        self.recorder.archive.close()
        self.temporary.cleanup()

    def ready_status(self, **changes):
        value = dict(generation=17, selection_stamp_ns=200, source_stamp_ns=300,
                     state='ready', detail='tracked', support_audit={'original': {'ratio': .74}})
        value.update(changes)
        return json.dumps(value, ensure_ascii=False, separators=(',', ':'))

    def admit(self, raw):
        self.recorder.process_status((raw, 400, time.monotonic()))

    def images(self, stamp=300):
        for stream in ('color', 'depth'):
            self.recorder.pairs.add(stream, Message(stamp, stream.encode()), 380, time.monotonic())

    def test_acquired_requires_new_generation_and_saved_original_pair(self):
        raw = self.ready_status()
        self.admit(raw)
        self.recorder.process_pending()
        self.assertFalse((self.output/'acquired.json').exists())
        self.images()
        self.recorder.process_pending()
        acquired = json.loads((self.output/'acquired.json').read_text())
        self.assertEqual((acquired['generation'], acquired['source_stamp_ns']), (17, 300))
        self.assertEqual(acquired['original_status_utf8_sha256'], m.sha(raw.encode()))
        self.assertFalse(acquired['motion_authorized'])
        self.recorder.archive.close()
        with zipfile.ZipFile(self.output/'diagnostic.zip') as reader:
            status = json.loads(reader.read('status/000000.json'))
            self.assertEqual(status['original_status_data'], raw)
            self.assertEqual(zlib.decompress(reader.read('pairs/300_color.rosmsg.zlib')), b'300\0color')
            self.assertEqual(zlib.decompress(reader.read('pairs/300_depth.rosmsg.zlib')), b'300\0depth')

    def test_old_generation_pending_and_wrong_source_never_acquire(self):
        self.images(299)
        for change in ({'generation':16}, {'state':'support_measurement_pending'}, {'selection_stamp_ns':100}):
            self.admit(self.ready_status(**change))
        self.recorder.process_pending(final=True)
        self.assertFalse((self.output/'acquired.json').exists())
        self.assertEqual(self.recorder.saved_pairs, {})

    def test_absent_source_never_uses_latest_pair(self):
        self.images()
        self.admit(json.dumps({'state':'lost', 'first_loss': {'last_good_source_stamp_ns': 301}}))
        self.recorder.process_pending(final=True)
        self.assertEqual(self.recorder.saved_pairs, {})
        self.assertEqual(self.recorder.counts['association_exact_pair_unavailable_in_original_2_second_cache'], 1)

    def test_elapsed_work_deadline_defers_compression(self):
        self.images()
        self.admit(self.ready_status())
        self.recorder.deadline = time.monotonic()-1.
        self.recorder.process_pending()
        self.assertEqual(self.recorder.saved_pairs, {})
        self.assertEqual(len(self.recorder.pending), 1)

    def test_original_static_partial_messages_both_retained(self):
        one, two = Message(1, b'first-static'), Message(2, b'second-static')
        one._connection_header = two._connection_header = {'callerid':'same-publisher'}
        self.recorder.tf(one, '/tf_static')
        self.recorder.tf(two, '/tf_static')
        self.assertEqual(len(self.recorder.static), 2)
        context = self.recorder.persist_context(self.recorder.context())
        self.assertEqual(len(context['tf_record_keys']), 2)
        entries = len(self.recorder.archive.entries)
        self.recorder.persist_context(self.recorder.context())
        self.assertEqual(len(self.recorder.archive.entries), entries)


if __name__ == '__main__':
    unittest.main()
