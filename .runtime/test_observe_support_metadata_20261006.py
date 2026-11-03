"""Pure finite-recorder tests: no ROS initialization, master, device or data."""
import ast
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

PATH = Path(__file__).with_name('observe_support_metadata_20261006.py')
SPEC = importlib.util.spec_from_file_location('support_metadata_recorder', PATH)
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def diagnostic_parameter(**overrides):
    payload=dict(schema=1,capacity=32,geometry_generation=3,diagnostic_only=True,
                 motion_authorized=False,entries=[dict(source_stamp_ns=1791282095700153589,
                     geometry_generation=3,evidence={'available':True})])
    payload.update(overrides)
    return dict(schema=1,encoding='json',payload_json=json.dumps(payload))


class PayloadTests(unittest.TestCase):
    def test_raw_payload_and_large_integer_are_saved_without_relabeling(self):
        raw = ' { "generation" : 20, "source_stamp_ns":1791282095700153589, "text":"电机", "first_loss":null }\n'
        row = dict(original_status_data=raw, annotation=M.status_annotation(raw, 20))
        after = json.loads(M.encode(row))
        self.assertEqual(after['original_status_data'], raw)
        self.assertEqual(json.loads(after['original_status_data'])['source_stamp_ns'], 1791282095700153589)
        self.assertTrue(after['annotation']['generation_matches'])
        self.assertFalse(after['annotation']['first_loss_for_expected_generation'])

    def test_other_generation_loss_cannot_end_expected_capture(self):
        result = M.status_annotation('{"generation":19,"first_loss":{"source_stamp_ns":1}}',20)
        self.assertFalse(result['generation_matches'])
        self.assertFalse(result['first_loss_for_expected_generation'])
        self.assertEqual(result['observed_generation'],19)

    def test_malformed_or_conflicting_first_loss_rejected(self):
        for generation in ('20',True,None,20.0):
            result=M.status_annotation(json.dumps(dict(generation=generation,first_loss={'source_stamp_ns':1})),20)
            self.assertFalse(result['first_loss_for_expected_generation'])
        for loss in ({},True,1,'lost',[],{'generation':19},{'generation':'20'}):
            self.assertFalse(M.status_annotation(json.dumps(dict(generation=20,first_loss=loss)),20)['first_loss_for_expected_generation'])
        for raw in ('not-json','[]','null'):
            self.assertFalse(M.status_annotation(raw,20)['first_loss_for_expected_generation'])

    def test_expected_integer_generation_real_loss_accepted(self):
        self.assertTrue(M.status_annotation('{"generation":20,"first_loss":{"source_stamp_ns":1791282095700153589}}',20)['first_loss_for_expected_generation'])

    def test_original_parameter_envelope_and_payload_retained(self):
        payload=' {"source_stamp_ns":1791282095700153589,"empty":null,"x":"\\n"}\n'
        value=dict(schema='diagnostic-v1',encoding='json',payload_json=payload)
        row=dict(original_parameter_value=value,annotation=M.parameter_annotation(value))
        after=json.loads(M.encode(row))
        self.assertEqual(after['original_parameter_value'],value)
        self.assertEqual(after['original_parameter_value']['payload_json'],payload)
        self.assertTrue(after['annotation']['parse_valid'])
        self.assertEqual(after['annotation']['payload_utf8_sha256'],M.digest(payload.encode()))

    def test_invalid_parameter_preserved_but_not_claimed_valid(self):
        for value in (None,{},dict(encoding='json',payload_json='{'),dict(encoding='xml',payload_json='{}')):
            self.assertFalse(M.parameter_annotation(value)['parse_valid'])

    def test_parseable_values_are_not_a_diagnostic_schema(self):
        for payload in ('null','[]','true','20','{}'):
            result=M.parameter_annotation(dict(schema=1,encoding='json',payload_json=payload))
            self.assertTrue(result['parse_valid'])
            self.assertFalse(result['schema_valid'])
        self.assertTrue(M.parameter_annotation(diagnostic_parameter())['schema_valid'])

    def test_schema_capacity_source_and_generation_are_checked(self):
        for changes in (dict(schema=True),dict(capacity=33),dict(geometry_generation='3'),
                        dict(diagnostic_only=False),dict(motion_authorized=True),
                        dict(entries=[{}]),dict(entries=[dict(source_stamp_ns=True,geometry_generation=3,evidence={})]),
                        dict(entries=[dict(source_stamp_ns=1,geometry_generation=2,evidence={})])):
            result=M.parameter_annotation(diagnostic_parameter(**changes))
            self.assertTrue(result['parse_valid']);self.assertFalse(result['schema_valid'])

    def test_one_valid_snapshot_does_not_hide_later_invalid_snapshot(self):
        capture=M.Capture(Path('/unused'),20,60,None)
        for value in (diagnostic_parameter(),dict(schema=1,encoding='json',payload_json='null')):
            capture.saved(dict(kind='camera_read_diagnostics_parameter',original_parameter_value=value,
                               annotation=M.parameter_annotation(value)))
        self.assertEqual(capture.counts['camera_parameter_read_success'],2)
        self.assertEqual(capture.counts['camera_parameter_schema_valid'],1)
        self.assertEqual(capture.counts['camera_parameter_with_source_entries'],1)
        self.assertEqual(sum(capture.metadata_errors.values()),1)


class QueueTests(unittest.TestCase):
    def test_item_and_byte_bounds_then_recover(self):
        q=M.BoundedInbox(maximum_items=2,maximum_bytes=7)
        self.assertTrue(q.put('a',3));self.assertTrue(q.put('b',4))
        self.assertFalse(q.put('c',1));self.assertEqual(q.pop(),'a')
        self.assertTrue(q.put('c',2));self.assertEqual(q.snapshot()['queued_bytes'],6)
        self.assertEqual(q.pop(),'b');self.assertEqual(q.pop(),'c');self.assertIsNone(q.pop())
        self.assertEqual(q.snapshot()['queued_bytes'],0)

    def test_size_and_close_fail_explicitly(self):
        q=M.BoundedInbox()
        self.assertFalse(q.put('oversize',M.ITEM_LIMIT+1))
        self.assertTrue(q.put('saved',2));q.close()
        self.assertFalse(q.put('late',2));self.assertEqual(q.pop(),'saved')
        self.assertEqual(q.snapshot()['rejected_after_close'],1)

    def test_concurrent_producers_respect_total_bounds(self):
        q=M.BoundedInbox(maximum_items=7,maximum_bytes=30)
        threads=[threading.Thread(target=lambda:[q.put('x',4) for _ in range(30)]) for _ in range(4)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        self.assertEqual(q.snapshot()['queued_items'],7)
        self.assertEqual(q.snapshot()['queued_bytes'],28)
        self.assertEqual(q.snapshot()['queue_bound_dropped'],113)


class SinkTests(unittest.TestCase):
    def test_byte_budget_never_truncates_a_json_record(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'metadata.jsonl';row={'raw':'原文'};size=len(M.encode(row))
            sink=M.JsonlSink(path,size)
            self.assertTrue(sink.append(row));self.assertFalse(sink.append(row));sink.close()
            self.assertEqual(path.read_bytes(),M.encode(row))
            self.assertTrue(sink.limit_reached);self.assertTrue(sink.closed)
            self.assertEqual(sink.hash.hexdigest(),M.digest(path.read_bytes()))

    def test_atomic_summary_has_valid_hashable_json(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'summary.json';value=dict(sealed=True,sha='x')
            self.assertEqual(M.atomic_json(path,value),len(M.encode(value)))
            self.assertEqual(json.loads(path.read_bytes()),value)
            self.assertFalse(path.with_suffix('.json.tmp').exists())


class RpcAndTimingTests(unittest.TestCase):
    def test_exact_parameter_only_and_read_interval_retained(self):
        calls=[];value={'schema':1,'encoding':'json','payload_json':' {"x":1} '}
        def get(caller,key):calls.append((caller,key));return 1,'original message',value
        stamps=iter([10.,10.2]);walls=iter([100,200]);ros=iter([300,400])
        row=M.read_parameter_once(SimpleNamespace(getParam=get),'/reader',lambda:next(ros),lambda:next(stamps),lambda:next(walls))
        self.assertEqual(calls,[('/reader','/camera_read_diagnostics')])
        self.assertEqual(row['original_parameter_value'],value)
        self.assertEqual(row['original_rpc_message'],'original message')
        self.assertEqual(row['rpc_started']['ros_ns'],300)
        self.assertEqual(row['rpc_finished']['ros_ns'],400)

    def test_missing_or_rpc_exception_never_substitutes_old_data(self):
        for get in (lambda *args:(-1,'missing',0),lambda *args:(_ for _ in ()).throw(TimeoutError('timeout'))):
            row=M.read_parameter_once(SimpleNamespace(getParam=get),'/reader',lambda:10)
            self.assertIn('error',row)
            self.assertNotIn('original_parameter_value',row)

    def test_only_persisted_matching_first_loss_sets_two_second_deadline(self):
        capture=M.Capture(Path('/unused'),20,60,None);capture.start=10.
        row=dict(kind='detector_status',original_status_data='{"generation":19,"first_loss":{"source_stamp_ns":1}}',
                 receipt={'monotonic':11.},original_payload_utf8_sha256='original')
        capture.prepare(row);capture.saved(row)
        self.assertIsNone(capture.due(20.))
        row['original_status_data']='{"generation":20,"first_loss":{"source_stamp_ns":2}}'
        capture.prepare(row)
        self.assertIsNone(capture.first_loss)
        capture.saved(row)
        self.assertIsNone(capture.due(12.999))
        self.assertEqual(capture.due(13.),'expected_generation_first_loss_plus_2_seconds')
        later=dict(row,receipt={'monotonic':30.});capture.saved(later)
        self.assertEqual(capture.first_loss['receipt']['monotonic'],11.)

    def test_duration_never_renewed_for_other_generations(self):
        capture=M.Capture(Path('/unused'),20,60,None);capture.start=10.
        self.assertIsNone(capture.due(69.999))
        self.assertEqual(capture.due(70.),'duration_limit')

    def test_startup_first_loss_is_retained_without_ending_new_capture(self):
        capture=M.Capture(Path('/unused'),20,60,None);capture.start=10.
        row=dict(kind='detector_status',original_status_data='{"generation":20,"first_loss":{"source_stamp_ns":1}}',
                 receipt={'monotonic':9.},original_payload_utf8_sha256='original')
        capture.prepare(row);capture.saved(row)
        self.assertEqual(row['annotation']['receipt_phase'],'startup_admitted_before_ready')
        self.assertIsNone(capture.first_loss)

    def test_ready_is_published_only_after_sink_admission_and_worker_ready(self):
        fake=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_nsec=lambda:1)),
            Subscriber=lambda *args,**kwargs:SimpleNamespace(unregister=lambda:None),is_shutdown=lambda:True)
        modules={'std_msgs':SimpleNamespace(),'std_msgs.msg':SimpleNamespace(String=object)}
        with tempfile.TemporaryDirectory() as temp,patch.dict(sys.modules,modules),redirect_stdout(io.StringIO()):
            capture=M.Capture(Path(temp)/'out',20,60,fake)
            def rpc():capture.rpc_initialized.set();capture.stop.wait(2.)
            capture.poll_parameters=rpc
            original=M.atomic_json
            def write(path,value):
                if path.name=='ready.json':
                    self.assertTrue((path.parent/'metadata.jsonl').exists())
                    self.assertTrue(capture.active.is_set());self.assertTrue(capture.rpc_initialized.is_set())
                    self.assertIsNone(capture.start)
                    capture.status_callback(SimpleNamespace(data='{"generation":20}'))
                return original(path,value)
            with patch.object(M,'atomic_json',side_effect=write):
                self.assertEqual(capture.run(),0)
            summary=json.loads((capture.output/'summary.json').read_text())
            self.assertTrue(summary['ready_published']);self.assertTrue(summary['sealed'])
            self.assertEqual(summary['records_by_kind']['detector_status'],1)

    def test_sink_creation_failure_never_publishes_ready_and_records_startup_failure(self):
        modules={'std_msgs':SimpleNamespace(),'std_msgs.msg':SimpleNamespace(String=object)}
        with tempfile.TemporaryDirectory() as temp,patch.dict(sys.modules,modules),redirect_stdout(io.StringIO()):
            capture=M.Capture(Path(temp)/'out',20,60,None)
            with patch.object(M,'JsonlSink',side_effect=OSError('injected sink failure')):
                self.assertEqual(capture.run(),2)
            self.assertFalse((capture.output/'ready.json').exists())
            summary=json.loads((capture.output/'summary.json').read_text())
            self.assertFalse(summary['ready_published']);self.assertFalse(summary['sealed'])
            self.assertIn('injected sink failure',summary['error'])
            self.assertIsNone(summary['capture_start_monotonic'])

    def test_script_has_no_publisher_service_image_or_write_parameter_api(self):
        tree=ast.parse(PATH.read_text())
        calls=[node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)]
        for forbidden in ('Publisher','ServiceProxy','Service','set_param','delete_param','publish'):
            self.assertNotIn(forbidden,calls)
        self.assertNotIn('sensor_msgs',PATH.read_text())
        self.assertNotIn('temperature',PATH.read_text())


if __name__=='__main__':
    unittest.main()
