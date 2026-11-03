"""Exercise the real wrapper with fake ROS transport and child processes."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

from test_confirmed_unknown_dispatch import fixture, measured_report

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('confirmed_wrapper_under_test',
    str(ROOT/'.runtime/run_confirmed_unknown_grasp_20261004.py'))
wrapper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wrapper)


@pytest.mark.parametrize('entry', ['metric','rgb','lost','enable_pending','selection_race'])
def test_real_wrapper_dispatches_only_serial_bound_children(monkeypatch, tmp_path, entry):
    import rospy
    decision, status, parameters = fixture(metric=entry == 'metric')
    selected = decision.selection
    receipt = dict(kind='explicit_user_target_alignment', target_aligned=True,
        confirmation_id='actual-user-receipt', confirmation_source={'kind':'user_message'})
    transition = tmp_path/'selection.json'
    transition.write_text(json.dumps(dict(preview_only=False, response={'success':True},
        alignment_receipt=receipt, new_selection=selected)))
    mount = tmp_path/'SanStand'
    out = mount/'experiment'/'record'
    out.mkdir(parents=True)
    class MountedPath(type(mount)):
        def is_mount(self): return True
    def path(value):
        return MountedPath(mount) if str(value) == '/media/zhuyupei/SanStand' else Path(value)
    monkeypatch.setattr(wrapper, 'Path', path)
    monkeypatch.setenv('GRASP_RECORD_DIR', str(out))
    monkeypatch.setattr(sys, 'argv', ['confirmed','--selection-result',str(transition)])
    monkeypatch.setattr(rospy, 'init_node', lambda *a, **k:None)
    monkeypatch.setattr(rospy, 'sleep', lambda *_:None)
    monkeypatch.setattr(rospy, 'is_shutdown', lambda:False)
    monkeypatch.setattr(rospy.Time, 'now', lambda:rospy.Time(10,100000000))
    monkeypatch.setattr(wrapper.time, 'monotonic', lambda:1.)
    params = {'/grasp_mode/selection':selected}
    monkeypatch.setattr(rospy, 'get_param', lambda key, default=None:params.get(key, default))
    monkeypatch.setattr(rospy, 'get_param_cached', lambda key, default=None:params.get(key, default))
    monkeypatch.setattr(rospy, 'set_param', lambda key, value:params.__setitem__(key,value))
    monkeypatch.setattr(wrapper, 'context_parameters', lambda _:parameters)
    timeline, callbacks = [], {}
    def subscribe(topic, kind, callback, **kwargs):
        timeline.append(('subscribe',topic))
        callbacks[topic] = callback
        if topic.endswith('detector_status'):
            if entry == 'lost': status['target_lost'] = True
            callback(NS(data=json.dumps(status)))
        else:
            callback(NS(frame_id='sdk_reference_epoch', stamp=NS(to_nsec=lambda:decision.epoch_ns)))
        return NS(unregister=lambda:timeline.append(('unsubscribe',topic)))
    monkeypatch.setattr(rospy, 'Subscriber', subscribe)
    monkeypatch.setattr(rospy, 'Publisher', lambda *a, **k:NS(
        publish=lambda _:timeline.append(('ownership',))))
    def child(command, **kwargs):
        # No blocking RPC can be timed out or sent concurrently by this wrapper.
        assert 'timeout' not in kwargs
        assert json.loads(command[command.index('--expected-selection-json')+1]) == selected
        if command[0] == 'rosrun':
            timeline.append(('metric',))
            return 0
        timeline.append(('rgb',))
        report = measured_report(decision)
        if entry == 'enable_pending':
            report['measured_target_available'] = False
            report['observation_completed'] = False
            report['requests'][0].update(success=False,
                message='RGB_ACQUISITION_REJECTED: MOTION_ENABLE_CONFIRMATION_REQUIRED: PENDING')
        session = mount/'实验记录/Robot-Controller/ros_log/rgb_acquisition_run_fixture'
        session.mkdir(parents=True)
        (session/'result.json').write_text(json.dumps(report))
        kwargs['stdout'].write('SESSION '+str(session)+'\n')
        callbacks['/perception/unknown/detector_status'](NS(data=json.dumps(decision.status)))
        if entry == 'selection_race': params['/grasp_mode/selection'] = dict(selected, generation=12)
        timeline.append(('rgb_returned',))
        return 0 if entry != 'enable_pending' else 2
    monkeypatch.setattr(wrapper.subprocess, 'call', child)
    code = wrapper.main()
    phases = [row[0] for row in timeline]
    assert phases[:2] == ['subscribe','subscribe']
    assert phases.index('ownership') > 1
    launched = [name for name in phases if name in ('rgb','rgb_returned','metric')]
    if entry == 'metric':
        assert code == 0 and launched == ['metric']
    elif entry == 'rgb':
        assert code == 0 and launched == ['rgb','rgb_returned','metric']
    else:
        assert code == 2
        assert launched == ([] if entry == 'lost' else ['rgb','rgb_returned'])
    recorded = json.loads((out/'dispatch.json').read_text())
    if entry == 'enable_pending':
        assert 'MOTION_ENABLE_CONFIRMATION_REQUIRED' in json.dumps(recorded)
    assert json.loads((out/'authorization.json').read_text())['budget_policy'].startswith('RGB acquisition')
