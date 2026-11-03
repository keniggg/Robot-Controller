"""Focused offline tests for current restart binding and online diagnostics."""
import copy
from collections import deque
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import unittest
from unittest.mock import Mock, patch

BASE=Path(__file__).resolve().parent
def load(name):
    spec=importlib.util.spec_from_file_location(name,BASE/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
session=load('depth_stage_camera_session_20261022')
entry=load('depth_stage_camera_entry_20261022')


class Checks(unittest.TestCase):
    def test_restart_binding_requires_passed_matching_current_contract(self):
        contract_ref=dict(path='contract',sha256='contract-sha')
        accept_ref=dict(path='accept',sha256='accept-sha')
        config_ref=dict(path='config',sha256='config-sha')
        contract=dict(kind='manual_full_stack_cold_start',workspace_root=str(session.W),
            output_root='/frozen/session',configuration=config_ref)
        accepted=dict(passed=True,manual_only=True,contract=contract_ref,selection=dict(mode='unknown'))
        config=dict(nodes={'/supervisor_camera':dict(respawn=False,required=False)},
            parameters={'/gui/joint_direct_mode':True})
        startup=dict(dispatched=True,contract=contract_ref,started_ns=100,
            components={'core':{'unit':'current-core'}},cold={'devices':dict(serial_device='/dev/tty',
                devices={'/dev/tty':1,'/dev/video0':2},camera_usb=[{'serial':'expected'}])})
        refs={'contract':contract,'accept':accepted,'config':config,'/frozen/session/startup.json':startup}
        with patch.object(session,'read_ref',side_effect=lambda r:refs[r['path']]),patch.object(session,'digest',return_value='startup-sha'):
            self.assertEqual(session.restart_binding(contract_ref,accept_ref)['startup_started_ns'],100)
            accepted['passed']=False
            with self.assertRaisesRegex(RuntimeError,'did not pass'):session.restart_binding(contract_ref,accept_ref)
            accepted['passed']=True;accepted['contract']=dict(path='old',sha256='old')
            with self.assertRaisesRegex(RuntimeError,'did not pass'):session.restart_binding(contract_ref,accept_ref)
            accepted['contract']=contract_ref;config['nodes']['/supervisor_camera']['respawn']=True
            with self.assertRaisesRegex(RuntimeError,'respawn'):session.restart_binding(contract_ref,accept_ref)

    def monitor(self,active=False,latching='1'):
        now=10_000_000_000
        stamp=lambda n:NS(to_nsec=lambda:n)
        owners=dict(joint='/alicia_d_driver_node',epoch='/alicia_d_driver_node',actuation='/alicia_d_driver_node',
            manual='/alicia_supervisor_gui',task='/grasp_task_node',mode='/grasp_mode_router',
            arm='/bessica_d_hw_interface',hand='/bessica_d_hw_interface')
        names=['Joint%d'%i for i in range(1,7)]+['right_finger']
        joint=lambda n:NS(header=NS(stamp=stamp(n),frame_id='sdk_measured'),name=names,position=[0.]*7,
            _connection_header={'callerid':owners['joint']})
        values=dict(joint=joint(now-50_000_000),epoch=NS(stamp=stamp(2_000_000_000)),
            actuation=NS(data='CONFIRMED'),manual=NS(data=True),task=NS(active=active),
            mode=NS(data=json.dumps(dict(active=False,start_reserved=False,pending=False))),
            arm=NS(header=NS(stamp=stamp(now)),status_list=[]),hand=NS(header=NS(stamp=stamp(now)),status_list=[]))
        for key,value in values.items():value._connection_header={'callerid':owners[key],'latching':latching}
        monitor=session.Monitor.__new__(session.Monitor)
        monitor.lock=threading.Lock();monitor.values={k:(-100.,v) for k,v in values.items()}
        monitor.joints=deque(joint(now-50_000_000-100_000_000*i) for i in reversed(range(20)))
        monitor.sdk_commands=deque();monitor.rospy=NS(Time=NS(now=lambda:stamp(now)))
        monitor.master=NS(getParam=lambda key:True if key=='/gui/joint_direct_mode' else {'mode':'unknown'})
        return monitor

    def test_old_received_inactive_latch_is_valid_with_fresh_real_feedback(self):
        state=self.monitor().sample()
        self.assertEqual(state['epoch_ns'],2_000_000_000)
        self.assertEqual(state['task_state_contract'],'event_driven_latched_current_publisher')

    def test_active_or_unlatched_task_is_rejected(self):
        for options in (dict(active=True),dict(latching='0')):
            with self.subTest(options=options),self.assertRaisesRegex(RuntimeError,'task active'):
                self.monitor(**options).sample()

    def test_restore_rebinds_current_epoch_without_claiming_diagnostic_pose_preserved(self):
        before=dict(epoch_ns=200,names=['Joint1'],positions=[0.],selection={'mode':'unknown'})
        now=dict(before,epoch_ns=300,positions=[.2])
        plan=dict(before=before,restart_evidence={'startup_started_ns':100})
        monitor=NS(sample=lambda:copy.deepcopy(now))
        with patch.object(session,'preserved'):
            state=session.validate_restored_control(plan,monitor)
        self.assertFalse(state['diagnostic_reference_preserved'])
        self.assertEqual(state['epoch_ns'],300)

    def instrument(self,preset='saved_baseline'):
        class Manager:
            def __init__(self,**kwargs):self.kwargs=kwargs
            def _raw_frame_diagnostics(self,frames):
                return {k:dict(metadata_raw={},unavailable={}) for k in ('color','depth')}
            def read(self):self.read_probe=self.depth_stage_probe;return ('same-color','same-depth')
        plan=dict(diagnostic_trials=[dict(label='B',preset=preset)],sources={})
        module=NS(RealSenseManager=Manager)
        entry.install_instrumentation(module,NS(),plan,NS(trial='B',sha256='plan'),Path('/unused'),[])
        return Manager

    def test_preset_override_is_local_and_queue_backpressure_uses_no_sample_slot(self):
        manager=self.instrument()
        original=dict(enabled=True,unknown_preset='high_accuracy',stationary_temporal_enabled=True)
        owner=manager(mode_depth_preset_cfg=original)
        self.assertEqual(original['unknown_preset'],'high_accuracy')
        self.assertEqual(owner.kwargs['mode_depth_preset_cfg']['unknown_preset'],'saved_baseline')
        probe=NS(queue=NS(full=Mock(side_effect=[True,False])),summary=lambda:{'closed':False})
        owner._finite_stage_probe_considered=True;owner._finite_stage_probe=probe;owner._finite_stage_skipped=0
        self.assertEqual(owner.read(),('same-color','same-depth'));self.assertIsNone(owner.read_probe)
        self.assertEqual(owner.read(),('same-color','same-depth'));self.assertIs(owner.read_probe,probe)
        self.assertEqual(owner._finite_stage_skipped,1)

    def test_unsupported_exposure_is_explicit_without_querying_values(self):
        manager=self.instrument(None);owner=manager()
        owner.rs=NS(frame_metadata_value=NS(actual_exposure=1,gain_level=2,auto_exposure=3))
        frame=NS(supports_frame_metadata=lambda field:False,get_frame_metadata=Mock())
        data=owner._raw_frame_diagnostics(NS(get_color_frame=lambda:frame,get_depth_frame=lambda:frame))
        self.assertTrue(all(set(row['unavailable'].values())=={'not_supported'} for row in data.values()))
        frame.get_frame_metadata.assert_not_called()

    def test_trial_contract_allows_only_the_planned_preset_delta(self):
        original=dict(configuration={'mode_depth_preset':{'unknown_preset':'high_accuracy'}},
            runtime={'mode_depth_preset':{'mode':'unknown','configured_unknown_preset':'high_accuracy',
                'preset':'high_accuracy','baseline_sha256':'same'},'geometry_intrinsics':{'fx':400.}})
        plan=dict(configuration_contract=original,diagnostic_trials=[dict(label='B',preset='saved_baseline')])
        expected=session.trial_configuration(plan,'B')
        self.assertEqual(expected['configuration'],original['configuration'])
        self.assertEqual(expected['runtime']['geometry_intrinsics'],original['runtime']['geometry_intrinsics'])
        self.assertEqual(expected['runtime']['mode_depth_preset']['preset'],'saved_baseline')
        self.assertEqual(session.trial_configuration(plan,'restored'),original)

    def test_interrupted_probe_still_restores_production(self):
        owner=dict(pid=123,start_ticks='1',parent_pid=10)
        plan=dict(interpreter='/fake/python',old_camera=owner,old_camera_uri='known',
            configuration_contract='same',diagnostic_trials=[dict(label='A1',preset='high_accuracy')],preserved={})
        monitor=NS(master=NS(getParam=lambda key:{}));events=[]
        restored=dict(new_camera={'pid':456},state={'diagnostic_reference_preserved':False})
        with patch.object(session.subprocess,'run',return_value=NS(returncode=0,stdout='',stderr='')), \
             patch.object(session,'digest',return_value='sha'),patch.object(session,'save'), \
             patch.object(session,'validate_control',return_value={}), \
             patch.object(session,'configuration_contract',return_value='same'), \
             patch.object(session,'current_camera',return_value=owner), \
             patch.object(session,'require_device_owner'),patch.object(session,'stop_camera'), \
             patch.object(session,'owned_unit_camera',return_value=None),patch.object(session,'stop_probe_unit'), \
             patch.object(session,'alive',return_value=False),patch.object(session,'preserved'), \
             patch.object(session,'require_camera_absent'), \
             patch.object(session,'launch',side_effect=lambda plan,prod,*args:events.append(prod)), \
             patch.object(session,'wait_ready',side_effect=[KeyboardInterrupt('offline interruption'),restored]):
            with self.assertRaisesRegex(RuntimeError,'production camera was restored'):
                session.run_session(plan,monitor)
        self.assertEqual(events,[False,True])


if __name__=='__main__':unittest.main()
