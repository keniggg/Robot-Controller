import copy
from types import SimpleNamespace
import numpy as np
import pytest
import evaluate_prospective_metric_candidate as E


def states():
    return {int(1e9+i*1e8):[0.]*6 for i in range(11)}


def test_complete_stationary_window():
    w,error=E.stationary_window(1_500_000_000,states(),[],1)
    assert error is None and w['end_ns']-w['start_ns']>=800_000_000 and w['sample_count']==9


def test_short_window_rejected():
    assert E.stationary_window(1_100_000_000,states(),[],1)[1]=='full_800ms_stationary_window_missing'


def test_gapped_window_rejected():
    q=states()
    for t in [1_300_000_000,1_400_000_000]:del q[t]
    assert E.stationary_window(1_500_000_000,q,[],1)[1]=='accepted_sample_gap'


@pytest.mark.parametrize('when',[1_100_000_000,1_500_000_000,1_900_000_000])
def test_any_command_inside_full_window_rejects(when):
    assert E.stationary_window(1_500_000_000,states(),[when],1)[1]=='window_contains_command'


def test_old_epoch_window_rejected():
    assert E.stationary_window(1_500_000_000,states(),[],1_100_000_000)[1]=='window_before_epoch'


def test_encoder_drift_not_settled_label():
    q=states();q[1_500_000_000][3]=np.radians(.151)
    assert E.stationary_window(1_500_000_000,q,[],1)[1]=='encoder_not_stationary'


def test_nonfinite_encoder_rejected():
    q=states();q[1_500_000_000][0]=float('nan')
    assert E.stationary_window(1_500_000_000,q,[],1)[1]=='invalid_accepted_positions'


def row(dx=0.,joint=0.,session='validation',phase='01_outbound'):
    T=np.eye(4);T[0,3]=dx
    return dict(session=session,phase=phase,q=[joint,0,0,0,0,0],T_base_board={'original':T.tolist(),'candidate':T.tolist()})


def test_baseline_alone_is_not_validation():
    assert E.evaluate([row(),row()],[])['status']=='baseline_only_not_validation'


def test_one_baseline_frame_insufficient():
    assert E.evaluate([row()],[row(),row()])['status']=='baseline_insufficient_multiframe'


def test_same_actual_posture_is_not_multipoise_validation():
    assert E.evaluate([row(),row()],[row(),row()])['status']=='no_distinct_posture_not_validation'


def test_new_posture_needs_multiple_frames():
    assert E.evaluate([row(),row()],[row(joint=.05)])['status']=='validation_phase_insufficient_multiframe'


def test_validation_Y_is_not_refit_to_new_bag():
    baseline=[row(),row()];validation=[row(.006,.05),row(.006,.05)]
    result=E.evaluate(baseline,validation)
    assert result['status']=='multi_posture_prospective_numeric_evaluation'
    for model in result['models'].values():
        m=model['all_validation'];assert m['position_rms_mm']==6. and m['position_max_mm']==6.
        assert not m['historical_3mm_5mm_1deg_2deg_numeric_pass']
        assert not m['local_4mm_4deg_numeric_pass']


def test_one_unobserved_phase_cannot_supply_multiframe():
    result=E.evaluate([row(),row()],[row(joint=.05,phase='a'),row(joint=.05,phase='b')])
    assert result['status']=='validation_phase_insufficient_multiframe'


def quality():
    return dict(pose_ok=True,charuco_count=85,reprojection_rms_px=.1,camera_model_fingerprint=E.CAMERA,
                camera_frame='camera_link',board_frame='charuco_board',board_size=[12,9],dictionary_id='DICT_5X5_100',square_length_m=.015,marker_length_m=.01125)


@pytest.mark.parametrize('key,value',[('camera_model_fingerprint','old'),('reprojection_rms_px',float('nan')),('pose_ok',False),('square_length_m',.020),('camera_model_change_latched',True),('camera_frame','optical')])
def test_quality_preserves_same_source_and_model_requirements(key,value):
    q=quality();assert E.quality_ok(q);q[key]=value;assert not E.quality_ok(q)


def test_rgb_payload_exact_size_and_digest():
    msg=SimpleNamespace(encoding='bgr8',width=640,height=480,step=1920,data=b'x'*(1920*480),header=SimpleNamespace(frame_id='camera_link',stamp=SimpleNamespace(to_nsec=lambda:123)))
    r=E.image_record(msg);assert r['source_stamp_ns']==123 and len(r['raw_sha256'])==64
    msg.data=msg.data[:-1]
    with pytest.raises(ValueError,match='payload'):E.image_record(msg)


def test_unfrozen_candidate_cannot_evaluate():
    with pytest.raises(ValueError,match='frozen model'):E.validate_model_pins({})


def test_first_session_keeps_all_postcommand_validation_without_recentring():
    rows=[row(session='first',phase='baseline'),row(session='first',phase='baseline'),
          row(.006,.05,session='first',phase='01_J2'),row(.006,.05,session='first',phase='01_J2'),
          row(.007,.06,session='first',phase='02_J3'),row(.007,.06,session='first',phase='02_J3')]
    baseline,validation=E.partition_initial_session(rows)
    assert len(baseline)==2 and len(validation)==4
    report=E.evaluate(baseline,validation)
    assert report['status']=='multi_posture_prospective_numeric_evaluation'
    m=report['models']['candidate']
    assert m['fixed_baseline_T_base_board'][0][3]==0
    assert [p['metrics']['position_max_mm'] for p in m['phases']]==[6.,7.]


def test_absent_command_phase_is_explicitly_incomplete():
    report=E.evaluate([row(),row()],[row(joint=.05),row(joint=.05)])
    audits=[dict(session='test',commands=[(2,'J2'),(3,'J3')],selected_per_phase={'01_J2':2})]
    E.add_coverage(report,audits)
    assert report['status']=='partial_command_phase_coverage'
    assert report['coverage']['command_phases'][1]['qualified_frame_count']==0


def test_one_frame_command_phase_is_not_complete():
    report={'status':'multi_posture_prospective_numeric_evaluation'}
    E.add_coverage(report,[dict(session='test',commands=[(2,'J2')],selected_per_phase={'01_J2':1})])
    assert report['status']=='partial_command_phase_coverage'


def test_archive_summary_absent_is_reported_not_invented_failure():
    a=E.archive_status(None)
    assert a['complete'] is None and a['available'] is False


def test_inconsistent_complete_flag_cannot_hide_dropped_archive():
    a=E.archive_status(dict(archive_complete=True,accepted=10,written=9,failed=0,rejected=1,pending=0,inflight=0,retained_failed_payloads=0,failure=None))
    assert a['complete'] is False
    r={'status':'multi_posture_prospective_numeric_evaluation'}
    E.add_coverage(r,[dict(session='s',commands=[],selected_per_phase={},archive_evidence=a)])
    assert r['status']=='incomplete_archive_diagnostic_only'


def test_complete_archive_and_all_command_phases_preserve_result():
    a=E.archive_status(dict(archive_complete=True,accepted=10,written=10,failed=0,rejected=0,pending=0,inflight=0,retained_failed_payloads=0,failure=None))
    assert a['complete'] is True
    r={'status':'multi_posture_prospective_numeric_evaluation'}
    E.add_coverage(r,[dict(session='s',commands=[(2,'J2')],selected_per_phase={'01_J2':3},archive_evidence=a)])
    assert r['status']=='multi_posture_prospective_numeric_evaluation'
