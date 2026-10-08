import importlib.util
from pathlib import Path
import types

spec=importlib.util.spec_from_file_location('first_loss_collector',Path(__file__).with_name('collect_unknown_first_loss.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_status_after_images_archives_unavailable_source():
    join=module.ExactDetectorSources(100)
    assert not join.image('rgb',200,'rgb',210)
    assert not join.image('depth',200,'depth',220)
    rows=join.status(dict(source_stamp_ns=200,state='unavailable',detail='target_lost'),230)
    assert len(rows)==1 and rows[0][1:]==('rgb','depth')
    assert rows[0][0]['detail']=='target_lost'
    assert join.report()['paired_count']==1


def test_status_before_images_waits_for_exact_pair_and_deduplicates():
    join=module.ExactDetectorSources(100)
    assert not join.status(dict(source_stamp_ns=200),210)
    assert not join.image('rgb',200,'rgb',220)
    assert not join.image('depth',201,'wrong',230)
    assert len(join.image('depth',200,'depth',240))==1
    assert not join.status(dict(source_stamp_ns=200),250)


def test_evicted_requested_source_is_explicitly_missing():
    join=module.ExactDetectorSources(100,image_limit=1)
    join.status(dict(source_stamp_ns=200),210)
    join.image('rgb',200,'old',210)
    join.image('rgb',201,'new',220)
    join.image('depth',200,'depth',230)
    result=join.report()
    assert result['missing_sources'][0]['source_stamp_ns']==200
    assert result['paired_count']==0 and result['image_evictions']['rgb']==1


def test_source_less_timeout_and_pre_start_status_do_not_invent_pair():
    join=module.ExactDetectorSources(100)
    join.status(dict(detail='exact_rgbd_stream_stale'),150)
    join.status(dict(source_stamp_ns=90),150)
    result=join.report()
    assert result['no_source_status_count']==1 and result['historical_source_status_count']==1
    assert result['requested_count']==0


def test_request_bound_is_not_silent_drop():
    import pytest
    join=module.ExactDetectorSources(100,request_limit=1)
    join.status(dict(source_stamp_ns=200),210)
    with pytest.raises(RuntimeError,match='request limit'):
        join.status(dict(source_stamp_ns=300),310)


def test_collector_has_no_publisher_or_ros_service_call():
    import ast
    tree=ast.parse(Path(module.__file__).read_text())
    names=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert not set(names)&{'Publisher','ServiceProxy','publish'}


def stamp(ns):
    return types.SimpleNamespace(to_nsec=lambda: ns)


def image(ns, width=4, height=3, encoding='rgb8', step=None):
    step = step or width*(2 if encoding == '16UC1' else 3)
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(ns), frame_id='camera_link'),
        width=width, height=height, encoding=encoding, step=step, is_bigendian=0, data=bytes(height*step))


def pair(join, ns):
    join.image('rgb', ns, image(ns), ns+10)
    join.image('depth', ns, image(ns, encoding='16UC1'), ns+20)


def object_message(ns, detected):
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(ns)), detected=detected)


def feedback(ns, label='accepted'):
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(ns), frame_id='sdk_measured'),
        name=list(module.JOINT_NAMES), position=[0.]*6, label=label)


def pair_join(image_limit=48):
    join = module.FirstLossPairSources(100, image_limit=image_limit)
    join.current_context('/alicia_d/control_reference_epoch', types.SimpleNamespace(stamp=stamp(50)), 110)
    join.current_context('/grasp_mode/selection', types.SimpleNamespace(data='selection'), 110)
    return join


def ready(join, ns=200, generation=9, selection=150):
    join.object(ns, object_message(ns, True), ns+25)
    payload = dict(source_stamp_ns=ns, state='ready', generation=generation, selection_stamp_ns=selection)
    join.status(payload, ns+30, types.SimpleNamespace(data='ready'))


def loss(join, ns=300, last_good=200, generation=9, selection=150, observed=330):
    first = dict(source_stamp_ns=ns, last_good_source_stamp_ns=last_good,
        generation=generation, selection_stamp_ns=selection, observed_stamp_ns=observed)
    payload = dict(source_stamp_ns=ns, state='unavailable', generation=generation,
        selection_stamp_ns=selection, first_loss=first)
    join.status(payload, observed, types.SimpleNamespace(data='loss'))
    if ns is not None:
        join.object(ns, object_message(ns, False), observed+1)


def hold(join, ns=300, last_good=200, gap='unique_contained_shrinking_fragment'):
    payload = dict(source_stamp_ns=ns, state='unavailable', generation=9, selection_stamp_ns=150,
        rgb_identity_hold=dict(state='holding_identity_only', source_stamp_ns=ns,
            reference_source_stamp_ns=last_good, depth_gap_kind=gap))
    join.status(payload, ns+30, types.SimpleNamespace(data='hold'))
    join.object(ns, object_message(ns, False), ns+31)


def test_pair_mode_pins_ready_native_images_across_cache_eviction():
    join = pair_join(image_limit=1)
    pair(join, 200); ready(join)
    pair(join, 300); loss(join)
    assert join.trigger['last_good_source_matches']
    assert join.trigger['epoch_matches']
    assert [slot['source_stamp_ns'] for slot in join.slots] == [200, 300]
    assert join.slots[0]['images']['depth'][0].encoding == '16UC1'
    assert join.report()['paired_count'] == 2


def test_pair_mode_keeps_latest_ready_and_never_emits_more_than_two_pairs():
    join = pair_join()
    pair(join, 200); ready(join)
    pair(join, 240); ready(join, 240)
    pair(join, 300); loss(join, last_good=240)
    pair(join, 400); ready(join, 400); loss(join, 500, 400)
    assert [slot['source_stamp_ns'] for slot in join.slots] == [240, 300]
    assert len(join.slots) == 2


def test_ready_requires_same_source_detected_true_object_and_ready_status():
    join = pair_join()
    join.status(dict(source_stamp_ns=200, state='ready', generation=9, selection_stamp_ns=150), 220)
    join.object(201, object_message(201, True), 221)
    assert join.last_ready is None
    join.object(200, object_message(200, False), 222)
    assert join.last_ready is None
    join.object(200, object_message(200, True), 223)
    assert join.last_ready['source_stamp_ns'] == 200


def test_hold_waits_for_source_stamped_detected_false_then_records_fragment_kind():
    join = pair_join(); pair(join, 200); ready(join)
    payload = dict(source_stamp_ns=300, state='unavailable', generation=9, selection_stamp_ns=150,
        rgb_identity_hold=dict(state='holding_identity_only', source_stamp_ns=300,
            reference_source_stamp_ns=200, depth_gap_kind='unique_contained_shrinking_fragment'))
    join.status(payload, 320, types.SimpleNamespace(data='hold'))
    assert join.trigger is None
    join.object(300, object_message(300, False), 330)
    assert join.trigger['kind'] == 'holding_identity_only'
    assert join.trigger['depth_gap_kind'] == 'unique_contained_shrinking_fragment'


def test_first_hold_stays_frozen_when_later_permanent_loss_arrives():
    join = pair_join(); pair(join, 200); ready(join); pair(join, 300); hold(join)
    pair(join, 400); loss(join, ns=400)
    assert join.trigger['kind'] == 'holding_identity_only'
    assert join.trigger['source_stamp_ns'] == 300
    assert join.slots[1]['status']['rgb_identity_hold']['source_stamp_ns'] == 300


def test_historical_and_unarmed_losses_do_not_end_new_capture():
    join = pair_join()
    loss(join, observed=90)
    loss(join, observed=120)
    assert join.trigger is None
    pair(join, 200); ready(join)
    loss(join, generation=8, observed=350)
    assert join.trigger is None
    pair(join, 400); loss(join, ns=400)
    assert join.trigger['source_stamp_ns'] == 400
    assert join.report()['ignored_historical_first_loss'] == 1
    assert join.report()['ignored_unarmed_or_other_selection_events'] == 2


def test_declared_last_good_mismatch_is_explicit_no_other_pair_substitution():
    join = pair_join(); pair(join, 200); ready(join); pair(join, 240); pair(join, 300)
    loss(join, last_good=240)
    assert join.trigger['last_good_source_matches'] is False
    assert join.slots[0]['source_stamp_ns'] == 240
    assert join.slots[0]['images'] == {}
    assert 'asserted_last_good_source_does_not_match_pinned_detected_ready' in join.report()['sources'][0]['unavailable']


def test_epoch_change_fails_same_source_pair_evidence():
    join = pair_join(); pair(join, 200); ready(join)
    join.current_context('/alicia_d/control_reference_epoch', types.SimpleNamespace(stamp=stamp(250)), 251)
    pair(join, 300); loss(join)
    assert not join.trigger['epoch_matches']
    assert 'control_reference_epoch_missing_or_changed' in join.report()['sources'][0]['unavailable']
    assert not join.report()['evidence_complete']


def test_missing_exact_image_is_unavailable_even_when_newer_pair_exists():
    join = pair_join(); pair(join, 200); ready(join)
    pair(join, 301); loss(join)
    assert join.slots[1]['images'] == {}
    row = join.report()['sources'][1]
    assert 'exact_rgb_unavailable' in row['unavailable']
    assert 'exact_depth_unavailable' in row['unavailable']
    pair(join, 300)
    assert join.report()['paired_count'] == 2


def test_timeout_loss_without_source_never_uses_observation_time_or_latest_image():
    join = pair_join(); pair(join, 200); ready(join); pair(join, 330)
    loss(join, ns=None, observed=330)
    assert join.slots[1]['source_stamp_ns'] is None
    assert join.slots[1]['images'] == {}
    assert 'first_event_has_no_post_start_exact_source' in join.report()['sources'][1]['unavailable']


def test_repeated_loss_status_does_not_relabel_later_source_as_first_loss():
    join = pair_join(); pair(join, 200); ready(join); pair(join, 300); pair(join, 400)
    payload = dict(source_stamp_ns=400, state='unavailable', generation=9, selection_stamp_ns=150,
        first_loss=dict(source_stamp_ns=300, last_good_source_stamp_ns=200, generation=9,
            selection_stamp_ns=150, observed_stamp_ns=330))
    join.status(payload, 430, types.SimpleNamespace(data='later status'))
    assert join.slots[1]['source_stamp_ns'] == 300
    assert join.slots[1]['images']['rgb'][0].header.stamp.to_nsec() == 300
    assert 'corresponding_detector_status_unavailable' in join.report()['sources'][1]['unavailable']


def test_feedback_requires_true_bracket_with_total_span_at_most_200ms():
    source = 1_000_000_000
    join = pair_join(); pair(join, source); ready(join, source)
    join.accepted_feedback(source-100_000_000, feedback(source-100_000_000, 'before'), source)
    join.accepted_feedback(source+100_000_001, feedback(source+100_000_001, 'too late'), source)
    assert join.last_ready['accepted_bracket'] is None
    join.accepted_feedback(source+100_000_000, feedback(source+100_000_000, 'after'), source)
    bracket = join.last_ready['accepted_bracket']
    assert [item[1].label for item in bracket] == ['before', 'after']
    assert bracket[1][0]-bracket[0][0] == 200_000_000


def test_pinned_feedback_and_context_survive_new_stream_messages():
    join = pair_join(); pair(join, 200); ready(join)
    join.accepted_feedback(199, feedback(199, 'before'), 220); join.accepted_feedback(201, feedback(201, 'after'), 220)
    for ns in range(400, 700):
        join.accepted_feedback(ns, feedback(ns, 'later'), ns)
    join.current_context('/grasp_mode/selection', types.SimpleNamespace(data='new'), 701)
    assert [item[0] for item in join.last_ready['accepted_bracket']] == [199, 201]
    assert join.last_ready['context']['/grasp_mode/selection'][0].data == 'selection'


def test_complete_pair_requires_status_objects_feedback_exact_tf_and_context():
    join = pair_join(); pair(join, 200); ready(join); pair(join, 300); loss(join)
    for ns in (200, 300):
        join.accepted_feedback(ns, feedback(ns), ns+1)
    assert not join.report()['evidence_complete']
    for slot in join.slots:
        slot['exact_tf'] = types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(slot['source_stamp_ns'])))
    assert join.report()['evidence_complete']
    assert [row['accepted_feedback_span_ns'] for row in join.report()['sources']] == [0, 0]


def test_budget_preflight_uses_actual_step_and_native_depth_bytes():
    join = pair_join(); pair(join, 200); ready(join); pair(join, 300); loss(join)
    join.slots[0]['images']['depth'] = (image(200, encoding='16UC1', step=12), 220)
    preflight = module.pair_preflight(join.slots, [], 100)
    assert preflight['native_image_bytes'] == 36+36+36+24
    assert preflight['actual_image_layouts'][0]['depth']['step'] == 12
    assert preflight['actual_image_layouts'][0]['depth']['encoding'] == '16UC1'
    assert preflight['fits']


def test_budget_refuses_actual_high_resolution_pair_and_oversized_sidecars():
    import pytest
    join = pair_join(); pair(join, 200); ready(join); loss(join)
    join.slots[0]['images'] = dict(rgb=(image(200, 1920, 1080), 210),
        depth=(image(200, 1920, 1080, '16UC1'), 220))
    with pytest.raises(ValueError, match='budget insufficient'):
        module.pair_preflight(join.slots, [], 100)
    with pytest.raises(ValueError, match='budget insufficient'):
        module.pair_preflight([], [], module.PAIR_BUDGET_BYTES)


def test_budget_refuses_more_than_two_pairs_malformed_data_and_wrong_stamp():
    import pytest
    join = pair_join(); pair(join, 200); ready(join); pair(join, 300); loss(join)
    with pytest.raises(ValueError, match='at most two'):
        module.pair_preflight(join.slots+[join.slots[0]], [], 0)
    join.slots[0]['images']['rgb'][0].data = b''
    with pytest.raises(ValueError, match='actual image shape'):
        module.pair_preflight(join.slots, [], 0)
    join.slots[0]['images']['rgb'] = (image(201), 220)
    with pytest.raises(ValueError, match='differs from pinned source'):
        module.pair_preflight(join.slots, [], 0)


def test_hard_bag_limit_checks_real_write_offset(tmp_path):
    import pytest
    capped = module.CappedBagFile(tmp_path/'capped.bag', 8)
    try:
        assert isinstance(capped, module.io.IOBase)  # rosbag's actual file-object admission check.
        capped.write(b'12345678')
        with pytest.raises(RuntimeError, match='hard artifact budget'):
            capped.write(b'9')
        capped.seek(7)
        with pytest.raises(RuntimeError, match='hard artifact budget'):
            capped.write(b'90')
        with pytest.raises(RuntimeError, match='hard artifact budget'):
            capped.truncate(9)
        capped.seek(0); capped.write(b'8')
    finally:
        capped.close()
    assert (tmp_path/'capped.bag').stat().st_size == 8


def test_delayed_detected_true_callback_can_arm_already_received_exact_loss():
    join = pair_join(); pair(join, 200); pair(join, 300)
    join.status(dict(source_stamp_ns=200, state='ready', generation=9, selection_stamp_ns=150),
        230, types.SimpleNamespace(data='ready'))
    loss(join)
    assert join.trigger is None
    join.object(200, object_message(200, True), 350)
    assert join.trigger['source_stamp_ns'] == 300
    assert join.trigger['last_good_source_matches']


def test_unarmed_old_hold_does_not_block_new_selection_ready_and_event():
    join = pair_join()
    payload = dict(source_stamp_ns=160, state='unavailable', generation=8, selection_stamp_ns=110,
        rgb_identity_hold=dict(state='holding_identity_only', source_stamp_ns=160,
            reference_source_stamp_ns=120))
    join.status(payload, 170, types.SimpleNamespace(data='old hold'))
    join.object(160, object_message(160, False), 180)
    pair(join, 200); ready(join); pair(join, 300); hold(join)
    assert join.trigger['source_stamp_ns'] == 300
    assert join.trigger['last_good_source_matches']


def test_feedback_rejects_wrong_frame_missing_duplicate_and_nonfinite_joints():
    join = pair_join(); pair(join, 200); ready(join)
    invalid = [feedback(200) for _ in range(4)]
    invalid[0].header.frame_id = 'commanded'
    invalid[1].name[-1] = 'unexpected_joint'
    invalid[2].name[-1] = invalid[2].name[0]
    invalid[3].position[2] = float('nan')
    for message in invalid:
        join.accepted_feedback(200, message, 210)
    assert join.last_ready['accepted_bracket'] is None
    assert sum(join.report()['rejected_accepted_feedback'].values()) == 4
    join.accepted_feedback(200, feedback(200), 210)
    assert join.last_ready['accepted_bracket'] is not None


def test_feedback_requires_latched_epoch_and_source_at_or_after_epoch():
    join = module.FirstLossPairSources(100)
    join.accepted_feedback(200, feedback(200), 210)
    assert not join.feedback
    join.current_context('/alicia_d/control_reference_epoch', types.SimpleNamespace(stamp=stamp(200)), 205)
    join.accepted_feedback(199, feedback(199), 210)
    assert not join.feedback
    join.accepted_feedback(200, feedback(200), 210)
    assert list(join.feedback) == [200]


def test_feedback_with_different_header_stamp_cannot_bracket_source():
    join = pair_join(); pair(join, 200); ready(join)
    join.accepted_feedback(200, feedback(201), 210)
    assert join.last_ready['accepted_bracket'] is None


def test_new_epoch_excludes_previously_cached_older_feedback_from_bracket():
    join = pair_join()
    join.accepted_feedback(199, feedback(199), 210)
    join.current_context('/alicia_d/control_reference_epoch', types.SimpleNamespace(stamp=stamp(200)), 211)
    pair(join, 201); ready(join, 201)
    join.accepted_feedback(202, feedback(202), 212)
    assert join.last_ready['accepted_bracket'] is None
    join.accepted_feedback(200, feedback(200), 213)
    assert [item[0] for item in join.last_ready['accepted_bracket']] == [200, 202]


def test_metadata_source_hashes_support_relative_python_entrypoint(monkeypatch):
    # Match `/usr/bin/python3 .runtime/collect_unknown_first_loss.py`, whose
    # __file__ can stay relative; importlib's normal absolute path hid this bug.
    monkeypatch.chdir(module.HERE.parent)
    monkeypatch.setattr(module, '__file__', '.runtime/collect_unknown_first_loss.py')
    paths = module.diagnostic_source_files()
    assert all(path.is_absolute() for path in paths)
    hashes = module.diagnostic_source_sha256()
    expected = module.hashlib.sha256((module.HERE/'collect_unknown_first_loss.py').read_bytes()).hexdigest()
    assert hashes['.runtime/collect_unknown_first_loss.py'] == expected
    assert len(hashes) == 4
    # Exercise the exact metadata JSON boundary used during finalization.
    assert module.json.loads(module.json.dumps(dict(source_sha256=hashes)))['source_sha256'] == hashes


def test_final_parameter_failure_becomes_explicit_unavailable():
    errors = []
    def offline_master():
        raise RuntimeError('ROS master unavailable')
    assert module.finalization_value('current_parameter:/handeye', offline_master, errors) is None
    assert errors == [dict(stage='current_parameter:/handeye', error='ROS master unavailable')]


def test_final_source_hash_failure_and_non_json_context_are_reportable():
    errors = []
    def missing_source():
        raise FileNotFoundError('source removed during capture')
    assert module.finalization_value('source_sha256', missing_source, errors) is None
    assert module.finalization_value('current_camera', lambda: {'unsupported': object()}, errors) is None
    assert [row['stage'] for row in errors] == ['source_sha256', 'current_camera']
    module.json.dumps(errors, allow_nan=False)


def test_bad_context_serialization_does_not_discard_either_raw_pair():
    def wire(payload, fail=False):
        def serialize(stream):
            if fail:
                raise ValueError('bad context message')
            stream.write(payload)
        return types.SimpleNamespace(serialize=serialize, _type='synthetic/Wire', _md5sum='synthetic')
    slots = []
    for source in (200, 300):
        slots.append(dict(images=dict(rgb=(wire(b'rgb'), source+1), depth=(wire(b'uint16'), source+2)),
            status_message=wire(b'status'), status_receipt_ns=source+3, object=None,
            accepted_bracket=None, exact_tf=None,
            context={'/bad_context': (wire(b'', fail=True), source+4)}))
    records, errors = module.serialize_pair_records(slots)
    images = [row for row in records if row[0].startswith('/unknown_loss/exact_')]
    assert len(images) == 4
    assert [row[1] for row in images] == [b'rgb', b'uint16', b'rgb', b'uint16']
    assert len(errors) == 2
    assert all(row['topic'] == '/bad_context' for row in errors)


def test_one_image_serialization_failure_preserves_other_valid_messages():
    def bad(stream):
        raise ValueError('bad image serialization')
    good = types.SimpleNamespace(serialize=lambda stream: stream.write(b'depth'),
        _type='synthetic/Wire', _md5sum='synthetic')
    broken = types.SimpleNamespace(serialize=bad)
    slot = dict(images=dict(rgb=(broken, 201), depth=(good, 202)), status_message=None,
        object=None, accepted_bracket=None, exact_tf=None, context={})
    records, errors = module.serialize_pair_records([slot])
    assert [(row[0], row[1]) for row in records] == [('/unknown_loss/exact_depth', b'depth')]
    assert errors[0]['topic'] == '/unknown_loss/exact_rgb'


def pending_qualification(join, ns, reference_ns, rejection=None):
    qualification = dict(state='pending_source_qualification', source_stamp_ns=ns,
        reference_source_stamp_ns=reference_ns, source_qualified=False, identity_verified=False,
        metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False,
        reason='accepted_feedback_does_not_bracket_source', feedback_rejection=rejection)
    payload = dict(source_stamp_ns=ns, state='unavailable', generation=9, selection_stamp_ns=150,
        detail='source_qualification_pending', source_qualification=qualification)
    join.status(payload, ns+30, types.SimpleNamespace(data='pending'))
    join.object(ns, object_message(ns, False), ns+31)


def test_first_pending_source_qualification_triggers_raw_pair_without_positive_hold():
    join = pair_join(); pair(join, 200); ready(join); pair(join, 300)
    pending_qualification(join, 300, 200)
    assert join.trigger['kind'] == 'pending_source_qualification'
    assert join.trigger['last_good_source_matches']
    assert join.trigger['source_qualification']['source_qualified'] is False
    assert join.trigger['source_qualification']['identity_verified'] is False
    assert 'rgb_identity_hold' not in join.slots[1]['status']
    assert join.report()['paired_count'] == 2
    assert not join.report()['evidence_complete']
    # A later positive identity hold cannot replace the first unqualified event.
    pair(join, 400); hold(join, 400, 200)
    assert join.trigger['source_stamp_ns'] == 300


def test_pending_keeps_actual_200043250ns_bracket_without_qualifying_it():
    lo, hi = 1791100581628347351, 1791100581828390601
    previous, source = lo-100_000_000, lo+70_000_000
    join = pair_join(); pair(join, previous); ready(join, previous)
    join.accepted_feedback(previous, feedback(previous), previous+10)
    pair(join, source)
    join.accepted_feedback(lo, feedback(lo), lo+10)
    join.accepted_feedback(hi, feedback(hi), hi+10)
    rejection = dict(accepted_before=[lo, [0.]*6], accepted_after=[hi, [0.]*6],
        span_ns=hi-lo, maximum_span_ns=200_000_000, bracket_qualified=False)
    pending_qualification(join, source, previous, rejection)
    for slot in join.slots:
        slot['exact_tf'] = object()
    row = join.report()['sources'][1]
    assert row['observed_feedback_source_ns'] == [lo, hi]
    assert row['observed_feedback_span_ns'] == 200_043_250
    assert row['observed_feedback_bracket_qualified'] is False
    assert row['accepted_feedback_source_ns'] is None
    assert 'accepted_feedback_bracket_within_200ms_unavailable' in row['unavailable']
    assert not join.report()['evidence_complete']
    assert join.report()['paired_count'] == 2
    assert join.trigger['source_qualification']['feedback_rejection'] == rejection


def test_unqualified_sdk_bracket_is_serialized_and_survives_feedback_eviction():
    lo, hi, source = 1_000_000_000, 1_200_043_250, 1_050_000_000
    join = pair_join(); pair(join, 800_000_000); ready(join, 800_000_000); pair(join, source)
    for ns in (lo, hi):
        message = feedback(ns)
        message.serialize = lambda stream, ns=ns: stream.write(str(ns).encode())
        message._type, message._md5sum = 'synthetic/JointState', 'synthetic'
        join.accepted_feedback(ns, message, ns+1)
    pending_qualification(join, source, 800_000_000)
    for ns in range(2_000_000_000, 2_000_000_300):
        join.accepted_feedback(ns, feedback(ns), ns+1)
    records, _ = module.serialize_pair_records(join.slots)
    saved = [row[1] for row in records if row[0] == '/alicia_d/accepted_joint_states']
    assert saved == [str(lo).encode(), str(hi).encode()]
    assert join.report()['sources'][1]['observed_feedback_span_ns'] == 200_043_250


def test_late_qualified_feedback_does_not_erase_original_observed_wide_interval():
    join = pair_join(); pair(join, 1_000_000_000); ready(join, 1_000_000_000)
    join.accepted_feedback(900_000_000, feedback(900_000_000), 1_300_000_000)
    join.accepted_feedback(1_100_043_250, feedback(1_100_043_250), 1_300_000_001)
    assert join.last_ready['accepted_bracket'] is None
    join.accepted_feedback(1_000_000_000, feedback(1_000_000_000), 1_300_000_002)
    assert [row[0] for row in join.last_ready['accepted_bracket']] == [1_000_000_000]*2
    assert [row[0] for row in join.last_ready['observed_bracket']] == [900_000_000, 1_100_043_250]


def test_pair_wait_explicit_180_seconds_keeps_defaults_and_full_source_limit():
    import pytest
    assert module.parse_arguments([]).seconds == 40.
    assert module.parse_arguments(['--first-loss-pair']).seconds == 40.
    assert module.parse_arguments(['--first-loss-pair', '--seconds', '180']).seconds == 180.
    assert module.parse_arguments(['--seconds', '45']).seconds == 45.
    for args in (['--seconds', '46'], ['--seconds', '180'],
                 ['--first-loss-pair', '--seconds', '29.999'],
                 ['--first-loss-pair', '--seconds', '180.001'],
                 ['--first-loss-pair', '--seconds', 'inf'],
                 ['--first-loss-pair', '--seconds', 'nan']):
        with pytest.raises(SystemExit):
            module.parse_arguments(args)


def test_long_wait_can_pin_ready_after_47_seconds_without_growing_pair_budget():
    join = pair_join()
    for second in range(1, 180):
        source = second*1_000_000_000
        pair(join, source)
        if second == 47:
            ready(join, source)
        elif second == 48:
            pending_qualification(join, source, 47_000_000_000)
    assert [slot['source_stamp_ns'] for slot in join.slots] == [47_000_000_000, 48_000_000_000]
    assert join.report()['paired_count'] == 2
    assert all(len(cache) <= 48 for cache in join.images.values())
    assert len(join.slots) == 2
    budget = module.pair_preflight(join.slots, [], 128*1024)
    assert budget['maximum_bytes'] == 8*1024*1024
    assert budget['native_image_bytes'] == 2*(4*3*3+4*3*2)
