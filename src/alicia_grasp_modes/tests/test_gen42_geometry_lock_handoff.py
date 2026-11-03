"""Real threaded ingress while the common geometry worker is computing.

Only scheduling, encoder/device boundaries and deliberate competing cache
writers are simulated. RGB-D qualification, registration/fusion, source joins,
common planning, public handoff, task acceptance and closing gates are real.
The same file runs against the unchanged GEN41 tree as a failing before case.
"""
from copy import deepcopy
from dataclasses import replace
import json
import threading
from types import SimpleNamespace as NS

import pytest

import test_gen35_minimal_chain_replay as chain
import test_gen41_pending_handoff_chain as wire
from test_gen38_cuboid_chain import cuboid_scene
from test_gen39_registration_chain import _install_real_reached_source_adapter
from test_remote_modes import base


class _IngressStarved(BaseException):
    """Escape broad production error handlers at the first failed handoff."""


class _PreparationObserved(BaseException):
    """Stop a rejection case before planning or any contact motion."""


@pytest.fixture(autouse=True)
def no_unsimulated_boundaries(monkeypatch):
    import rospy
    import socket

    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('GEN42 unsimulated boundary '+repr(args))

    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, forbidden)


def _source(c):
    record = c.remote._station_compute_reference
    return None if record is None else record['contract']['source_ns']


def _task_source(c):
    record = c.task._station_task_record
    return None if record is None else record['contract']['source_ns']


def _event(c, name, **facts):
    c.gen42_trace.append(dict(name=name,
        thread_name=threading.current_thread().name,
        now_ns=wire._now(c), remote_source_ns=_source(c),
        task_source_ns=_task_source(c), **facts))


def _owners(node):
    return {name: getattr(node, name, None) for name in (
        '_latest_target_observation', '_latest_raw_geometry_source',
        '_near_field_reference_view', '_near_field_fused_surface',
        '_latest_registration_evidence', '_latest_box_registration_choice',
        '_latest_fused_scene_audit', '_regular_box_snapshot')}


def _fresh_triplet(c):
    """One real producer source plus independent explicit SDK-provider samples."""
    c.clock[0] += .2
    source_sec = c.clock[0]-.1
    for stamp_sec in (source_sec-.05, source_sec+.05):
        stamp = chain.producer_module.rospy.Time.from_sec(stamp_sec).to_nsec()
        if stamp > c.task.latest_accepted_joint_state.header.stamp.to_nsec():
            wire._sdk(c, stamp)
    # The unchanged controller simulator owns the current-time sample. Its
    # feed wrapper updates its private last-emitted clock as well as Task.
    # Emit only bracket endpoints above, then relay/reuse that exact current
    # sample for Remote; a second Task insertion would break settle ordering.
    pair = c.feed()
    wire._sdk(c, wire._now(c))
    row = chain.publication(c.producer)
    assert row['status']['state'] == 'ready', row['status']
    assert row['object'].detected
    stamp = row['object'].header.stamp.to_nsec()
    assert stamp == pair[0].header.stamp.to_nsec()
    assert stamp > _source(c)
    c.remote.frames.update_color(c.rgb, pair[0].header.stamp, pair[0].header.frame_id)
    c.remote.frames.update_depth(c.scene.depth, pair[1].header.stamp, pair[1].header.frame_id)
    from alicia_grasp_modes.rgb_observation_source import bracket_feedback
    bracket_feedback(list(c.remote._station_compute_feedback), stamp,
        c.producer._rgb_interest_epoch)
    return row


def _install(monkeypatch, *, order=('status', 'object', 'mask'),
             boundary='registration', conflict=None):
    original_contact = chain.same_source_contact
    original_heavy = getattr(base, ('append_box_consistent_registered_view'
        if boundary == 'registration' else 'enclose_inferred_box_geometry'))
    captured = []

    def contact(c):
        captured.append(c)
        c.gen42_trace, c.gen42_threads, c.gen42_errors = [], [], []
        c.gen41_sdk_provider = []
        c.gen42_boundary, c.gen42_conflict = boundary, conflict
        c.gen42_heavy_released = threading.Event()
        c.gen42_callback_entered = threading.Event()
        c.gen42_callback_done = threading.Event()
        c.gen42_fired = False
        n = c.remote
        publish = n.geometry_pub.publish

        def public_geometry(message):
            publish(message)
            c.task.target_geometry_cb(deepcopy(message))
        monkeypatch.setattr(n.geometry_pub, 'publish', public_geometry)

        if conflict is not None:
            prepare = (n._prepare_snapshot_geometry if boundary == 'registration'
                       else n._prepare_regular_box_snapshot)

            def observed(*args, **kwargs):
                try:
                    result = prepare(*args, **kwargs)
                except base.StreamResultCancelled as exc:
                    if not c.gen42_fired:
                        raise
                    c.gen42_cancel_code = exc.code
                    c.gen42_after_owners = _owners(n)
                    _event(c, 'obsolete_preparation_cancelled', code=exc.code)
                    raise _PreparationObserved() from exc
                if c.gen42_fired:
                    c.gen42_result = result
                    c.gen42_after_owners = _owners(n)
                    _event(c, 'obsolete_preparation_returned')
                    raise _PreparationObserved()
                return result
            monkeypatch.setattr(n, ('_prepare_snapshot_geometry' if boundary == 'registration'
                else '_prepare_regular_box_snapshot'), observed)
        return original_contact(c)

    def heavy(*args, **kwargs):
        if not captured:
            return original_heavy(*args, **kwargs)
        c = captured[-1]
        n = c.remote
        ticket = getattr(c, 'ticket', None)
        stamp = (args[2].stamp_ns if boundary == 'registration'
                 else getattr(getattr(n, '_latest_target_observation', None), 'stamp_ns', None))
        if (c.gen42_fired or ticket is None or stamp != ticket.payload[0].stamp_ns):
            return original_heavy(*args, **kwargs)
        c.gen42_fired = True
        parent = deepcopy(n._station_compute_reference['contract'])
        phase_deadline = n._near_field_phase_deadline_sec
        lifecycle = n._multiview_lifecycle_token()
        c.gen42_parent = parent
        _event(c, 'heavy_computation_entered', boundary=boundary,
            original_identity_deadline_ns=parent['identity_deadline_ns'])
        row = _fresh_triplet(c)
        c.gen42_transport_row = deepcopy(row)
        assert wire._now(c) < parent['identity_deadline_ns']
        fresh_source = row['status']['source_stamp_ns']

        def ingress():
            try:
                c.gen42_callback_entered.set()
                _event(c, 'original_callbacks_started')
                # Exact duplicate requests exercise the original source history
                # and may not cause a repeated observation or contact goal.
                wire._deliver(c, row, order)
                wire._deliver(c, row, tuple(reversed(order)))
                assert _source(c) == fresh_source
                assert _task_source(c) == fresh_source
                assert not n._station_compute_retired
                assert not c.task._station_task_retired
                assert n._near_field_phase_deadline_sec == phase_deadline
                assert n._multiview_lifecycle_token() == lifecycle
                assert not c.gen42_heavy_released.is_set()
                _event(c, 'fresh_join_completed_before_heavy_release',
                    fresh_source_ns=fresh_source)
                if conflict is not None:
                    with n._stream_condition, n._geometry_state_guard():
                        if conflict == 'same_generation_observation':
                            previous = n._latest_target_observation
                            n._latest_target_observation = replace(previous,
                                stamp_ns=previous.stamp_ns+1)
                            previous_raw = n._latest_raw_geometry_source
                            n._latest_raw_geometry_source = dict(
                                previous_raw or {'diagnostic_only': True,
                                    'fixture_competing_cache_owner': True,
                                    'qualified_measurement_claimed': False},
                                observation=n._latest_target_observation)
                        elif conflict == 'same_generation_surface':
                            n._near_field_fused_surface = replace(n._near_field_fused_surface)
                        elif conflict == 'same_generation_reference':
                            n._near_field_reference_view = replace(n._near_field_reference_view)
                        elif conflict == 'invalidation_generation':
                            n._geometry_invalidation_generation += 1
                        elif conflict == 'same_observation_box':
                            # An explicitly competing cache writer, not new
                            # measured box evidence or an executable plan.
                            n._regular_box_snapshot = {'competing_owner': object()}
                        else:
                            raise AssertionError('unmodeled writer conflict '+str(conflict))
                        c.gen42_competing_owners = _owners(n)
                        _event(c, 'competing_cache_writer_committed', conflict=conflict)
            except BaseException as exc:
                c.gen42_errors.append(exc)
            finally:
                c.gen42_callback_done.set()

        thread = threading.Thread(target=ingress, name='gen42-real-station-ingress')
        c.gen42_threads.append(thread)
        thread.start()
        try:
            assert c.gen42_callback_entered.wait(3.), 'callback thread was not scheduled'
            progressed = c.gen42_callback_done.wait(3.)
            _event(c, 'bounded_callback_progress_checked', completed=progressed)
            if not progressed:
                raise _IngressStarved('original station callbacks blocked behind '
                    +boundary+' computation before heavy-release barrier')
            if c.gen42_errors:
                raise _IngressStarved('original station callback rejected fresh source: '
                    +repr(c.gen42_errors[0]))
        finally:
            c.gen42_heavy_released.set()
            _event(c, 'heavy_computation_released')
        # The real registration/enclosure runs with unchanged inputs/results.
        return original_heavy(*args, **kwargs)

    monkeypatch.setattr(chain, 'same_source_contact', contact)
    monkeypatch.setattr(base, ('append_box_consistent_registered_view'
        if boundary == 'registration' else 'enclose_inferred_box_geometry'), heavy)
    return captured


def _join_save(captured, tmp_path):
    for c in captured:
        for thread in c.gen42_threads:
            thread.join(3.)
            assert not thread.is_alive(), 'offline ingress thread did not terminate'
        data = dict(scope='threaded scheduling proof with explicit device simulators',
            physical_grasp_verified=False, production_metrics_stubbed=False,
            trace=c.gen42_trace, parent_contract=getattr(c, 'gen42_parent', None),
            callback_errors=[repr(e) for e in c.gen42_errors],
            sdk_provider=c.gen41_sdk_provider,
            boundary=c.gen42_boundary, conflict=c.gen42_conflict,
            device_calls=getattr(c, 'device_calls', []))
        (tmp_path/'gen42_thread_handoff.json').write_text(
            json.dumps(data, default=str, indent=2)+'\n')


@pytest.mark.parametrize('boundary,order', [
    ('registration', ('status', 'object', 'mask')),
    ('registration', ('mask', 'object', 'status')),
    ('box_enclosure', ('object', 'status', 'mask')),
])
def test_fresh_callbacks_finish_while_geometry_is_computing_then_common_chain_closes_once(
        monkeypatch, tmp_path, boundary, order):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install(monkeypatch, order=order, boundary=boundary)
    try:
        try:
            c = chain.unified_replay(monkeypatch, tmp_path, perturb=True)
        except _IngressStarved as exc:
            pytest.fail(str(exc))
    finally:
        _join_save(captured, tmp_path)
    assert captured == [c] and c.gen42_fired
    names = [r['name'] for r in c.gen42_trace]
    assert names.index('fresh_join_completed_before_heavy_release') < names.index('heavy_computation_released')
    assert not c.gen42_errors
    assert c.execution_result is False and c.states[-1].state == 'HOLDING'
    assert len(c.task._frozen_contact_pub.messages) == 1
    assert sum(x[0] == 'motion' and x[1] == c.origin.plan_id for x in c.device_calls) == 1
    assert sum(x[0] == 'motion' and x[1] == c.contact.plan_id for x in c.device_calls) == 3
    assert sum(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls) == 1
    assert not any(x[0] == 'lift' for x in c.device_calls)
    assert 'duplicate_start_rejected' in c.injected_events


@pytest.mark.parametrize('conflict', [
    'same_generation_observation', 'same_generation_surface',
    'same_generation_reference', 'invalidation_generation',
])
def test_obsolete_registration_cannot_overwrite_same_generation_or_invalidated_owner(
        monkeypatch, tmp_path, conflict):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install(monkeypatch, conflict=conflict)
    try:
        with pytest.raises(_PreparationObserved):
            c = chain.arrived_same_scene(monkeypatch)
            chain.same_source_contact(c)
    finally:
        _join_save(captured, tmp_path)
    assert captured == [c] and c.gen42_fired
    expected_code = (str(getattr(c.remote, '_last_geometry_invalidation_code', '')
        or 'PLAN_STALE') if conflict == 'invalidation_generation'
        else 'TARGET_PROCESSING_PENDING')
    assert c.gen42_cancel_code == expected_code
    for name, owner in c.gen42_competing_owners.items():
        assert c.gen42_after_owners[name] is owner, 'obsolete writer overwrote '+name
    assert not hasattr(c, 'contact')
    assert not getattr(c.task, '_frozen_contact_pub', NS(messages=[])).messages


def test_box_enclosure_cannot_overwrite_a_new_box_owner_for_the_same_observation(
        monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install(monkeypatch, boundary='box_enclosure', conflict='same_observation_box')
    try:
        with pytest.raises(_PreparationObserved):
            c = chain.arrived_same_scene(monkeypatch)
            chain.same_source_contact(c)
    finally:
        _join_save(captured, tmp_path)
    assert captured == [c] and c.gen42_fired
    assert c.gen42_cancel_code == 'TARGET_PROCESSING_PENDING'
    assert c.gen42_after_owners['_regular_box_snapshot'] is c.gen42_competing_owners['_regular_box_snapshot']
    assert not hasattr(c, 'contact')


class _StationSourceConfigured(BaseException):
    """Finish fixture configuration before the real worker preparation starts."""


def _original_prepare_race(monkeypatch, tmp_path, race, revocation=None):
    """Actual source-to-common-prepare input with an explicit concurrent writer."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    c = chain.arrived_same_scene(monkeypatch)
    real_station_samples = chain.station_samples

    def station_only(context):
        real_station_samples(context)
        raise _StationSourceConfigured()

    with monkeypatch.context() as patch:
        patch.setattr(chain, 'station_samples', station_only)
        with pytest.raises(_StationSourceConfigured):
            chain.same_source_contact(c)
    n, ticket = c.remote, c.ticket
    assert n._station_compute_work_certificate(ticket)
    assert ticket.payload[0].original_sample is not None
    c.gen41_sdk_provider = []
    monkeypatch.setattr(n, '_refresh_runtime_params', lambda: None)
    ctx = NS(c=c, node=n, ticket=ticket, race=race, revocation=revocation,
        counters=dict(geometry=0, qualification=0, enclosure=0,
            candidates=0, paths=0, learned=0, accept=0),
        qualified=[], writers=[], writer_errors=[], injected=False,
        expected_owners=None, box_owner=None)
    actual_geometry = n._prepare_snapshot_geometry
    actual_box = n._prepare_regular_box_snapshot
    actual_qualify = base.qualify_regular_box_snapshot
    actual_enclose = base.enclose_inferred_box_geometry
    actual_register = base.append_box_consistent_registered_view
    ctx.channels = ('geometry_pub', 'preview_plan_pub', 'preview_rich_plan_pub',
                    'plan_pub', 'rich_plan_pub')
    ctx.publications = {name: len(getattr(n, name).messages) for name in ctx.channels}
    ctx.generation, ctx.phase = n._stream_generation, n._near_field_phase_id
    ctx.phase_deadline = n._near_field_phase_deadline_sec
    ctx.session_deadline = n._observe_return_session['deadline_ns']
    ctx.identity = n._current_stream_target_identity()
    ctx.watermark = n.last_submitted_stamp_ns
    ctx.contract = deepcopy(n._station_compute_reference['contract'])

    def writer(mutation):
        done = threading.Event()

        def commit():
            try:
                with n._stream_condition, n._geometry_state_guard():
                    mutation()
                    if revocation == 'manual':
                        c.params['/gui/joint_direct_mode'] = True
                    elif revocation == 'target_epoch':
                        n._advance_target_instance_epoch('TARGET_INSTANCE_CHANGED')
                    ctx.expected_owners = _owners(n)
                    ctx.injected = True
            except BaseException as exc:
                ctx.writer_errors.append(exc)
            finally:
                done.set()
        thread = threading.Thread(target=commit, name='gen42-common-'+race+'-writer')
        ctx.writers.append(thread)
        thread.start()
        assert done.wait(3.), 'common '+race+' retained input/geometry commit locks'
        assert not ctx.writer_errors

    def geometry(*args, **kwargs):
        ctx.counters['geometry'] += 1
        return actual_geometry(*args, **kwargs)

    def qualify(*args, **kwargs):
        result = actual_qualify(*args, **kwargs)
        ctx.counters['qualification'] += 1
        ctx.qualified.append(result)
        return result

    def box_owner(geometry, qualified, enclosed):
        assert qualified.ok
        return dict(lifecycle=n._multiview_lifecycle_token(),
            observation=n._latest_target_observation, result=qualified,
            geometry=geometry, planning_geometry=enclosed[0], enclosure=enclosed[1],
            current_tool=None)

    def enclose(geometry, model):
        result = actual_enclose(geometry, model)
        ctx.counters['enclosure'] += 1
        assert ctx.qualified[-1].ok and ctx.qualified[-1].model is model
        if race == 'box_commit' and not ctx.injected:
            ctx.box_owner = box_owner(geometry, ctx.qualified[-1], result)
            writer(lambda: setattr(n, '_regular_box_snapshot', ctx.box_owner))
        return result

    def box_entry(snapshot, geometry, transform):
        observation = n._latest_target_observation
        camera = n._camera_intrinsics()
        qualified = qualify(snapshot, camera, float(camera.depth_scale), transform,
            geometry, observation.identity, observation.stamp_ns)
        assert qualified.ok
        ctx.box_owner = box_owner(geometry, qualified, enclose(geometry, qualified.model))

        def supersede_source_owner():
            # The qualified box really came from the current source. Simulate
            # loss/replacement of its current observation owner separately;
            # never claim the +1 marker is a new measured/qualified frame.
            n._regular_box_snapshot = ctx.box_owner
            n._latest_target_observation = (None if race == 'box_entry_missing'
                else replace(observation, stamp_ns=observation.stamp_ns+1))
        writer(supersede_source_owner)
        return actual_box(snapshot, geometry, transform)

    def register(*args, **kwargs):
        result = actual_register(*args, **kwargs)
        if not ctx.injected:
            def supersede_geometry_owner():
                observation = n._latest_target_observation
                n._latest_target_observation = (None if observation is None
                    else replace(observation, stamp_ns=observation.stamp_ns+1))
                n._latest_raw_geometry_source = dict(
                    n._latest_raw_geometry_source or {},
                    diagnostic_only=True, fixture_competing_cache_owner=True,
                    qualified_measurement_claimed=False)
            writer(supersede_geometry_owner)
        return result

    def forbidden(stage):
        def called(*args, **kwargs):
            ctx.counters[stage] += 1
            raise AssertionError('obsolete common preparation entered '+stage)
        return called

    monkeypatch.setattr(n, '_prepare_snapshot_geometry', geometry)
    monkeypatch.setattr(base, 'qualify_regular_box_snapshot', qualify)
    monkeypatch.setattr(base, 'enclose_inferred_box_geometry', enclose)
    if race.startswith('box_entry'):
        monkeypatch.setattr(n, '_prepare_regular_box_snapshot', box_entry)
    elif race == 'geometry_commit':
        monkeypatch.setattr(base, 'append_box_consistent_registered_view', register)
    monkeypatch.setattr(n, '_generate_tabletop_candidates', forbidden('candidates'))
    monkeypatch.setattr(n, '_strict_moveit_evaluation', forbidden('paths'))
    monkeypatch.setattr(n, '_predict_remote', forbidden('learned'))
    monkeypatch.setattr(n, '_accept_prediction', forbidden('accept'))
    ctx.actual_prepare = n._prepare_and_predict
    return ctx


def _join_common_writers(ctx):
    for thread in ctx.writers:
        thread.join(3.)
        assert not thread.is_alive()


def _assert_local_preparation_preserved(ctx):
    n = ctx.node
    assert ctx.injected and not ctx.writer_errors
    for name, owner in ctx.expected_owners.items():
        assert getattr(n, name, None) is owner, 'superseded writer changed '+name
    assert ctx.counters['geometry'] == 1
    assert ctx.counters['enclosure'] == int(ctx.race != 'geometry_commit')
    assert ctx.counters['candidates'] == ctx.counters['paths'] == ctx.counters['learned'] == ctx.counters['accept'] == 0
    assert {name: len(getattr(n, name).messages) for name in ctx.channels} == ctx.publications
    assert n._near_field_phase_deadline_sec == ctx.phase_deadline
    assert n._observe_return_session['deadline_ns'] == ctx.session_deadline
    assert n.last_submitted_stamp_ns == ctx.watermark
    assert n._current_stream_target_identity() == ctx.identity
    assert len(ctx.c.motion) == 1


@pytest.mark.parametrize('race,revocation', [
    ('box_commit', None), ('box_entry_missing', None),
    ('box_entry_source_mismatch', None), ('geometry_commit', None),
    ('box_commit', 'manual'), ('box_commit', 'target_epoch'),
])
def test_original_main_prepare_cancels_owner_race_before_any_candidate_path_or_publish(
        monkeypatch, tmp_path, race, revocation):
    """Real common prepare; a real hard revocation outranks local retry."""
    ctx = _original_prepare_race(monkeypatch, tmp_path, race, revocation)
    n = ctx.node
    try:
        with pytest.raises(base.StreamResultCancelled) as canceled:
            ctx.actual_prepare(ctx.ticket)
        if revocation is None:
            assert canceled.value.code == 'TARGET_PROCESSING_PENDING'
            _assert_local_preparation_preserved(ctx)
        else:
            assert canceled.value.code != 'TARGET_PROCESSING_PENDING'
            assert canceled.value.code in ('TARGET_LOST', 'TARGET_EPOCH_STALE',
                'GENERATION_STALE', 'OBSERVE_RETURN_PHASE_PAUSED', 'GRASP_MODE_CHANGED')
            assert ctx.counters['candidates'] == ctx.counters['paths'] == ctx.counters['learned'] == ctx.counters['accept'] == 0
            assert all(not message.valid for name in ctx.channels
                if name in ('geometry_pub', 'preview_rich_plan_pub', 'rich_plan_pub')
                for message in getattr(n, name).messages[ctx.publications[name]:])
        assert ctx.injected and not ctx.writer_errors
        (tmp_path/'gen42_common_owner_cancellation.json').write_text(json.dumps(dict(
            race=race, revocation=revocation, cancellation_code=canceled.value.code,
            counters=ctx.counters, source_watermark_ns=ctx.watermark,
            phase_deadline_sec=ctx.phase_deadline, session_deadline_ns=ctx.session_deadline,
            main_entry_original=True, source_geometry_registration_and_box_original=True,
            current_runtime_parameter_provider='fixture frozen configuration',
            competing_writer='explicit cache-owner scheduling simulator',
            marker_is_new_measured_source=False, real_worker_recovery_proven=False,
            physical_grasp_verified=False), indent=2)+'\n')
    finally:
        _join_common_writers(ctx)


@pytest.mark.parametrize('race', ['box_commit', 'box_entry_missing', 'geometry_commit'])
def test_real_worker_owner_cancellation_releases_slot_then_admits_three_new_station_sources(
        monkeypatch, tmp_path, race):
    """Original worker cleanup and fresh-source polling; second prepare stops at claim."""
    ctx = _original_prepare_race(monkeypatch, tmp_path, race)
    c, n, ticket = ctx.c, ctx.node, ctx.ticket
    assert n._stream_worker_ticket is ticket
    assert n._direct_near_field_submission_generation == ticket.generation
    actual_release = n._release_preparation_cancelled_direct_snapshot_locked
    actual_observe_failure = n._observe_stream_prediction_failure
    actual_publish_metrics = n.pipeline_metrics_pub.publish
    failures, metrics, releases, newer_tickets, poll_results, worker_errors = [], [], [], [], [], []
    first_metric = threading.Event()
    first_release = threading.Event()
    second_claimed = threading.Event()
    second_release = threading.Event()
    worker_finished = threading.Event()

    def prepare(actual_ticket):
        if actual_ticket.request_id == ticket.request_id:
            return ctx.actual_prepare(actual_ticket)
        n._require_stream_ticket_current(actual_ticket)
        newer_tickets.append(dict(request_id=actual_ticket.request_id,
            source_stamps_ns=list(actual_ticket.payload[0].sample_stamp_ns),
            private_certificate=bool(n._station_compute_work_certificate(actual_ticket)),
            generation=actual_ticket.generation, target_epoch=actual_ticket.target_epoch))
        second_claimed.set()
        # Explicitly stop the second actual admitted ticket. No second plan,
        # candidate check, executable motion or physical grasp is claimed.
        raise base.StreamResultCancelled('TARGET_PROCESSING_PENDING')

    def observe_failure(*args, **kwargs):
        failures.append(dict(args=[str(arg) for arg in args], kwargs=kwargs))
        return actual_observe_failure(*args, **kwargs)

    def publish_metrics(message):
        actual_publish_metrics(message)
        metrics.append(json.loads(message.data))
        first_metric.set()

    def release(actual_ticket, error):
        assert n._stream_condition._is_owned()
        assert n._stream_worker_busy is False
        result = actual_release(actual_ticket, error)
        releases.append(dict(request_id=actual_ticket.request_id, code=error.code,
            released=result, busy=n._stream_worker_busy,
            direct_slot=n._direct_near_field_submission_generation,
            source_watermark_ns=n.last_submitted_stamp_ns,
            phase_id=n._near_field_phase_id,
            phase_deadline_sec=n._near_field_phase_deadline_sec,
            session_deadline_ns=n._observe_return_session['deadline_ns']))
        (first_release if actual_ticket.request_id == ticket.request_id
         else second_release).set()
        return result

    monkeypatch.setattr(n, '_prepare_and_predict', prepare)
    monkeypatch.setattr(n, '_observe_stream_prediction_failure', observe_failure)
    monkeypatch.setattr(n.pipeline_metrics_pub, 'publish', publish_metrics)
    monkeypatch.setattr(n, '_release_preparation_cancelled_direct_snapshot_locked', release)

    def run_original_worker():
        try:
            n._stream_worker_main()
        except BaseException as exc:
            worker_errors.append(exc)
        finally:
            worker_finished.set()

    worker = threading.Thread(target=run_original_worker, name='gen42-original-stream-worker')
    n._stream_worker = worker
    worker.start()
    try:
        assert first_metric.wait(30.), dict(worker_errors=[repr(e) for e in worker_errors])
        assert metrics[0]['status'] == 'TARGET_PROCESSING_PENDING', metrics[0]
        assert first_release.wait(3.), 'real worker did not run cancellation cleanup'
        with n._stream_condition:
            assert releases[0]['released'] is True
            assert n._stream_worker_busy is False
            assert n._direct_near_field_submission_generation is None
            assert int(ticket.request_id) not in n._station_compute_work
            assert n._station_compute_reference['contract'] == ctx.contract
            assert n._stream_generation == ctx.generation
            assert n._near_field_phase_id == ctx.phase
            _assert_local_preparation_preserved(ctx)
        assert not failures and not worker_errors
        assert n._poll_direct_snapshot() is False, 'old submitted sources were reused'

        for index in range(3):
            row = _fresh_triplet(c)
            wire._deliver(c, row, ('mask', 'object', 'status'))
            admitted = n._poll_direct_snapshot()
            poll_results.append(dict(source_ns=row['status']['source_stamp_ns'], admitted=admitted))
            assert admitted is (index == 2), getattr(n, '_direct_snapshot_last_rejection', None)
        assert second_claimed.wait(3.), 'original worker never claimed the new measured ticket'
        assert second_release.wait(3.), 'bounded second preparation did not finish'
        with n._stream_condition:
            assert len(newer_tickets) == 1 and newer_tickets[0]['private_certificate']
            assert len(newer_tickets[0]['source_stamps_ns']) == 3
            assert all(stamp > ctx.watermark for stamp in newer_tickets[0]['source_stamps_ns'])
            assert n._pipeline_counters['submitted'] == 2
            assert n.last_submitted_stamp_ns > ctx.watermark
            assert n._stream_worker_busy is False and releases[-1]['released'] is True
            assert n._near_field_phase_id == ctx.phase
            assert n._near_field_phase_deadline_sec == ctx.phase_deadline
            assert n._observe_return_session['deadline_ns'] == ctx.session_deadline
            assert n._current_stream_target_identity() == ctx.identity
        assert not failures and not ctx.writer_errors and not worker_errors
        assert ctx.counters['candidates'] == ctx.counters['paths'] == ctx.counters['learned'] == ctx.counters['accept'] == 0
        assert {name: len(getattr(n, name).messages) for name in ctx.channels} == ctx.publications
        assert len(c.motion) == 1
        assert all(m['status'] == 'TARGET_PROCESSING_PENDING' for m in metrics)
        (tmp_path/'gen42_real_worker_owner_recovery.json').write_text(json.dumps(dict(
            scope='original worker cleanup and genuinely newer source admission', race=race,
            physical_grasp_verified=False, second_grasp_plan_generated=False,
            counters=ctx.counters, releases=releases, metrics=metrics,
            original_contract=ctx.contract, new_tickets=newer_tickets,
            poll_results=poll_results, sdk_provider=c.gen41_sdk_provider,
            no_candidate_path_or_publication=True, no_deadline_extension=True,
            source_watermark_not_rewound=True, worker_errors=[], writer_errors=[]),
            default=str, indent=2)+'\n')
    finally:
        n.shutdown_streaming_worker(timeout_sec=3.)
        worker.join(3.)
        assert worker_finished.is_set() and not worker.is_alive()
        _join_common_writers(ctx)


def test_failed_strict_policy_read_cannot_clear_another_box_cache_writer(
        monkeypatch, tmp_path):
    """The live policy stays hard while a concurrent newer cache survives."""
    ctx = _original_prepare_race(monkeypatch, tmp_path, 'policy_failure')
    n, snapshot = ctx.node, ctx.ticket.payload[0]
    geometry, _depth, transform = n._prepare_snapshot_geometry(
        snapshot, wire._at(snapshot.stamp_ns))
    assert geometry.ok
    n._prepare_regular_box_snapshot(snapshot, geometry, transform)
    previous = n._regular_box_snapshot
    assert previous is not None and previous['result'].ok
    competing = dict(previous)
    done = threading.Event()
    errors, writers, policy_calls = [], [], []

    def unavailable(name, *args, **kwargs):
        policy_calls.append(name)
        assert name == '/grasp/unknown_require_measured_bilateral_contact'

        def commit():
            try:
                with n._geometry_state_guard():
                    n._regular_box_snapshot = competing
            except BaseException as exc:
                errors.append(exc)
            finally:
                done.set()
        thread = threading.Thread(target=commit, name='gen42-strict-policy-cache-writer')
        writers.append(thread)
        thread.start()
        assert done.wait(3.), 'strict policy RPC retained the geometry lock'
        assert not errors
        raise RuntimeError('explicit unavailable strict-policy transport provider')

    monkeypatch.setattr(base.rospy, 'get_param_cached', unavailable)
    try:
        with pytest.raises(base.CandidateContractError) as rejected:
            n._prepare_regular_box_snapshot(snapshot, geometry, transform)
        assert rejected.value.code == 'GRASP_CONFIG_UNAVAILABLE'
        assert policy_calls == ['/grasp/unknown_require_measured_bilateral_contact']
        assert n._regular_box_snapshot is competing
        assert n._latest_target_observation is competing['observation']
        assert ctx.counters['candidates'] == ctx.counters['paths'] == ctx.counters['learned'] == ctx.counters['accept'] == 0
        (tmp_path/'gen42_policy_failure_cache_owner.json').write_text(json.dumps(dict(
            policy_code=rejected.value.code, policy_calls=policy_calls,
            old_cache_writer_superseded=True, new_cache_writer_preserved=True,
            physical_grasp_verified=False, strict_live_policy_defaulted=False), indent=2)+'\n')
    finally:
        for thread in writers:
            thread.join(3.)
            assert not thread.is_alive()
        _join_common_writers(ctx)


@pytest.mark.parametrize('conflict', [None, 'new_owner', 'target_epoch'])
def test_original_common_prepare_keeps_only_coherent_registration_after_full_enclosure_rejection(
        monkeypatch, tmp_path, conflict):
    """A downstream rejection retains measurements, never an executable plan.

    The actual full enclosure first passes unchanged mathematical checks. An
    explicit negative boundary simulator then rejects it to exercise failure
    delivery. This is not evidence that these fixture points exceed a real
    extent bound; the unchanged historical bound test covers that predicate.
    """
    ctx = _original_prepare_race(monkeypatch, tmp_path, 'enclosure_rejection')
    n, snapshot = ctx.node, ctx.ticket.payload[0]
    actual_enclose = base.enclose_registered_surface
    actual_prepare_geometry = n._prepare_snapshot_geometry
    before_surface = n._near_field_fused_surface
    before_box = n._regular_box_snapshot
    finished = threading.Event()
    writers, writer_errors, expected_owners = [], [], []
    post_geometry_owners = []
    enclosed_inputs = []

    def observe_geometry(*args, **kwargs):
        result = actual_prepare_geometry(*args, **kwargs)
        post_geometry_owners.append(_owners(n))
        return result

    def reject_after_original_enclosure(geometry, surface, **kwargs):
        result = actual_enclose(geometry, surface, **kwargs)
        assert result[0].ok
        enclosed_inputs.append(dict(source_ns=snapshot.stamp_ns,
            view_stamps_ns=list(surface.view_stamps_ns),
            original_enclosure_passed=True))
        if conflict is not None:
            def change_owner():
                try:
                    with n._stream_condition, n._geometry_state_guard():
                        if conflict == 'new_owner':
                            n._latest_target_observation = None
                            n._latest_raw_geometry_source = dict(
                                diagnostic_only=True, fixture_competing_cache_owner=True,
                                qualified_measurement_claimed=False)
                        else:
                            n._advance_target_instance_epoch('TARGET_INSTANCE_CHANGED')
                        expected_owners.append(_owners(n))
                except BaseException as exc:
                    writer_errors.append(exc)
                finally:
                    finished.set()
            thread = threading.Thread(target=change_owner,
                name='gen42-rejected-envelope-'+conflict)
            writers.append(thread)
            thread.start()
            assert finished.wait(3.), 'enclosure rejection retained commit locks'
            assert not writer_errors
        raise ValueError('explicit downstream envelope-rejection simulator after original calculation')

    monkeypatch.setattr(n, '_prepare_snapshot_geometry', observe_geometry)
    monkeypatch.setattr(base, 'enclose_registered_surface', reject_after_original_enclosure)
    try:
        with pytest.raises((base.CandidateContractError, base.StreamResultCancelled)) as rejected:
            ctx.actual_prepare(ctx.ticket)
        assert len(enclosed_inputs) == 1
        assert ctx.counters['geometry'] == 1
        assert ctx.counters['candidates'] == ctx.counters['paths'] == ctx.counters['learned'] == ctx.counters['accept'] == 0
        if conflict is None:
            assert isinstance(rejected.value, base.CandidateContractError)
            assert rejected.value.code == 'FUSED_SCENE_GEOMETRY_INVALID'
            observation = n._latest_target_observation
            evidence = n._latest_registration_evidence
            assert observation.stamp_ns == snapshot.stamp_ns
            assert evidence.result.ok and evidence.stamp_ns == snapshot.stamp_ns
            assert evidence.identity == observation.identity == snapshot.target_identity
            assert n._latest_raw_geometry_source['snapshot'] is snapshot
            assert n._latest_raw_geometry_source['observation'] is observation
            assert n._near_field_fused_surface.view_stamps_ns == (
                before_surface.view_stamps_ns+(snapshot.stamp_ns,))
            assert n._latest_fused_scene_audit is None
            assert n._regular_box_snapshot is before_box
            assert n._current_inferred_box() is None
            choice = n._latest_box_registration_choice
            if choice is not None:
                assert choice['observation'] is observation
                assert choice['lifecycle'] == n._multiview_lifecycle_token()
        elif conflict == 'new_owner':
            assert isinstance(rejected.value, base.StreamResultCancelled)
            assert rejected.value.code == 'TARGET_PROCESSING_PENDING'
            for name, owner in expected_owners[0].items():
                assert getattr(n, name, None) is owner, 'partial failure overwrote '+name
        else:
            assert isinstance(rejected.value, base.StreamResultCancelled)
            assert rejected.value.code != 'TARGET_PROCESSING_PENDING'
            assert post_geometry_owners
            # Inspect immediately after the original geometry method, before
            # the common entry's required hard revocation/retirement proceeds.
            for name, owner in expected_owners[0].items():
                assert post_geometry_owners[0][name] is owner, 'revoked partial result wrote '+name
        if conflict != 'target_epoch':
            assert {name: len(getattr(n, name).messages) for name in ctx.channels} == ctx.publications
        else:
            assert all(not message.valid for name in ctx.channels
                if name in ('geometry_pub', 'preview_rich_plan_pub', 'rich_plan_pub')
                for message in getattr(n, name).messages[ctx.publications[name]:])
        assert len(ctx.c.motion) == 1
        (tmp_path/'gen42_partial_registration_rejection.json').write_text(json.dumps(dict(
            conflict=conflict, result_code=rejected.value.code,
            counters=ctx.counters, enclosure_inputs=enclosed_inputs,
            full_enclosure_rejection_provider='explicit negative simulator after unchanged enclosure',
            actual_extent_bound_failure_claimed=False,
            coherent_registration_retained=conflict is None,
            no_candidate_or_motion_authorized=True,
            physical_grasp_verified=False), indent=2)+'\n')
    finally:
        for thread in writers:
            thread.join(3.)
            assert not thread.is_alive()
        _join_common_writers(ctx)
