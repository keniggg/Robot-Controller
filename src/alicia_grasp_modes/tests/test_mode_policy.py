import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes.policy import ModePolicy


def ready(mode='unknown', stamp=100):
    policy = ModePolicy(state_known=True)
    assert policy.begin_switch(mode, stamp) == 'SWITCHING'
    policy.finish_switch(True)
    return policy


def test_old_frames_and_wrong_source_never_cross_switch():
    policy = ready()
    assert not policy.admit_frame('unknown', 100)
    assert not policy.admit_frame('carton', 101)
    assert policy.admit_frame('unknown', 101)


def test_old_plan_at_same_position_cannot_start_new_mode():
    policy = ready('carton')
    policy.observe_plan('plan', True, 'old', 110, 'carton_segment', 'carton_segment')
    assert policy.begin_switch('unknown', 200) == 'SWITCHING'
    policy.finish_switch(True)
    policy.observe_plan('plan', True, 'late', 150, 'unknown_tabletop', 'unknown_tabletop')
    assert policy.reserve_start('old', True) == 'PLAN_NOT_BOUND_TO_CURRENT_MODE'
    assert policy.reserve_start('late', True) == 'PLAN_NOT_BOUND_TO_CURRENT_MODE'


def test_plan_model_and_execution_reservation():
    policy = ready()
    policy.observe_plan('plan', True, 'wrong', 110, 'carton_segment', 'unknown_tabletop')
    assert policy.reserve_start('wrong', True) == 'PLAN_NOT_BOUND_TO_CURRENT_MODE'
    policy.observe_plan('plan', True, 'new', 111, 'unknown_tabletop', 'unknown_tabletop')
    assert policy.reserve_start('new', False) == 'TARGET_UNAVAILABLE'
    assert policy.reserve_start('new', True) == ''
    assert policy.begin_switch('carton', 200) == 'QUEUED'
    assert policy.mode == 'unknown' and policy.generation == 1
    assert policy.reserve_start('new', True) == 'TASK_BUSY_OR_UNAVAILABLE'


def test_failed_switch_stays_unavailable_and_never_falls_back():
    policy = ready('carton')
    policy.begin_switch('unknown', 200)
    policy.finish_switch(False)
    assert policy.mode == 'unknown'
    assert not policy.admit_frame('carton', 250)
    assert not policy.admit_frame('unknown', 250)
    assert policy.reserve_start('anything', True) == 'MODE_NOT_READY'


def test_switch_back_creates_new_generation_and_clears_authority():
    policy = ready('carton')
    policy.begin_switch('unknown', 200)
    policy.finish_switch(True)
    policy.begin_switch('carton', 300)
    policy.finish_switch(True)
    assert policy.generation == 3
    assert not policy.admit_frame('carton', 250)
    assert policy.admit_frame('carton', 350)


def test_unknown_task_state_and_active_task_do_not_switch():
    policy = ModePolicy()
    assert policy.begin_switch('unknown', 100) == 'TASK_STATE_UNAVAILABLE'
    policy.state_known = True
    policy.active = True
    assert policy.begin_switch('unknown', 100) == 'QUEUED'
    assert policy.generation == 0


def test_strategy_change_invalidates_plans_even_when_target_mode_unchanged():
    policy = ready()
    policy.observe_plan('plan', True, 'observation', 110, 'unknown_tabletop', 'unknown_tabletop')
    assert policy.begin_switch('unknown', 200, 'direct') == 'SWITCHING'
    policy.finish_switch(True)
    assert policy.selection()['strategy'] == 'direct'
    assert policy.reserve_start('observation', True) == 'PLAN_NOT_BOUND_TO_CURRENT_MODE'
    assert not policy.admit_frame('unknown', 199)


def test_active_task_queues_target_and_strategy_as_one_selection():
    policy = ready()
    policy.active = True
    assert policy.begin_switch('carton', 200, 'direct') == 'QUEUED'
    assert (policy.mode, policy.strategy) == ('unknown', 'two_stage')
    assert (policy.pending, policy.pending_strategy) == ('carton', 'direct')
    policy.active = False
    assert policy.begin_switch(policy.pending, 300, policy.pending_strategy) == 'SWITCHING'
    assert (policy.mode, policy.strategy) == ('carton', 'direct')


def test_legacy_mode_service_keeps_selected_execution_strategy():
    policy = ready()
    policy.begin_switch('unknown', 200, 'direct')
    policy.finish_switch(True)
    policy.begin_switch('carton', 300)
    assert policy.strategy == 'direct'
