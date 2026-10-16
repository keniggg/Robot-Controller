"""GEN22 source timing through real remote callbacks; no ROS/hardware calls."""
from copy import deepcopy
import itertools

import pytest

from test_station_compute_wait_remote import station, context, arm, clock, row_at, feedback
from test_unknown_pending_identity import deliver


# Recorded first invalidation and latest complete producer READY source. The
# recorder does not prove the remote's callback order; the test enumerates it.
OLD_SOURCE = 1791638797145746231
NEW_SOURCE = 1791638797473508119
INVALIDATION = 1791638798010373830


@pytest.mark.parametrize('order', list(itertools.permutations(('mask', 'object', 'status'))))
def test_complete_fresh_ready_precedes_old_clock_expiry(station, order):
    c = station; n = c.node
    parent = arm(c)
    start = parent['source_ns']
    new_ns = start + NEW_SOURCE - OLD_SOURCE
    row = row_at(c, new_ns, recovery=parent)
    feedback(c, new_ns-50_000_000)
    feedback(c, new_ns+50_000_000)
    # Two topics arrive while the previous measurement is still valid. The
    # third makes an exact fresh join at the recorded relative expiry instant.
    clock(c, start + 790_000_000)
    deliver(n, row, order[:-1])
    assert n._station_compute_reference['contract'] == parent
    before = len(n.geometry_pub.messages)
    clock(c, start + INVALIDATION - OLD_SOURCE)
    deliver(n, row, order[-1:])
    assert n._station_compute_reference['contract']['source_ns'] == new_ns
    assert not n._station_compute_pending
    assert not n._station_compute_retired
    assert not [m for m in n.geometry_pub.messages[before:]
                if m.failure_reason.startswith('TARGET_PROCESSING_PENDING')]


@pytest.mark.parametrize('missing', ['mask', 'object', 'status'])
def test_incomplete_fresh_ready_does_not_keep_old_geometry(station, missing):
    c = station; n = c.node; parent = arm(c)
    row = row_at(c, parent['source_ns'] + NEW_SOURCE - OLD_SOURCE, recovery=parent)
    feedback(c, row['status']['source_stamp_ns']-50_000_000)
    feedback(c, row['status']['source_stamp_ns']+50_000_000)
    clock(c, parent['source_ns'] + INVALIDATION - OLD_SOURCE)
    deliver(n, row, tuple(k for k in ('mask', 'object', 'status') if k != missing))
    assert n._station_compute_pending
    assert not n.geometry_pub.messages[-1].valid
    assert n._station_compute_reference['contract'] == parent


@pytest.mark.parametrize('bad', ['missing_recovery', 'expired_source', 'mask_hash',
                                  'low_confidence', 'true_loss', 'epoch', 'manual'])
def test_early_join_keeps_recovery_identity_and_original_measurement_gates(station, bad):
    c = station; n = c.node; parent = arm(c)
    ns = parent['source_ns'] + NEW_SOURCE - OLD_SOURCE
    row = row_at(c, ns, recovery=parent)
    if bad == 'low_confidence': row['object'].confidence=.01
    feedback(c, ns-50_000_000); feedback(c, ns+50_000_000)
    clock(c, parent['source_ns']+790_000_000)
    deliver(n, row, ('mask', 'object'))
    if bad == 'missing_recovery': row['status'].pop('station_compute_recovery')
    elif bad == 'mask_hash': row['status']['station_compute_mask']['sha256']='f'*64
    elif bad == 'true_loss': row['status']['target_lost']=True
    elif bad == 'epoch': n._measurement_remote_epoch_ns += 1
    elif bad == 'manual': c.params['/gui/joint_direct_mode']=True
    later = ns+801_000_000 if bad == 'expired_source' else parent['source_ns']+INVALIDATION-OLD_SOURCE
    clock(c, later)
    deliver(n, row, ('status',))
    assert n._station_compute_pending or n._station_compute_retired
    assert n._station_compute_reference is None or n._station_compute_reference['contract'] == parent
    assert not n.geometry_pub.messages[-1].valid
