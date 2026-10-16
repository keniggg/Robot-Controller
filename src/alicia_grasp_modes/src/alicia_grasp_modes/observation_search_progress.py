"""Scheduling memory only: never cache a pose, geometry, IK or path result.

The same measured station can supply many fresh requests. Remembering which
construction families consumed checks prevents each request from restarting
the same prefix. A timeout remains unresolved; every family stays eligible.
"""
import json
import math


POLICY = 'same_station_observation_search_budget'
BUDGET_CODE = 'OBSERVATION_SEARCH_BUDGET_EXHAUSTED'
BUDGET_SOURCE = 'observation_search_budget_exhausted'


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def family_key(variant, pose_fingerprint):
    """Use constructor coordinates, never a distance tolerance around a pose.

    Fresh depth changes an endpoint slightly without changing its construction
    family. Missing constructor coordinates fall back to the exact pose, so
    unrelated endpoints cannot be folded into a guessed family.
    """
    audit = variant.get('observation_view', {})
    proposal = audit.get('proposal', {})
    if (variant.get('search_family') == 'joint_space_fk'
            and 'direction_gap_index' in proposal
            and 'exposure_target_noise_multiplier' in proposal):
        return canonical(['joint_space_fk', proposal['direction_gap_index'],
                          proposal['exposure_target_noise_multiplier']])
    fields = ('orientation_source', 'camera_distance_preference_index',
              'camera_roll_preference_index', 'camera_tilt_preference_index')
    if all(key in audit for key in fields):
        return canonical(['camera_orientation_lattice', *[audit[key] for key in fields]])
    return canonical(['exact_pose_fallback', str(pose_fingerprint)])


class SearchProgress:
    """Single worker's bounded, owner-specific scheduling facts.

    Callers serialize bind/start/finish with their existing stream lock and
    revalidate original station/ticket contracts. Tokens discard late results
    after an owner change. Counts affect ordering only, never admission.
    """
    def __init__(self):
        self.owner = None
        self.token = 0
        self.families = {}
        self.maximum_check_sec = 0.
        self.deadline_monotonic = 0.

    def bind(self, owner, *, remaining_sec, monotonic_now):
        key = canonical(owner)
        remaining, now = float(remaining_sec), float(monotonic_now)
        if not all(math.isfinite(x) for x in (remaining, now)):
            raise ValueError('finite original station budget required')
        if key != self.owner:
            self.owner = key
            self.token += 1
            self.families = {}
            self.maximum_check_sec = 0.
            self.deadline_monotonic = now + max(0., remaining)
        else:
            # A fresh frame cannot move the station's original deadline.
            self.deadline_monotonic = min(self.deadline_monotonic, now + max(0., remaining))
        return self.token

    def count(self, family):
        return self.families.get(family, {}).get('attempts', 0)

    def order(self, rows):
        """Rows are (family, current_positive_endpoint, original_rank, value)."""
        return tuple(row[3] for row in sorted(rows,
            key=lambda row: (self.count(row[0]), not row[1], row[2])))

    def start(self, token, family):
        if token != self.token:
            return False
        record = self.families.setdefault(family, dict(attempts=0, timeouts=0, last_code=''))
        record['attempts'] += 1
        return True

    def finish(self, token, family, *, elapsed_sec, code):
        elapsed = float(elapsed_sec)
        if token != self.token or family not in self.families:
            return False
        if not math.isfinite(elapsed) or elapsed < 0.:
            raise ValueError('finite nonnegative strict-check duration required')
        record = self.families[family]
        record['last_code'] = str(code)
        record['timeouts'] += int(code in ('OBSERVATION_PLANNING_TIMEOUT', 'MOVEIT_TIMEOUT'))
        self.maximum_check_sec = max(self.maximum_check_sec, elapsed)
        return True

    def reserve(self, *, remaining_sec, monotonic_now, planning_sec, publication_sec):
        values = tuple(float(v) for v in
            (remaining_sec, monotonic_now, planning_sec, publication_sec))
        if not all(math.isfinite(v) for v in values) or min(values[2:]) <= 0.:
            raise ValueError('finite positive planning/publication reserve required')
        remaining, now, planning, publication = values
        remaining = min(remaining, self.deadline_monotonic-now)
        # .25 is the unchanged strict service's discovery wait. Observed wall
        # duration includes gateway overhead; this estimate is not a latency SLA.
        estimate = max(planning + .25, self.maximum_check_sec)
        return dict(can_start=remaining > estimate + publication,
                    remaining_sec=max(0., remaining), estimated_strict_sec=estimate,
                    publication_reserve_sec=publication,
                    attempted_count=sum(r['attempts'] for r in self.families.values()),
                    timeout_count=sum(r['timeouts'] for r in self.families.values()))
