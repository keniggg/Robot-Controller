"""A bounded prefix of an already collision-checked observation trajectory."""
from copy import deepcopy
import math


PENDING_BOOTSTRAP_STATES = frozenset((
    'PENDING:POSITIVE_ENABLE_REQUESTED', 'PENDING:COMMAND_SYNCHRONIZED',
))


def bounded_observation_prefix(plan, max_delta_rad=0.025):
    if not math.isfinite(max_delta_rad) or not 0.023 <= max_delta_rad <= 0.025:
        raise ValueError('bootstrap joint bound must be within [0.023, 0.025] rad')
    result = deepcopy(plan)
    trajectory = result.joint_trajectory
    points = list(trajectory.points)
    if len(trajectory.joint_names) != 6 or len(set(trajectory.joint_names)) != 6:
        raise ValueError('bootstrap requires exactly six distinct arm joints')
    if len(points) < 2 or getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', []):
        raise ValueError('bootstrap requires a nonempty arm-only trajectory')
    for point in points:
        if len(point.positions) != 6 or not all(math.isfinite(v) for v in point.positions):
            raise ValueError('bootstrap trajectory positions are incomplete or nonfinite')
    origin = list(points[0].positions)
    prefix = [points[0]]
    for point in points[1:]:
        previous = prefix[-1].positions
        delta = [v - q for v, q in zip(point.positions, origin)]
        if max(abs(v) for v in delta) < max_delta_rad:
            prefix.append(point)
            continue
        # The clipped endpoint lies on the existing checked joint-space edge.
        fractions = []
        for old, new, start in zip(previous, point.positions, origin):
            if abs(new - start) >= max_delta_rad and new != old:
                boundary = start + math.copysign(max_delta_rad, new - start)
                fractions.append((boundary - old) / (new - old))
        fraction = min(fractions)
        if not 0.0 < fraction <= 1.0:
            raise ValueError('bootstrap prefix boundary is invalid')
        point.positions = [a + fraction * (b - a) for a, b in zip(previous, point.positions)]
        prefix.append(point)
        break
    endpoint = prefix[-1].positions
    if max(abs(a - b) for a, b in zip(endpoint, origin)) < 0.023 - 1e-9:
        raise ValueError('observation path too short to establish actuation response')
    # A small initial prefix must not reverse any joint or leave its endpoint
    # interval. Retiming then preserves this exact, bounded path geometry.
    for first, second in zip(prefix, prefix[1:]):
        for a, b, start, end in zip(first.positions, second.positions, origin, endpoint):
            if (b - a) * (end - start) < -1e-12 or not min(start, end)-1e-9 <= b <= max(start, end)+1e-9:
                raise ValueError('observation prefix reverses a joint')
    for point in prefix:
        point.velocities = []
        point.accelerations = []
        point.effort = []
    trajectory.points = prefix
    return result
