"""Bounded stale ROS registration handling for the reviewed visual recovery.

No imports of rospy, no process signals, no registration writes. The caller must
retain the original validated failed-deployment identity; a connection failure
alone is never evidence that a process is dead. This helper does not authorize
starting a duplicate process or resuming an unverified deployment.
"""

VISUAL_NODES = frozenset(('/supervisor_camera', '/unknown_tabletop_perception',
                          '/remote_grasp6d_node', '/motion_gateway'))


def registered_or_pending(rospy, name, original_identity, helper):
    """Return live registered PID, or None for absent/dead-old stale endpoint.

Only ECONNREFUSED's concrete ConnectionRefusedError is recoverable after the
existing identity helper proves the original PID/start pair no longer lives.
Timeouts, other transport errors and an alive original owner remain failures.
"""
    if (name not in VISUAL_NODES or not isinstance(original_identity, dict)
            or type(original_identity.get('pid')) is not int or original_identity['pid'] <= 0
            or type(original_identity.get('start_ticks')) is not int
            or original_identity['start_ticks'] <= 0):
        raise RuntimeError('validated original visual identity required')
    try:
        return helper.node_pid(rospy, name)
    except helper.CameraRegistrationPending:
        return None
    except ConnectionRefusedError:
        if helper.alive(original_identity):
            raise
        return None
