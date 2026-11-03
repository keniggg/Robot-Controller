"""Read complete parameter namespaces without rospy's partial subtree cache."""
from copy import deepcopy


def read_authoritative_mapping(master, name):
    """Use MasterProxy's direct RPC, never its cache-backed ``__getitem__``.

    A cached child can create a parent dictionary containing only that child.
    Reading that dictionary through get_param is not a complete namespace read.
    Do not repair the process cache here: readers keep their subscriptions and
    each caller receives its own complete, independently mutable snapshot.
    """
    if not isinstance(name, str) or not name.startswith('/'):
        raise ValueError('authoritative parameter name must be absolute')
    response = master.getParam(name)
    if not isinstance(response, (tuple, list)) or len(response) != 3:
        raise ValueError('invalid authoritative parameter response for '+name)
    code, message, value = response
    if type(code) is not int or code != 1:
        raise ValueError('authoritative parameter unavailable for %s: %s' % (name, message))
    if not isinstance(value, dict):
        raise ValueError('authoritative parameter must be a mapping: '+name)
    return deepcopy(value)
