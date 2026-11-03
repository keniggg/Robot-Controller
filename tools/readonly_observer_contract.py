"""Exclude only freshly checked, explicitly declared observer instances.

This module performs no ROS, process or filesystem I/O itself. Callbacks must
resolve the current process identity and check current source bytes at every
getSystemState boundary. Other master methods and all control graph entries
remain unchanged. Standard diagnostic service names are checked here; service
URI/type and arbitrary process behaviour are not inferred from those names.
"""
from copy import deepcopy
from urllib.parse import urlsplit


STANDARD_SERVICES = ('get_loggers', 'set_logger_level', 'tf2_frames')


class ReadonlyObserverViolation(ValueError):
    pass


def _require(ok, reason):
    if not ok:
        raise ReadonlyObserverViolation(reason)


def _name(value):
    _require(isinstance(value, str) and value.startswith('/') and len(value) > 1,
             'observer_name_invalid')
    parts = value[1:].split('/')
    _require(all(part and all(c.isascii() and (c.isalnum() or c == '_') for c in part)
                 for part in parts), 'observer_name_must_be_literal')
    return value


def _identity(value):
    _require(type(value) is dict and set(value) == {'pid', 'parent_pid', 'start_ticks'},
             'observer_identity_fields_invalid')
    result = {}
    for field, number in value.items():
        _require(type(number) is int or (type(number) is str and number.isascii()
                 and number.isdecimal()), 'observer_identity_value_invalid')
        result[field] = int(number)
        _require(result[field] > 0, 'observer_identity_value_invalid')
    return result


def _declarations(values):
    _require(type(values) in (list, tuple), 'observer_declarations_invalid')
    result = []
    for value in values:
        _require(type(value) is dict and set(value) == {
            'name', 'uri', 'identity', 'source_sha256', 'subscriptions', 'services'},
            'observer_declaration_fields_invalid')
        row = deepcopy(value)
        name = _name(row['name'])
        uri = row['uri']
        _require(type(uri) is str, 'observer_uri_invalid')
        parsed = urlsplit(uri)
        _require(parsed.scheme == 'http' and parsed.hostname and parsed.port
                 and parsed.path == '/' and not parsed.query and not parsed.fragment
                 and parsed.username is None and parsed.password is None,
                 'observer_uri_invalid')
        row['identity'] = _identity(row['identity'])
        sources = row['source_sha256']
        _require(type(sources) is dict and sources, 'observer_sources_missing')
        for path, digest in sources.items():
            _require(type(path) is str and path.startswith('/')
                     and all(p not in ('', '.', '..') for p in path[1:].split('/'))
                     and type(digest) is str and len(digest) == 64
                     and all(c in '0123456789abcdef' for c in digest),
                     'observer_source_pin_invalid')
        for field in ('subscriptions', 'services'):
            names = row[field]
            _require(type(names) in (list, tuple) and len(names) == len(set(names)),
                     'observer_endpoint_declaration_invalid')
            for endpoint in names:
                _name(endpoint)
            row[field] = tuple(names)
        _require(set(row['services']) <= {name+'/'+s for s in STANDARD_SERVICES},
                 'observer_service_declaration_not_standard')
        result.append(row)
    _require(len({r['name'] for r in result}) == len(result), 'duplicate_observer_name')
    _require(len({r['uri'] for r in result}) == len(result), 'duplicate_observer_uri')
    _require(len({(r['identity']['pid'], r['identity']['start_ticks']) for r in result})
             == len(result), 'duplicate_observer_process')
    return tuple(result)


def _state(value):
    _require(type(value) in (list, tuple) and len(value) == 3, 'master_state_invalid')
    for table in value:
        _require(type(table) in (list, tuple), 'master_table_invalid')
        for row in table:
            _require(type(row) in (list, tuple) and len(row) == 2
                     and type(row[0]) is str and type(row[1]) in (list, tuple)
                     and all(type(owner) is str for owner in row[1]),
                     'master_row_invalid')
    return deepcopy(value)


def _without(table, checked):
    rows = []
    for row in table:
        if not any(owner in checked for owner in row[1]):
            rows.append(row)
            continue
        owners = [owner for owner in row[1] if owner not in checked]
        if owners:
            rows.append(type(row)((row[0], type(row[1])(owners))))
    return type(table)(rows)


class _CheckedMaster:
    def __init__(self, master, caller, declarations, identity_resolver, source_checker):
        _require(type(caller) is str and caller, 'observer_checker_caller_missing')
        _require(callable(identity_resolver) and callable(source_checker),
                 'observer_checker_callbacks_missing')
        self._master = master
        self._declarations = _declarations(declarations)
        self._identity_resolver = identity_resolver
        self._source_checker = source_checker

    def __getattr__(self, name):
        return getattr(self._master, name)

    def _owner(self, row):
        _require(self._master.lookupNode(row['name']) == row['uri'],
                 'observer_uri_changed')
        current = _identity(self._identity_resolver(row['name']))
        _require(current == row['identity'], 'observer_process_identity_changed')

    def getSystemState(self):
        state = _state(self._master.getSystemState())
        checked = set()
        for row in self._declarations:
            name = row['name']
            owned = [[entry for entry in table if name in entry[1]] for table in state]
            if not any(owned):
                continue  # A departed observer is not a control dependency.
            _require(not owned[0], 'observer_has_publications')
            _require({x[0] for x in owned[1]} <= set(row['subscriptions']),
                     'observer_has_undeclared_subscription')
            _require({x[0] for x in owned[2]} <= set(row['services']),
                     'observer_has_undeclared_service')
            _require(all(list(x[1]) == [name] for x in owned[2]),
                     'observer_standard_service_owner_conflict')
            self._owner(row)
            _require(self._source_checker(deepcopy(row['source_sha256'])) is True,
                     'observer_current_source_check_failed')
            self._owner(row)  # Reject a replacement during source validation.
            checked.add(name)
        return type(state)((state[0], _without(state[1], checked),
                            _without(state[2], checked)))


def checked_master(master, caller, declarations, identity_resolver, source_checker):
    """Wrap an existing Master without changing its other methods.

    Declarations contain exact name, URI, pid/parent_pid/start_ticks, current
    source SHA256 map, data topic names and permitted standard service names.
    identity_resolver(name) returns those three process fields. source_checker
    receives the source map and must freshly verify it and return exactly True.
    Lookup/identity/source exceptions propagate; they never authorize filtering.
    """
    return _CheckedMaster(master, caller, declarations, identity_resolver, source_checker)
