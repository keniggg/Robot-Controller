"""Pure graph fixtures; no ROS, process discovery or control operations."""
from copy import deepcopy
import unittest

from readonly_observer_contract import ReadonlyObserverViolation, checked_master


NAME = '/operator_capture_37'
URI = 'http://host:1234/'
IDENTITY = dict(pid=37, parent_pid=11, start_ticks='91')
DECLARATION = dict(name=NAME, uri=URI, identity=IDENTITY,
    source_sha256={'/project/tools/capture.py': 'a'*64},
    subscriptions=['/camera/rgb', '/camera/depth', '/accepted_joint_states'],
    services=[NAME+'/'+s for s in ('get_loggers', 'set_logger_level', 'tf2_frames')])


class Master:
    def __init__(self):
        self.state = [
            [['/camera/rgb', ['/camera']], ['/command', ['/controller']],
             ['/unexpected_publication', [NAME+'_other']]],
            [['/camera/rgb', ['/perception', NAME, NAME+'_other']],
             ['/camera/depth', [NAME]], ['/accepted_joint_states', ['/controller', NAME]],
             ['/command', ['/driver']]],
            [[s, [NAME]] for s in DECLARATION['services']]
            + [['/control_service', ['/controller']], ['/unknown_service', [NAME+'_other']]]]
        self.uri = URI
        self.calls = []

    def getSystemState(self):
        self.calls.append('state')
        return self.state

    def lookupNode(self, name):
        self.calls.append(('lookup', name))
        return self.uri

    def lookupService(self, name):
        return ('unchanged service method', name)


class ReadonlyObserverContractTests(unittest.TestCase):
    def setUp(self):
        self.master = Master()
        self.identity = deepcopy(IDENTITY)
        self.source_ok = True
        self.checks = []
        self.identities = []
        self.declarations = [deepcopy(DECLARATION)]
        self.proxy = checked_master(self.master, '/verification', self.declarations,
                                    self.resolve, self.check_sources)

    def resolve(self, name):
        self.identities.append(name)
        return deepcopy(self.identity)

    def check_sources(self, sources):
        self.checks.append(deepcopy(sources))
        return self.source_ok

    def test_only_exact_checked_subscription_and_service_owners_are_removed(self):
        before = deepcopy(self.master.state)
        result = self.proxy.getSystemState()
        self.assertEqual(result[0], before[0])
        self.assertEqual(result[1], [
            ['/camera/rgb', ['/perception', NAME+'_other']],
            ['/accepted_joint_states', ['/controller']], ['/command', ['/driver']]])
        self.assertEqual(result[2], [
            ['/control_service', ['/controller']], ['/unknown_service', [NAME+'_other']]])
        self.assertEqual(self.master.state, before)
        self.assertEqual(self.identities, [NAME, NAME])
        self.assertEqual(self.checks, [DECLARATION['source_sha256']])
        self.assertEqual(self.proxy.lookupService('/control_service'),
                         ('unchanged service method', '/control_service'))

    def test_departed_observer_does_not_become_a_control_dependency(self):
        remaining = self.proxy.getSystemState()
        self.master.state = remaining
        self.master.uri = 'http://stale:9999/'
        self.identity['start_ticks'] = '999'
        self.source_ok = False
        self.checks.clear()
        self.identities.clear()
        self.assertEqual(self.proxy.getSystemState(), remaining)
        self.assertEqual(self.checks, [])
        self.assertEqual(self.identities, [])

    def test_each_boundary_rechecks_graph_and_rejects_new_publication(self):
        self.proxy.getSystemState()
        self.master.state[0].append(['/command', [NAME]])
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'has_publications'):
            self.proxy.getSystemState()

    def test_undeclared_subscription_is_not_hidden(self):
        self.master.state[1].append(['/unreviewed_data', [NAME]])
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'undeclared_subscription'):
            self.proxy.getSystemState()

    def test_control_service_is_not_hidden(self):
        self.master.state[2].append(['/move_robot', [NAME]])
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'undeclared_service'):
            self.proxy.getSystemState()

    def test_standard_service_cannot_hide_a_second_provider(self):
        self.master.state[2][0][1].append('/controller')
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'service_owner_conflict'):
            self.proxy.getSystemState()

    def test_pid_parent_and_start_tick_changes_each_reject(self):
        for field in ('pid', 'parent_pid', 'start_ticks'):
            with self.subTest(field=field):
                self.identity = deepcopy(IDENTITY)
                self.identity[field] = int(self.identity[field])+1
                with self.assertRaisesRegex(ReadonlyObserverViolation, 'process_identity_changed'):
                    self.proxy.getSystemState()

    def test_same_name_with_changed_uri_rejects(self):
        self.master.uri = 'http://host:9999/'
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'uri_changed'):
            self.proxy.getSystemState()

    def test_current_source_check_is_not_cached(self):
        self.proxy.getSystemState()
        self.source_ok = False
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'source_check_failed'):
            self.proxy.getSystemState()
        self.assertEqual(len(self.checks), 2)

    def test_replacement_during_source_validation_rejects(self):
        def changed(sources):
            self.identity['start_ticks'] = '92'
            return True
        proxy = checked_master(self.master, '/verification', self.declarations,
                               self.resolve, changed)
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'process_identity_changed'):
            proxy.getSystemState()

    def test_checker_errors_propagate_without_filtering(self):
        before = deepcopy(self.master.state)
        def failed(sources):
            raise OSError('source unavailable')
        proxy = checked_master(self.master, '/verification', self.declarations,
                               self.resolve, failed)
        with self.assertRaisesRegex(OSError, 'source unavailable'):
            proxy.getSystemState()
        self.assertEqual(self.master.state, before)

    def test_declaration_cannot_whitelist_control_service_or_name_pattern(self):
        control = deepcopy(DECLARATION)
        control['services'].append(NAME+'/move')
        wildcard = deepcopy(DECLARATION)
        wildcard['name'] += '*'
        for declaration in (control, wildcard):
            with self.subTest(declaration=declaration['name']):
                with self.assertRaises(ReadonlyObserverViolation):
                    checked_master(self.master, '/verification', [declaration],
                                   self.resolve, self.check_sources)

    def test_declaration_copy_prevents_external_mutation_widening_scope(self):
        self.declarations[0]['subscriptions'].append('/new_data')
        self.declarations[0]['identity']['pid'] = 123
        self.master.state[1].append(['/new_data', [NAME]])
        with self.assertRaisesRegex(ReadonlyObserverViolation, 'undeclared_subscription'):
            self.proxy.getSystemState()

    def test_empty_declarations_leave_every_graph_item_unchanged(self):
        proxy = checked_master(self.master, '/verification', [], self.resolve,
                               self.check_sources)
        self.assertEqual(proxy.getSystemState(), self.master.state)
        self.assertEqual(self.identities, [])
        self.assertEqual(self.checks, [])


if __name__ == '__main__':
    unittest.main()
