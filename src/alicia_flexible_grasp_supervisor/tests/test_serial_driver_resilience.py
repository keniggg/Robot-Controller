#!/usr/bin/env python3
import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
DRIVER_SRC = ROOT / 'real-arm' / 'alicia_d_driver' / 'src'


def _function_body(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == '{':
            depth += 1
        elif char == '}':
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise AssertionError('function body not found: %s' % signature)


class SerialDriverResilienceTest(unittest.TestCase):
    def test_read_thread_does_not_call_full_disconnect_from_inside_itself(self):
        source = (DRIVER_SRC / 'serial_communicator.cpp').read_text()
        body = _function_body(source, 'void SerialCommunicator::read_thread_loop()')

        self.assertNotRegex(
            body,
            re.compile(r'\bdisconnect\s*\('),
            'read_thread_loop must not call disconnect(); self-disconnect races with writers and can crash',
        )
        self.assertIn('handle_read_error_disconnect()', body)

    def test_disconnect_closes_serial_port_under_serial_mutex(self):
        source = (DRIVER_SRC / 'serial_communicator.cpp').read_text()
        body = _function_body(source, 'void SerialCommunicator::disconnect()')

        self.assertIn('std::lock_guard<std::mutex> lock(serial_mutex_)', body)
        self.assertIn('serial_port_.close()', body)

    def test_driver_keeps_reconnect_timer_alive_after_successful_initial_connect(self):
        source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
        ctor_body = _function_body(source, 'AliciaDDriverNode::AliciaDDriverNode()')
        reconnect_body = _function_body(source, 'void AliciaDDriverNode::reconnect_callback')

        self.assertIn('reconnect_timer_ = nh_.createTimer', ctor_body)
        self.assertNotIn('reconnect_timer_.stop()', ctor_body)
        self.assertNotIn('reconnect_timer_.stop()', reconnect_body)


if __name__ == '__main__':
    unittest.main()
