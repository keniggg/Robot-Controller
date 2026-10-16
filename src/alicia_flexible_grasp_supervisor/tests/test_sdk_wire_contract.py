"""Compile the real wire-building slice offline; no ROS or serial client.

Protocol reference: Synria-Robotics/Alicia-D-SDK,
cb6c456a76a0238b98a3c1d77934e15b5da4659c, servo_driver.py.
The deliberate nearest-count encoder fix is NOT reverted to SDK truncation.
"""
import math
import re
import subprocess
import zlib

import pytest

from test_serial_driver_resilience import DRIVER_SRC, DRIVER_PACKAGE, _function_body


def compile_wire_encoder(directory):
    source = (DRIVER_SRC / 'alicia_d_driver_node.cpp').read_text()
    helpers = '\n'.join(signature + '{' + _function_body(source, signature) + '}'
                        for signature in (
                            'static uint8_t sdk_crc32_low8(const std::vector<uint8_t>& data)',
                            'static uint16_t speed_deg_s_to_hw(double speed_deg_s)',
                        ))
    constants = '\n'.join(re.search(r'constexpr uint8_t ' + name + r'\s*=.*?;', source).group()
                          for name in ('SDK_CMD_JOINT', 'SDK_FUNC_SET_JOINT_GRIPPER',
                                       'SDK_DATA_LEN_JOINT_GRIPPER'))
    body = _function_body(source, 'void AliciaDDriverNode::send_command_timer_callback')
    assembly = body[body.index('const size_t frame_size = 34;'):
                    body.index('if (suppress_redundant_commands_ && servo_frame ==')]
    conversion = _function_body(source, 'uint16_t AliciaDDriverNode::rad_to_hardware_value(')
    code = '''
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>
#include "alicia_d_driver/sdk_position_codec.hpp"
constexpr uint8_t FRAME_START_BYTE = 0xAA, FRAME_END_BYTE = 0xFF;
''' + constants + helpers + '\nuint16_t rad_to_hardware_value(double angle_rad) {' + conversion + '''}
int main() {
  std::vector<double> sdk_joint_angles(6);
  double cmd_gripper_rad_, joint_speed_deg_s_;
  while (std::cin >> sdk_joint_angles[0]) {
    for (int i = 1; i < 6; ++i) std::cin >> sdk_joint_angles[i];
    std::cin >> cmd_gripper_rad_ >> joint_speed_deg_s_;
''' + assembly + '''
    for (auto byte : servo_frame) std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(byte);
    std::cout << std::endl;
  }
}
'''
    cpp, executable = directory / 'actual_wire.cpp', directory / 'actual_wire'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++14', '-O2', '-I', str(DRIVER_PACKAGE / 'include'),
                    str(cpp), '-o', str(executable)], check=True, capture_output=True, text=True)
    return executable


def encode_cases(executable, cases):
    stdin = ''.join(' '.join(format(v, '.17g') for v in q + [grip, speed]) + '\n'
                    for q, grip, speed in cases)
    result = subprocess.run([str(executable)], input=stdin, text=True,
                            capture_output=True, check=True)
    return [bytes.fromhex(line) for line in result.stdout.splitlines()]


@pytest.fixture(scope='module')
def wire_encoder(tmp_path_factory):
    return compile_wire_encoder(tmp_path_factory.mktemp('wire_contract'))


def joint_words(frame):
    return [int.from_bytes(frame[4 + 4*i:6 + 4*i], 'little') for i in range(6)]


def test_actual_cpp_combined_frame_has_six_independent_joint_slots(wire_encoder):
    angles = [0, -math.pi/2, math.pi/2, -math.pi, math.pi, math.pi/4]
    frame, = encode_cases(wire_encoder, [(angles, math.radians(50), 15)])
    assert len(frame) == 34 and frame[:4] == bytes.fromhex('AA06031C') and frame[-1] == 0xFF
    assert joint_words(frame) == [2048, 1024, 3072, 0, 4095, 2560]
    assert [int.from_bytes(frame[6 + 4*i:8 + 4*i], 'little') for i in range(6)] == [150]*6
    assert int.from_bytes(frame[28:30], 'little') == 500
    assert int.from_bytes(frame[30:32], 'little') == 5500
    assert frame[-2] == zlib.crc32(frame[1:-2]) & 0xFF


def test_one_axis_change_does_not_change_other_position_or_speed_slots(wire_encoder):
    count = [831, 2361, 1794, 2050, 2215, 2052]
    decode = lambda values: [(v / 4096 * 360 - 180) * math.pi/180 for v in values]
    cases = [(decode(count), 0.5, 15)]
    for axis in range(6):
        changed = list(count)
        changed[axis] += 4
        cases.append((decode(changed), 0.5, 15))
    baseline, *frames = encode_cases(wire_encoder, cases)
    for axis, frame in enumerate(frames):
        assert joint_words(frame) == [c + (4 if j == axis else 0) for j, c in enumerate(count)]
        assert frame[28:32] == baseline[28:32]


def test_simultaneous_joint2_joint3_joint5_changes_all_reach_wire(wire_encoder):
    count = [831, 2361, 1794, 2050, 2215, 2052]
    changed = [c + (4 if i in (1, 2, 4) else 0) for i, c in enumerate(count)]
    q = [(v / 4096 * 360 - 180) * math.pi/180 for v in changed]
    frame, = encode_cases(wire_encoder, [(q, .5, 15)])
    assert joint_words(frame) == changed


@pytest.mark.parametrize('speed, expected', [(0.1, 50), (10, 100), (15, 150), (360, 4100), (1000, 5000)])
def test_speed_field_quantization_is_not_requested_trajectory_velocity(wire_encoder, speed, expected):
    frame, = encode_cases(wire_encoder, [([0]*6, 0, speed)])
    assert int.from_bytes(frame[6:8], 'little') == expected


def test_all_4096_returned_counts_round_trip_without_one_count_drift(wire_encoder):
    cases = [([(c / 4096 * 360 - 180) * math.pi/180]*6, 0, 15) for c in range(4096)]
    for c, frame in enumerate(encode_cases(wire_encoder, cases)):
        assert joint_words(frame) == [c]*6
        assert frame[-2] == zlib.crc32(frame[1:-2]) & 0xFF
