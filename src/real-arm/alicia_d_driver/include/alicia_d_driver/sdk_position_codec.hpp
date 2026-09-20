#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

// Use the same codec for transmitted setpoints and retained holds. Truncation
// makes some encode(decode(count)) round trips lose a count through FP error.
inline uint16_t sdk_joint_position_encode(double radians)
{
    const double deg = std::max(-180.0, std::min(180.0, radians * 180.0 / M_PI));
    return static_cast<uint16_t>(std::max(0L, std::min(4095L,
        std::lround((deg + 180.0) / 360.0 * 4096.0))));
}

inline double sdk_joint_position_decode(uint16_t count)
{
    return (-180.0 + std::min<unsigned>(4095, count) / 4096.0 * 360.0) * M_PI / 180.0;
}

inline uint16_t sdk_gripper_position_encode(double radians)
{
    const double deg = std::max(0.0, std::min(100.0, radians * 180.0 / M_PI));
    return static_cast<uint16_t>(std::lround(deg * 10.0));
}

inline double sdk_gripper_position_decode(uint16_t count)
{
    return std::min<unsigned>(1000, count) / 10.0 * M_PI / 180.0;
}

// Only for our cached, successfully written SDK command, not serial input.
// Its lifetime ends on clear_retained_command_state (enable/reconnect/reset).
inline bool decode_retained_sdk_hold(
    const std::vector<uint8_t>& frame, std::vector<double>& joints, double& gripper)
{
    if (frame.size() != 34 || frame[0] != 0xAA || frame[1] != 0x06 ||
        frame[2] != 0x03 || frame[3] != 0x1C || frame[33] != 0xFF) {
        return false;
    }
    std::vector<double> decoded;
    for (size_t i = 0; i < 7; ++i) {
        const uint16_t count = frame[4 + i * 4] | (frame[5 + i * 4] << 8);
        if (count > (i < 6 ? 4095 : 1000)) return false;
        if (i < 6) decoded.push_back(sdk_joint_position_decode(count));
        else gripper = sdk_gripper_position_decode(count);
    }
    joints = decoded;
    return true;
}
