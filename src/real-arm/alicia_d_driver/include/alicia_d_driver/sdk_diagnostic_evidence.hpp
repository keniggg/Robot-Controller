#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

// Official Alicia-D-SDK cb6c456a76a0238b98a3c1d77934e15b5da4659c,
// servo_driver.py INFO_COMMANDS['version']. This is a read-only whitelist,
// NOT an interface for arbitrary commands (temperature writes share CMD 06).
inline const std::vector<uint8_t>& sdk_readonly_version_query()
{
    static const std::vector<uint8_t> frame{0xAA, 0x01, 0x00, 0x01, 0xFE, 0x23, 0xFF};
    return frame;
}

enum class SdkReadonlyMotionQuery { None, Velocity, SelfCheck };

inline const std::vector<uint8_t>& sdk_readonly_motion_query(SdkReadonlyMotionQuery query)
{
    // Exact SDK cb6c456 / ROS1 eb703102 read-only frames. No arbitrary payload.
    static const std::vector<uint8_t> velocity{0xAA, 0x06, 0x02, 0x01, 0xFE, 0xF4, 0xFF};
    static const std::vector<uint8_t> self_check{0xAA, 0xFE, 0x00, 0x00, 0xFE, 0x93, 0xFF};
    static const std::vector<uint8_t> none;
    if (query == SdkReadonlyMotionQuery::Velocity) return velocity;
    if (query == SdkReadonlyMotionQuery::SelfCheck) return self_check;
    return none;
}

// Caller holds its mutex. Times MUST come from a monotonic host clock, not ROS
// simulated time. Each explicit request yields at most two writes in separate
// existing quiet poll slots, expires in 5 s and is never retried on write error.
class SdkReadonlyMotionBatch
{
public:
    // Drop unsent observations on transport loss; keep the request rate limit.
    void cancel() { pending_ = 0; }

    bool request(double now)
    {
        if (!std::isfinite(now) || now <= 0 || now < last_request_ + 5.0) return false;
        last_request_ = now;
        pending_ = 3;
        return true;
    }

    SdkReadonlyMotionQuery take(double now, bool eligible)
    {
        if (!std::isfinite(now) || now < last_request_ || now >= last_request_ + 5.0) {
            pending_ = 0;
            return SdkReadonlyMotionQuery::None;
        }
        if (!eligible || !pending_) return SdkReadonlyMotionQuery::None;
        if (pending_ & 1) {
            pending_ &= ~1u;
            return SdkReadonlyMotionQuery::Velocity;
        }
        pending_ = 0;
        return SdkReadonlyMotionQuery::SelfCheck;
    }

private:
    double last_request_ = -5.0;
    unsigned pending_ = 0;
};

// Raw ten-servo words, deliberately NOT sensor_msgs/JointState velocities.
// Firmware sign encoding and channel-to-joint map are not established here.
inline bool sdk_decode_follower_velocity_words(const std::vector<uint8_t>& payload,
                                               std::vector<uint16_t>& result)
{
    if (payload.size() != 20) return false;
    std::vector<uint16_t> words;
    for (size_t i = 0; i < 20; i += 2) {
        words.push_back(static_cast<uint16_t>(payload[i]) |
                        (static_cast<uint16_t>(payload[i + 1]) << 8));
    }
    result = words;
    return true;
}

struct SdkSelfCheckEvidence
{
    uint16_t raw_mask = 0;
    uint16_t abnormal_channel_bits = 0;
    uint16_t reserved_bits = 0;
};

inline bool sdk_decode_follower_self_check(const std::vector<uint8_t>& payload,
                                         SdkSelfCheckEvidence& result)
{
    if (payload.size() != 2) return false;
    SdkSelfCheckEvidence decoded;
    decoded.raw_mask = static_cast<uint16_t>(payload[0]) |
                       (static_cast<uint16_t>(payload[1]) << 8);
    decoded.abnormal_channel_bits = (~decoded.raw_mask) & 0x03FF;
    decoded.reserved_bits = decoded.raw_mask & 0xFC00;
    result = decoded;
    return true;
}

struct SdkDeviceIdentity
{
    std::string serial;
    uint32_t hardware_raw = 0;
    uint32_t firmware_raw = 0;
};

// Raw integers are authoritative. Do not guess compatibility or silently
// interpret an extended/unknown layout as this documented 24-byte layout.
inline bool sdk_decode_device_identity(const std::vector<uint8_t>& data,
                                       SdkDeviceIdentity& result)
{
    if (data.size() != 24) return false;
    SdkDeviceIdentity decoded;
    bool padding = false;
    for (size_t i = 0; i < 16; ++i) {
        const auto byte = data[i];
        if (byte == 0) { padding = true; continue; }
        if (byte < 0x20 || byte > 0x7E || (padding && byte != ' ')) return false;
        if (!padding) decoded.serial.push_back(static_cast<char>(byte));
    }
    while (!decoded.serial.empty() && decoded.serial.back() == ' ') decoded.serial.pop_back();
    if (decoded.serial.empty()) return false;
    for (size_t i = 0; i < 4; ++i) {
        decoded.hardware_raw |= static_cast<uint32_t>(data[16 + i]) << (8 * i);
        decoded.firmware_raw |= static_cast<uint32_t>(data[20 + i]) << (8 * i);
    }
    result = decoded;
    return true;
}

struct SdkTemperatureEvidence
{
    bool complete_follower_layout = false;
    bool all_channels_accepted = false;
    bool raw_over_limit = false;
    unsigned raw_max_c = 0;
    std::string channel_quality;

    bool healthy_sample() const
    {
        return complete_follower_layout && all_channels_accepted && !raw_over_limit;
    }
};

// Evidence only: no torque, enable, command permission, or motor/joint mapping.
// A rejected channel is UNKNOWN, never evidence of a cool/healthy channel.
inline SdkTemperatureEvidence sdk_temperature_evidence(
    const std::vector<uint8_t>& raw, const std::vector<float>& filtered,
    double limit_c)
{
    SdkTemperatureEvidence result;
    result.complete_follower_layout = raw.size() == 10;
    result.all_channels_accepted = !raw.empty() && raw.size() == filtered.size();
    result.raw_over_limit = !std::isfinite(limit_c) || limit_c <= 0;
    for (size_t i = 0; i < raw.size(); ++i) {
        result.raw_max_c = raw[i] > result.raw_max_c ? raw[i] : result.raw_max_c;
        result.raw_over_limit = result.raw_over_limit || raw[i] >= limit_c;
        const bool accepted = i < filtered.size() && std::isfinite(filtered[i]) &&
                              filtered[i] == static_cast<float>(raw[i]);
        result.all_channels_accepted = result.all_channels_accepted && accepted;
        if (i) result.channel_quality += ',';
        result.channel_quality += accepted ? "accepted" : "unknown_rejected";
    }
    return result;
}
