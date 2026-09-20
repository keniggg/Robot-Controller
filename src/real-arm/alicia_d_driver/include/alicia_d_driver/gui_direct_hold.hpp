#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include "alicia_d_driver/sdk_position_codec.hpp"

inline bool joint_source_allowed_by_control_mode(bool direct_mode, const std::string& source)
{
    const bool slider = source == "gui_direct" || source == "gui_direct_sync";
    return direct_mode ? slider : !slider;
}

inline bool controller_handoff_matches_feedback(
    const std::vector<double>& command,
    const std::vector<double>& feedback,
    bool feedback_is_fresh)
{
    if (!feedback_is_fresh || command.size() != 6 || feedback.size() != 6) {
        return false;
    }
    for (size_t i = 0; i < command.size(); ++i) {
        if (!std::isfinite(command[i]) || !std::isfinite(feedback[i]) ||
            std::abs(command[i] - feedback[i]) > 0.003) {
            return false;
        }
    }
    return true;
}

// With an established SDK hold, an ownership handoff must not change any
// transmitted position word. A servo's measured residual is not a new goal.
// An absent frame is the separate initial/reset epoch; a present but invalid,
// stale or non-frozen frame must never fall back to encoder initialization.
inline bool controller_handoff_matches_frozen_sdk_or_initial(
    const std::vector<double>& command,
    double command_gripper_rad,
    const std::vector<double>& feedback,
    bool feedback_is_fresh,
    const std::vector<uint8_t>& last_written_frame,
    bool retained_frozen_and_fresh)
{
    if (!feedback_is_fresh || command.size() != 6 || feedback.size() != 6 ||
        !std::isfinite(command_gripper_rad)) {
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(command[i]) || !std::isfinite(feedback[i]) ||
            command[i] < -M_PI || command[i] > M_PI) {
            return false;
        }
    }
    if (last_written_frame.empty()) {
        return controller_handoff_matches_feedback(
            command, feedback, feedback_is_fresh);
    }
    if (!retained_frozen_and_fresh) {
        return false;
    }
    std::vector<double> retained;
    double retained_gripper_rad = 0.0;
    if (!decode_retained_sdk_hold(last_written_frame, retained, retained_gripper_rad)) {
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        if (sdk_joint_position_encode(command[i]) !=
            sdk_joint_position_encode(retained[i])) {
            return false;
        }
    }
    return sdk_gripper_position_encode(command_gripper_rad) ==
           sdk_gripper_position_encode(retained_gripper_rad);
}

// Dedicated no-new-reference messages must belong to a positively established
// command epoch. Compare ROS nanoseconds as integers: a double timestamp loses
// the one-nanosecond old-queue boundary at current epoch magnitudes.
inline uint64_t reference_epoch_next_stamp_ns(uint64_t now_ns, uint64_t previous_ns)
{
    return now_ns > previous_ns ? now_ns : previous_ns + 1;
}

inline bool reference_sync_stamp_in_current_epoch(
    uint64_t message_ns, uint64_t reset_ns, uint64_t ownership_ns, uint64_t now_ns)
{
    return message_ns > 0 && reset_ns > 0 &&
        message_ns >= reset_ns && message_ns >= ownership_ns && message_ns <= now_ns;
}

// Unlike ordinary tracking commands, a reference-only message ALWAYS checks
// the current successful wire words. It may not become a new motion target if
// the ownership gate was admitted while that message was queued.
inline bool reference_sync_matches_successful_sdk_or_initial(
    const std::vector<double>& command,
    double command_gripper_rad,
    const std::vector<double>& feedback,
    bool feedback_is_fresh,
    const std::vector<uint8_t>& last_written_frame,
    bool successful_sdk_is_fresh,
    const std::string& actuation_status)
{
    const bool initialization_pending =
        actuation_status == "PENDING:POSITIVE_ENABLE_REQUESTED" ||
        actuation_status == "PENDING:COMMAND_SYNCHRONIZED";
    if ((!initialization_pending && actuation_status.compare(0, 10, "CONFIRMED:") != 0) ||
        (last_written_frame.empty() && !initialization_pending)) {
        return false;
    }
    return controller_handoff_matches_frozen_sdk_or_initial(
        command, command_gripper_rad, feedback, feedback_is_fresh,
        last_written_frame, successful_sdk_is_fresh);
}

// A position-controlled servo can hold a repeatable measured offset from its
// SDK setpoint.  Replacing an unedited joint's live SDK setpoint with its raw
// encoder value therefore creates a real target change and can look like
// multi-joint coupling.  Preserve the most recently transmitted setpoint when
// it is current; fall back to measured feedback only before a usable command
// has been sent.
inline std::vector<double> select_gui_direct_hold_reference(
    const std::vector<double>& feedback_joint_angles,
    const std::vector<double>& streamed_joint_angles,
    bool streamed_reference_is_fresh
)
{
    const bool streamed_reference_is_valid =
        streamed_reference_is_fresh &&
        !feedback_joint_angles.empty() &&
        streamed_joint_angles.size() == feedback_joint_angles.size() &&
        std::all_of(
            streamed_joint_angles.begin(),
            streamed_joint_angles.end(),
            [](double value) { return std::isfinite(value); }
        );
    return streamed_reference_is_valid
        ? streamed_joint_angles
        : feedback_joint_angles;
}

// One-hot intent describes which channel THIS message edits, not which joint
// may move. Previously admitted manual goals survive slider changes/releases
// and gesture timeouts, including goals the interpolator has not yet reached.
// The caller clears manual_target at every ownership/actuation epoch change.
inline std::vector<double> select_gui_direct_command_reference(
    const std::vector<double>& feedback,
    const std::vector<double>& streamed,
    bool streamed_valid,
    const std::vector<double>& manual_target)
{
    if (manual_target.size() == 6 && std::all_of(
            manual_target.begin(), manual_target.end(),
            [](double v) { return std::isfinite(v); })) {
        return manual_target;
    }
    return select_gui_direct_hold_reference(feedback, streamed, streamed_valid);
}
