#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

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
