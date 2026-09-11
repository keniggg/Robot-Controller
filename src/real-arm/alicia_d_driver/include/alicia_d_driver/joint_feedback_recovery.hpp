#ifndef ALICIA_D_DRIVER_JOINT_FEEDBACK_RECOVERY_HPP
#define ALICIA_D_DRIVER_JOINT_FEEDBACK_RECOVERY_HPP

#include <cmath>
#include <vector>

namespace alicia_d_driver
{

// Repeated telemetry may recover a lost position only when the entire sample
// agrees with the last successfully transmitted target. The tolerance describes
// encoder/position agreement, independently of the serial polling interval.
// The caller still requires repeated samples and revokes actuation confirmation.
inline bool command_consistent_feedback_recovery(
    const std::vector<double>& previous,
    const std::vector<double>& candidate,
    const std::vector<double>& command,
    double tolerance_rad
)
{
    if (previous.empty() || previous.size() != candidate.size() ||
        candidate.size() != command.size() ||
        !std::isfinite(tolerance_rad) || tolerance_rad <= 0.0) {
        return false;
    }

    bool improved_discontinuity = false;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
        if (!std::isfinite(previous[i]) || !std::isfinite(candidate[i]) ||
            !std::isfinite(command[i])) {
            return false;
        }
        const double candidate_error = std::abs(candidate[i] - command[i]);
        if (candidate_error > tolerance_rad) {
            return false;
        }
        if (std::abs(candidate[i] - previous[i]) > tolerance_rad) {
            if (candidate_error + tolerance_rad >=
                std::abs(previous[i] - command[i])) {
                return false;
            }
            improved_discontinuity = true;
        }
    }
    return improved_discontinuity;
}

}  // namespace alicia_d_driver

#endif
