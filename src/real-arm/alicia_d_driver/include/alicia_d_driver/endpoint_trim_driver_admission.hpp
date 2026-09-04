#ifndef ALICIA_D_DRIVER_ENDPOINT_TRIM_DRIVER_ADMISSION_HPP
#define ALICIA_D_DRIVER_ENDPOINT_TRIM_DRIVER_ADMISSION_HPP

#include "alicia_d_driver/endpoint_trim_continuity.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

// Orders accepted upstream targets and release requests without sharing the
// coordinator's rebased reference as upstream-command identity.
class EndpointTrimCommandOrder {
public:
    EndpointTrimCommandOrder(
        size_t joint_count = 6,
        double change_tolerance_rad = 2.0 * M_PI / 4096.0)
        : joint_count_(joint_count),
          change_tolerance_rad_(change_tolerance_rad)
    {
    }

    bool observe_upstream_command(
        const std::vector<double>& target,
        bool force_handoff)
    {
        if (target.size() != joint_count_) {
            return false;
        }
        for (const double value : target) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        bool changed = force_handoff ||
            upstream_target_.size() != target.size();
        if (!changed) {
            for (size_t i = 0; i < target.size(); ++i) {
                if (std::abs(target[i] - upstream_target_[i]) >
                        change_tolerance_rad_) {
                    changed = true;
                    break;
                }
            }
        }
        if (changed) {
            upstream_target_ = target;
            command_generation_ = next_generation();
        }
        return changed;
    }

    void mark_command_applied()
    {
        applied_command_generation_ = command_generation_;
    }

    void note_release()
    {
        release_generation_ = next_generation();
    }

    bool has_unapplied_command() const
    {
        return command_generation_ != applied_command_generation_;
    }

    bool newer_command_requires_handoff() const
    {
        return has_unapplied_command() &&
            command_generation_ > release_generation_;
    }

    const std::vector<double>& upstream_target() const
    {
        return upstream_target_;
    }

    uint64_t command_generation() const
    {
        return command_generation_;
    }

    uint64_t release_generation() const
    {
        return release_generation_;
    }

    void reset()
    {
        sequence_ = 0;
        command_generation_ = 0;
        applied_command_generation_ = 0;
        release_generation_ = 0;
        upstream_target_.clear();
    }

private:
    uint64_t next_generation()
    {
        if (sequence_ == std::numeric_limits<uint64_t>::max()) {
            // This cannot be reached at physical command rates, but fail
            // closed instead of allowing a wrapped command to outrank release.
            return sequence_;
        }
        return ++sequence_;
    }

    size_t joint_count_;
    double change_tolerance_rad_;
    uint64_t sequence_ = 0;
    uint64_t command_generation_ = 0;
    uint64_t applied_command_generation_ = 0;
    uint64_t release_generation_ = 0;
    std::vector<double> upstream_target_;
};

struct EndpointTrimTransmissionGate {
    bool motion_enabled;
    bool actuation_overheat_blocked;
    bool protection_latched;
    bool pause_when_feedback_stale;
    bool feedback_ready;
    bool feedback_stale;

    bool allows_correction() const
    {
        return motion_enabled &&
            !actuation_overheat_blocked &&
            !protection_latched &&
            (!pause_when_feedback_stale ||
             (feedback_ready && !feedback_stale));
    }
};

inline std::vector<double> endpoint_trim_stream_target(
    const std::vector<double>& fallback,
    const EndpointTrimDecision& decision)
{
    if (decision.phase != EndpointTrimPhase::IDLE &&
        decision.composed_target.size() == fallback.size()) {
        return decision.composed_target;
    }
    return fallback;
}

#endif  // ALICIA_D_DRIVER_ENDPOINT_TRIM_DRIVER_ADMISSION_HPP
