#ifndef ALICIA_D_DRIVER_ENDPOINT_TRIM_DRIVER_ADMISSION_HPP
#define ALICIA_D_DRIVER_ENDPOINT_TRIM_DRIVER_ADMISSION_HPP

#include "alicia_d_driver/endpoint_trim_continuity.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

enum class EndpointTrimCommandSource {
    TASK_CONTROLLER,
    GUI_DIRECT_EDIT,
    GUI_DIRECT_SYNC,
};

enum class EndpointTrimReleaseStatus {
    PENDING,
    COMPLETED,
    NOOP,
    REJECTED,
};

inline bool endpoint_trim_release_request_allowed(EndpointTrimPhase phase)
{
    return phase == EndpointTrimPhase::ACTIVE_READY ||
        phase == EndpointTrimPhase::WAITING_RESPONSE ||
        phase == EndpointTrimPhase::PENDING_RELEASE;
}

// Orders accepted upstream targets and release requests without sharing the
// coordinator's rebased reference as upstream-command identity.
class EndpointTrimCommandOrder {
public:
    EndpointTrimCommandOrder(
        size_t joint_count = 6,
        double change_tolerance_rad = 2.0 * M_PI / 4096.0,
        double gui_task_holdoff_sec = 0.25)
        : joint_count_(joint_count),
          change_tolerance_rad_(change_tolerance_rad),
          gui_task_holdoff_sec_(
              std::isfinite(gui_task_holdoff_sec) &&
                  gui_task_holdoff_sec >= 0.0
                  ? gui_task_holdoff_sec
                  : 0.25)
    {
    }

    bool observe_upstream_command(
        const std::vector<double>& target,
        EndpointTrimCommandSource source)
    {
        return observe_upstream_command(target, source, 0.0);
    }

    bool observe_upstream_command(
        const std::vector<double>& target,
        EndpointTrimCommandSource source,
        double now_sec)
    {
        last_observation_accepted_ = false;
        if (
            target.size() != joint_count_ ||
            !std::isfinite(now_sec) ||
            now_sec < 0.0
        ) {
            return false;
        }
        for (const double value : target) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        if (source == EndpointTrimCommandSource::TASK_CONTROLLER) {
            if (
                gui_task_holdoff_active_ &&
                now_sec <= gui_task_holdoff_until_sec_
            ) {
                return false;
            }
            gui_task_holdoff_active_ = false;
        } else if (source == EndpointTrimCommandSource::GUI_DIRECT_SYNC) {
            // A tagged synchronization explicitly ends the edit holdoff.
            gui_task_holdoff_active_ = false;
        } else if (source == EndpointTrimCommandSource::GUI_DIRECT_EDIT) {
            if (
                now_sec > std::numeric_limits<double>::max() -
                    gui_task_holdoff_sec_
            ) {
                return false;
            }
            gui_task_holdoff_active_ = true;
            gui_task_holdoff_until_sec_ =
                now_sec + gui_task_holdoff_sec_;
        }
        last_observation_accepted_ = true;
        bool changed =
            source == EndpointTrimCommandSource::GUI_DIRECT_EDIT ||
            source == EndpointTrimCommandSource::GUI_DIRECT_SYNC ||
            !has_authoritative_source_ ||
            source != authoritative_source_ ||
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
            authoritative_source_ = source;
            has_authoritative_source_ = true;
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

    EndpointTrimReleaseStatus record_release(
        EndpointTrimPhase phase_before,
        const EndpointTrimDecision& decision,
        bool request_routed)
    {
        if (phase_before == EndpointTrimPhase::FAULT) {
            last_release_status_ = EndpointTrimReleaseStatus::REJECTED;
        } else if (
            phase_before == EndpointTrimPhase::IDLE ||
            phase_before == EndpointTrimPhase::QUIESCENT
        ) {
            last_release_status_ = EndpointTrimReleaseStatus::NOOP;
        } else if (!request_routed) {
            last_release_status_ = EndpointTrimReleaseStatus::REJECTED;
        } else if (decision.phase == EndpointTrimPhase::PENDING_RELEASE) {
            last_release_status_ = EndpointTrimReleaseStatus::PENDING;
        } else if (decision.release_completed) {
            last_release_status_ = EndpointTrimReleaseStatus::COMPLETED;
        } else {
            last_release_status_ = EndpointTrimReleaseStatus::REJECTED;
        }
        if (
            last_release_status_ == EndpointTrimReleaseStatus::PENDING ||
            last_release_status_ == EndpointTrimReleaseStatus::COMPLETED
        ) {
            // Only an active coordinator release can start a terminal-event
            // generation. No-op and rejected calls must not re-arm an older
            // completed state after its event has been consumed.
            note_release();
            if (last_release_status_ == EndpointTrimReleaseStatus::COMPLETED) {
                capture_terminal_release(decision);
            }
        }
        return last_release_status_;
    }

    void capture_terminal_release(const EndpointTrimDecision& decision)
    {
        if (
            !decision.release_completed ||
            !terminal_release_code_.empty() ||
            terminal_release_captured_generation_ == release_generation_
        ) {
            return;
        }
        if (decision.code.empty()) {
            terminal_release_code_ = "ENDPOINT_TRIM_RELEASE_SETTLED";
        } else if (decision.code == "ENDPOINT_TRIM_RESPONSE_TIMEOUT") {
            terminal_release_code_ = "ENDPOINT_TRIM_RESPONSE_TIMEOUT";
        } else {
            terminal_release_code_ = "ENDPOINT_TRIM_CONTINUITY_VIOLATION";
        }
        terminal_release_captured_generation_ = release_generation_;
    }

    EndpointTrimReleaseStatus release_status() const
    {
        return last_release_status_;
    }

    bool has_terminal_release_event() const
    {
        return !terminal_release_code_.empty();
    }

    std::string consume_terminal_release_code()
    {
        const std::string code = terminal_release_code_;
        terminal_release_code_.clear();
        return code;
    }

    bool has_unapplied_command() const
    {
        return command_generation_ != applied_command_generation_;
    }

    bool newer_explicit_gui_command_requires_handoff() const
    {
        return has_unapplied_command() &&
            command_generation_ > release_generation_ &&
            (authoritative_source_ ==
                EndpointTrimCommandSource::GUI_DIRECT_EDIT ||
             authoritative_source_ ==
                EndpointTrimCommandSource::GUI_DIRECT_SYNC);
    }

    bool newer_task_command_requires_handoff() const
    {
        return has_unapplied_command() &&
            command_generation_ > release_generation_ &&
            authoritative_source_ ==
                EndpointTrimCommandSource::TASK_CONTROLLER;
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

    bool last_observation_accepted() const
    {
        return last_observation_accepted_;
    }

    EndpointTrimCommandSource authoritative_source() const
    {
        return authoritative_source_;
    }

    void reset()
    {
        sequence_ = 0;
        command_generation_ = 0;
        applied_command_generation_ = 0;
        release_generation_ = 0;
        upstream_target_.clear();
        has_authoritative_source_ = false;
        authoritative_source_ = EndpointTrimCommandSource::TASK_CONTROLLER;
        last_observation_accepted_ = false;
        last_release_status_ = EndpointTrimReleaseStatus::NOOP;
        terminal_release_code_.clear();
        terminal_release_captured_generation_ = 0;
        gui_task_holdoff_active_ = false;
        gui_task_holdoff_until_sec_ = 0.0;
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
    double gui_task_holdoff_sec_;
    uint64_t sequence_ = 0;
    uint64_t command_generation_ = 0;
    uint64_t applied_command_generation_ = 0;
    uint64_t release_generation_ = 0;
    std::vector<double> upstream_target_;
    bool has_authoritative_source_ = false;
    EndpointTrimCommandSource authoritative_source_ =
        EndpointTrimCommandSource::TASK_CONTROLLER;
    bool last_observation_accepted_ = false;
    EndpointTrimReleaseStatus last_release_status_ =
        EndpointTrimReleaseStatus::NOOP;
    std::string terminal_release_code_;
    uint64_t terminal_release_captured_generation_ = 0;
    bool gui_task_holdoff_active_ = false;
    double gui_task_holdoff_until_sec_ = 0.0;
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
