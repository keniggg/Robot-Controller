#ifndef ALICIA_D_DRIVER_ACTUATION_CONFIRMATION_HPP
#define ALICIA_D_DRIVER_ACTUATION_CONFIRMATION_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

enum class ActuationState
{
    DISABLED,
    PENDING,
    CONFIRMED,
    UNCONFIRMED,
    OVERHEAT_BLOCKED,
};

struct ActuationConfirmationConfig
{
    double sync_tolerance_rad = 0.05;
    double command_probe_min_delta_rad = 0.02;
    double measured_response_min_delta_rad = 0.003;
    double response_timeout_sec = 1.0;
    double confirmation_freshness_sec = 2.0;
};

class ActuationConfirmation
{
public:
    explicit ActuationConfirmation(
        const ActuationConfirmationConfig& config =
            ActuationConfirmationConfig()
    )
        : config_(config)
    {
        config_.sync_tolerance_rad =
            std::max(0.0, config_.sync_tolerance_rad);
        config_.command_probe_min_delta_rad =
            std::max(0.0, config_.command_probe_min_delta_rad);
        config_.measured_response_min_delta_rad =
            std::max(0.0, config_.measured_response_min_delta_rad);
        config_.response_timeout_sec =
            std::max(0.0, config_.response_timeout_sec);
        config_.confirmation_freshness_sec =
            std::max(0.0, config_.confirmation_freshness_sec);
    }

    bool can_preserve_positive_enable(
        bool connected,
        bool motion_commands_enabled,
        double now_sec
    ) const
    {
        return connected && motion_commands_enabled && motion_confirmed(now_sec);
    }

    void reset_command_synchronization()
    {
        synchronized_ = false;
        clear_probe();
        if (state_ == ActuationState::PENDING) {
            reason_ = "POSITIVE_ENABLE_REQUESTED";
        }
    }

    void reset_for_positive_enable(double now_sec)
    {
        (void)now_sec;
        state_ = ActuationState::PENDING;
        reason_ = "POSITIVE_ENABLE_REQUESTED";
        synchronized_ = false;
        has_feedback_ = false;
        latest_feedback_.clear();
        latest_feedback_stamp_sec_ = 0.0;
        clear_probe();
    }

    void set_disabled(double now_sec)
    {
        (void)now_sec;
        state_ = ActuationState::DISABLED;
        reason_ = "EXPLICIT_TORQUE_OFF";
        synchronized_ = false;
        has_feedback_ = false;
        latest_feedback_.clear();
        latest_feedback_stamp_sec_ = 0.0;
        clear_probe();
    }

    void mark_overheat_blocked(
        const std::string& reason,
        double now_sec
    )
    {
        (void)now_sec;
        state_ = ActuationState::OVERHEAT_BLOCKED;
        reason_ = reason.empty()
            ? "SUSTAINED_SAME_CHANNEL_TEMPERATURE"
            : reason;
        synchronized_ = false;
        clear_probe();
    }

    void mark_unconfirmed(
        const std::string& reason,
        double now_sec
    )
    {
        (void)now_sec;
        state_ = ActuationState::UNCONFIRMED;
        reason_ = reason.empty() ? "ACTUATION_UNCONFIRMED" : reason;
        clear_probe();
    }

    void note_feedback(
        const std::vector<double>& joints,
        double stamp_sec
    )
    {
        if (!valid_joints(joints) || !std::isfinite(stamp_sec)) {
            return;
        }

        latest_feedback_ = joints;
        latest_feedback_stamp_sec_ = stamp_sec;
        has_feedback_ = true;

        if (
            state_ != ActuationState::PENDING ||
            !probe_active_ ||
            probe_baseline_.size() != joints.size() ||
            probe_target_.size() != joints.size()
        ) {
            return;
        }

        for (std::size_t i = 0; i < joints.size(); ++i) {
            const double requested_delta =
                probe_target_[i] - probe_baseline_[i];
            if (
                std::abs(requested_delta) <
                config_.command_probe_min_delta_rad
            ) {
                continue;
            }
            const double measured_delta =
                joints[i] - probe_baseline_[i];
            if (
                std::abs(measured_delta) >=
                    config_.measured_response_min_delta_rad &&
                requested_delta * measured_delta > 0.0
            ) {
                state_ = ActuationState::CONFIRMED;
                reason_ = "MEASURED_DIRECTIONAL_RESPONSE";
                clear_probe();
                return;
            }
        }
    }

    bool admit_command(
        const std::vector<double>& target,
        double stamp_sec,
        std::string* rejection_reason
    )
    {
        if (rejection_reason != nullptr) {
            rejection_reason->clear();
        }
        if (
            !valid_joints(target) ||
            !std::isfinite(stamp_sec)
        ) {
            set_rejection(rejection_reason, "INVALID_JOINT_COMMAND");
            return false;
        }
        if (state_ == ActuationState::DISABLED) {
            set_rejection(rejection_reason, "MOTION_NOT_REQUESTED");
            return false;
        }
        if (state_ == ActuationState::OVERHEAT_BLOCKED) {
            set_rejection(rejection_reason, "OVERHEAT_BLOCKED");
            return false;
        }
        if (!has_feedback_ || latest_feedback_.size() != target.size()) {
            set_rejection(
                rejection_reason,
                "FEEDBACK_REQUIRED_FOR_SYNC"
            );
            return false;
        }
        if (synchronized_) {
            return true;
        }

        for (std::size_t i = 0; i < target.size(); ++i) {
            if (
                std::abs(target[i] - latest_feedback_[i]) >
                config_.sync_tolerance_rad
            ) {
                set_rejection(
                    rejection_reason,
                    "STALE_COMMAND_AFTER_RECONNECT"
                );
                return false;
            }
        }
        synchronized_ = true;
        reason_ = "COMMAND_SYNCHRONIZED";
        return true;
    }

    void note_streamed_target(
        const std::vector<double>& target,
        double stamp_sec
    )
    {
        if (
            !synchronized_ ||
            !has_feedback_ ||
            probe_active_ ||
            (state_ != ActuationState::PENDING &&
             state_ != ActuationState::UNCONFIRMED) ||
            !valid_joints(target) ||
            latest_feedback_.size() != target.size() ||
            !std::isfinite(stamp_sec) ||
            stamp_sec < latest_feedback_stamp_sec_ ||
            stamp_sec - latest_feedback_stamp_sec_ > config_.confirmation_freshness_sec
        ) {
            return;
        }

        if (stream_baseline_.empty()) {
            bool unchanged = true;
            for (std::size_t i = 0; i < target.size(); ++i) {
                unchanged = unchanged && std::abs(target[i] - latest_feedback_[i]) <= 1e-9;
            }
            if (unchanged) {
                return;
            }
        }
        // Smooth trajectories arrive as small increments. Measuring each
        // increment against the latest feedback can wait forever when the
        // servo follows well. Keep a short, continuously streamed command
        // window and measure the requested/actual displacement from its start.
        bool feedback_ahead_of_command = false;
        if (stream_baseline_.size() == target.size() &&
            stream_target_.size() == target.size()) {
            for (std::size_t i = 0; i < target.size(); ++i) {
                feedback_ahead_of_command = feedback_ahead_of_command ||
                    std::abs(latest_feedback_[i] - stream_baseline_[i]) >
                        std::abs(stream_target_[i] - stream_baseline_[i]) +
                            config_.measured_response_min_delta_rad;
            }
        }
        if (feedback_ahead_of_command || stream_baseline_.size() != target.size() ||
            stamp_sec < stream_start_sec_ ||
            stamp_sec - stream_last_sec_ > config_.response_timeout_sec ||
            stamp_sec - stream_start_sec_ >
                config_.response_timeout_sec + config_.confirmation_freshness_sec) {
            stream_baseline_ = latest_feedback_;
            stream_start_sec_ = stamp_sec;
        }
        stream_target_ = target;
        stream_last_sec_ = stamp_sec;
        bool non_trivial = false;
        for (std::size_t i = 0; i < target.size(); ++i) {
            if (
                std::abs(target[i] - stream_baseline_[i]) >=
                config_.command_probe_min_delta_rad
            ) {
                non_trivial = true;
                break;
            }
        }
        if (!non_trivial) {
            return;
        }

        probe_active_ = true;
        probe_baseline_ = stream_baseline_;
        probe_target_ = target;
        probe_start_sec_ = stamp_sec;
        state_ = ActuationState::PENDING;
        reason_ = "AWAITING_ENCODER_RESPONSE";
    }

    void update(double now_sec)
    {
        if (!std::isfinite(now_sec)) {
            return;
        }
        if (
            probe_active_ &&
            now_sec - probe_start_sec_ >
                config_.response_timeout_sec
        ) {
            state_ = ActuationState::UNCONFIRMED;
            reason_ = "ENCODER_RESPONSE_TIMEOUT";
            clear_probe();
            return;
        }
        if (
            state_ == ActuationState::CONFIRMED &&
            (
                !has_feedback_ ||
                now_sec - latest_feedback_stamp_sec_ >
                    config_.confirmation_freshness_sec
            )
        ) {
            state_ = ActuationState::UNCONFIRMED;
            reason_ = "CONFIRMED_FEEDBACK_STALE";
            clear_probe();
        }
    }

    ActuationState state() const
    {
        return state_;
    }

    bool synchronized() const
    {
        return synchronized_;
    }

    bool motion_confirmed(double now_sec) const
    {
        return (
            state_ == ActuationState::CONFIRMED &&
            has_feedback_ &&
            std::isfinite(now_sec) &&
            now_sec >= latest_feedback_stamp_sec_ &&
            now_sec - latest_feedback_stamp_sec_ <=
                config_.confirmation_freshness_sec
        );
    }

    std::string status_text() const
    {
        return state_name(state_) + ":" + reason_;
    }

private:
    static bool valid_joints(const std::vector<double>& joints)
    {
        if (joints.size() != 6) {
            return false;
        }
        for (double value : joints) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return true;
    }

    static std::string state_name(ActuationState state)
    {
        switch (state) {
        case ActuationState::DISABLED:
            return "DISABLED";
        case ActuationState::PENDING:
            return "PENDING";
        case ActuationState::CONFIRMED:
            return "CONFIRMED";
        case ActuationState::UNCONFIRMED:
            return "UNCONFIRMED";
        case ActuationState::OVERHEAT_BLOCKED:
            return "OVERHEAT_BLOCKED";
        }
        return "UNCONFIRMED";
    }

    static void set_rejection(
        std::string* rejection_reason,
        const std::string& reason
    )
    {
        if (rejection_reason != nullptr) {
            *rejection_reason = reason;
        }
    }

    void clear_probe()
    {
        stream_baseline_.clear();
        stream_target_.clear();
        stream_start_sec_ = 0.0;
        stream_last_sec_ = 0.0;
        probe_active_ = false;
        probe_baseline_.clear();
        probe_target_.clear();
        probe_start_sec_ = 0.0;
    }

    std::vector<double> stream_baseline_;
    std::vector<double> stream_target_;
    double stream_start_sec_ = 0.0;
    double stream_last_sec_ = 0.0;
    ActuationConfirmationConfig config_;
    ActuationState state_ = ActuationState::DISABLED;
    std::string reason_ = "NOT_REQUESTED";
    bool synchronized_ = false;
    bool has_feedback_ = false;
    std::vector<double> latest_feedback_;
    double latest_feedback_stamp_sec_ = 0.0;
    bool probe_active_ = false;
    std::vector<double> probe_baseline_;
    std::vector<double> probe_target_;
    double probe_start_sec_ = 0.0;
};

#endif  // ALICIA_D_DRIVER_ACTUATION_CONFIRMATION_HPP
