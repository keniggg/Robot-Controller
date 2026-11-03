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
        clear_committed_reference();
        if (state_ == ActuationState::PENDING) {
            reason_ = "POSITIVE_ENABLE_REQUESTED";
        }
    }

    // A control-owner change cancels the previous motion observation, but is
    // not a torque reset and cannot invalidate an already synchronized servo.
    // The driver separately requires fresh measured automatic handoff.
    void reset_motion_observation()
    {
        clear_probe();
        if (state_ == ActuationState::PENDING) {
            reason_ = synchronized_ ? "COMMAND_SYNCHRONIZED" : "POSITIVE_ENABLE_REQUESTED";
        }
    }

    void reset_for_positive_enable(double now_sec)
    {
        (void)now_sec;
        initial_probe_failed_ = false;
        state_ = ActuationState::PENDING;
        reason_ = "POSITIVE_ENABLE_REQUESTED";
        synchronized_ = false;
        has_feedback_ = false;
        latest_feedback_.clear();
        latest_feedback_stamp_sec_ = 0.0;
        clear_probe();
        clear_committed_reference();
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
        clear_committed_reference();
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
        if (!valid_joints(joints) || !std::isfinite(stamp_sec) ||
            (has_feedback_ && stamp_sec < latest_feedback_stamp_sec_)) {
            return;
        }

        latest_feedback_ = joints;
        latest_feedback_stamp_sec_ = stamp_sec;
        has_feedback_ = true;

        // A delayed source-stamped sample from before this axis's first
        // committed movement may refine its pre-command baseline, never count
        // as response. Ordinary post-command streamed movement still accrues.
        if (stream_axis_started_.size() == joints.size() &&
            stream_axis_start_sec_.size() == joints.size() &&
            stream_baseline_.size() == joints.size()) {
            for (std::size_t i = 0; i < joints.size(); ++i) {
                if (stream_axis_started_[i] && stamp_sec <= stream_axis_start_sec_[i]) {
                    stream_baseline_[i] = joints[i];
                }
            }
        }

        if (
            (state_ != ActuationState::PENDING &&
             state_ != ActuationState::CONFIRMED) ||
            !probe_active_ ||
            stamp_sec - probe_start_sec_ > config_.response_timeout_sec ||
            probe_baseline_.size() != joints.size() ||
            probe_requested_delta_.size() != joints.size() ||
            probe_response_start_sec_.size() != joints.size() ||
            probe_motion_start_sec_.size() != joints.size()
        ) {
            return;
        }

        for (std::size_t i = 0; i < joints.size(); ++i) {
            const double requested_delta = probe_requested_delta_[i];
            if (stamp_sec <= probe_motion_start_sec_[i]) {
                probe_baseline_[i] = joints[i];
                continue;
            }
            if (
                std::abs(requested_delta) <
                    config_.command_probe_min_delta_rad ||
                stamp_sec <= probe_response_start_sec_[i]
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
        if (initial_probe_failed_) {
            set_rejection(rejection_reason, "INITIAL_PROBE_FAILED_REQUIRES_POSITIVE_ENABLE");
            return false;
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

    // A successfully written reference-only/ownership hold supplies an SDK
    // origin, never motion intent. The driver calls this only after write
    // success, including hold-only writes which do not enter the motion path.
    void note_committed_reference(
        const std::vector<double>& target,
        double stamp_sec
    )
    {
        if (!valid_joints(target) || !std::isfinite(stamp_sec) ||
            (!committed_target_.empty() && stamp_sec < committed_stamp_sec_)) {
            return;
        }
        committed_target_ = target;
        committed_stamp_sec_ = stamp_sec;
        // A reference must not renew or cancel an active response deadline.
        // Ownership/reset explicitly cancels the old observation separately.
        if (!probe_active_) clear_stream();
    }

    void note_streamed_target(
        const std::vector<double>& target,
        double stamp_sec
    )
    {
        // This API is evidence of a successful SDK write, not admission or
        // publication of an upstream target. Failed/retired requests cannot
        // rearm it; a new positive-enable epoch discards every old reference.
        if (!synchronized_ || initial_probe_failed_ ||
            (state_ != ActuationState::PENDING && state_ != ActuationState::CONFIRMED) ||
            !valid_joints(target) || !std::isfinite(stamp_sec) ||
            (!committed_target_.empty() && stamp_sec < committed_stamp_sec_)) {
            return;
        }
        const std::vector<double> previous = committed_target_;
        const double previous_stamp = committed_stamp_sec_;
        committed_target_ = target;
        committed_stamp_sec_ = stamp_sec;
        if (!has_feedback_ || latest_feedback_.size() != target.size() ||
            stamp_sec < latest_feedback_stamp_sec_ ||
            stamp_sec - latest_feedback_stamp_sec_ > config_.confirmation_freshness_sec) {
            if (!probe_active_) clear_stream();
            return;
        }
        // The first successful SDK vector is a reference. Without a prior
        // successful write, encoder/setpoint difference cannot distinguish a
        // cold-start alignment or retained owner hold from new motion intent.
        // The first real step is measured from this committed SDK origin.
        if (previous.size() != target.size()) {
            if (!probe_active_) clear_stream();
            return;
        }
        if (probe_active_ &&
            stamp_sec > probe_start_sec_ + config_.response_timeout_sec) {
            return; // Never extend an expired window through another write.
        }
        if (!probe_active_ &&
            (stream_baseline_.size() != target.size() ||
             stamp_sec - previous_stamp > config_.response_timeout_sec ||
             stamp_sec - stream_start_sec_ >
                 config_.response_timeout_sec + config_.confirmation_freshness_sec)) {
            stream_command_baseline_ = previous;
            stream_baseline_ = latest_feedback_;
            stream_axis_started_.assign(target.size(), false);
            stream_axis_start_sec_.assign(target.size(), 0.0);
            stream_start_sec_ = stamp_sec;
        }
        if (stream_baseline_.size() != target.size() ||
            stream_command_baseline_.size() != target.size() ||
            stream_axis_started_.size() != target.size() ||
            stream_axis_start_sec_.size() != target.size()) {
            return;
        }

        for (std::size_t i = 0; i < target.size(); ++i) {
            if (probe_active_ && std::abs(probe_requested_delta_[i]) >=
                    config_.command_probe_min_delta_rad) {
                continue; // Existing eligible axes use the reversal rule below.
            }
            const double committed_step = target[i] - previous[i];
            const double previous_command_delta =
                previous[i] - stream_command_baseline_[i];
            const bool direction_reversed = stream_axis_started_[i] &&
                committed_step * previous_command_delta < 0.0;
            if (!stream_axis_started_[i] || direction_reversed) {
                // Before eligibility, a reversal starts a new causal run at
                // the preceding successful SDK vector. Earlier movement in
                // the newly requested direction did not follow that request.
                // The active multi-axis response deadline remains unchanged.
                stream_command_baseline_[i] = previous[i];
                stream_baseline_[i] = latest_feedback_[i];
                stream_axis_started_[i] = committed_step != 0.0;
                stream_axis_start_sec_[i] = stamp_sec;
            } else {
                const double command_delta = target[i] - stream_command_baseline_[i];
                const double measured_delta = latest_feedback_[i] - stream_baseline_[i];
                const double previous_delta = previous[i] - stream_command_baseline_[i];
                if (command_delta * measured_delta > 0.0 &&
                    std::abs(measured_delta) > std::abs(previous_delta) +
                        config_.measured_response_min_delta_rad) {
                    // Exclude this axis's response which preceded sufficient
                    // committed command. Keep its actual SDK command origin;
                    // never replace all axes' origins with encoder offsets.
                    stream_baseline_[i] = latest_feedback_[i];
                }
            }
        }
        stream_target_ = target;
        stream_last_sec_ = stamp_sec;

        if (probe_active_) {
            for (std::size_t i = 0; i < target.size(); ++i) {
                const double requested = probe_requested_delta_[i];
                if (std::abs(requested) >= config_.command_probe_min_delta_rad) {
                    const double reversal = target[i] - probe_command_extreme_[i];
                    if (reversal * requested >= 0.0) {
                        probe_command_extreme_[i] = target[i];
                    } else if (std::abs(reversal) >= config_.command_probe_min_delta_rad) {
                        const double remaining = target[i] - latest_feedback_[i];
                        // Preserve the existing genuine reversal contract:
                        // transmitted reversal and remaining encoder demand
                        // must agree, then a NEW measured response is required.
                        if (remaining * reversal > 0.0 &&
                            std::abs(remaining) >= config_.command_probe_min_delta_rad) {
                            probe_baseline_[i] = latest_feedback_[i];
                            probe_requested_delta_[i] = remaining;
                            probe_command_extreme_[i] = target[i];
                            probe_response_start_sec_[i] = stamp_sec;
                            probe_motion_start_sec_[i] = stamp_sec;
                        }
                    }
                    continue;
                }
                const double delta = target[i] - stream_command_baseline_[i];
                if (stream_axis_started_[i] &&
                    std::abs(delta) >= config_.command_probe_min_delta_rad) {
                    // A subthreshold offset/sign never fixes a future axis's
                    // direction. Its own committed displacement decides it,
                    // with a subsequent feedback frame and ORIGINAL deadline.
                    probe_requested_delta_[i] = delta;
                    probe_baseline_[i] = stream_baseline_[i];
                    probe_command_extreme_[i] = target[i];
                    probe_response_start_sec_[i] = stamp_sec;
                    probe_motion_start_sec_[i] = stream_axis_start_sec_[i];
                }
            }
            return;
        }

        bool non_trivial = false;
        for (std::size_t i = 0; i < target.size(); ++i) {
            non_trivial = non_trivial || (stream_axis_started_[i] &&
                std::abs(target[i] - stream_command_baseline_[i]) >=
                    config_.command_probe_min_delta_rad);
        }
        if (!non_trivial) return;
        probe_active_ = true;
        probe_baseline_ = stream_baseline_;
        probe_requested_delta_.resize(target.size());
        probe_command_extreme_ = target;
        probe_response_start_sec_.assign(target.size(), stamp_sec);
        probe_motion_start_sec_ = stream_axis_start_sec_;
        for (std::size_t i = 0; i < target.size(); ++i) {
            probe_requested_delta_[i] = stream_axis_started_[i]
                ? target[i] - stream_command_baseline_[i] : 0.0;
        }
        probe_start_sec_ = stamp_sec;
        if (state_ != ActuationState::CONFIRMED) {
            state_ = ActuationState::PENDING;
            reason_ = "AWAITING_ENCODER_RESPONSE";
        }
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
            if (state_ == ActuationState::PENDING) {
                initial_probe_failed_ = true;
            }
            reason_ = state_ == ActuationState::CONFIRMED
                ? "ENCODER_RESPONSE_LOST" : "ENCODER_RESPONSE_TIMEOUT";
            state_ = ActuationState::UNCONFIRMED;
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

    // A failed initial probe retires its host target until another explicit
    // positive-enable handshake. This is not a torque/enable command, and it
    // deliberately excludes response loss after confirmation and stale input.
    bool initial_probe_failed() const
    {
        return initial_probe_failed_;
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

    void clear_committed_reference()
    {
        committed_target_.clear();
        committed_stamp_sec_ = 0.0;
    }

    void clear_stream()
    {
        stream_baseline_.clear();
        stream_target_.clear();
        stream_command_baseline_.clear();
        stream_axis_started_.clear();
        stream_axis_start_sec_.clear();
        stream_start_sec_ = 0.0;
        stream_last_sec_ = 0.0;
    }

    void clear_probe()
    {
        clear_stream();
        probe_active_ = false;
        probe_baseline_.clear();
        probe_requested_delta_.clear();
        probe_command_extreme_.clear();
        probe_response_start_sec_.clear();
        probe_motion_start_sec_.clear();
        probe_start_sec_ = 0.0;
    }

    // A successful physical SDK reference survives ordinary response/owner
    // observation resets. A reset of command synchronization/positive enable
    // discards it explicitly; no encoder sample can replace this origin.
    std::vector<double> committed_target_;
    double committed_stamp_sec_ = 0.0;
    std::vector<bool> stream_axis_started_;
    std::vector<double> stream_axis_start_sec_;
    std::vector<double> probe_motion_start_sec_;
    std::vector<double> stream_baseline_;
    std::vector<double> stream_target_;
    std::vector<double> stream_command_baseline_;
    double stream_start_sec_ = 0.0;
    double stream_last_sec_ = 0.0;
    ActuationConfirmationConfig config_;
    ActuationState state_ = ActuationState::DISABLED;
    std::string reason_ = "NOT_REQUESTED";
    bool synchronized_ = false;
    bool initial_probe_failed_ = false;
    bool has_feedback_ = false;
    std::vector<double> latest_feedback_;
    double latest_feedback_stamp_sec_ = 0.0;
    bool probe_active_ = false;
    std::vector<double> probe_baseline_;
    std::vector<double> probe_requested_delta_;
    std::vector<double> probe_command_extreme_;
    std::vector<double> probe_response_start_sec_;
    double probe_start_sec_ = 0.0;
};

#endif  // ALICIA_D_DRIVER_ACTUATION_CONFIRMATION_HPP
