#ifndef ALICIA_D_DRIVER_ENDPOINT_TRIM_CONTINUITY_HPP
#define ALICIA_D_DRIVER_ENDPOINT_TRIM_CONTINUITY_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

enum class EndpointTrimPhase {
    IDLE,
    ACTIVE_READY,
    WAITING_RESPONSE,
    PENDING_RELEASE,
    QUIESCENT,
    FAULT,
};

struct EndpointTrimConfig {
    size_t joint_count = 6;
    double sdk_quantum_rad = 2.0 * M_PI / 4096.0;
    double max_step_rad = 8.0 * M_PI / 4096.0;
    double response_min_rad = 2.0 * M_PI / 4096.0;
    double settle_error_rad = 4.0 * M_PI / 4096.0;
    double stable_sec = 0.30;
    double response_deadline_sec = 1.0;
    double max_total_trim_rad = 0.12;
};

struct EndpointTrimDecision {
    EndpointTrimPhase phase;
    std::vector<double> reference;
    std::vector<double> offsets;
    std::vector<double> composed_target;
    std::vector<double> applied_step;
    bool command_changed;
    bool release_completed;
    std::string code;
};

// This coordinator is deliberately independent of ROS and hardware.  Its
// caller owns all command dispatch, feedback acquisition, and safety actions.
class EndpointTrimContinuity {
public:
    explicit EndpointTrimContinuity(const EndpointTrimConfig& config)
        : config_(config)
    {
        state_.phase = EndpointTrimPhase::IDLE;
        state_.command_changed = false;
        state_.release_completed = false;
        valid_config_ = config_.joint_count > 0 &&
            finite_nonnegative(config_.sdk_quantum_rad) &&
            config_.sdk_quantum_rad > 0.0 &&
            finite_nonnegative(config_.max_step_rad) &&
            finite_nonnegative(config_.response_min_rad) &&
            finite_nonnegative(config_.settle_error_rad) &&
            finite_nonnegative(config_.stable_sec) &&
            finite_nonnegative(config_.response_deadline_sec) &&
            finite_nonnegative(config_.max_total_trim_rad);
    }

    EndpointTrimDecision activate(
        const std::vector<double>& reference,
        const std::vector<double>& offsets,
        double now_sec)
    {
        if (!valid_time(now_sec) || !valid_joints(reference) ||
            !valid_joints(offsets)) {
            return violate();
        }
        if (state_.phase == EndpointTrimPhase::FAULT) {
            return state_;
        }
        if (state_.phase == EndpointTrimPhase::WAITING_RESPONSE ||
            state_.phase == EndpointTrimPhase::PENDING_RELEASE) {
            return unchanged("ENDPOINT_TRIM_RESPONSE_OUTSTANDING");
        }

        clear_response();
        clear_release();
        state_.phase = EndpointTrimPhase::ACTIVE_READY;
        state_.reference = reference;
        state_.offsets = clamp_offsets(offsets);
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.composed_target = compose(state_.reference, state_.offsets);
        state_.command_changed = true;
        state_.release_completed = false;
        state_.code.clear();
        return state_;
    }

    EndpointTrimDecision request_correction(
        const std::vector<double>& measured,
        const std::vector<uint8_t>& stable_joint_mask,
        double gain,
        double now_sec)
    {
        if (!valid_time(now_sec) || !valid_joints(measured) ||
            stable_joint_mask.size() != config_.joint_count ||
            !std::isfinite(gain) || gain < 0.0) {
            return violate();
        }
        if (state_.phase == EndpointTrimPhase::FAULT) {
            return state_;
        }
        if (state_.phase == EndpointTrimPhase::WAITING_RESPONSE ||
            state_.phase == EndpointTrimPhase::PENDING_RELEASE) {
            return unchanged("ENDPOINT_TRIM_RESPONSE_OUTSTANDING");
        }
        if (state_.phase != EndpointTrimPhase::ACTIVE_READY) {
            return unchanged("ENDPOINT_TRIM_NOT_ACTIVE");
        }

        const std::vector<double> previous_target = state_.composed_target;
        std::vector<double> next_offsets = state_.offsets;
        std::vector<double> applied(config_.joint_count, 0.0);
        std::vector<int8_t> response_direction(config_.joint_count, 0);
        for (size_t i = 0; i < config_.joint_count; ++i) {
            if (stable_joint_mask[i] == 0) {
                continue;
            }
            const double error = saturating_subtract(
                state_.reference[i], measured[i]
            );
            const double requested_step = clamp(
                saturating_multiply(error, gain),
                -config_.max_step_rad,
                config_.max_step_rad
            );
            next_offsets[i] = clamp(
                saturating_add(state_.offsets[i], requested_step),
                -config_.max_total_trim_rad,
                config_.max_total_trim_rad
            );
            applied[i] = saturating_subtract(
                next_offsets[i], state_.offsets[i]
            );
            response_direction[i] = applied[i] > 0.0
                ? 1
                : (applied[i] < 0.0 ? -1 : 0);
        }

        state_.offsets = next_offsets;
        state_.applied_step = applied;
        state_.composed_target = compose(state_.reference, state_.offsets);
        state_.command_changed = state_.composed_target != previous_target;
        state_.release_completed = false;
        state_.code.clear();
        if (!state_.command_changed) {
            state_.phase = EndpointTrimPhase::ACTIVE_READY;
            return state_;
        }

        response_baseline_ = measured;
        // Verify the response to this increment, independently of any fixed
        // command-to-feedback bias.  The composed target remains the SDK
        // over-command, while the immutable response goal is exactly the
        // measured baseline plus the increment that was actually admitted.
        // This admits another same-direction increment only after the prior
        // one has demonstrably moved and settled.
        response_goal_ = response_baseline_;
        for (size_t i = 0; i < config_.joint_count; ++i) {
            response_goal_[i] = saturating_add(
                response_baseline_[i], applied[i]
            );
        }
        response_direction_ = response_direction;
        response_start_sec_ = now_sec;
        response_settle_since_sec_ = std::numeric_limits<double>::quiet_NaN();
        last_feedback_stamp_sec_ = -std::numeric_limits<double>::infinity();
        state_.phase = EndpointTrimPhase::WAITING_RESPONSE;
        return state_;
    }

    EndpointTrimDecision note_feedback(
        const std::vector<double>& measured,
        double feedback_stamp_sec,
        double now_sec)
    {
        if (!valid_time(now_sec) || !valid_time(feedback_stamp_sec) ||
            !valid_joints(measured)) {
            return violate();
        }
        if (state_.phase == EndpointTrimPhase::FAULT) {
            return state_;
        }
        if (feedback_stamp_sec > now_sec ||
            feedback_stamp_sec <= last_feedback_stamp_sec_) {
            return unchanged();
        }

        if (state_.phase != EndpointTrimPhase::WAITING_RESPONSE &&
            state_.phase != EndpointTrimPhase::PENDING_RELEASE) {
            last_feedback_stamp_sec_ = feedback_stamp_sec;
            return unchanged();
        }
        if (response_deadline_reached(feedback_stamp_sec, now_sec)) {
            return timeout_response();
        }
        if (response_baseline_.size() != config_.joint_count ||
            response_goal_.size() != config_.joint_count ||
            response_direction_.size() != config_.joint_count ||
            feedback_stamp_sec <= response_start_sec_) {
            return unchanged();
        }
        last_feedback_stamp_sec_ = feedback_stamp_sec;
        latest_feedback_ = measured;
        has_fresh_feedback_ = true;

        bool matching = true;
        bool within_settle_error = true;
        for (size_t i = 0; i < config_.joint_count; ++i) {
            if (response_direction_[i] == 0) {
                continue;
            }
            const double measured_delta = measured[i] - response_baseline_[i];
            if (static_cast<double>(response_direction_[i]) * measured_delta <= 0.0 ||
                std::abs(measured_delta) < config_.response_min_rad) {
                matching = false;
            }
            if (std::abs(measured[i] - response_goal_[i]) >
                config_.settle_error_rad) {
                within_settle_error = false;
            }
        }
        if (!matching || !within_settle_error) {
            response_settle_since_sec_ = std::numeric_limits<double>::quiet_NaN();
            return unchanged();
        }
        if (!std::isfinite(response_settle_since_sec_)) {
            response_settle_since_sec_ = feedback_stamp_sec;
            return unchanged();
        }
        if (feedback_stamp_sec - response_settle_since_sec_ < config_.stable_sec) {
            return unchanged();
        }

        clear_response();
        if (release_requested_) {
            return complete_release(false);
        }
        clear_release();
        state_.phase = EndpointTrimPhase::ACTIVE_READY;
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.command_changed = false;
        state_.release_completed = false;
        state_.code.clear();
        return state_;
    }

    EndpointTrimDecision request_release(
        const std::vector<double>& measured,
        bool feedback_fresh,
        double now_sec)
    {
        if (!valid_time(now_sec) || !valid_joints(measured)) {
            return violate();
        }
        if (state_.phase == EndpointTrimPhase::FAULT) {
            return unchanged(state_.code);
        }
        if (state_.phase == EndpointTrimPhase::PENDING_RELEASE) {
            return unchanged();
        }
        if (state_.phase == EndpointTrimPhase::QUIESCENT) {
            return unchanged();
        }
        if (state_.phase == EndpointTrimPhase::IDLE) {
            return violate();
        }
        if (!release_requested_) {
            release_requested_ = true;
            release_preserved_target_ = state_.composed_target;
        }
        if (feedback_fresh) {
            latest_feedback_ = measured;
            has_fresh_feedback_ = true;
        }
        state_.command_changed = false;
        state_.release_completed = false;
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.code.clear();
        if (state_.phase == EndpointTrimPhase::WAITING_RESPONSE) {
            state_.phase = EndpointTrimPhase::PENDING_RELEASE;
            return state_;
        }
        return complete_release(false);
    }

    EndpointTrimDecision update(double now_sec)
    {
        if (!valid_time(now_sec)) {
            return violate();
        }
        if (state_.phase == EndpointTrimPhase::FAULT) {
            return unchanged(state_.code);
        }
        if ((state_.phase == EndpointTrimPhase::WAITING_RESPONSE ||
             state_.phase == EndpointTrimPhase::PENDING_RELEASE) &&
            now_sec - response_start_sec_ >= config_.response_deadline_sec) {
            return timeout_response();
        }
        return unchanged();
    }

    EndpointTrimDecision explicit_gui_handoff(
        const std::vector<double>& gui_target,
        double now_sec)
    {
        return handoff_idle_target(gui_target, now_sec);
    }

    EndpointTrimDecision task_controller_handoff(
        const std::vector<double>& task_target,
        double now_sec)
    {
        if (!valid_time(now_sec) || !valid_joints(task_target)) {
            return violate();
        }
        if (state_.phase == EndpointTrimPhase::FAULT) {
            return unchanged(state_.code);
        }
        return handoff_idle_target(task_target, now_sec);
    }

    const EndpointTrimDecision& state() const
    {
        return state_;
    }

private:
    EndpointTrimDecision handoff_idle_target(
        const std::vector<double>& target,
        double now_sec)
    {
        if (!valid_time(now_sec) || !valid_joints(target)) {
            return violate();
        }
        clear_response();
        clear_release();
        state_.phase = EndpointTrimPhase::IDLE;
        state_.reference = target;
        state_.offsets.assign(config_.joint_count, 0.0);
        state_.composed_target = target;
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.command_changed = true;
        state_.release_completed = false;
        state_.code.clear();
        return state_;
    }

    static bool finite_nonnegative(double value)
    {
        return std::isfinite(value) && value >= 0.0;
    }

    bool valid_time(double value) const
    {
        return valid_config_ && std::isfinite(value);
    }

    bool valid_joints(const std::vector<double>& values) const
    {
        if (!valid_config_ || values.size() != config_.joint_count) {
            return false;
        }
        for (const double value : values) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return true;
    }

    static double clamp(double value, double lower, double upper)
    {
        return std::max(lower, std::min(value, upper));
    }

    static double saturating_add(double left, double right)
    {
        const double maximum = std::numeric_limits<double>::max();
        if (right > 0.0 && left > maximum - right) {
            return maximum;
        }
        if (right < 0.0 && left < -maximum - right) {
            return -maximum;
        }
        return left + right;
    }

    static double saturating_subtract(double left, double right)
    {
        return saturating_add(left, -right);
    }

    static double saturating_multiply(double left, double right)
    {
        if (left == 0.0 || right == 0.0) {
            return 0.0;
        }
        const double maximum = std::numeric_limits<double>::max();
        if (std::abs(left) > maximum / std::abs(right)) {
            return std::signbit(left) == std::signbit(right)
                ? maximum
                : -maximum;
        }
        return left * right;
    }

    std::vector<double> clamp_offsets(const std::vector<double>& offsets) const
    {
        std::vector<double> clamped = offsets;
        for (double& value : clamped) {
            value = clamp(
                value, -config_.max_total_trim_rad, config_.max_total_trim_rad
            );
        }
        return clamped;
    }

    static std::vector<double> compose(
        const std::vector<double>& reference,
        const std::vector<double>& offsets
    )
    {
        std::vector<double> target(reference.size(), 0.0);
        for (size_t i = 0; i < reference.size(); ++i) {
            target[i] = saturating_add(reference[i], offsets[i]);
        }
        return target;
    }

    double sdk_quantize(double radians) const
    {
        const double ticks = saturating_add(radians, M_PI) /
            config_.sdk_quantum_rad;
        const double bounded_ticks = clamp(
            ticks,
            static_cast<double>(std::numeric_limits<int>::min()),
            static_cast<double>(std::numeric_limits<int>::max())
        );
        return saturating_subtract(
            static_cast<double>(static_cast<int>(bounded_ticks)) *
                config_.sdk_quantum_rad,
            M_PI
        );
    }

    bool response_deadline_reached(
        double feedback_stamp_sec,
        double now_sec
    ) const
    {
        return feedback_stamp_sec - response_start_sec_ >=
                config_.response_deadline_sec ||
            now_sec - response_start_sec_ >= config_.response_deadline_sec;
    }

    EndpointTrimDecision timeout_response()
    {
        clear_response();
        if (release_requested_) {
            return complete_release(true);
        }
        clear_release();
        state_.phase = EndpointTrimPhase::FAULT;
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.command_changed = false;
        state_.release_completed = false;
        state_.code = "ENDPOINT_TRIM_RESPONSE_TIMEOUT";
        return state_;
    }

    EndpointTrimDecision complete_release(bool timed_out)
    {
        const std::vector<double> preserved = release_preserved_target_;
        state_.phase = EndpointTrimPhase::QUIESCENT;
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.command_changed = false;
        state_.release_completed = true;
        state_.code = timed_out ? "ENDPOINT_TRIM_RESPONSE_TIMEOUT" : "";
        if (timed_out) {
            state_.reference = preserved;
            state_.offsets.assign(config_.joint_count, 0.0);
            state_.composed_target = preserved;
        } else if (has_fresh_feedback_ && valid_joints(latest_feedback_)) {
            state_.reference = latest_feedback_;
            state_.offsets.assign(config_.joint_count, 0.0);
            for (size_t i = 0; i < config_.joint_count; ++i) {
                state_.offsets[i] = sdk_quantize(saturating_subtract(
                    preserved[i], state_.reference[i]
                ));
            }
            state_.composed_target = compose(state_.reference, state_.offsets);
            for (size_t i = 0; i < config_.joint_count; ++i) {
                if (std::abs(state_.composed_target[i] - preserved[i]) >
                    config_.sdk_quantum_rad + 1e-12) {
                    state_.reference = preserved;
                    state_.offsets.assign(config_.joint_count, 0.0);
                    state_.composed_target = preserved;
                    state_.code = "ENDPOINT_TRIM_CONTINUITY_VIOLATION";
                    break;
                }
            }
        } else {
            state_.reference = preserved;
            state_.offsets.assign(config_.joint_count, 0.0);
            state_.composed_target = preserved;
        }
        clear_release();
        return state_;
    }

    EndpointTrimDecision unchanged(const std::string& code = std::string())
    {
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.command_changed = false;
        state_.release_completed = false;
        state_.code = code;
        return state_;
    }

    EndpointTrimDecision violate()
    {
        clear_response();
        clear_release();
        state_.phase = EndpointTrimPhase::FAULT;
        state_.applied_step.assign(config_.joint_count, 0.0);
        state_.command_changed = false;
        state_.release_completed = false;
        state_.code = "ENDPOINT_TRIM_CONTINUITY_VIOLATION";
        return state_;
    }

    void clear_response()
    {
        response_baseline_.clear();
        response_goal_.clear();
        response_direction_.clear();
        response_start_sec_ = 0.0;
        response_settle_since_sec_ = std::numeric_limits<double>::quiet_NaN();
    }

    void clear_release()
    {
        release_requested_ = false;
        release_preserved_target_.clear();
        has_fresh_feedback_ = false;
        latest_feedback_.clear();
    }

    EndpointTrimConfig config_;
    bool valid_config_ = false;
    EndpointTrimDecision state_;
    std::vector<double> response_baseline_;
    std::vector<double> response_goal_;
    std::vector<int8_t> response_direction_;
    double response_start_sec_ = 0.0;
    double response_settle_since_sec_ = std::numeric_limits<double>::quiet_NaN();
    double last_feedback_stamp_sec_ = -std::numeric_limits<double>::infinity();
    bool release_requested_ = false;
    std::vector<double> release_preserved_target_;
    bool has_fresh_feedback_ = false;
    std::vector<double> latest_feedback_;
};

#endif  // ALICIA_D_DRIVER_ENDPOINT_TRIM_CONTINUITY_HPP
