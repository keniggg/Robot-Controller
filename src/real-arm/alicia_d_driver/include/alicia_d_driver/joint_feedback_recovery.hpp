#ifndef ALICIA_D_DRIVER_JOINT_FEEDBACK_RECOVERY_HPP
#define ALICIA_D_DRIVER_JOINT_FEEDBACK_RECOVERY_HPP

#include <cmath>
#include <vector>

namespace alicia_d_driver
{

// Acquisition is separate from recovery of an already accepted position. A
// controller can report a continuous boot/return-to-home sequence faster than
// our host motion limit. Do not make its first frame the accepted baseline.
// This window is only used after feedback was explicitly discarded, and never
// supplies authority to accept a discontinuity during commanded motion.
struct InitialJointFeedbackConfig
{
    double tolerance_rad;
    double minimum_span_sec;
    double maximum_frame_gap_sec;
    int minimum_samples;
};

class InitialJointFeedbackWindow
{
public:
    void reset()
    {
        minimum_.clear();
        maximum_.clear();
        first_time_ = 0.0;
        last_time_ = 0.0;
        samples_ = 0;
    }

    bool observe(const std::vector<double>& candidate, double receipt_time_sec,
                 bool has_streamed_command_reference,
                 const InitialJointFeedbackConfig& config)
    {
        if (has_streamed_command_reference || candidate.size() != 6 ||
            !std::isfinite(receipt_time_sec) || receipt_time_sec <= 0.0 ||
            !std::isfinite(config.tolerance_rad) || config.tolerance_rad <= 0.0 ||
            !std::isfinite(config.minimum_span_sec) || config.minimum_span_sec <= 0.0 ||
            !std::isfinite(config.maximum_frame_gap_sec) ||
            config.maximum_frame_gap_sec <= 0.0 || config.minimum_samples < 3) {
            reset();
            return false;
        }
        for (double value : candidate) {
            if (!std::isfinite(value)) {
                reset();
                return false;
            }
        }
        if (samples_ && receipt_time_sec <= last_time_) {
            // Replayed timestamps or a clock reset are not independent samples.
            reset();
            return false;
        }
        bool restart = !samples_ ||
            receipt_time_sec - last_time_ > config.maximum_frame_gap_sec + 1e-9;
        if (!restart) {
            for (std::size_t i = 0; i < candidate.size(); ++i) {
                const double low = candidate[i] < minimum_[i] ? candidate[i] : minimum_[i];
                const double high = candidate[i] > maximum_[i] ? candidate[i] : maximum_[i];
                // Bound the whole window, not consecutive differences: slowly
                // moving boot samples must not accumulate into a stable pose.
                if (high - low > config.tolerance_rad + 1e-12) {
                    restart = true;
                    break;
                }
            }
        }
        if (restart) {
            minimum_ = maximum_ = candidate;
            first_time_ = last_time_ = receipt_time_sec;
            samples_ = 1;
            return false;
        }
        for (std::size_t i = 0; i < candidate.size(); ++i) {
            if (candidate[i] < minimum_[i]) minimum_[i] = candidate[i];
            if (candidate[i] > maximum_[i]) maximum_[i] = candidate[i];
        }
        last_time_ = receipt_time_sec;
        ++samples_;
        return samples_ >= config.minimum_samples &&
            span_sec() + 1e-9 >= config.minimum_span_sec;
    }

    int sample_count() const { return samples_; }
    double span_sec() const { return samples_ ? last_time_ - first_time_ : 0.0; }

private:
    std::vector<double> minimum_, maximum_;
    double first_time_ = 0.0;
    double last_time_ = 0.0;
    int samples_ = 0;
};

// Repeated telemetry may recover a lost position only when the entire sample
// agrees with the last successfully transmitted target. The tolerance describes
// encoder/position agreement, independently of the serial polling interval.
// The caller still requires repeated samples and revokes actuation confirmation.
inline bool command_consistent_feedback_recovery(
    const std::vector<double>& previous,
    const std::vector<double>& candidate,
    const std::vector<double>& command,
    double tolerance_rad,
    double physical_jump_limit_rad,
    double minimum_improvement_rad
)
{
    if (previous.empty() || previous.size() != candidate.size() ||
        candidate.size() != command.size() ||
        !std::isfinite(tolerance_rad) || tolerance_rad <= 0.0 ||
        !std::isfinite(physical_jump_limit_rad) || physical_jump_limit_rad <= 0.0 ||
        !std::isfinite(minimum_improvement_rad) || minimum_improvement_rad <= 0.0 ||
        minimum_improvement_rad > tolerance_rad) {
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
        const double delta = std::abs(candidate[i] - previous[i]);
        const double jump_limit = physical_jump_limit_rad < tolerance_rad
            ? physical_jump_limit_rad : tolerance_rad;
        if (delta > jump_limit) {
            // The 0.05 rad sample-agreement tolerance must not hide a jump
            // already rejected by the velocity bound (e.g. 32 counts at
            // 10 Hz). For such sub-tolerance jumps require improvement above
            // the encoder slack. Larger recovery keeps the original margin.
            const double improvement = delta > tolerance_rad
                ? tolerance_rad : minimum_improvement_rad;
            if (candidate_error + improvement >=
                std::abs(previous[i] - command[i])) {
                return false;
            }
            improved_discontinuity = true;
        }
    }
    return improved_discontinuity;
}

// Preserve the historical contract for callers without a physical jump bound.
inline bool command_consistent_feedback_recovery(
    const std::vector<double>& previous,
    const std::vector<double>& candidate,
    const std::vector<double>& command,
    double tolerance_rad
)
{
    return command_consistent_feedback_recovery(
        previous, candidate, command, tolerance_rad, tolerance_rad, tolerance_rad);
}

}  // namespace alicia_d_driver

#endif
