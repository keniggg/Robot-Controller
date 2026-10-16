#pragma once

#include "alicia_d_driver/sdk_position_codec.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <vector>

// A command shaper, not a measurement or an arrival detector. Only GUI edits
// use it. Its clock is advanced by the driver after a successful write (or an
// identical already-written SDK word); it never follows encoder residuals.
struct GuiDirectProfileLimits {
    double velocity = 0.23;       // rad/s
    double acceleration = 1.0;   // rad/s^2
    double jerk = 10.0;          // rad/s^3
    double minimum_duration = 0.2;
    double maximum_duration = 60.0;
    double maximum_tick = 0.02;
};

struct GuiDirectProfileSample {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
};

class GuiDirectProfile {
public:
    struct Axis {
        std::array<double, 6> control{};
        double duration = 0.0;
        double elapsed = 0.0;
        double lower = 0.0;
        double upper = 0.0;
        double velocity_bound = 0.0;
        double acceleration_bound = 0.0;
        double jerk_bound = 0.0;
        uint16_t target_word = 0;
    };

    explicit GuiDirectProfile(const GuiDirectProfileLimits& limits = {})
        : limits_(limits) {
        if (!positive(limits.velocity) || !positive(limits.acceleration) ||
            !positive(limits.jerk) || !positive(limits.minimum_duration) ||
            !positive(limits.maximum_duration) || !positive(limits.maximum_tick) ||
            limits.minimum_duration > limits.maximum_duration) {
            throw std::invalid_argument("invalid GUI direct profile limits");
        }
    }

    void clear() { initialized_ = false; axes_ = {}; }
    bool initialized() const { return initialized_; }
    const Axis& axis(size_t i) const { return axes_.at(i); }

    bool seed(const std::vector<double>& successful_wire) {
        if (successful_wire.size() != axes_.size()) return false;
        for (double q : successful_wire) if (!valid_position(q)) return false;
        for (size_t i = 0; i < axes_.size(); ++i) {
            Axis axis;
            axis.target_word = sdk_joint_position_encode(successful_wire[i]);
            const double q = sdk_joint_position_decode(axis.target_word);
            axis.control.fill(q);
            axis.lower = axis.upper = q;
            axes_[i] = axis;
        }
        initialized_ = true;
        return true;
    }

    // Returns false without mutation if a continuous bounded redirect cannot
    // be represented. There is no hidden queue and no step-command fallback.
    // A redirect may brake inside the previous target envelope, but may never
    // leave the envelope of the current position and the old/new goals.
    bool request(size_t edited_axis, double target) {
        if (!initialized_ || edited_axis >= axes_.size() || !valid_position(target)) return false;
        const uint16_t word = sdk_joint_position_encode(target);
        const Axis& previous = axes_[edited_axis];
        if (word == previous.target_word) return true; // repeated GUI/SDK word does not restart time
        const auto start = sample(edited_axis);
        const double goal = sdk_joint_position_decode(word);
        const double old_goal = sdk_joint_position_decode(previous.target_word);
        Axis next;
        next.target_word = word;
        next.lower = std::min(start.position, std::min(old_goal, goal));
        next.upper = std::max(start.position, std::max(old_goal, goal));
        const double distance = std::abs(goal - start.position);
        double duration = std::max(limits_.minimum_duration, std::max(
            1.875 * distance / limits_.velocity, std::max(
                std::sqrt(5.773502691896258 * distance / limits_.acceleration),
                std::cbrt(60.0 * distance / limits_.jerk))));
        // A moving start may reach the new goal sooner than a rest-to-rest
        // estimate, so search from the minimum for redirects. Bounds, not the
        // estimate, decide admission. Work remains finite for any request.
        if (std::abs(start.velocity) > 1e-12 || std::abs(start.acceleration) > 1e-12)
            duration = limits_.minimum_duration;
        for (size_t attempt = 0; attempt < 200 && duration <= limits_.maximum_duration; ++attempt, duration *= 1.04) {
            next.duration = duration;
            next.control = {{start.position,
                start.position + start.velocity * duration / 5.0,
                start.position + 2.0 * start.velocity * duration / 5.0 +
                    start.acceleration * duration * duration / 20.0,
                goal, goal, goal}};
            // The Bezier convex hull proves the whole continuous curve stays
            // inside the authorized envelope, not only its timer samples.
            if (std::any_of(next.control.begin(), next.control.end(), [&next](double q) {
                    return !std::isfinite(q) || q < next.lower - 1e-12 || q > next.upper + 1e-12;
                })) continue;
            auto derivative = derivatives(next.control, duration);
            next.velocity_bound = absolute_bound(derivative[0], 5);
            next.acceleration_bound = absolute_bound(derivative[1], 5);
            next.jerk_bound = absolute_bound(derivative[2], 5);
            if (next.velocity_bound > limits_.velocity * (1.0 + 1e-10) ||
                next.acceleration_bound > limits_.acceleration * (1.0 + 1e-10) ||
                next.jerk_bound > limits_.jerk * (1.0 + 1e-10)) continue;
            axes_[edited_axis] = next;
            return true;
        }
        return false;
    }

    GuiDirectProfileSample sample(size_t i) const {
        const Axis& axis = axes_.at(i);
        GuiDirectProfileSample out;
        if (axis.duration <= 0.0 || axis.elapsed >= axis.duration) {
            out.position = sdk_joint_position_decode(axis.target_word);
            return out;
        }
        const double u = axis.elapsed / axis.duration;
        out.position = evaluate(std::vector<double>(axis.control.begin(), axis.control.end()), u);
        auto derivative = derivatives(axis.control, axis.duration);
        out.velocity = evaluate(derivative[0], u);
        out.acceleration = evaluate(derivative[1], u);
        out.jerk = evaluate(derivative[2], u);
        return out;
    }

    void advance(double elapsed_seconds) {
        if (!initialized_ || !std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0) return;
        const double dt = std::min(elapsed_seconds, limits_.maximum_tick);
        for (Axis& axis : axes_) axis.elapsed = std::min(axis.duration, axis.elapsed + dt);
    }

    std::vector<double> positions() const {
        std::vector<double> result;
        if (initialized_) for (size_t i = 0; i < axes_.size(); ++i) result.push_back(sample(i).position);
        return result;
    }

private:
    static bool positive(double v) { return std::isfinite(v) && v > 0.0; }
    static bool valid_position(double q) { return std::isfinite(q) && q >= -M_PI && q <= M_PI; }
    static double evaluate(std::vector<double> control, double u) {
        for (size_t n = control.size(); n > 1; --n)
            for (size_t i = 0; i + 1 < n; ++i) control[i] = (1.0-u)*control[i] + u*control[i+1];
        return control.front();
    }
    static std::array<std::vector<double>, 3> derivatives(const std::array<double, 6>& control, double duration) {
        std::array<std::vector<double>, 3> result;
        std::vector<double> previous(control.begin(), control.end());
        for (auto& current : result) {
            const double scale = (previous.size() - 1) / duration;
            for (size_t i = 0; i + 1 < previous.size(); ++i) current.push_back((previous[i+1]-previous[i])*scale);
            previous = current;
        }
        return result;
    }
    // Subdivision tightens a conservative derivative bound. Each sub-curve
    // still lies in its control polygon: this is not sampled validation.
    static double absolute_bound(const std::vector<double>& control, size_t depth) {
        if (!depth) {
            double result = 0.0;
            for (double value : control) {
                if (!std::isfinite(value)) return std::numeric_limits<double>::infinity();
                result = std::max(result, std::abs(value));
            }
            return result;
        }
        std::vector<double> work(control), left{work.front()}, right{work.back()};
        for (size_t n = work.size(); n > 1; --n) {
            for (size_t i = 0; i + 1 < n; ++i) work[i] = (work[i]+work[i+1])*0.5;
            left.push_back(work.front());
            right.push_back(work[n-2]);
        }
        std::reverse(right.begin(), right.end());
        return std::max(absolute_bound(left, depth-1), absolute_bound(right, depth-1));
    }
    GuiDirectProfileLimits limits_;
    std::array<Axis, 6> axes_{};
    bool initialized_ = false;
};
