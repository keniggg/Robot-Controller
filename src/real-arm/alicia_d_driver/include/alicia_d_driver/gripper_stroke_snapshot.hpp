#ifndef ALICIA_D_DRIVER_GRIPPER_STROKE_SNAPSHOT_HPP
#define ALICIA_D_DRIVER_GRIPPER_STROKE_SNAPSHOT_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

namespace alicia_d_driver {

// Parameter I/O belongs to this object's worker, never to feedback/command
// callbacks. Missing, nonnumeric or failed responses retain param<double>'s
// 0.05 fallback; numeric values retain their existing conversion semantics.
class GripperStrokeSnapshot {
public:
    GripperStrokeSnapshot(const std::string& master_host, int master_port,
                         const std::string& master_path,
                         const std::string& caller_id, const std::string& key,
                         std::chrono::milliseconds period = std::chrono::milliseconds(1000),
                         std::chrono::milliseconds timeout = std::chrono::milliseconds(250));
    ~GripperStrokeSnapshot();
    GripperStrokeSnapshot(const GripperStrokeSnapshot&) = delete;
    GripperStrokeSnapshot& operator=(const GripperStrokeSnapshot&) = delete;

    double load() const noexcept { return value_.load(std::memory_order_relaxed); }
    void stop();
    static std::string master_path(const std::string& uri);

private:
    // Declared before Impl so the joined worker can never outlive its storage.
    std::atomic<double> value_{0.05};
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace alicia_d_driver
#endif
