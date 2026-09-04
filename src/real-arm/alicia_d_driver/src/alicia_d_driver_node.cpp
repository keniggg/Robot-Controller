#include "alicia_d_driver/alicia_d_driver_node.hpp"
#include <cmath>
#include <numeric> // For std::accumulate
#include <map>
#include <set>   // For std::set
#include <thread> // for std::this_thread
#include <chrono> // for std::chrono
#include <algorithm>
#include <array>
#include <sstream>
#include <iomanip>
#include <limits>

// --- Command IDs, Data Identifiers, etc. remain the same ---
constexpr uint8_t CMD_SERVO_CONTROL = 0x04;
constexpr uint8_t CMD_GRIPPER_CONTROL = 0x02;
constexpr uint8_t CMD_ZERO_CAL = 0x03;
constexpr uint8_t CMD_DEMO_CONTROL = 0x13;

// SDK v6 joint+gripper control protocol:
// AA 06 03 1C [6*(pos_lo pos_hi speed_lo speed_hi)] [gripper_pos_lo gripper_pos_hi gripper_speed_lo gripper_speed_hi] CRC FF
constexpr uint8_t SDK_CMD_JOINT = 0x06;
constexpr uint8_t SDK_FUNC_QUERY_JOINT_GRIPPER = 0x00;
constexpr uint8_t SDK_FUNC_QUERY_TEMPERATURE = 0x01;
constexpr uint8_t SDK_FUNC_SET_JOINT_GRIPPER = 0x03;
constexpr uint8_t SDK_DATA_LEN_JOINT_GRIPPER = 0x1C;
constexpr uint8_t SDK_CMD_SELF_CHECK = 0xFE;
constexpr uint8_t SDK_FUNC_SELF_CHECK = 0x00;
constexpr double SDK_JOINT_QUANTIZATION_RAD = 2.0 * M_PI / 4096.0;
// A commanded joint value and its returned feedback are independently
// quantized, so their comparison cannot resolve error below two SDK quanta.
constexpr double ENDPOINT_FEEDBACK_TRIM_ROUND_TRIP_FLOOR_RAD =
    2.0 * SDK_JOINT_QUANTIZATION_RAD;

static uint8_t sdk_crc32_low8(const std::vector<uint8_t>& data)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint8_t b : data)
    {
        crc ^= static_cast<uint32_t>(b);
        for (int i = 0; i < 8; ++i)
        {
            if (crc & 1u)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }
    crc ^= 0xFFFFFFFFu;
    return static_cast<uint8_t>(crc & 0xFFu);
}

static uint16_t speed_deg_s_to_hw(double speed_deg_s)
{
    // SDK mapping: 360 deg/s = 4096 ticks/s, hardware range 50~5000, step 50
    const double deg_per_tick = 360.0 / 4096.0;
    double hw = speed_deg_s / deg_per_tick;

    if (hw < 50.0) hw = 50.0;
    if (hw > 5000.0) hw = 5000.0;

    int rounded = static_cast<int>(std::round(hw / 50.0) * 50.0);
    if (rounded < 50) rounded = 50;
    if (rounded > 5000) rounded = 5000;
    return static_cast<uint16_t>(rounded);
}

static std::string format_radians_as_degrees(const std::vector<double>& values)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0) ss << ", ";
        ss << values[i] * 180.0 / M_PI;
    }
    ss << "]";
    return ss.str();
}

static std::string format_bytes_as_hex(const std::vector<uint8_t>& values)
{
    std::ostringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) ss << " ";
        ss << std::setw(2) << static_cast<unsigned>(values[i]);
    }
    return ss.str();
}

static std::string format_temperatures(const std::vector<float>& values)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) ss << ", ";
        ss << values[i];
    }
    ss << "]";
    return ss.str();
}

static const char* endpoint_trim_phase_name(EndpointTrimPhase phase)
{
    switch (phase) {
        case EndpointTrimPhase::IDLE: return "IDLE";
        case EndpointTrimPhase::ACTIVE_READY: return "ACTIVE_READY";
        case EndpointTrimPhase::WAITING_RESPONSE: return "WAITING_RESPONSE";
        case EndpointTrimPhase::PENDING_RELEASE: return "PENDING_RELEASE";
        case EndpointTrimPhase::QUIESCENT: return "QUIESCENT";
        case EndpointTrimPhase::FAULT: return "FAULT";
    }
    return "UNKNOWN";
}
// Protocol Constants for feedback frames
constexpr uint8_t FEEDBACK_GRIPPER_STATE = 0x02;
constexpr uint8_t FEEDBACK_SERVO_STATE = 0x04;
constexpr uint8_t FEEDBACK_SERVO_STATE_EXT = 0x06;
constexpr uint8_t FEEDBACK_ERROR = 0xEE;

// Gripper HW mapping constants (map 0..100 deg -> 2048..3590)
constexpr double GRIPPER_HW_MIN = 2048.0;      // Fully open
constexpr double GRIPPER_HW_MAX = 3590.0;      // Fully closed
constexpr double GRIPPER_DEG_MAX = 100.0;      // Logical gripper range in degrees
constexpr double GRIPPER_HW_PER_DEG = (GRIPPER_HW_MAX - GRIPPER_HW_MIN) / GRIPPER_DEG_MAX; // ~15.42 counts/deg


AliciaDDriverNode::AliciaDDriverNode()
    : pnh_("~"),
      endpoint_trim_continuity_(EndpointTrimConfig()),
      last_process_time_(0.0)
{
    load_parameters();
    endpoint_trim_config_.joint_count = 6;
    endpoint_trim_config_.sdk_quantum_rad = SDK_JOINT_QUANTIZATION_RAD;
    endpoint_trim_config_.max_step_rad =
        endpoint_feedback_trim_max_step_quantums_ *
        SDK_JOINT_QUANTIZATION_RAD;
    endpoint_trim_config_.response_min_rad =
        endpoint_feedback_trim_response_min_quantums_ *
        SDK_JOINT_QUANTIZATION_RAD;
    endpoint_trim_config_.settle_error_rad =
        ENDPOINT_FEEDBACK_TRIM_ROUND_TRIP_FLOOR_RAD;
    endpoint_trim_config_.stable_sec = endpoint_feedback_trim_stable_sec_;
    endpoint_trim_config_.response_deadline_sec =
        endpoint_feedback_trim_response_deadline_sec_;
    endpoint_trim_config_.max_total_trim_rad = endpoint_feedback_trim_max_rad_;
    endpoint_trim_continuity_ = EndpointTrimContinuity(endpoint_trim_config_);
    endpoint_trim_command_order_ = EndpointTrimCommandOrder(
        endpoint_trim_config_.joint_count,
        endpoint_trim_config_.sdk_quantum_rad
    );
    ActuationConfirmationConfig actuation_config;
    actuation_config.sync_tolerance_rad = reconnect_sync_tolerance_rad_;
    actuation_config.command_probe_min_delta_rad =
        actuation_command_probe_min_delta_rad_;
    actuation_config.measured_response_min_delta_rad =
        actuation_measured_response_min_delta_rad_;
    actuation_config.response_timeout_sec =
        actuation_confirmation_timeout_sec_;
    actuation_config.confirmation_freshness_sec =
        actuation_confirmation_freshness_sec_;
    actuation_confirmation_ = ActuationConfirmation(actuation_config);
    setup_ros_communications();
    publish_actuation_status();

    // Open the transport first. Torque is deliberately opt-in so the driver can
    // monitor controller startup and protection states before loading the arm.
    if (communicator_->connect()) {
        if (auto_torque_on_startup_) {
            ROS_WARN("Initial connection successful. auto_torque_on_startup is enabled; torque-on will be requested.");
            request_positive_enable("startup");
        } else {
            ROS_INFO("Initial connection successful in monitor-only startup mode; torque remains unchanged.");
        }
    } else {
        ROS_ERROR("Initial connection failed. Reconnect timer will keep trying.");
    }
    reconnect_timer_ = nh_.createTimer(ros::Duration(5.0), &AliciaDDriverNode::reconnect_callback, this);

    // Initialize last feedback time to now so we don't immediately consider it stale
    last_feedback_time_ = ros::Time::now();
    last_temperature_query_time_ = last_feedback_time_;
    last_self_check_query_time_ = last_feedback_time_;
}


AliciaDDriverNode::~AliciaDDriverNode()
{
    if (communicator_) {
        communicator_->disconnect();
    }
}


void AliciaDDriverNode::load_parameters()
{
    std::string port;
    int baud_rate;
    
    pnh_.param<std::string>("port", port, "/dev/alicia_arm");
    // Compatible with both parameter names used by different launch files:
    // baud_rate and baudrate. Prefer baudrate if provided.
    pnh_.param<int>("baud_rate", baud_rate, 921600);
    pnh_.param<int>("baudrate", baud_rate, baud_rate);
    pnh_.param<int>("servo_count", servo_count_, 9);
    pnh_.param<bool>("debug_mode", debug_mode_, false);
    pnh_.param<bool>("auto_torque_on_startup", auto_torque_on_startup_, false);
    pnh_.param<double>("rate_limit_sec", rate_limit_sec_, 0.01);
    pnh_.param<double>("command_rate_hz", command_rate_hz_, 200.0);
    pnh_.param<double>("state_poll_rate_hz", state_poll_rate_hz_, 20.0);
    pnh_.param<double>("temperature_poll_rate_hz", temperature_poll_rate_hz_, 1.0);
    pnh_.param<double>("self_check_poll_rate_hz", self_check_poll_rate_hz_, 0.5);
    pnh_.param<bool>(
        "suppress_diagnostic_queries_while_motion_active",
        suppress_diagnostic_queries_while_motion_active_,
        true
    );
    pnh_.param<double>(
        "diagnostic_query_motion_quiet_sec",
        diagnostic_query_motion_quiet_sec_,
        0.5
    );
    pnh_.param<double>(
        "diagnostic_query_motion_error_rad",
        diagnostic_query_motion_error_rad_,
        0.02
    );
    diagnostic_query_motion_quiet_sec_ = std::max(
        0.0,
        diagnostic_query_motion_quiet_sec_
    );
    diagnostic_query_motion_error_rad_ = std::max(
        0.0,
        diagnostic_query_motion_error_rad_
    );
    pnh_.param<bool>("mirror_commanded_state_when_feedback_stale", mirror_commanded_state_when_feedback_stale_, false);
    pnh_.param<bool>("log_command_flow", log_command_flow_, true);
    pnh_.param<bool>("suppress_redundant_commands", suppress_redundant_commands_, true);
    pnh_.param<bool>("pause_commands_when_feedback_stale", pause_commands_when_feedback_stale_, true);
    pnh_.param<double>("feedback_stale_timeout_sec", feedback_stale_timeout_sec_, 1.0);
    // The SDK position target must be periodically reasserted while the ROS
    // controller holds its final setpoint.  With redundant-frame suppression
    // and a zero keepalive, a small final correction can stop short after the
    // last distinct frame even though FollowJointTrajectory is still waiting.
    // Match the default real-feedback polling cadence so every fresh feedback
    // interval can be followed by one identical setpoint confirmation.
    pnh_.param<double>("command_keepalive_rate_hz", command_keepalive_rate_hz_, 10.0);
    command_keepalive_rate_hz_ = std::max(
        0.0,
        std::min(command_keepalive_rate_hz_, command_rate_hz_));
    // The SDK position loop can hold a repeatable measured offset from a
    // stable ROS endpoint. The continuity coordinator admits one bounded
    // correction, waits for a fresh directional encoder response to settle,
    // and preserves that composed target through release or timeout. Never
    // bake in a joint-specific angle or integrate while the trajectory moves.
    pnh_.param<bool>("endpoint_feedback_trim_enabled", endpoint_feedback_trim_enabled_, false);
    pnh_.param<double>(
        "endpoint_feedback_trim_task_lease_timeout_sec",
        endpoint_feedback_trim_task_lease_timeout_sec_,
        120.0
    );
    pnh_.param<double>("endpoint_feedback_trim_stable_sec", endpoint_feedback_trim_stable_sec_, 0.30);
    pnh_.param<double>("endpoint_feedback_trim_activation_error_rad",
                       endpoint_feedback_trim_activation_error_rad_, 0.035);
    pnh_.param<double>("endpoint_feedback_trim_max_rad", endpoint_feedback_trim_max_rad_, 0.12);
    pnh_.param<double>("endpoint_feedback_trim_gain", endpoint_feedback_trim_gain_, 1.0);
    pnh_.param<int>(
        "endpoint_feedback_trim_max_step_quantums",
        endpoint_feedback_trim_max_step_quantums_,
        4
    );
    pnh_.param<int>(
        "endpoint_feedback_trim_response_min_quantums",
        endpoint_feedback_trim_response_min_quantums_,
        2
    );
    pnh_.param<double>(
        "endpoint_feedback_trim_response_deadline_sec",
        endpoint_feedback_trim_response_deadline_sec_,
        1.0
    );
    const bool endpoint_trim_parameters_valid =
        endpoint_feedback_trim_max_step_quantums_ >= 1 &&
        endpoint_feedback_trim_max_step_quantums_ <= 16 &&
        endpoint_feedback_trim_response_min_quantums_ >= 1 &&
        endpoint_feedback_trim_response_min_quantums_ <=
            endpoint_feedback_trim_max_step_quantums_ &&
        std::isfinite(endpoint_feedback_trim_response_deadline_sec_) &&
        endpoint_feedback_trim_response_deadline_sec_ >= 0.30 &&
        endpoint_feedback_trim_response_deadline_sec_ <= 3.0;
    if (!endpoint_trim_parameters_valid) {
        ROS_WARN(
            "Rejected endpoint trim continuity parameters "
            "(max_step_quantums=%d response_min_quantums=%d "
            "response_deadline_sec=%.3f); retaining safe defaults",
            endpoint_feedback_trim_max_step_quantums_,
            endpoint_feedback_trim_response_min_quantums_,
            endpoint_feedback_trim_response_deadline_sec_
        );
        endpoint_feedback_trim_max_step_quantums_ = 4;
        endpoint_feedback_trim_response_min_quantums_ = 2;
        endpoint_feedback_trim_response_deadline_sec_ = 1.0;
    }
    ROS_INFO(
        "Endpoint trim continuity effective values: "
        "max_step_quantums=%d response_min_quantums=%d "
        "response_deadline_sec=%.3f",
        endpoint_feedback_trim_max_step_quantums_,
        endpoint_feedback_trim_response_min_quantums_,
        endpoint_feedback_trim_response_deadline_sec_
    );
    endpoint_feedback_trim_stable_sec_ =
        std::max(0.0, endpoint_feedback_trim_stable_sec_);
    endpoint_feedback_trim_activation_error_rad_ =
        std::max(0.0, endpoint_feedback_trim_activation_error_rad_);
    endpoint_feedback_trim_max_rad_ = std::max(
        endpoint_feedback_trim_activation_error_rad_,
        endpoint_feedback_trim_max_rad_);
    endpoint_feedback_trim_gain_ =
        std::max(0.0, std::min(1.0, endpoint_feedback_trim_gain_));
    endpoint_feedback_trim_task_lease_timeout_sec_ = std::max(
        1.0,
        endpoint_feedback_trim_task_lease_timeout_sec_
    );
    pnh_.param<double>("protection_clear_stable_sec", protection_clear_stable_sec_, 30.0);
    pnh_.param<double>("max_enable_temperature_c", max_enable_temperature_c_, 60.0);
    pnh_.param<double>(
        "max_plausible_temperature_c",
        max_plausible_temperature_c_,
        125.0
    );
    max_plausible_temperature_c_ = std::max(
        max_enable_temperature_c_,
        max_plausible_temperature_c_
    );
    pnh_.param<int>("e1_confirm_consecutive_frames", e1_confirm_consecutive_frames_, 3);
    pnh_.param<int>("temperature_over_limit_confirm_samples", temperature_over_limit_confirm_samples_, 3);
    e1_confirm_consecutive_frames_ = std::max(1, e1_confirm_consecutive_frames_);
    temperature_over_limit_confirm_samples_ = std::max(1, temperature_over_limit_confirm_samples_);
    pnh_.param<double>(
        "reconnect_sync_tolerance_rad",
        reconnect_sync_tolerance_rad_,
        0.05
    );
    pnh_.param<double>(
        "actuation_command_probe_min_delta_rad",
        actuation_command_probe_min_delta_rad_,
        0.02
    );
    pnh_.param<double>(
        "actuation_measured_response_min_delta_rad",
        actuation_measured_response_min_delta_rad_,
        0.003
    );
    pnh_.param<double>(
        "actuation_confirmation_timeout_sec",
        actuation_confirmation_timeout_sec_,
        1.0
    );
    pnh_.param<double>(
        "actuation_confirmation_freshness_sec",
        actuation_confirmation_freshness_sec_,
        2.0
    );
    reconnect_sync_tolerance_rad_ =
        std::max(0.0, reconnect_sync_tolerance_rad_);
    actuation_command_probe_min_delta_rad_ =
        std::max(0.0, actuation_command_probe_min_delta_rad_);
    actuation_measured_response_min_delta_rad_ =
        std::max(0.0, actuation_measured_response_min_delta_rad_);
    actuation_confirmation_timeout_sec_ =
        std::max(0.0, actuation_confirmation_timeout_sec_);
    actuation_confirmation_freshness_sec_ =
        std::max(0.0, actuation_confirmation_freshness_sec_);
    // Smoothing & input interpretation
    pnh_.param<bool>("use_trajectory_smoothing", use_trajectory_smoothing_, true);
    pnh_.param<double>("max_joint_velocity_rad_s", max_joint_velocity_rad_s_, 2.5);
    pnh_.param<double>("max_gripper_velocity_rad_s", max_gripper_velocity_rad_s_, 1.5);
    pnh_.param<bool>("gripper_input_is_percent", gripper_input_is_percent_, true);
    pnh_.param<double>("max_joint_accel_rad_s2", max_joint_accel_rad_s2_, 8.0);
    pnh_.param<double>("max_gripper_accel_rad_s2", max_gripper_accel_rad_s2_, 10.0);
    pnh_.param<double>("joint_speed_deg_s", joint_speed_deg_s_, 15.0);
    // Feedback is CRC-checked, but retained hardware evidence contains a
    // one-frame channel-like jump large enough to abort ros_control while the
    // neighbouring accepted samples and outgoing SDK targets remained stable.
    // Bound a single sample by elapsed time and the configured physical speed,
    // then require a second nearby sample before accepting a discontinuity.
    // This policy is identical for all joints and independent of grasp target.
    pnh_.param<bool>(
        "reject_implausible_joint_feedback",
        reject_implausible_joint_feedback_,
        true
    );
    const double configured_physical_speed_rad_s = std::max(
        max_joint_velocity_rad_s_,
        joint_speed_deg_s_ * M_PI / 180.0
    );
    pnh_.param<double>(
        "feedback_max_velocity_rad_s",
        feedback_max_velocity_rad_s_,
        4.0 * configured_physical_speed_rad_s
    );
    pnh_.param<double>(
        "feedback_jump_base_tolerance_rad",
        feedback_jump_base_tolerance_rad_,
        8.0 * SDK_JOINT_QUANTIZATION_RAD
    );
    pnh_.param<double>(
        "feedback_jump_confirmation_tolerance_rad",
        feedback_jump_confirmation_tolerance_rad_,
        0.05
    );
    pnh_.param<int>(
        "feedback_jump_confirm_samples",
        feedback_jump_confirm_samples_,
        2
    );
    pnh_.param<bool>(
        "reject_command_inconsistent_joint_feedback",
        reject_command_inconsistent_joint_feedback_,
        true
    );
    feedback_max_velocity_rad_s_ =
        std::max(configured_physical_speed_rad_s, feedback_max_velocity_rad_s_);
    feedback_jump_base_tolerance_rad_ = std::max(
        2.0 * SDK_JOINT_QUANTIZATION_RAD,
        feedback_jump_base_tolerance_rad_
    );
    feedback_jump_confirmation_tolerance_rad_ = std::max(
        2.0 * SDK_JOINT_QUANTIZATION_RAD,
        feedback_jump_confirmation_tolerance_rad_
    );
    feedback_jump_confirm_samples_ = std::max(
        2,
        feedback_jump_confirm_samples_
    );

    communicator_ = std::make_unique<SerialCommunicator>(port, baud_rate, debug_mode_);

    joint_to_servo_map_index_ = {0, 0, 1, 1, 2, 2, 3, 4, 5};
    joint_to_servo_map_direction_ = {1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    servo_to_joint_map_index_ = {0, -1, 1, -1, 2, -1, 3, 4, 5}; // -1 means ignore
    servo_to_joint_map_direction_ = {1.0, 0, 1.0, 0, 1.0, 0, 1.0, 1.0, 1.0};

    // Initialize global state variables
    joint_names_ = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "right_finger"};
    current_joint_positions_.resize(6, 0.0); // 6 arm joints
    current_gripper_position_ = 0.0;
    cmd_joint_angles_.assign(6, 0.0);
    cmd_gripper_rad_ = 0.0;
    cmd_joint_velocities_.assign(6, 0.0);
    cmd_gripper_vel_rad_s_ = 0.0;

}



void AliciaDDriverNode::setup_ros_communications()
{
    joint_state_pub_std_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    feedback_ready_pub_ = nh_.advertise<std_msgs::Bool>("/alicia_d/feedback_ready", 1, true);
    run_status_pub_ = nh_.advertise<std_msgs::UInt8>("/alicia_d/run_status", 1, true);
    temperature_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/alicia_d/temperatures_c", 1, true);
    self_check_mask_pub_ = nh_.advertise<std_msgs::UInt16>("/alicia_d/self_check_mask", 1, true);
    protection_latched_pub_ = nh_.advertise<std_msgs::Bool>("/alicia_d/protection_latched", 1, true);
    motion_enabled_pub_ = nh_.advertise<std_msgs::Bool>("/alicia_d/motion_enabled", 1, true);
    actuation_status_pub_ = nh_.advertise<std_msgs::String>("/alicia_d/actuation_status", 1, true);
    task_endpoint_precision_service_ = pnh_.advertiseService(
        "set_task_endpoint_precision",
        &AliciaDDriverNode::set_task_endpoint_precision_callback,
        this
    );
    joint_command_sub_ = nh_.subscribe("/joint_commands", 10, &AliciaDDriverNode::joint_command_callback, this);
    zero_calib_sub_ = nh_.subscribe("/zero_calibrate", 10, &AliciaDDriverNode::zero_calibrate_callback, this);
    demo_mode_sub_ = nh_.subscribe("/demonstration", 10, &AliciaDDriverNode::demonstration_mode_callback, this);
    processing_timer_ = nh_.createTimer(ros::Duration(0.01), &AliciaDDriverNode::process_serial_data_callback, this);
    // Heartbeat to ensure fresh /joint_states even when hardware frames are sparse
    heartbeat_timer_ = nh_.createTimer(ros::Duration(0.02), &AliciaDDriverNode::heartbeat_publish_callback, this);
    if (state_poll_rate_hz_ > 0.0) {
        const double state_poll_period = 1.0 / std::max(1.0, state_poll_rate_hz_);
        state_poll_timer_ = nh_.createTimer(ros::Duration(state_poll_period), &AliciaDDriverNode::state_poll_timer_callback, this);
    }
    // Timer to send serialized commands at fixed rate, decoupled from subscriber callback
    const double command_period = 1.0 / std::max(1.0, command_rate_hz_);
    command_timer_ = nh_.createTimer(ros::Duration(command_period), &AliciaDDriverNode::send_command_timer_callback, this);
}

bool AliciaDDriverNode::set_task_endpoint_precision_callback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response
)
{
    const ros::Time now = ros::Time::now();
    std::vector<double> release_feedback;
    bool release_feedback_is_fresh = false;
    if (!request.data) {
        std::lock_guard<std::mutex> data_lock(data_mutex_);
        if (current_joint_positions_.size() == 6) {
            release_feedback = current_joint_positions_;
        }
        release_feedback_is_fresh =
            has_real_feedback_ &&
            release_feedback.size() == 6 &&
            !last_accepted_joint_feedback_time_.isZero() &&
            (now - last_accepted_joint_feedback_time_).toSec() >= 0.0 &&
            (now - last_accepted_joint_feedback_time_).toSec() <=
                feedback_stale_timeout_sec_;
    }
    EndpointTrimReleaseStatus release_status =
        EndpointTrimReleaseStatus::NOOP;
    std::string release_failure_code;
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        if (request.data) {
            const EndpointTrimDecision& current =
                endpoint_trim_continuity_.state();
            const std::vector<double>& held_reference =
                current.reference.size() == 6
                    ? current.reference
                    : endpoint_trim_command_order_.upstream_target();
            if (held_reference.size() != 6) {
                response.success = false;
                response.message =
                    "task endpoint precision lease rejected: no held six-joint target";
                return true;
            }
            endpoint_feedback_trim_task_lease_active_ = true;
            endpoint_feedback_trim_task_lease_expires_at_ =
                now + ros::Duration(
                    endpoint_feedback_trim_task_lease_timeout_sec_);
            endpoint_feedback_trim_task_lease_reference_ =
                held_reference;
        } else {
            endpoint_feedback_trim_task_lease_active_ = false;
            endpoint_feedback_trim_task_lease_expires_at_ = ros::Time(0);
            endpoint_feedback_trim_task_lease_reference_.clear();
            const EndpointTrimDecision& current =
                endpoint_trim_continuity_.state();
            const EndpointTrimPhase phase_before_release = current.phase;
            std::vector<double> release_measurement = release_feedback;
            if (release_measurement.size() != 6) {
                release_measurement = current.composed_target;
            }
            bool request_routed = false;
            EndpointTrimDecision release_decision = current;
            if (
                endpoint_trim_release_request_allowed(
                    phase_before_release
                ) &&
                release_measurement.size() == 6
            ) {
                release_decision = endpoint_trim_continuity_.request_release(
                    release_measurement,
                    release_feedback_is_fresh,
                    now.toSec()
                );
                request_routed = true;
            }
            release_status = endpoint_trim_command_order_.record_release(
                phase_before_release,
                release_decision,
                request_routed
            );
            release_failure_code = release_decision.code;
        }
    }
    response.success = true;
    if (request.data) {
        std::ostringstream message;
        message << "task endpoint precision lease active for at most "
                << std::fixed << std::setprecision(1)
                << endpoint_feedback_trim_task_lease_timeout_sec_ << "s";
        response.message = message.str();
        ROS_INFO("%s", response.message.c_str());
    } else {
        if (release_status == EndpointTrimReleaseStatus::PENDING) {
            response.success = true;
            response.message =
                "task endpoint precision lease release pending serialized encoder response";
        } else if (release_status == EndpointTrimReleaseStatus::COMPLETED) {
            response.success = true;
            response.message =
                "task endpoint precision lease release completed; serialized target install pending";
        } else if (release_status == EndpointTrimReleaseStatus::REJECTED) {
            response.success = false;
            response.message = "task endpoint precision lease release rejected: " +
                (release_failure_code.empty()
                    ? std::string("invalid coordinator state")
                    : release_failure_code);
        } else {
            response.success = true;
            response.message =
                "task endpoint precision lease already inactive; no release handoff required";
        }
        ROS_INFO("%s", response.message.c_str());
    }
    return true;
}

void AliciaDDriverNode::clear_retained_command_state()
{
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        has_latest_command_ = false;
        latest_joint_angles_.clear();
        endpoint_trim_continuity_ =
            EndpointTrimContinuity(endpoint_trim_config_);
        endpoint_trim_command_order_.reset();
        endpoint_trim_reference_joint_angles_.clear();
        endpoint_trim_upstream_reference_since_ = ros::Time(0);
        endpoint_feedback_trim_active_ = false;
        endpoint_feedback_trim_quiescent_ = false;
        endpoint_feedback_trim_offsets_.clear();
        endpoint_trim_feedback_anchor_joint_angles_.clear();
        endpoint_trim_feedback_stable_since_.clear();
        endpoint_trim_last_feedback_sample_time_ = ros::Time(0);
        endpoint_trim_waiting_for_feedback_response_ = false;
        endpoint_trim_response_start_joint_angles_.clear();
        endpoint_trim_response_joint_mask_.clear();
        endpoint_trim_response_wait_since_ = ros::Time(0);
        endpoint_trim_last_response_latency_sec_ = 0.0;
        endpoint_trim_stalled_retry_count_ = 0;
        endpoint_feedback_trim_iteration_ = 0;
        endpoint_feedback_trim_task_lease_active_ = false;
        endpoint_feedback_trim_task_lease_expires_at_ = ros::Time(0);
        endpoint_feedback_trim_task_lease_reference_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(send_mutex_);
        command_state_seeded_from_feedback_ = false;
        cmd_joint_velocities_.assign(6, 0.0);
        cmd_gripper_vel_rad_s_ = 0.0;
        last_sent_sdk_command_frame_.clear();
        last_sent_sdk_command_time_ = ros::Time(0);
    }
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        has_real_feedback_ = false;
        last_feedback_time_ = ros::Time(0);
        last_accepted_joint_feedback_time_ = ros::Time(0);
        last_joint_feedback_frame_time_ = ros::Time(0);
        pending_joint_feedback_.clear();
        pending_joint_feedback_count_ = 0;
        last_streamed_joint_positions_.clear();
        last_streamed_joint_positions_time_ = ros::Time(0);
    }
}

void AliciaDDriverNode::publish_actuation_status()
{
    const double now_sec = ros::Time::now().toSec();
    std::string status;
    bool confirmed = false;
    {
        std::lock_guard<std::mutex> lock(actuation_mutex_);
        actuation_confirmation_.update(now_sec);
        status = actuation_confirmation_.status_text();
        confirmed = actuation_confirmation_.motion_confirmed(now_sec);
    }

    std_msgs::String status_msg;
    status_msg.data = status;
    actuation_status_pub_.publish(status_msg);
    std_msgs::Bool motion_msg;
    motion_msg.data = confirmed;
    motion_enabled_pub_.publish(motion_msg);

    if (
        status != last_published_actuation_status_ ||
        !has_published_motion_enabled_ ||
        confirmed != last_published_motion_enabled_
    ) {
        ROS_WARN(
            "Actuation state changed: status=%s motion_enabled=%s",
            status.c_str(),
            confirmed ? "true" : "false"
        );
        last_published_actuation_status_ = status;
        last_published_motion_enabled_ = confirmed;
        has_published_motion_enabled_ = true;
    }
}

bool AliciaDDriverNode::request_positive_enable(const std::string& source)
{
    const ros::Time now = ros::Time::now();
    bool sustained_temperature_protection = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const bool temperature_fresh =
            has_temperature_feedback_ &&
            !last_temperature_time_.isZero() &&
            (now - last_temperature_time_).toSec() <= 3.0;
        sustained_temperature_protection =
            temperature_fresh &&
            (last_run_status_ == 0xE1 || last_run_status_ == 0xE2) &&
            consecutive_high_temperature_samples_ >=
                temperature_over_limit_confirm_samples_;
    }
    if (sustained_temperature_protection) {
        {
            std::lock_guard<std::mutex> lock(actuation_mutex_);
            actuation_confirmation_.mark_overheat_blocked(
                "SUSTAINED_SAME_CHANNEL_TEMPERATURE",
                now.toSec()
            );
        }
        publish_actuation_status();
        ROS_ERROR(
            "Rejected positive enable from %s: sustained same-channel temperature protection is active.",
            source.c_str()
        );
        return false;
    }

    clear_retained_command_state();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        motion_commands_enabled_ = true;
    }
    {
        std::lock_guard<std::mutex> lock(actuation_mutex_);
        actuation_confirmation_.reset_for_positive_enable(now.toSec());
    }
    publish_actuation_status();

    ROS_INFO(
        "Requesting positive SDK torque_on from %s; encoder confirmation remains pending.",
        source.c_str()
    );
    const std::vector<uint8_t> torque_on_frame = {
        0xAA, 0x05, 0x00, 0x01, 0x01, 0xF9, 0xFF
    };
    const bool wrote = communicator_->write_raw_frame(torque_on_frame);
    if (!wrote) {
        {
            std::lock_guard<std::mutex> lock(actuation_mutex_);
            actuation_confirmation_.mark_unconfirmed(
                "TORQUE_ON_WRITE_FAILED",
                ros::Time::now().toSec()
            );
        }
        publish_actuation_status();
        ROS_ERROR(
            "Positive SDK torque_on write failed for source %s.",
            source.c_str()
        );
    }
    return wrote;
}

void AliciaDDriverNode::state_poll_timer_callback(const ros::TimerEvent& event)
{
    if (!communicator_ || !communicator_->is_connected()) return;

    // The controller can interleave responses when two query timers fire at
    // once. Schedule every diagnostic query in a dedicated state-poll slot so
    // only one request is outstanding on the half-duplex transport.
    const ros::Time now = ros::Time::now();
    const bool diagnostic_queries_suppressed =
        diagnostic_query_suppressed_for_motion(now);
    const bool temperature_due = !diagnostic_queries_suppressed &&
        temperature_poll_rate_hz_ > 0.0 &&
        (last_temperature_query_time_.isZero() ||
         (now - last_temperature_query_time_).toSec() >= 1.0 / temperature_poll_rate_hz_);
    const bool self_check_due = !diagnostic_queries_suppressed &&
        self_check_poll_rate_hz_ > 0.0 &&
        (last_self_check_query_time_.isZero() ||
         (now - last_self_check_query_time_).toSec() >= 1.0 / self_check_poll_rate_hz_);

    if (diagnostic_queries_suppressed) {
        ROS_INFO_THROTTLE(
            5.0,
            "Deferring SDK temperature/self-check queries while the arm target is active; joint feedback polling remains enabled."
        );
    }

    if (self_check_due) {
        // Exact official SDK self-check query.
        static const std::vector<uint8_t> self_check_query_frame = {
            FRAME_START_BYTE, SDK_CMD_SELF_CHECK, SDK_FUNC_SELF_CHECK,
            0x00, 0xFE, 0x93, FRAME_END_BYTE
        };
        if (communicator_->write_raw_frame(self_check_query_frame)) {
            last_self_check_query_time_ = now;
        }
        return;
    }

    if (temperature_due) {
        static const std::vector<uint8_t> temperature_query_frame = {
            FRAME_START_BYTE, SDK_CMD_JOINT, SDK_FUNC_QUERY_TEMPERATURE,
            0x01, 0xFE, 0xAD, FRAME_END_BYTE
        };
        if (communicator_->write_raw_frame(temperature_query_frame)) {
            last_temperature_query_time_ = now;
        }
        return;
    }

    // SDK joint+gripper query:
    // AA 06 00 01 FE 9A FF
    static const std::vector<uint8_t> joint_state_query_frame = {
        FRAME_START_BYTE, SDK_CMD_JOINT, SDK_FUNC_QUERY_JOINT_GRIPPER,
        0x01, 0xFE, 0x9A, FRAME_END_BYTE
    };
    communicator_->write_raw_frame(joint_state_query_frame);
}

bool AliciaDDriverNode::diagnostic_query_suppressed_for_motion(
    const ros::Time& now
)
{
    if (!suppress_diagnostic_queries_while_motion_active_) return false;

    bool has_command = false;
    std::vector<double> target_joint_angles;
    double target_gripper_rad = 0.0;
    ros::Time last_reference_change;
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        has_command = has_latest_command_;
        target_joint_angles = latest_joint_angles_;
        target_gripper_rad = latest_gripper_rad_;
        last_reference_change = last_motion_reference_change_time_;
    }
    if (!has_command) return false;

    if (
        !last_reference_change.isZero() &&
        std::max(0.0, (now - last_reference_change).toSec()) <
            diagnostic_query_motion_quiet_sec_
    ) {
        return true;
    }

    bool feedback_is_fresh = false;
    std::vector<double> feedback_joint_angles;
    double feedback_gripper_rad = 0.0;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        feedback_is_fresh =
            has_real_feedback_ &&
            !last_feedback_time_.isZero() &&
            std::max(0.0, (now - last_feedback_time_).toSec()) <=
                feedback_stale_timeout_sec_;
        feedback_joint_angles = current_joint_positions_;
        feedback_gripper_rad = current_gripper_position_;
    }
    if (
        !feedback_is_fresh ||
        target_joint_angles.size() != feedback_joint_angles.size() ||
        target_joint_angles.empty()
    ) {
        return true;
    }

    for (size_t i = 0; i < target_joint_angles.size(); ++i) {
        const double target = target_joint_angles[i];
        const double feedback = feedback_joint_angles[i];
        if (
            !std::isfinite(target) ||
            !std::isfinite(feedback) ||
            std::abs(target - feedback) >
                diagnostic_query_motion_error_rad_
        ) {
            return true;
        }
    }
    if (
        !std::isfinite(target_gripper_rad) ||
        !std::isfinite(feedback_gripper_rad) ||
        std::abs(target_gripper_rad - feedback_gripper_rad) >
            diagnostic_query_motion_error_rad_
    ) {
        return true;
    }
    return false;
}


void AliciaDDriverNode::reconnect_callback(const ros::TimerEvent& event)
{
    if (!communicator_->is_connected()) {
        ROS_INFO("Attempting to reconnect...");
        if (communicator_->connect()) {
            if (auto_torque_on_startup_) {
                ROS_WARN("Reconnect successful. auto_torque_on_startup is enabled; requesting torque-on.");
                request_positive_enable("serial_reconnect");
            } else {
                clear_retained_command_state();
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    motion_commands_enabled_ = false;
                }
                {
                    std::lock_guard<std::mutex> lock(actuation_mutex_);
                    actuation_confirmation_.mark_unconfirmed(
                        "SERIAL_RECONNECTED_TORQUE_UNCHANGED",
                        ros::Time::now().toSec()
                    );
                }
                publish_actuation_status();
                ROS_INFO("Reconnect successful in monitor-only mode; torque remains unchanged.");
            }
        }
    }

}

void AliciaDDriverNode::joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!communicator_->is_connected()) return;
    const ros::Time command_time = ros::Time::now();
    const bool endpoint_trim_explicit_gui_command =
        msg->header.frame_id == "gui_direct" ||
        msg->header.frame_id == "gui_direct_sync";

    // Measure incoming /joint_commands rate (logs once per second)
    // {
    //     static bool s_initialized = false;
    //     static bool s_log_rates = true;
    //     static size_t s_msg_count = 0;
    //     static ros::Time s_last_log(0, 0);
    //     if (!s_initialized) {
    //         ros::param::param("~log_rates", s_log_rates, true);
    //         s_last_log = ros::Time::now();
    //         s_initialized = true;
    //     }
    //     ++s_msg_count;
    //     const ros::Time now = ros::Time::now();
    //     const double dt = (now - s_last_log).toSec();
    //     if (s_log_rates && dt >= 1.0) {
    //         const double hz = static_cast<double>(s_msg_count) / dt;
    //         ROS_INFO("[Rate] /joint_commands incoming: %.1f Hz (window %.2fs, %zu msgs)", hz, dt, s_msg_count);
    //         s_msg_count = 0;
    //         s_last_log = now;
    //     }
    // }

    // Only parse and store latest command quickly; do not block the callback
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
    }

    std::vector<std::string> hardware_joint_names = {
        "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"
    };

    std::vector<double> joint_angles;
    double gripper_value = 0.0; // incoming normalized value -> radians for gripper
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        joint_angles = latest_joint_angles_;
        gripper_value = latest_gripper_rad_;
    }
    if (joint_angles.size() != hardware_joint_names.size()) {
        joint_angles.assign(hardware_joint_names.size(), 0.0);
    }

    for (size_t i = 0; i < hardware_joint_names.size(); ++i) {
        auto it = joint_map.find(hardware_joint_names[i]);
        if (it != joint_map.end()) {
            joint_angles[i] = it->second;
        }
    }

    auto it_grip = joint_map.find("right_finger");
    if (it_grip != joint_map.end()) {
        if (gripper_input_is_percent_) {
            // map [0..1] percent to radians [0..100deg]
            const double pct = std::max(0.0, std::min(1.0, it_grip->second));
            const double deg = pct * 100.0;
            gripper_value = deg * M_PI / 180.0;
        } else {
            // Treat incoming value as meters for the prismatic joint [0..stroke]
            double stroke_m = 0.05; // default 5 cm stroke per URDF
            pnh_.param<double>("gripper_stroke_m", stroke_m, 0.05);
            const double m = std::max(0.0, std::min(stroke_m, it_grip->second));
            const double pct = (stroke_m > 1e-6) ? (m / stroke_m) : 0.0; // 0..1
            const double deg = pct * 100.0; // 0..100 deg displayed in HW space
            gripper_value = deg * M_PI / 180.0; // radians for internal smoothing + HW mapping
        }
    }

    std::string actuation_rejection;
    {
        std::lock_guard<std::mutex> lock(actuation_mutex_);
        if (!actuation_confirmation_.admit_command(
                joint_angles,
                command_time.toSec(),
                &actuation_rejection
            )) {
            ROS_WARN_THROTTLE(
                1.0,
                "Rejected /joint_commands target: %s joints_deg=%s",
                actuation_rejection.c_str(),
                format_radians_as_degrees(joint_angles).c_str()
            );
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        const bool gripper_reference_changed =
            std::abs(gripper_value - latest_gripper_rad_) > 1e-6;
        const bool reference_changed =
            endpoint_trim_command_order_.observe_upstream_command(
                joint_angles,
                endpoint_trim_explicit_gui_command
                    ? EndpointTrimCommandSource::EXPLICIT_GUI
                    : EndpointTrimCommandSource::TASK_CONTROLLER,
                // An untagged task command reaches this point only after the
                // active gui_direct gesture has timed out. gui_direct_sync is
                // tagged GUI authority itself, so it cannot authorize a task
                // handoff by accident.
                !endpoint_trim_explicit_gui_command
            );
        if (!endpoint_trim_command_order_.last_observation_accepted()) {
            ROS_WARN_THROTTLE(
                1.0,
                "Ignored task/controller command while explicit GUI authority is active"
            );
            return;
        }
        if (endpoint_trim_explicit_gui_command) {
            const EndpointTrimDecision gui_handoff =
                endpoint_trim_continuity_.explicit_gui_handoff(
                    joint_angles,
                    command_time.toSec()
                );
            endpoint_trim_command_order_.mark_command_applied();
            endpoint_trim_reference_joint_angles_ =
                gui_handoff.reference;
            endpoint_feedback_trim_offsets_ = gui_handoff.offsets;
            endpoint_feedback_trim_active_ = false;
            endpoint_feedback_trim_quiescent_ = false;
            endpoint_trim_waiting_for_feedback_response_ = false;
            endpoint_trim_response_start_joint_angles_.clear();
            endpoint_trim_response_joint_mask_.clear();
            endpoint_trim_response_wait_since_ = ros::Time(0);
            endpoint_trim_last_response_latency_sec_ = 0.0;
            endpoint_trim_stalled_retry_count_ = 0;
            endpoint_feedback_trim_iteration_ = 0;
            endpoint_feedback_trim_task_lease_active_ = false;
            endpoint_feedback_trim_task_lease_expires_at_ = ros::Time(0);
            endpoint_feedback_trim_task_lease_reference_.clear();
        }
        if (reference_changed) {
            if (endpoint_feedback_trim_task_lease_active_) {
                endpoint_feedback_trim_task_lease_active_ = false;
                endpoint_feedback_trim_task_lease_expires_at_ = ros::Time(0);
                endpoint_feedback_trim_task_lease_reference_.clear();
            }
            endpoint_trim_upstream_reference_since_ = command_time;
            endpoint_trim_feedback_anchor_joint_angles_.clear();
            endpoint_trim_feedback_stable_since_.assign(
                joint_angles.size(),
                ros::Time(0)
            );
        }
        if (reference_changed || gripper_reference_changed) {
            last_motion_reference_change_time_ = command_time;
        }
        latest_joint_angles_ = joint_angles;
        latest_gripper_rad_ = gripper_value;
        has_latest_command_ = true;
    }

    if (log_command_flow_) {
        ROS_INFO_THROTTLE(1.0, "Received /joint_commands target: joints_deg=%s gripper_rad=%.3f",
                          format_radians_as_degrees(joint_angles).c_str(), gripper_value);
    }
}

void AliciaDDriverNode::send_command_timer_callback(const ros::TimerEvent& event)
{
    if (!communicator_ || !communicator_->is_connected()) return;

    std::unique_lock<std::mutex> send_lock(send_mutex_, std::try_to_lock);
    if (!send_lock.owns_lock()) return;

    const ros::Time now = ros::Time::now();

    bool feedback_ready = false;
    bool feedback_stale = true;
    bool protection_latched = false;
    bool motion_enabled = false;
    bool actuation_overheat_blocked = false;
    std::vector<double> feedback_joint_angles;
    double feedback_gripper_rad = 0.0;
    ros::Time feedback_sample_time;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        feedback_ready = has_real_feedback_;
        feedback_sample_time = last_accepted_joint_feedback_time_;
        feedback_stale =
            !has_real_feedback_ ||
            feedback_sample_time.isZero() ||
            (now - feedback_sample_time).toSec() < 0.0 ||
            (now - feedback_sample_time).toSec() >
                feedback_stale_timeout_sec_;
        protection_latched = protection_fault_latched_;
        motion_enabled = motion_commands_enabled_;
        feedback_joint_angles = current_joint_positions_;
        feedback_gripper_rad = current_gripper_position_;
    }
    {
        std::lock_guard<std::mutex> lock(actuation_mutex_);
        actuation_overheat_blocked =
            actuation_confirmation_.state() ==
                ActuationState::OVERHEAT_BLOCKED;
    }
    const EndpointTrimTransmissionGate endpoint_trim_gate{
        motion_enabled,
        actuation_overheat_blocked,
        protection_latched,
        pause_commands_when_feedback_stale_,
        feedback_ready,
        feedback_stale,
    };

    std::vector<double> joint_angles;
    double gripper_value = 0.0;
    EndpointTrimDecision endpoint_trim_decision{};
    EndpointTrimPhase endpoint_trim_phase_before = EndpointTrimPhase::IDLE;
    std::string endpoint_trim_code_before;
    std::vector<double> endpoint_trim_target_before;
    double endpoint_trim_response_age_sec = 0.0;
    bool endpoint_feedback_trim_task_lease_expired = false;
    bool accepted_feedback_sample_is_new = false;
    bool endpoint_trim_target_changed = false;
    std::string endpoint_trim_terminal_release_code;

    // Endpoint trim continuity begin
    {
        std::lock_guard<std::mutex> lock(latest_cmd_mutex_);
        if (!has_latest_command_) return;

        joint_angles = latest_joint_angles_;
        gripper_value = latest_gripper_rad_;
        const EndpointTrimDecision& initial_state =
            endpoint_trim_continuity_.state();
        endpoint_trim_phase_before = initial_state.phase;
        endpoint_trim_code_before = initial_state.code;
        endpoint_trim_target_before = initial_state.composed_target;
        endpoint_trim_command_order_.capture_terminal_release(initial_state);

        if (
            endpoint_trim_command_order_.
                newer_task_command_requires_handoff() &&
            endpoint_trim_command_order_.upstream_target().size() == 6
        ) {
            endpoint_trim_decision =
                endpoint_trim_continuity_.task_controller_handoff(
                    endpoint_trim_command_order_.upstream_target(),
                    now.toSec()
                );
            endpoint_trim_command_order_.mark_command_applied();
        } else {
            endpoint_trim_decision = initial_state;
        }

        const bool accepted_feedback_is_fresh =
            feedback_ready &&
            !feedback_stale &&
            feedback_joint_angles.size() == 6 &&
            !feedback_sample_time.isZero();
        accepted_feedback_sample_is_new =
            accepted_feedback_is_fresh &&
            (
                endpoint_trim_last_feedback_sample_time_.isZero() ||
                feedback_sample_time >
                    endpoint_trim_last_feedback_sample_time_
            );
        if (
            endpoint_feedback_trim_task_lease_active_ &&
            !endpoint_feedback_trim_task_lease_expires_at_.isZero() &&
            now >= endpoint_feedback_trim_task_lease_expires_at_
        ) {
            endpoint_feedback_trim_task_lease_active_ = false;
            endpoint_feedback_trim_task_lease_expires_at_ = ros::Time(0);
            endpoint_feedback_trim_task_lease_reference_.clear();
            endpoint_feedback_trim_task_lease_expired = true;
            const EndpointTrimPhase phase_before_release =
                endpoint_trim_decision.phase;

            std::vector<double> release_measurement =
                feedback_joint_angles;
            if (release_measurement.size() != 6) {
                release_measurement =
                    endpoint_trim_decision.composed_target;
            }
            bool request_routed = false;
            if (
                endpoint_trim_release_request_allowed(
                    phase_before_release
                ) &&
                release_measurement.size() == 6
            ) {
                endpoint_trim_decision =
                    endpoint_trim_continuity_.request_release(
                        release_measurement,
                        accepted_feedback_is_fresh,
                        now.toSec()
                );
                request_routed = true;
            }
            endpoint_trim_command_order_.record_release(
                phase_before_release,
                endpoint_trim_decision,
                request_routed
            );
        }
        // Lease-expiry release request end

        if (accepted_feedback_sample_is_new) {
            endpoint_trim_decision =
                endpoint_trim_continuity_.note_feedback(
                    feedback_joint_angles,
                    feedback_sample_time.toSec(),
                    now.toSec()
                );
            endpoint_trim_last_feedback_sample_time_ =
                feedback_sample_time;
            endpoint_trim_command_order_.capture_terminal_release(
                endpoint_trim_decision
            );

            if (
                endpoint_trim_feedback_anchor_joint_angles_.size() != 6 ||
                endpoint_trim_feedback_stable_since_.size() != 6
            ) {
                endpoint_trim_feedback_anchor_joint_angles_ =
                    feedback_joint_angles;
                endpoint_trim_feedback_stable_since_.assign(
                    6,
                    feedback_sample_time
                );
            } else {
                for (size_t i = 0; i < 6; ++i) {
                    if (
                        std::abs(
                            feedback_joint_angles[i] -
                            endpoint_trim_feedback_anchor_joint_angles_[i]
                        ) >
                            SDK_JOINT_QUANTIZATION_RAD + 1e-12
                    ) {
                        endpoint_trim_feedback_anchor_joint_angles_[i] =
                            feedback_joint_angles[i];
                        endpoint_trim_feedback_stable_since_[i] =
                            feedback_sample_time;
                    }
                }
            }
        }


        endpoint_trim_decision =
            endpoint_trim_continuity_.update(now.toSec());
        endpoint_trim_command_order_.capture_terminal_release(
            endpoint_trim_decision
        );

        std::vector<uint8_t> endpoint_feedback_stable_by_joint(6, 0);
        double maximum_feedback_error_rad = 0.0;
        bool has_stable_joint_above_round_trip_floor = false;
        const bool feedback_shape_valid =
            feedback_joint_angles.size() == 6 &&
            endpoint_trim_decision.reference.size() == 6 &&
            endpoint_trim_feedback_stable_since_.size() == 6;
        if (feedback_shape_valid) {
            for (size_t i = 0; i < 6; ++i) {
                maximum_feedback_error_rad = std::max(
                    maximum_feedback_error_rad,
                    std::abs(
                        endpoint_trim_decision.reference[i] -
                        feedback_joint_angles[i]
                    )
                );
                const double joint_stable_age_sec =
                    endpoint_trim_feedback_stable_since_[i].isZero()
                        ? 0.0
                        : (
                            now -
                            endpoint_trim_feedback_stable_since_[i]
                        ).toSec();
                const bool stable_above_floor =
                    joint_stable_age_sec >=
                        endpoint_feedback_trim_stable_sec_ &&
                    std::abs(
                        endpoint_trim_decision.reference[i] -
                        feedback_joint_angles[i]
                    ) >
                        ENDPOINT_FEEDBACK_TRIM_ROUND_TRIP_FLOOR_RAD;
                endpoint_feedback_stable_by_joint[i] =
                    stable_above_floor ? 1 : 0;
                has_stable_joint_above_round_trip_floor =
                    has_stable_joint_above_round_trip_floor ||
                    stable_above_floor;
            }
        }

        double maximum_reference_delta_rad = 0.0;
        if (
            endpoint_trim_decision.reference.size() ==
                joint_angles.size()
        ) {
            for (size_t i = 0; i < joint_angles.size(); ++i) {
                maximum_reference_delta_rad = std::max(
                    maximum_reference_delta_rad,
                    std::abs(
                        joint_angles[i] -
                        endpoint_trim_decision.reference[i]
                    )
                );
            }
        }
        const double endpoint_reference_stable_age_sec =
            endpoint_trim_upstream_reference_since_.isZero()
                ? 0.0
                : (now - endpoint_trim_upstream_reference_since_).toSec();
        const bool endpoint_reference_stable =
            feedback_shape_valid &&
            endpoint_reference_stable_age_sec >=
                endpoint_feedback_trim_stable_sec_ &&
            maximum_reference_delta_rad <=
                SDK_JOINT_QUANTIZATION_RAD;
        const bool endpoint_error_inside_trim_window =
            maximum_feedback_error_rad <=
                endpoint_feedback_trim_max_rad_;
        const bool endpoint_feedback_trim_update_allowed =
            endpoint_feedback_trim_enabled_ ||
            endpoint_feedback_trim_task_lease_active_;

        if (
            endpoint_trim_gate.allows_correction() &&
            endpoint_feedback_trim_update_allowed &&
            accepted_feedback_sample_is_new &&
            endpoint_reference_stable &&
            endpoint_error_inside_trim_window &&
            has_stable_joint_above_round_trip_floor &&
            (
                endpoint_trim_decision.phase == EndpointTrimPhase::IDLE ||
                endpoint_trim_decision.phase ==
                    EndpointTrimPhase::QUIESCENT
            )
        ) {
            std::vector<double> activation_offsets =
                endpoint_trim_decision.offsets;
            if (activation_offsets.size() != 6) {
                activation_offsets.assign(6, 0.0);
            }
            endpoint_trim_decision =
                endpoint_trim_continuity_.activate(
                    endpoint_trim_decision.reference,
                    activation_offsets,
                    now.toSec()
                );
        }

        if (
            endpoint_trim_gate.allows_correction() &&
            endpoint_trim_decision.phase == EndpointTrimPhase::ACTIVE_READY &&
            endpoint_feedback_trim_update_allowed &&
            accepted_feedback_sample_is_new &&
            endpoint_reference_stable &&
            endpoint_error_inside_trim_window &&
            has_stable_joint_above_round_trip_floor
        ) {
            endpoint_trim_decision =
                endpoint_trim_continuity_.request_correction(
                    feedback_joint_angles,
                    endpoint_feedback_stable_by_joint,
                    endpoint_feedback_trim_gain_,
                    now.toSec()
                );
        }

        // Install the coordinator-owned pair atomically. The serialized target
        // below is always taken from the same immutable decision.
        endpoint_trim_reference_joint_angles_ =
            endpoint_trim_decision.reference;
        endpoint_feedback_trim_offsets_ =
            endpoint_trim_decision.offsets;
        endpoint_feedback_trim_active_ =
            endpoint_trim_decision.phase != EndpointTrimPhase::IDLE;
        endpoint_feedback_trim_quiescent_ =
            endpoint_trim_decision.phase == EndpointTrimPhase::QUIESCENT;
        endpoint_trim_waiting_for_feedback_response_ =
            endpoint_trim_decision.phase ==
                EndpointTrimPhase::WAITING_RESPONSE ||
            endpoint_trim_decision.phase ==
                EndpointTrimPhase::PENDING_RELEASE;

        endpoint_trim_target_changed =
            endpoint_trim_target_before !=
                endpoint_trim_decision.composed_target;
        const bool endpoint_trim_response_generation_started =
            endpoint_trim_waiting_for_feedback_response_ &&
            endpoint_trim_target_changed;
        if (
            endpoint_trim_waiting_for_feedback_response_ &&
            (
                endpoint_trim_response_generation_started ||
                endpoint_trim_response_wait_since_.isZero()
            )
        ) {
            endpoint_trim_response_wait_since_ = now;
        }
        if (!endpoint_trim_response_wait_since_.isZero()) {
            endpoint_trim_response_age_sec =
                (now - endpoint_trim_response_wait_since_).toSec();
        }
        if (!endpoint_trim_waiting_for_feedback_response_) {
            endpoint_trim_response_wait_since_ = ros::Time(0);
        }
        if (endpoint_trim_target_changed) {
            ++endpoint_feedback_trim_iteration_;
        }
        endpoint_trim_terminal_release_code =
            endpoint_trim_command_order_.consume_terminal_release_code();
    }
    // Endpoint trim continuity end

    if (
        endpoint_trim_phase_before != endpoint_trim_decision.phase ||
        endpoint_trim_target_changed ||
        endpoint_feedback_trim_task_lease_expired ||
        !endpoint_trim_terminal_release_code.empty() ||
        endpoint_trim_code_before != endpoint_trim_decision.code
    ) {
        const char* code = nullptr;
        if (!endpoint_trim_terminal_release_code.empty()) {
            if (
                endpoint_trim_terminal_release_code ==
                    "ENDPOINT_TRIM_RELEASE_SETTLED"
            ) {
                code = "ENDPOINT_TRIM_RELEASE_SETTLED";
            } else if (
                endpoint_trim_terminal_release_code ==
                    "ENDPOINT_TRIM_RESPONSE_TIMEOUT"
            ) {
                code = "ENDPOINT_TRIM_RESPONSE_TIMEOUT";
            } else {
                code = "ENDPOINT_TRIM_CONTINUITY_VIOLATION";
            }
        } else {
            code = endpoint_trim_decision.code.empty()
                ? "ENDPOINT_TRIM_STATE_TRANSITION"
                : endpoint_trim_decision.code.c_str();
        }
        ROS_INFO(
            "Endpoint trim transition code=%s phase=%s->%s "
            "response_age=%.3fs applied_step_deg=%s "
            "target_before_deg=%s target_after_deg=%s",
            code,
            endpoint_trim_phase_name(endpoint_trim_phase_before),
            endpoint_trim_phase_name(endpoint_trim_decision.phase),
            endpoint_trim_response_age_sec,
            format_radians_as_degrees(
                endpoint_trim_decision.applied_step
            ).c_str(),
            format_radians_as_degrees(
                endpoint_trim_target_before
            ).c_str(),
            format_radians_as_degrees(
                endpoint_trim_decision.composed_target
            ).c_str()
        );
    }
    if (endpoint_feedback_trim_task_lease_expired) {
        ROS_WARN(
            "Task endpoint precision lease expired; serialized release "
            "preserved the coordinator target"
        );
    }

    if (!motion_enabled) {
        ROS_WARN_THROTTLE(1.0, "Blocking SDK command stream: motion has not been explicitly enabled.");
        return;
    }

    if (actuation_overheat_blocked) {
        ROS_ERROR_THROTTLE(
            1.0,
            "Blocking SDK command stream: measured actuation state is OVERHEAT_BLOCKED; no torque_off frame was sent."
        );
        return;
    }

    if (protection_latched) {
        ROS_ERROR_THROTTLE(1.0, "Blocking SDK command stream: hardware protection fault is latched.");
        return;
    }

    if (pause_commands_when_feedback_stale_ && (!feedback_ready || feedback_stale)) {
        ROS_WARN_THROTTLE(1.0,
                          "Pausing SDK command stream: hardware feedback is %s (timeout %.2fs).",
                          feedback_ready ? "stale" : "not ready",
                          feedback_stale_timeout_sec_);
        return;
    }

    // A driver-only restart can receive controller commands before its first
    // serial feedback frame because ros_control remains online. Seed the
    // hardware command interpolator from that first valid measured state, not
    // from constructor zeroes, then continue toward the current ROS target.
    if (
        !command_state_seeded_from_feedback_ &&
        feedback_joint_angles.size() == cmd_joint_angles_.size()
    ) {
        cmd_joint_angles_ = feedback_joint_angles;
        cmd_gripper_rad_ = feedback_gripper_rad;
        cmd_joint_velocities_.assign(cmd_joint_angles_.size(), 0.0);
        cmd_gripper_vel_rad_s_ = 0.0;
        command_state_seeded_from_feedback_ = true;
        ROS_INFO(
            "Seeded SDK command interpolation from first valid real feedback: joints_deg=%s",
            format_radians_as_degrees(cmd_joint_angles_).c_str()
        );
    }

    // Interpolate toward latest command (slew limiting)
    if (use_trajectory_smoothing_) {
        const double dt = 1.0 / std::max(1.0, command_rate_hz_);

        // Trapezoidal profile per joint: accelerate to velocity, cruise, decelerate toward target
        for (size_t i = 0; i < 6 && i < joint_angles.size(); ++i) {
            const double pos = cmd_joint_angles_[i];
            double vel = cmd_joint_velocities_[i];
            const double target = joint_angles[i];
            const double error = target - pos;

            // Compute desired sign and braking velocity needed
            const double sign = (error >= 0.0) ? 1.0 : -1.0;
            const double v_max = max_joint_velocity_rad_s_;
            const double a_max = max_joint_accel_rad_s2_;

            // Distance needed to brake to zero from current speed
            const double brake_dist = (vel * vel) / (2.0 * std::max(1e-6, a_max));
            const double dist = std::abs(error);

            // Decide whether to accelerate or decelerate
            if (brake_dist >= dist) {
                // Need to decelerate
                vel -= sign * a_max * dt * ((vel * sign) > 0 ? 1.0 : -1.0);
            } else {
                // Can accelerate toward target
                vel += sign * a_max * dt;
            }
            // Clamp velocity
            if (vel > v_max) vel = v_max;
            if (vel < -v_max) vel = -v_max;

            // Integrate position
            double new_pos = pos + vel * dt;

            // If we would cross the target this step, snap to target and zero velocity
            if ((target - pos) * (target - new_pos) <= 0.0) {
                new_pos = target;
                vel = 0.0;
            }

            cmd_joint_angles_[i] = new_pos;
            cmd_joint_velocities_[i] = vel;
        }

        // Gripper profile
        {
            const double pos = cmd_gripper_rad_;
            double vel = cmd_gripper_vel_rad_s_;
            const double target = gripper_value;
            const double error = target - pos;
            const double sign = (error >= 0.0) ? 1.0 : -1.0;
            const double v_max = max_gripper_velocity_rad_s_;
            const double a_max = max_gripper_accel_rad_s2_;
            const double brake_dist = (vel * vel) / (2.0 * std::max(1e-6, a_max));
            const double dist = std::abs(error);

            if (brake_dist >= dist) {
                vel -= sign * a_max * dt * ((vel * sign) > 0 ? 1.0 : -1.0);
            } else {
                vel += sign * a_max * dt;
            }
            if (vel > v_max) vel = v_max;
            if (vel < -v_max) vel = -v_max;

            double new_pos = pos + vel * dt;
            if ((target - pos) * (target - new_pos) <= 0.0) {
                new_pos = target;
                vel = 0.0;
            }
            cmd_gripper_rad_ = new_pos;
            cmd_gripper_vel_rad_s_ = vel;
        }
    } else {
        cmd_joint_angles_ = joint_angles;
        cmd_gripper_rad_ = gripper_value;
        cmd_joint_velocities_.assign(6, 0.0);
        cmd_gripper_vel_rad_s_ = 0.0;
    }

    const std::vector<double> sdk_joint_angles = endpoint_trim_stream_target(
        cmd_joint_angles_,
        endpoint_trim_decision
    );

    ROS_DEBUG_THROTTLE(
        1.0,
        "Endpoint trim hold phase=%s code=%s response_age=%.3fs "
        "applied_step_deg=%s target_before_deg=%s target_after_deg=%s",
        endpoint_trim_phase_name(endpoint_trim_decision.phase),
        endpoint_trim_decision.code.empty()
            ? "NONE"
            : endpoint_trim_decision.code.c_str(),
        endpoint_trim_response_age_sec,
        format_radians_as_degrees(
            endpoint_trim_decision.applied_step
        ).c_str(),
        format_radians_as_degrees(endpoint_trim_target_before).c_str(),
        format_radians_as_degrees(
            endpoint_trim_decision.composed_target
        ).c_str()
    );

    // Build and send SDK-style joint + gripper frame:
    // [AA] [06] [03] [1C] [J1 pos lo hi speed lo hi] ... [J6] [gripper pos lo hi speed lo hi] [CRC] [FF]
    const size_t frame_size = 34;
    std::vector<uint8_t> servo_frame(frame_size, 0);

    servo_frame[0] = FRAME_START_BYTE;                // 0xAA
    servo_frame[1] = SDK_CMD_JOINT;                   // 0x06
    servo_frame[2] = SDK_FUNC_SET_JOINT_GRIPPER;      // 0x03
    servo_frame[3] = SDK_DATA_LEN_JOINT_GRIPPER;      // 0x1C
    servo_frame[frame_size - 1] = FRAME_END_BYTE;     // 0xFF

    const size_t data_start = 4;
    const uint16_t joint_speed_hw = speed_deg_s_to_hw(joint_speed_deg_s_);

    for (int joint_idx = 0; joint_idx < 6; ++joint_idx)
    {
        uint16_t hw_val = rad_to_hardware_value(sdk_joint_angles[joint_idx]);

        size_t offset = data_start + joint_idx * 4;
        servo_frame[offset]     = hw_val & 0xFF;
        servo_frame[offset + 1] = (hw_val >> 8) & 0xFF;
        servo_frame[offset + 2] = joint_speed_hw & 0xFF;
        servo_frame[offset + 3] = (joint_speed_hw >> 8) & 0xFF;
    }

    // SDK gripper value range: 0~1000
    size_t gripper_offset = data_start + 6 * 4;
    double gripper_deg = std::max(0.0, std::min(100.0, cmd_gripper_rad_ * 180.0 / M_PI));
    uint16_t gripper_hw_val = static_cast<uint16_t>((gripper_deg / 100.0) * 1000.0);
    uint16_t gripper_speed_hw = 5500;

    servo_frame[gripper_offset]     = gripper_hw_val & 0xFF;
    servo_frame[gripper_offset + 1] = (gripper_hw_val >> 8) & 0xFF;
    servo_frame[gripper_offset + 2] = gripper_speed_hw & 0xFF;
    servo_frame[gripper_offset + 3] = (gripper_speed_hw >> 8) & 0xFF;

    std::vector<uint8_t> crc_payload(servo_frame.begin() + 1, servo_frame.end() - 2);
    servo_frame[frame_size - 2] = sdk_crc32_low8(crc_payload);

    if (suppress_redundant_commands_ && servo_frame == last_sent_sdk_command_frame_) {
        bool keepalive_due = false;
        if (command_keepalive_rate_hz_ > 0.0) {
            const double keepalive_period = 1.0 / command_keepalive_rate_hz_;
            keepalive_due = last_sent_sdk_command_time_.isZero() ||
                            (now - last_sent_sdk_command_time_).toSec() >= keepalive_period;
        }
        if (!keepalive_due) {
            ROS_DEBUG_THROTTLE(5.0, "Skipping redundant SDK command frame while holding target.");
            return;
        }
    }

    const bool wrote = communicator_->write_raw_frame(servo_frame);
    if (wrote) {
        last_sent_sdk_command_frame_ = servo_frame;
        last_sent_sdk_command_time_ = now;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_streamed_joint_positions_ = sdk_joint_angles;
            last_streamed_joint_positions_time_ = now;
        }
        {
            std::lock_guard<std::mutex> lock(actuation_mutex_);
            actuation_confirmation_.note_streamed_target(
                sdk_joint_angles,
                now.toSec()
            );
        }
        publish_actuation_status();
    }
    if (log_command_flow_) {
        ROS_INFO_THROTTLE(1.0, "Streaming SDK command (%s): joints_deg=%s gripper_raw=%u crc=0x%02X",
                          wrote ? "ok" : "write_failed",
                          format_radians_as_degrees(sdk_joint_angles).c_str(),
                          static_cast<unsigned>(gripper_hw_val),
                          servo_frame[frame_size - 2]);
    }
}

void AliciaDDriverNode::process_serial_data_callback(const ros::TimerEvent& event)
{
    process_serial_data();
}

void AliciaDDriverNode::heartbeat_publish_callback(const ros::TimerEvent& event)
{
    // Republish the latest known state with a current timestamp to keep MoveIt happy
    const ros::Time now = ros::Time::now();

    // Optional compatibility mode: mirror commanded state when feedback is
    // stale. Keep disabled on real hardware so MoveIt does not report success
    // just because a command was sent.
    const double feedback_timeout = 0.1; // seconds
    if (mirror_commanded_state_when_feedback_stale_ &&
        (now - last_feedback_time_).toSec() > feedback_timeout) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < current_joint_positions_.size() && i < cmd_joint_angles_.size(); ++i) {
            current_joint_positions_[i] = cmd_joint_angles_[i];
        }
        // Map commanded gripper radians to current for consistency
        current_gripper_position_ = cmd_gripper_rad_;
    }

    std::lock_guard<std::mutex> lock2(data_mutex_);
    const double feedback_age = (now - last_feedback_time_).toSec();
    const bool feedback_ready = has_real_feedback_ && feedback_age <= feedback_stale_timeout_sec_;
    std_msgs::Bool ready_msg;
    ready_msg.data = feedback_ready;
    feedback_ready_pub_.publish(ready_msg);
    if (!feedback_ready) {
        ROS_WARN_THROTTLE(
            1.0,
            "Suppressing /joint_states heartbeat: real hardware feedback is %s (age %.2fs).",
            has_real_feedback_ ? "stale" : "not ready",
            feedback_age);
        return;
    }
    publish_joint_state();
}


void AliciaDDriverNode::process_serial_data()
{
    if (!communicator_->is_connected()) return;

    std::vector<uint8_t> packet;
    // Process all packets currently in the queue.
    while (communicator_->get_packet(packet))
    {
        if (packet.size() < 3) {
            ROS_WARN("Received short SDK packet (%zu bytes).", packet.size());
            continue;
        }
        uint8_t command_id = packet[0];
        uint8_t function_id = packet[1];
        uint8_t data_len = packet[2];
        if (packet.size() < static_cast<size_t>(data_len) + 3) {
            ROS_WARN("SDK packet size mismatch: cmd=0x%02X func=0x%02X len=%u packet=%zu",
                     command_id, function_id, data_len, packet.size());
            continue;
        }

        std::vector<uint8_t> data_payload(packet.begin() + 3, packet.begin() + 3 + data_len);
        switch (command_id) {
            case SDK_CMD_JOINT:
                if (function_id == SDK_FUNC_QUERY_JOINT_GRIPPER) {
                    parse_sdk_joint_state_frame(data_payload);
                } else if (function_id == SDK_FUNC_QUERY_TEMPERATURE) {
                    parse_sdk_temperature_frame(data_payload);
                } else if (debug_mode_) {
                    ROS_INFO("Received unhandled SDK joint frame func=0x%02X len=%u",
                             function_id, data_len);
                }
                break;
            case SDK_CMD_SELF_CHECK:
                if (function_id == SDK_FUNC_SELF_CHECK) {
                    parse_sdk_self_check_frame(data_payload);
                }
                break;
            case FEEDBACK_ERROR:
                parse_error_frame(data_payload);
                break;
            default:
                if (debug_mode_) {
                    ROS_INFO("Received unhandled SDK frame cmd=0x%02X func=0x%02X len=%u",
                             command_id, function_id, data_len);
                }
                break;
        }
    }
}

void AliciaDDriverNode::parse_sdk_joint_state_frame(const std::vector<uint8_t>& data_payload)
{
    // SDK joint+gripper feedback DATA:
    // 6 joint positions (2 bytes each) + gripper raw 0..1000 (2 bytes) + run status (1 byte)
    if (data_payload.size() < 15) {
        ROS_WARN("SDK joint state DATA too short: expected >=15 bytes, got %zu", data_payload.size());
        return;
    }

    std::array<uint16_t, 6> raw_joints{};
    bool all_zero = true;
    bool all_full_scale = true;
    for (size_t i = 0; i < raw_joints.size(); ++i) {
        const size_t idx = i * 2;
        raw_joints[i] = data_payload[idx] | (data_payload[idx + 1] << 8);
        all_zero = all_zero && raw_joints[i] == 0;
        all_full_scale = all_full_scale && raw_joints[i] >= 4095;
    }
    if (all_zero || all_full_scale) {
        // A powered-down or booting controller can first emit an invalid
        // encoder sentinel and then a syntactically valid but transient
        // position. Do not retain the pre-power-cycle feedback baseline or a
        // command target across that boundary: either can make the first
        // stable encoder sample look like an impossible jump and leave the
        // GUI latched to stale joints. This only clears software bootstrap
        // state; it does not change motion_commands_enabled_ or transmit a
        // stop, disable, or torque command.
        clear_retained_command_state();
        {
            std::lock_guard<std::mutex> lock(actuation_mutex_);
            actuation_confirmation_.mark_unconfirmed(
                "ENCODER_FEEDBACK_NOT_READY",
                ros::Time::now().toSec()
            );
        }
        publish_actuation_status();
        ROS_WARN_THROTTLE(
            1.0,
            "Rejected invalid SDK joint feedback: all six encoders are %s; waiting for controller power/encoder readiness.",
            all_zero ? "zero" : "full-scale");
        return;
    }

    std::vector<double> candidate_joint_positions(raw_joints.size(), 0.0);
    for (size_t i = 0; i < raw_joints.size(); ++i) {
        candidate_joint_positions[i] = hardware_value_to_rad(raw_joints[i]);
    }

    {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const ros::Time feedback_time = ros::Time::now();

    bool accept_joint_positions = true;
    bool accepted_joint_feedback_discontinuity = false;
    double maximum_joint_delta_rad = 0.0;
    size_t maximum_joint_delta_index = 0;
    double allowed_joint_delta_rad = 0.0;
    if (
        reject_implausible_joint_feedback_ &&
        !last_accepted_joint_feedback_time_.isZero() &&
        !last_joint_feedback_frame_time_.isZero() &&
        candidate_joint_positions.size() == current_joint_positions_.size()
    ) {
        const double frame_interval_sec = std::max(
            0.0,
            (feedback_time - last_joint_feedback_frame_time_).toSec()
        );
        // A missing response must not make the next jump arbitrarily large.
        // Use at most two nominal poll periods for direct physical
        // plausibility. A larger recovery still has the separate repeated,
        // command-consistent path below; without a streamed command it stays
        // rejected instead of becoming valid merely because telemetry paused.
        const double maximum_jump_interval_sec =
            2.0 / std::max(1.0, state_poll_rate_hz_);
        const double bounded_frame_interval_sec = std::min(
            frame_interval_sec,
            maximum_jump_interval_sec
        );
        allowed_joint_delta_rad =
            feedback_jump_base_tolerance_rad_ +
            feedback_max_velocity_rad_s_ * bounded_frame_interval_sec;
        for (size_t i = 0; i < candidate_joint_positions.size(); ++i) {
            const double delta = std::abs(
                candidate_joint_positions[i] - current_joint_positions_[i]
            );
            if (delta > maximum_joint_delta_rad) {
                maximum_joint_delta_rad = delta;
                maximum_joint_delta_index = i;
            }
        }
        if (maximum_joint_delta_rad > allowed_joint_delta_rad) {
            bool has_streamed_command_reference = false;
            bool command_inconsistent_jump = false;
            bool command_consistent_recovery = false;
            double previous_command_error_rad = 0.0;
            double candidate_command_error_rad = 0.0;
            double command_consistency_margin_rad = 0.0;
            double streamed_command_age_sec = std::numeric_limits<double>::infinity();
            if (
                reject_command_inconsistent_joint_feedback_ &&
                last_streamed_joint_positions_.size() ==
                    candidate_joint_positions.size() &&
                !last_streamed_joint_positions_time_.isZero()
            ) {
                streamed_command_age_sec = (
                    feedback_time - last_streamed_joint_positions_time_
                ).toSec();
                if (streamed_command_age_sec >= 0.0) {
                    has_streamed_command_reference = true;
                    // A successfully streamed position remains the physical
                    // hold reference after command streaming pauses.  Do not
                    // let two repeated, multi-radian corrupt frames become
                    // acceptable merely because the retained command is more
                    // than one feedback timeout old.  Genuine manual or
                    // unpowered motion still arrives as physically bounded
                    // incremental feedback; reconnect clears this retained
                    // reference before accepting a new powered state.
                    previous_command_error_rad = std::abs(
                        current_joint_positions_[maximum_joint_delta_index] -
                        last_streamed_joint_positions_[
                            maximum_joint_delta_index
                        ]
                    );
                    candidate_command_error_rad = std::abs(
                        candidate_joint_positions[
                            maximum_joint_delta_index
                        ] -
                        last_streamed_joint_positions_[
                            maximum_joint_delta_index
                        ]
                    );
                    command_consistency_margin_rad = std::max(
                        allowed_joint_delta_rad,
                        feedback_jump_confirmation_tolerance_rad_
                    );
                    command_inconsistent_jump =
                        candidate_command_error_rad >
                        previous_command_error_rad +
                            command_consistency_margin_rad;
                    command_consistent_recovery =
                        candidate_command_error_rad +
                            command_consistency_margin_rad <
                        previous_command_error_rad;
                }
            }
            bool matches_pending =
                pending_joint_feedback_.size() ==
                candidate_joint_positions.size();
            if (matches_pending) {
                for (size_t i = 0; i < candidate_joint_positions.size(); ++i) {
                    if (
                        std::abs(
                            candidate_joint_positions[i] -
                            pending_joint_feedback_[i]
                        ) > feedback_jump_confirmation_tolerance_rad_
                    ) {
                        matches_pending = false;
                        break;
                    }
                }
            }
            if (matches_pending) {
                ++pending_joint_feedback_count_;
            } else {
                pending_joint_feedback_ = candidate_joint_positions;
                pending_joint_feedback_count_ = 1;
            }
            accept_joint_positions =
                pending_joint_feedback_count_ >=
                    feedback_jump_confirm_samples_ &&
                command_consistent_recovery;
            if (command_inconsistent_jump) {
                accept_joint_positions = false;
                ROS_WARN(
                    "Rejected SDK joint feedback discontinuity inconsistent with fresh streamed command: joint=%zu delta=%.6frad allowed=%.6frad previous_command_error=%.6frad candidate_command_error=%.6frad margin=%.6frad command_age=%.3fs confirmation=%d/%d previous_deg=%s candidate_deg=%s command_deg=%s payload=[%s]",
                    maximum_joint_delta_index + 1,
                    maximum_joint_delta_rad,
                    allowed_joint_delta_rad,
                    previous_command_error_rad,
                    candidate_command_error_rad,
                    command_consistency_margin_rad,
                    streamed_command_age_sec,
                    pending_joint_feedback_count_,
                    feedback_jump_confirm_samples_,
                    format_radians_as_degrees(current_joint_positions_).c_str(),
                    format_radians_as_degrees(candidate_joint_positions).c_str(),
                    format_radians_as_degrees(
                        last_streamed_joint_positions_
                    ).c_str(),
                    format_bytes_as_hex(data_payload).c_str()
                );
            } else if (
                pending_joint_feedback_count_ >=
                    feedback_jump_confirm_samples_ &&
                !command_consistent_recovery
            ) {
                ROS_WARN(
                    "Rejected repeated SDK joint feedback discontinuity without command-consistent recovery: joint=%zu delta=%.6frad allowed=%.6frad command_reference=%s previous_command_error=%.6frad candidate_command_error=%.6frad margin=%.6frad command_age=%.3fs confirmation=%d/%d previous_deg=%s candidate_deg=%s payload=[%s]",
                    maximum_joint_delta_index + 1,
                    maximum_joint_delta_rad,
                    allowed_joint_delta_rad,
                    has_streamed_command_reference ? "available" : "unavailable",
                    previous_command_error_rad,
                    candidate_command_error_rad,
                    command_consistency_margin_rad,
                    streamed_command_age_sec,
                    pending_joint_feedback_count_,
                    feedback_jump_confirm_samples_,
                    format_radians_as_degrees(current_joint_positions_).c_str(),
                    format_radians_as_degrees(candidate_joint_positions).c_str(),
                    format_bytes_as_hex(data_payload).c_str()
                );
            } else if (!accept_joint_positions) {
                ROS_WARN(
                    "Rejected implausible one-frame SDK joint feedback: joint=%zu delta=%.6frad allowed=%.6frad frame_dt=%.3fs confirmation=%d/%d previous_deg=%s candidate_deg=%s payload=[%s]",
                    maximum_joint_delta_index + 1,
                    maximum_joint_delta_rad,
                    allowed_joint_delta_rad,
                    frame_interval_sec,
                    pending_joint_feedback_count_,
                    feedback_jump_confirm_samples_,
                    format_radians_as_degrees(current_joint_positions_).c_str(),
                    format_radians_as_degrees(candidate_joint_positions).c_str(),
                    format_bytes_as_hex(data_payload).c_str()
                );
            } else {
                accepted_joint_feedback_discontinuity = true;
                ROS_WARN(
                    "Accepted command-consistent SDK joint feedback recovery after %d nearby frames: joint=%zu delta=%.6frad allowed=%.6frad previous_deg=%s confirmed_deg=%s",
                    pending_joint_feedback_count_,
                    maximum_joint_delta_index + 1,
                    maximum_joint_delta_rad,
                    allowed_joint_delta_rad,
                    format_radians_as_degrees(current_joint_positions_).c_str(),
                    format_radians_as_degrees(candidate_joint_positions).c_str()
                );
            }
        }
    }
    last_joint_feedback_frame_time_ = feedback_time;
    if (accept_joint_positions) {
        current_joint_positions_ = candidate_joint_positions;
        last_feedback_time_ = feedback_time;
        has_real_feedback_ = true;
        last_accepted_joint_feedback_time_ = feedback_time;
        pending_joint_feedback_.clear();
        pending_joint_feedback_count_ = 0;
        {
            std::lock_guard<std::mutex> actuation_lock(actuation_mutex_);
            if (accepted_joint_feedback_discontinuity) {
                // A discontinuity can move toward a held target because bad
                // telemetry recovered, not because a servo responded. Drop
                // the active probe before recording the recovered sample so
                // it cannot manufacture positive actuation confirmation.
                actuation_confirmation_.mark_unconfirmed(
                    "DISCONTINUOUS_FEEDBACK_RECOVERY",
                    feedback_time.toSec()
                );
            }
            actuation_confirmation_.note_feedback(
                candidate_joint_positions,
                feedback_time.toSec()
            );
        }
    }

    uint16_t gripper_raw = data_payload[12] | (data_payload[13] << 8);
    gripper_raw = std::max<uint16_t>(0, std::min<uint16_t>(1000, gripper_raw));
    current_gripper_position_ = (static_cast<double>(gripper_raw) / 1000.0) * GRIPPER_DEG_MAX * M_PI / 180.0;
    last_run_status_ = data_payload[14];

    std_msgs::UInt8 status_msg;
    status_msg.data = last_run_status_;
    run_status_pub_.publish(status_msg);

    const bool temperature_fresh = has_temperature_feedback_ &&
        (feedback_time - last_temperature_time_).toSec() <= 3.0;
    float max_temperature = 0.0f;
    for (float value : latest_temperatures_c_) {
        if (std::isfinite(value)) {
            max_temperature = std::max(max_temperature, value);
        }
    }
    const bool sustained_high_temperature_telemetry = temperature_fresh &&
        consecutive_high_temperature_samples_ >= temperature_over_limit_confirm_samples_;

    if (last_run_status_ == 0xE1) {
        ++consecutive_e1_frames_;
        ROS_WARN_THROTTLE(
            2.0,
            "Hardware reports E1 overheat warning (%d/%d consecutive, temperature %s, max %.1f C).",
            consecutive_e1_frames_, e1_confirm_consecutive_frames_,
            temperature_fresh ? "fresh" : "unavailable/stale", max_temperature);
    } else {
        consecutive_e1_frames_ = 0;
    }

    const bool protection_status_event = last_run_status_ == 0xE1 || last_run_status_ == 0xE2;
    if (
        protection_status_event &&
        sustained_high_temperature_telemetry
    ) {
        std::lock_guard<std::mutex> actuation_lock(actuation_mutex_);
        actuation_confirmation_.mark_overheat_blocked(
            "SUSTAINED_SAME_CHANNEL_TEMPERATURE",
            feedback_time.toSec()
        );
    }
    if (sustained_high_temperature_telemetry) {
        // Temperature bytes are diagnostic evidence only. Retained real-arm
        // data contains physically impossible repeated values (174 C followed
        // by 45 C one second later), so this parser must never autonomously
        // remove torque or change the motion-enable state. Explicit operator
        // zero-torque mode remains the only runtime torque-off path.
        ROS_ERROR_THROTTLE(
            2.0,
            "Over-temperature telemetry diagnostic: status=0x%02X temperature_channel=%d streak=%d joint_payload=[%s] temperatures_c=%s self_check_mask=%s0x%04X; autonomous torque_off is disabled and motion enable is unchanged",
            last_run_status_,
            consecutive_high_temperature_channel_index_ >= 0
                ? consecutive_high_temperature_channel_index_ + 1
                : -1,
            consecutive_high_temperature_samples_,
            format_bytes_as_hex(data_payload).c_str(),
            format_temperatures(latest_temperatures_c_).c_str(),
            has_self_check_feedback_ ? "" : "unavailable/",
            static_cast<unsigned>(latest_self_check_mask_));
    } else if (protection_status_event) {
        ROS_WARN_THROTTLE(
            2.0,
            "Treating hardware status 0x%02X as a status event; no torque_off sent without sustained measured over-temperature.",
            last_run_status_);
    }

    std_msgs::Bool protection_msg;
    protection_msg.data = protection_fault_latched_;
    protection_latched_pub_.publish(protection_msg);
    publish_actuation_status();

    bool should_seed_command_state = false;
    {
        std::lock_guard<std::mutex> latest_lock(latest_cmd_mutex_);
        should_seed_command_state =
            accept_joint_positions && !has_latest_command_;
        if (should_seed_command_state) {
            // A ros_control arm trajectory intentionally omits right_finger.
            // Seed the partial-command merge source from measured hardware so
            // the first arm-only command after driver startup preserves the
            // physical gripper position instead of inheriting the constructor
            // default (zero/closed).  Keep has_latest_command_ false: feedback
            // alone must not start a command stream.
            latest_joint_angles_ = current_joint_positions_;
            latest_gripper_rad_ = current_gripper_position_;
        }
    }
    if (should_seed_command_state) {
        cmd_joint_angles_ = current_joint_positions_;
        cmd_gripper_rad_ = current_gripper_position_;
        cmd_joint_velocities_.assign(6, 0.0);
        cmd_gripper_vel_rad_s_ = 0.0;
    }

    if (accept_joint_positions && log_command_flow_) {
        ROS_INFO_THROTTLE(1.0, "Real SDK feedback: joints_deg=%s gripper_raw=%u status=0x%02X",
                          format_radians_as_degrees(current_joint_positions_).c_str(),
                          static_cast<unsigned>(gripper_raw),
                          data_payload[14]);
    }

    if (accept_joint_positions) {
        publish_joint_state();
    }
    }
}

void AliciaDDriverNode::parse_sdk_self_check_frame(const std::vector<uint8_t>& data_payload)
{
    if (data_payload.size() < 2) {
        ROS_WARN("SDK self-check DATA too short: expected >=2 bytes, got %zu", data_payload.size());
        return;
    }

    const uint16_t raw_mask = static_cast<uint16_t>(data_payload[0]) |
                              (static_cast<uint16_t>(data_payload[1]) << 8);
    std_msgs::UInt16 msg;
    msg.data = raw_mask;
    self_check_mask_pub_.publish(msg);

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_self_check_mask_ = raw_mask;
        has_self_check_feedback_ = true;
        last_self_check_time_ = ros::Time::now();
    }

    std::ostringstream set_bits;
    bool first = true;
    for (unsigned i = 0; i < 10; ++i) {
        if ((raw_mask & (1u << i)) == 0) continue;
        if (!first) set_bits << ",";
        set_bits << i;
        first = false;
    }
    ROS_INFO_THROTTLE(2.0, "Official self-check: raw_mask=0x%04X set_bits=[%s]",
                      static_cast<unsigned>(raw_mask), set_bits.str().c_str());
}

void AliciaDDriverNode::parse_sdk_temperature_frame(const std::vector<uint8_t>& data_payload)
{
    if (data_payload.empty()) {
        ROS_WARN("Received empty SDK temperature frame.");
        return;
    }

    std_msgs::Float32MultiArray msg;
    msg.data.reserve(data_payload.size());
    float max_temperature = 0.0f;
    size_t invalid_temperature_channels = 0;
    for (uint8_t raw : data_payload) {
        float value = static_cast<float>(raw);
        if (
            value >
            static_cast<float>(max_plausible_temperature_c_)
        ) {
            value = std::numeric_limits<float>::quiet_NaN();
            ++invalid_temperature_channels;
        }
        msg.data.push_back(value);
        if (std::isfinite(value)) {
            max_temperature = std::max(max_temperature, value);
        }
    }

    if (invalid_temperature_channels > 0) {
        // The retained real-arm failure contained CRC-valid response bursts
        // that simultaneously produced impossible 164--250 C temperature
        // bytes, changing self-check masks, and 130/180 degree encoder jumps.
        // Exclude only those impossible channel values.  Other channels in
        // the same response remain independent protection evidence: the
        // recorded 60--61 C channel must still be able to accumulate the
        // unchanged consecutive same-channel over-temperature block.
        ROS_ERROR_THROTTLE(
            1.0,
            "Rejected %zu implausible SDK temperature channel(s) above telemetry plausibility ceiling %.1f C; raw=[%s] accepted_values_c=%s",
            invalid_temperature_channels,
            max_plausible_temperature_c_,
            format_bytes_as_hex(data_payload).c_str(),
            format_temperatures(msg.data).c_str()
        );
    }

    int high_temperature_sample_count = 0;
    int high_temperature_channel_index = -1;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_temperatures_c_ = msg.data;
        last_temperature_time_ = ros::Time::now();
        has_temperature_feedback_ = true;
        if (
            consecutive_high_temperature_samples_by_channel_.size() !=
            msg.data.size()
        ) {
            consecutive_high_temperature_samples_by_channel_.assign(
                msg.data.size(),
                0
            );
        }
        for (size_t i = 0; i < msg.data.size(); ++i) {
            int& channel_streak =
                consecutive_high_temperature_samples_by_channel_[i];
            if (
                msg.data[i] >=
                static_cast<float>(max_enable_temperature_c_)
            ) {
                channel_streak = std::min(
                    temperature_over_limit_confirm_samples_,
                    channel_streak + 1
                );
            } else {
                channel_streak = 0;
            }
            if (channel_streak > high_temperature_sample_count) {
                high_temperature_sample_count = channel_streak;
                high_temperature_channel_index = static_cast<int>(i);
            }
        }
        consecutive_high_temperature_samples_ =
            high_temperature_sample_count;
        consecutive_high_temperature_channel_index_ =
            high_temperature_channel_index;
    }
    temperature_pub_.publish(msg);
    if (max_temperature >= 50.0f) {
        ROS_WARN("High or anomalous SDK temperature sample (same-channel max streak %d/%d at channel %d): values_c=%s",
                 high_temperature_sample_count,
                 temperature_over_limit_confirm_samples_,
                 high_temperature_channel_index >= 0
                     ? high_temperature_channel_index + 1
                     : -1,
                 format_temperatures(msg.data).c_str());
    }
    ROS_INFO_THROTTLE(2.0, "Real SDK temperatures: count=%zu max=%.1f C", msg.data.size(), max_temperature);
}


void AliciaDDriverNode::parse_servo_states_frame(const std::vector<uint8_t>& data_payload)
{

    if (data_payload.empty()) {
      ROS_WARN("Received empty servo state frame.");
      return;
    }

    uint8_t data_length = data_payload[0];
    if (data_payload.size() < data_length + 1) {
        ROS_WARN("Servo state frame payload size mismatch: expected %d, got %zu", data_length + 1, data_payload.size());
        return;
    }
    int servos_in_frame = data_length / 2;

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_feedback_time_ = ros::Time::now();
    // Data Processing & State Update
    for (int i = 0; i < servos_in_frame && i < servo_count_; ++i) {
        size_t data_idx = 1 + i * 2;
        if (data_idx + 1 >= data_payload.size()) {
            ROS_WARN("Incomplete data for servo %d in frame.", i);
            break;
        }
        uint16_t hw_val = data_payload[data_idx] | (data_payload[data_idx + 1] << 8);
        double rad_val = hardware_value_to_rad(hw_val);
        
        if (static_cast<size_t>(i) < servo_to_joint_map_index_.size()) {
            int joint_idx = servo_to_joint_map_index_[i];
            if(joint_idx != -1 && joint_idx < static_cast<int>(current_joint_positions_.size())) {
                current_joint_positions_[joint_idx] = rad_val * servo_to_joint_map_direction_[i];
            }
        }
    }
    
    // Publish complete joint state
    publish_joint_state();
}



void AliciaDDriverNode::parse_gripper_state_frame(const std::vector<uint8_t>& data_payload)
{
    // The data_payload is what comes *after* the command ID (0x02).
    // The structure is [LEN, ID, Low, High, ?, ?, BTN1, BTN2].
    if (data_payload.size() < 8) {
         ROS_WARN("Gripper state data payload is smaller than the expected 8 bytes: %zu bytes", data_payload.size());
        return;
    }
   uint16_t gripper_hw_val = data_payload[2] | (data_payload[3] << 8);
   std::lock_guard<std::mutex> lock(data_mutex_);
   last_feedback_time_ = ros::Time::now();
   current_gripper_position_ = hardware_value_to_rad_grip(gripper_hw_val);
   publish_joint_state();

}

void AliciaDDriverNode::publish_joint_state()
{
    std::lock_guard<std::mutex> lock(topic_mutex_);
    
    sensor_msgs::JointState js_msg;
    // Ensure strictly monotonically increasing timestamps to avoid robot_state_publisher warnings
    static ros::Time s_last_stamp(0, 0);
    ros::Time now = ros::Time::now();
    if (!s_last_stamp.isZero() && (now <= s_last_stamp)) {
        now = s_last_stamp + ros::Duration(0, 1); // add 1 ns
    }
    js_msg.header.stamp = now;
    s_last_stamp = now;
    js_msg.name = joint_names_;
    
    // Combine arm joints and gripper
    js_msg.position = current_joint_positions_;
    // Convert internal gripper radians back to meters for the prismatic joint interface
    double stroke_m = 0.05;
    pnh_.param<double>("gripper_stroke_m", stroke_m, 0.05);
    // Internal representation: 0..(100deg in rad). Map to 0..stroke_m
    double gripper_deg = std::max(0.0, std::min(100.0, current_gripper_position_ * 180.0 / M_PI));
    double gripper_m = (gripper_deg / 100.0) * stroke_m;
    js_msg.position.push_back(gripper_m);
    
    joint_state_pub_std_.publish(js_msg);

    // // Measure outgoing /joint_states publish rate (logs once per second)
    // {
    //     static bool s_initialized = false;
    //     static bool s_log_rates = true;
    //     static size_t s_pub_count = 0;
    //     static ros::Time s_last_log(0, 0);
    //     if (!s_initialized) {
    //         ros::param::param("~log_rates", s_log_rates, true);
    //         s_last_log = ros::Time::now();
    //         s_initialized = true;
    //     }
    //     ++s_pub_count;
    //     const ros::Time now = ros::Time::now();
    //     const double dt = (now - s_last_log).toSec();
    //     if (s_log_rates && dt >= 1.0) {
    //         const double hz = static_cast<double>(s_pub_count) / dt;
    //         ROS_INFO("[Rate] /joint_states published: %.1f Hz (window %.2fs, %zu msgs)", hz, dt, s_pub_count);
    //         s_pub_count = 0;
    //         s_last_log = now;
    //     }
    // }
}


void AliciaDDriverNode::parse_error_frame(const std::vector<uint8_t>& data_payload)
{
    if (data_payload.size() < 2) {
        ROS_WARN("Error frame data payload is too short: %zu bytes", data_payload.size());
        return;
    }
    uint8_t error_type = data_payload[0];
    uint8_t error_param = data_payload[1];
    ROS_ERROR("Received Error Frame from Hardware: Type=0x%02X, Param=0x%02X", error_type, error_param);
}


uint8_t AliciaDDriverNode::calculate_checksum(const std::vector<uint8_t>& frame_data)
{
    // The checksum is the sum of the DATA PAYLOAD bytes, modulo 2.
    // The frame_data vector is passed in *before* the checksum is calculated and inserted.
    // The payload starts at index 3 and its length is specified at index 2.
    if (frame_data.size() < 4) {
        return 0; // Frame is too short to have a payload
    }
    
    // The length of the actual data payload.
    const uint8_t payload_len = frame_data[2];

    // Ensure the frame is large enough to contain the declared payload
    if (frame_data.size() < (size_t)3 + payload_len) {
        return 0; 
    }

    // Sum from the beginning of the payload (index 3) for the length of the payload.
    int sum = std::accumulate(frame_data.begin() + 3, 
                              frame_data.begin() + 3 + payload_len, 
                              0);

    return static_cast<uint8_t>(sum % 2);
}



std::vector<uint8_t> AliciaDDriverNode::generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum)
{
    std::vector<uint8_t> frame(6);
    frame[0] = FRAME_START_BYTE;
    frame[1] = command;
    frame[2] = 0x01; // Data length is always 1 for these simple frames
    frame[3] = data & 0xFF; // Data byte

    if (use_checksum) {
        // Checksum is just the data byte modulo 2, as per the Python logic
        frame[4] = data % 2;
    } else {
        frame[4] = 0x00;
    }

    frame[5] = FRAME_END_BYTE;
    return frame;
}


void AliciaDDriverNode::zero_calibrate_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Received Zero Calibration command.");
        auto frame = generate_simple_frame(CMD_ZERO_CAL, 0x00, false);
        communicator_->write_raw_frame(frame);
    }
} 


void AliciaDDriverNode::demonstration_mode_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Enabling zero-torque mode with SDK torque_off frame.");
        clear_retained_command_state();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            motion_commands_enabled_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(actuation_mutex_);
            actuation_confirmation_.set_disabled(ros::Time::now().toSec());
        }
        publish_actuation_status();
        const std::vector<uint8_t> torque_off_frame = {
            0xAA, 0x05, 0x00, 0x01, 0x00, 0x6F, 0xFF
        };
        communicator_->write_raw_frame(torque_off_frame);
    } else {
        request_positive_enable("demonstration_false");
    }
}



uint16_t AliciaDDriverNode::rad_to_hardware_value(double angle_rad) {
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
    int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
    return std::max(0, std::min(4095, value));
}

uint16_t AliciaDDriverNode::rad_to_hardware_value_grip(double angle_rad)
{
    double angle_deg = angle_rad * 180.0 / M_PI;
    // Clamp to expected [0, 100] deg range
    angle_deg = std::max(0.0, std::min(GRIPPER_DEG_MAX, angle_deg));
    // Map 0..100deg -> 2048..3590
    const double hw = angle_deg * GRIPPER_HW_PER_DEG + GRIPPER_HW_MIN;
    const int hardware_value = static_cast<int>(std::round(hw));
    return std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(GRIPPER_HW_MAX), hardware_value));
}


double AliciaDDriverNode::hardware_value_to_rad(uint16_t hw_value) {
    hw_value = std::max(0, std::min(4095, (int)hw_value));
    double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4096.0) * 360.0;
    return angle_deg * M_PI / 180.0;
}
double AliciaDDriverNode::hardware_value_to_rad_grip(uint16_t hw_value)
{
    hw_value = std::max(static_cast<int>(GRIPPER_HW_MIN), std::min(static_cast<int>(GRIPPER_HW_MAX), (int)hw_value));
    // Inverse map 2048..3590 -> 0..100deg
    const double angle_deg = (static_cast<double>(hw_value) - GRIPPER_HW_MIN) / GRIPPER_HW_PER_DEG;
    return angle_deg * M_PI / 180.0; // Convert to radians
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "alicia_d_driver_node");
    AliciaDDriverNode node;
    // Use AsyncSpinner with 2 threads to avoid blocking callbacks when serial writes are heavy
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
