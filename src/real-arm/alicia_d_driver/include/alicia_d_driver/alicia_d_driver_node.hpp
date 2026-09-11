#ifndef ALICiA_D_DRIVER_NODE_H
#define ALICiA_D_DRIVER_NODE_H

#include "ros/ros.h"
#include "alicia_d_driver/actuation_confirmation.hpp"
#include "alicia_d_driver/endpoint_trim_continuity.hpp"
#include "alicia_d_driver/endpoint_trim_driver_admission.hpp"
#include "serial_communicator.hpp" // Assuming this is a non-ROS helper class
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"
#include <memory>
#include <vector>
#include <string>
#include <mutex>


constexpr uint8_t CMD_DUAL_ARM = 0x06;
constexpr size_t MAX_FRAME_LENGTH = 64;
constexpr size_t FIXED_FRAME_LENGTH = 50;
constexpr size_t DUAL_ARM_FEEDBACK_PAYLOAD_SIZE = 45;
constexpr size_t MIN_FRAME_LENGTH = 5; // Minimum valid frame length (AA CMD LEN CHK FF)

class AliciaDDriverNode
{
public:
   AliciaDDriverNode();
   ~AliciaDDriverNode();

private:
    // ROS 1 NodeHandle
   ros::NodeHandle nh_;
   ros::NodeHandle pnh_; // Private NodeHandle for parameters

   // Initialization
   void load_parameters();
   void setup_ros_communications();
   void clear_retained_command_state();
   bool request_positive_enable(
       const std::string& source, bool preserve_fresh_confirmation = false);
   void publish_actuation_status();
   bool set_task_endpoint_precision_callback(
       std_srvs::SetBool::Request& request,
       std_srvs::SetBool::Response& response
   );
    
   // Callbacks for incoming commands
   void joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg);
   void zero_calibrate_callback(const std_msgs::Bool::ConstPtr& msg);
   void demonstration_mode_callback(const std_msgs::Bool::ConstPtr& msg);
    
    // Timer callbacks
   void process_serial_data_callback(const ros::TimerEvent& event);
   void reconnect_callback(const ros::TimerEvent& event);
	   void send_command_timer_callback(const ros::TimerEvent& event);
	   bool diagnostic_query_suppressed_for_motion(const ros::Time& now);
	   void heartbeat_publish_callback(const ros::TimerEvent& event);
	   void state_poll_timer_callback(const ros::TimerEvent& event);

   // Main processing loop
   void process_serial_data();
	   void parse_sdk_joint_state_frame(const std::vector<uint8_t>& data_payload);
	   void parse_sdk_temperature_frame(const std::vector<uint8_t>& data_payload);
	   void parse_sdk_self_check_frame(const std::vector<uint8_t>& data_payload);
   void parse_servo_states_frame(const std::vector<uint8_t>& payload);
   void parse_gripper_state_frame(const std::vector<uint8_t>& payload); // Add this
   void parse_error_frame(const std::vector<uint8_t>& payload);

    // Data Conversion & Framing
   uint16_t rad_to_hardware_value(double angle_rad);
   uint16_t rad_to_hardware_value_grip(double angle_rad);
   double hardware_value_to_rad(uint16_t hw_value);
   double hardware_value_to_rad_grip(uint16_t hw_value);
   std::vector<uint8_t> generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum);
   uint8_t calculate_checksum(const std::vector<uint8_t>& frame_data);

    // Member Variables
   std::unique_ptr<SerialCommunicator> communicator_;
   ros::Timer processing_timer_;
   ros::Timer reconnect_timer_;
   ros::Timer command_timer_;
	   ros::Timer heartbeat_timer_;
	   ros::Timer state_poll_timer_;

   // Publishers & Subscribers
	   ros::Publisher joint_state_pub_std_;
	   ros::Publisher feedback_ready_pub_;
	   ros::Publisher run_status_pub_;
	   ros::Publisher temperature_pub_;
	   ros::Publisher self_check_mask_pub_;
	   ros::Publisher protection_latched_pub_;
	   ros::Publisher motion_enabled_pub_;
	   ros::Publisher actuation_status_pub_;
   ros::ServiceServer task_endpoint_precision_service_;
   ros::Subscriber joint_command_sub_;
   ros::Subscriber zero_calib_sub_;
   ros::Subscriber demo_mode_sub_;

   // Configuration and State
   int servo_count_;
	   bool debug_mode_;
	   bool auto_torque_on_startup_ = false;
   double rate_limit_sec_;
	   double command_rate_hz_;
	   double state_poll_rate_hz_;
	   double temperature_poll_rate_hz_ = 1.0;
	   double self_check_poll_rate_hz_ = 0.5;
	   bool suppress_diagnostic_queries_while_motion_active_ = true;
	   double diagnostic_query_motion_quiet_sec_ = 0.5;
	   double diagnostic_query_motion_error_rad_ = 0.02;
	   bool mirror_commanded_state_when_feedback_stale_;
	   bool log_command_flow_;
	   bool suppress_redundant_commands_ = true;
	   bool pause_commands_when_feedback_stale_ = true;
	   double feedback_stale_timeout_sec_ = 1.0;
	   double command_keepalive_rate_hz_ = 10.0;
	   bool reject_implausible_joint_feedback_ = true;
	   double feedback_max_velocity_rad_s_ = 1.0;
	   double feedback_jump_base_tolerance_rad_ = 0.02;
	   double feedback_jump_confirmation_tolerance_rad_ = 0.05;
	   int feedback_jump_confirm_samples_ = 2;
	   bool reject_command_inconsistent_joint_feedback_ = true;
	   bool endpoint_feedback_trim_enabled_ = false;
	   bool endpoint_feedback_trim_task_lease_active_ = false;
	   double endpoint_feedback_trim_task_lease_timeout_sec_ = 120.0;
	   ros::Time endpoint_feedback_trim_task_lease_expires_at_;
	   std::vector<double> endpoint_feedback_trim_task_lease_reference_;
	   double endpoint_feedback_trim_stable_sec_ = 0.30;
	   double endpoint_feedback_trim_activation_error_rad_ = 0.035;
	   double endpoint_feedback_trim_max_rad_ = 0.12;
	   double endpoint_feedback_trim_gain_ = 1.0;
	   int endpoint_feedback_trim_max_step_quantums_ = 4;
	   int endpoint_feedback_trim_response_min_quantums_ = 2;
	   double endpoint_feedback_trim_response_deadline_sec_ = 1.0;
	   EndpointTrimConfig endpoint_trim_config_;
	   EndpointTrimContinuity endpoint_trim_continuity_;
	   EndpointTrimCommandOrder endpoint_trim_command_order_;
	   double gui_direct_gesture_timeout_sec_ = 0.25;
		   double protection_clear_stable_sec_ = 30.0;
	   double max_enable_temperature_c_ = 60.0;
	   double max_plausible_temperature_c_ = 125.0;
	   double max_temperature_slew_c_per_sec_ = 8.0;
	   double temperature_slew_tolerance_c_ = 5.0;
	   int e1_confirm_consecutive_frames_ = 3;
	   int temperature_over_limit_confirm_samples_ = 3;
	   double reconnect_sync_tolerance_rad_ = 0.05;
	   double actuation_command_probe_min_delta_rad_ = 0.02;
	   double actuation_measured_response_min_delta_rad_ = 0.003;
	   double actuation_confirmation_timeout_sec_ = 1.0;
	   double actuation_confirmation_freshness_sec_ = 2.0;
	   ros::Time last_process_time_;
    // Trajectory smoothing parameters
    bool use_trajectory_smoothing_ = true;
    double max_joint_velocity_rad_s_ = 2.5;     // per-joint max command slew rate (rad/s)
    double max_gripper_velocity_rad_s_ = 1.5;   // gripper slew rate (rad/s)
    double max_joint_accel_rad_s2_ = 8.0;       // per-joint max acceleration (rad/s^2)
    double max_gripper_accel_rad_s2_ = 10.0;    // gripper max acceleration (rad/s^2)
    double joint_speed_deg_s_ = 15.0;           // SDK joint motion speed field (deg/s)
    bool gripper_input_is_percent_ = true;      // interpret /joint_commands right_finger as [0..1] percent

   // Mutex for thread safety
	   std::mutex data_mutex_;
	   std::mutex topic_mutex_;
	   std::mutex latest_cmd_mutex_;
	   std::mutex send_mutex_;
	   std::mutex actuation_mutex_;
   std::vector<double> servo_to_joint_map_index_;
   std::vector<double> servo_to_joint_map_direction_;
   std::vector<double> joint_to_servo_map_index_;
   std::vector<double> joint_to_servo_map_direction_;


   // Global state variables for joint states
   std::vector<double> current_joint_positions_;
   double current_gripper_position_;
   std::vector<std::string> joint_names_;
	   ros::Time last_accepted_joint_feedback_time_;
	   ros::Time last_joint_feedback_frame_time_;
	   std::vector<double> pending_joint_feedback_;
	   int pending_joint_feedback_count_ = 0;

   void publish_joint_state();
   bool has_data;

   // Latest command (decoupled from ROS subscriber thread)
   std::vector<double> latest_joint_angles_; // size 6
   double latest_gripper_rad_ = 0.0;          // radians
   bool has_latest_command_ = false;
	   ros::Time last_motion_reference_change_time_;
	   std::vector<double> endpoint_trim_reference_joint_angles_;
	   ros::Time endpoint_trim_upstream_reference_since_;
	   bool endpoint_feedback_trim_active_ = false;
	   bool endpoint_feedback_trim_quiescent_ = false;
	   std::vector<double> endpoint_feedback_trim_offsets_;
	   std::vector<double> endpoint_trim_feedback_anchor_joint_angles_;
	   std::vector<ros::Time> endpoint_trim_feedback_stable_since_;
	   ros::Time endpoint_trim_last_feedback_sample_time_;
	   bool endpoint_trim_waiting_for_feedback_response_ = false;
	   std::vector<double> endpoint_trim_response_start_joint_angles_;
	   std::vector<uint8_t> endpoint_trim_response_joint_mask_;
	   ros::Time endpoint_trim_response_wait_since_;
	   double endpoint_trim_last_response_latency_sec_ = 0.0;
	   size_t endpoint_trim_stalled_retry_count_ = 0;
	   size_t endpoint_feedback_trim_iteration_ = 0;
	   bool gui_direct_gesture_active_ = false;
	   int gui_direct_edited_index_ = -1;
	   ros::Time gui_direct_last_command_time_;
	   std::vector<double> gui_direct_hold_joint_angles_;
	   double gui_direct_hold_gripper_rad_ = 0.0;
	   ros::Time last_command_sent_time_;
	   std::vector<uint8_t> last_sent_sdk_command_frame_;
	   ros::Time last_sent_sdk_command_time_;
	   std::vector<double> last_streamed_joint_positions_;
	   ros::Time last_streamed_joint_positions_time_;

   // Throttling/gripper smooth send
   double gripper_send_rate_hz_ = 50.0;        // default gripper send rate
   double gripper_min_delta_deg_ = 1.0;        // only send if change > 1 deg
   ros::Time last_gripper_send_time_;
   double last_sent_gripper_deg_ = 0.0;        // last sent gripper angle in degree space

    // Interpolated command state (what we actually stream to hardware)
    std::vector<double> cmd_joint_angles_;      // size 6, radians
    std::vector<double> cmd_joint_velocities_;  // size 6, rad/s
    double cmd_gripper_rad_ = 0.0;              // radians
    double cmd_gripper_vel_rad_s_ = 0.0;        // rad/s
    bool command_state_seeded_from_feedback_ = false;

    // Timestamp of the last feedback received from hardware. When stale, we
    // fall back to publishing the commanded state so visualizers remain in sync.
	   ros::Time last_feedback_time_;
	   bool has_real_feedback_ = false;
		   uint8_t last_run_status_ = 0x00;
		   int consecutive_e1_frames_ = 0;
		   int consecutive_high_temperature_samples_ = 0;
		   int consecutive_high_temperature_channel_index_ = -1;
		   std::vector<int> consecutive_high_temperature_samples_by_channel_;
	   bool protection_fault_latched_ = false;
	   bool motion_commands_enabled_ = false;
	   ActuationConfirmation actuation_confirmation_;
	   std::string last_published_actuation_status_;
	   bool last_published_motion_enabled_ = false;
	   bool has_published_motion_enabled_ = false;
	   bool has_temperature_feedback_ = false;
	   bool has_self_check_feedback_ = false;
	   ros::Time last_protection_time_;
	   ros::Time last_temperature_time_;
	   ros::Time last_temperature_query_time_;
	   ros::Time last_self_check_query_time_;
	   ros::Time last_self_check_time_;
	   std::vector<float> latest_temperatures_c_;
	   std::vector<float> last_plausible_temperatures_c_;
	   std::vector<ros::Time> last_plausible_temperature_times_;
	   uint16_t latest_self_check_mask_ = 0;
};
#endif // ALICiA_D_DRIVER_NODE_H
