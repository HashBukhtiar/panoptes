#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "interfaces/msg/camera_orientation_command.hpp"
#include "interfaces/msg/joint_command.hpp"
#include "kinematics/kinematics.hpp"
#include "motion_control/alerter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;
using JointState = sensor_msgs::msg::JointState;
using JointCommand = interfaces::msg::JointCommand;

using RosClockTime = rclcpp::Time;
using LocalClockTime = std::chrono::steady_clock::time_point;

/* (IMPORTANT) Naming convention for timestamps:
 *   _stamp_ = ROS clock timestamp from the message header (set by the publisher, preserved)
 *   _time_  = local steady-clock timestamp from this node (used for elapsed-time calculations)
 */

class MotionControlNode : public rclcpp::Node {
   public:
    MotionControlNode();

   private:
    /* Types and state */
    enum class State { CALIBRATING, NORMAL, HOLDING, TERMINAL };
    State state_ = State::CALIBRATING;

    /* Configuration - command generation frequency */
    const double LOOP_RATE_HZ_ = declare_and_get("loop_rate_hz", 100.0);

    /* Configuration - hardware limits */
    const double MAX_SPEED_RAD_S_ = declare_and_get("max_speed_rad_s", 0.5);
    const std::vector<double> JOINT_MIN_RAD_ = declare_and_get_vec("joint_min_rad", {-1.57, -1.57});
    const std::vector<double> JOINT_MAX_RAD_ = declare_and_get_vec("joint_max_rad", {1.57, 1.57});
    const std::vector<double> CAMERA_MIN_RAD_ =
        declare_and_get_vec("camera_min_rad", {-1.57, -1.57, -1.57});
    const std::vector<double> CAMERA_MAX_RAD_ =
        declare_and_get_vec("camera_max_rad", {1.57, 1.57, 1.57});

    /* Configuration - staleness tolerances */
    const double HOLD_MAX_DURATION_MS_ = declare_and_get("hold_max_duration_ms", 2000.0);
    const double CAMERA_COMMAND_MAX_AGE_MS_ = declare_and_get("camera_command_max_age_ms", 100.0);
    const double JOINT_STATE_MAX_AGE_MS_ = declare_and_get("joint_state_max_age_ms", 100.0);
    const double JOINT_STATE_MAX_GAP_MS_ = declare_and_get("joint_state_max_gap_ms", 100.0);

    /* Configuration - transient fault and recovery durations */
    const double JOINT_STATE_FAULT_STREAK_MAX_MS_ =
        declare_and_get("joint_state_fault_streak_max_ms", 500.0);
    const double JOINT_STATE_GOOD_STREAK_MIN_MS_ =
        declare_and_get("joint_state_good_streak_min_ms", 500.0);

    /* ROS interfaces */
    rclcpp::Subscription<CameraOrientationCommand>::SharedPtr camera_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<JointCommand>::SharedPtr joint_command_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    Alerter alerter_{get_logger()};

    /* Received data */
    kinematics::JointAngles last_target_joint_angles_; /* computed from last camera command */
    std::optional<RosClockTime> last_camera_command_stamp_;

    std::vector<double> last_joint_state_;
    std::optional<RosClockTime>
        last_joint_state_stamp_;           /* used to reject out-of-order joint states */
    LocalClockTime last_joint_state_time_; /* used to check if there was no new joint state message
                                              after N ms */

    /* Published data */
    JointCommand last_joint_command_;
    LocalClockTime last_joint_command_time_;

    /* Internal timers */
    std::optional<LocalClockTime> hold_start_time_;
    std::optional<LocalClockTime> joint_state_fault_start_time_;
    std::optional<LocalClockTime> joint_state_good_start_time_;

    /* Configuration helper functions */
    double declare_and_get(const std::string& name, double fallback);
    std::vector<double> declare_and_get_vec(const std::string& name,
                                            const std::vector<double>& fallback);

    /* State management */
    void hold();
    void recalibrate();
    void terminate(const std::string& reason);

    void start_joint_state_fault_timer();
    void start_joint_state_good_timer();
    void update_state(LocalClockTime now);

    /* Message handling */
    void on_camera_command(const CameraOrientationCommand::SharedPtr msg);
    void on_joint_state(const JointState::SharedPtr msg);

    /* Command generation */
    void compute_and_send_joint_command();
};