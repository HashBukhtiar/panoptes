#pragma once

#include <chrono>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "interfaces/msg/camera_orientation_command.hpp"
#include "interfaces/msg/joint_command.hpp"
#include "kinematics/kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;
using JointState = sensor_msgs::msg::JointState;
using JointCommand = interfaces::msg::JointCommand;

class MotionControlNode : public rclcpp::Node {
   public:
    MotionControlNode();

   private:
    enum class State { CALIBRATING, NORMAL, HOLDING };

    const double LOOP_RATE_HZ_ = declare_and_get("loop_rate_hz", 100.0);
    const double MAX_SPEED_RAD_S_ = declare_and_get("max_speed_rad_s", 0.5);

    /* Staleness tolerances */
    const double CAMERA_COMMAND_MAX_AGE_MS_ = declare_and_get("camera_command_max_age_ms", 100.0);
    const double JOINT_STATE_MAX_AGE_MS_ = declare_and_get("joint_state_max_age_ms", 100.0);
    const double JOINT_STATE_MAX_GAP_MS_ = declare_and_get("joint_state_max_gap_ms", 100.0);

    /* Joint state streak durations (for the deferred hold/recalibrate behaviour) */
    const double JOINT_STATE_FAULT_STREAK_MAX_MS_ = declare_and_get(
        "joint_state_fault_streak_max_ms", 500.0); /* if faulty this long, command hold */
    const double JOINT_STATE_GOOD_STREAK_MIN_MS_ = declare_and_get(
        "joint_state_good_streak_min_ms", 500.0); /* if good this long, recalibrate */

    /* Numerical limits */
    const std::vector<double> JOINT_MIN_RAD_ = declare_and_get_vec("joint_min_rad", {-1.57, -1.57});
    const std::vector<double> JOINT_MAX_RAD_ = declare_and_get_vec("joint_max_rad", {1.57, 1.57});
    const std::vector<double> CAMERA_MIN_RAD_ =
        declare_and_get_vec("camera_min_rad", {-1.57, -1.57, -1.57}); /* order: yaw, pitch, roll */
    const std::vector<double> CAMERA_MAX_RAD_ =
        declare_and_get_vec("camera_max_rad", {1.57, 1.57, 1.57}); /* order: yaw, pitch, roll */

    rclcpp::Subscription<CameraOrientationCommand>::SharedPtr camera_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<JointCommand>::SharedPtr joint_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::CALIBRATING;

    /* (IMPORTANT) Naming convention for timestamps:
     *   _stamp_ = the message's own header timestamp (set by the publisher, preserved as-is)
     *   _time_  = this node's local steady-clock time (used for local calculations only)
     */

    /* Received */
    kinematics::JointAngles
        target_joint_angles_; /* target joint angles computed from the camera orientation command */

    std::optional<rclcpp::Time> last_camera_command_stamp_;
    std::optional<rclcpp::Time>
        last_joint_state_stamp_; /* used to reject out-of-order joint states */
    std::chrono::steady_clock::time_point
        last_good_joint_state_time_; /* used to check if there was no new joint state message after
                                        N ms */
    std::vector<double> latest_joint_position_;

    /* Published */
    JointCommand last_joint_command_; /* last joint command sent to the joint controller */
    std::chrono::steady_clock::time_point last_joint_command_time_;

    double declare_and_get(const std::string& name, double fallback);
    std::vector<double> declare_and_get_vec(const std::string& name,
                                            const std::vector<double>& fallback);

    void on_camera_command(const CameraOrientationCommand::SharedPtr msg);
    void on_joint_state(const JointState::SharedPtr msg);
    void compute_and_send_joint_command();
};