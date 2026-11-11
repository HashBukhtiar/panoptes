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
    const double LOOP_RATE_HZ_ = declare_and_get("loop_rate_hz", 100.0);
    const double MAX_SPEED_RAD_S_ = declare_and_get("max_speed_rad_s", 0.5);

    rclcpp::Subscription<CameraOrientationCommand>::SharedPtr camera_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<JointCommand>::SharedPtr joint_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* Received */
    kinematics::JointAngles
        target_joint_angles_; /* target joint angles computed from the camera orientation command */
    builtin_interfaces::msg::Time
        target_joint_angles_time_; /* timestamp of the target joint angles received (preserved) */

    /* Published */
    JointCommand last_joint_command_; /* last joint command sent to the joint controller */
    std::chrono::steady_clock::time_point
        last_joint_command_time_; /* timestamp of the last joint command sent (used for local
                                     calculations only) */

    double declare_and_get(const std::string& name, double fallback);
    void on_camera_command(const CameraOrientationCommand::SharedPtr msg);
    void on_joint_state(const JointState::SharedPtr msg);
    void compute_and_send_joint_command();
};