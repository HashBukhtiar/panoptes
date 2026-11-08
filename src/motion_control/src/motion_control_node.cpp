#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "interfaces/msg/camera_orientation_command.hpp"
#include "interfaces/msg/joint_command.hpp"
#include "kinematics/kinematics.hpp"
#include "motion_control/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;
using JointState = sensor_msgs::msg::JointState;
using JointCommand = interfaces::msg::JointCommand;

class MotionControlNode : public rclcpp::Node {
   public:
    MotionControlNode() : Node("motion_control") {
        camera_sub_ = create_subscription<CameraOrientationCommand>(
            "camera_orientation_command", /* name of the topic */
            rclcpp::QoS(1).best_effort(), /* queue of up to 1 message (keeps newest only) */
            [this](const CameraOrientationCommand::SharedPtr msg) { on_camera_command(msg); });

        joint_sub_ = create_subscription<JointState>(
            "joint_state",                /* name of the topic*/
            rclcpp::QoS(1).best_effort(), /* queue of up to 1 message (keeps newest only) */
            [this](const JointState::SharedPtr msg) { on_joint_state(msg); });

        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / LOOP_RATE_HZ_),
                                   [this]() { compute_and_send_joint_command(); });
    }

   private:
    const double LOOP_RATE_HZ_ = declare_and_get("loop_rate_hz", 100.0);
    const double MAX_SPEED_RAD_S_ = declare_and_get("max_speed_rad_s", 0.5);

    rclcpp::Subscription<CameraOrientationCommand>::SharedPtr camera_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    kinematics::JointAngles
        target_joint_angles_; /* target joint angles computed from the camera orientation command */
    JointCommand joint_command_; /* last joint command sent to the joint controller */
    std::chrono::steady_clock::time_point last_command_time_ =
        std::chrono::steady_clock::now(); /* timestamp of the last joint command sent */

    /**
     * Declares a parameter with the given name and fallback value, and returns its value.
     *
     * @param name      The name of the parameter to declare and get.
     * @param fallback  The fallback value to use if the parameter is not already declared.
     */
    double declare_and_get(const std::string& name, double fallback) {
        declare_parameter(name, fallback);
        return get_parameter(name).as_double();
    }

    /**
     * Callback function for receiving camera orientation commands.
     *
     * @param msg       The received CameraOrientationCommand message.
     */
    void on_camera_command(const CameraOrientationCommand::SharedPtr msg) {
        kinematics::CameraOrientation orientation{msg->yaw, msg->pitch, msg->roll};

        auto joint_angles = kinematics::inverse_kinematics(orientation);

        if (joint_angles) {
            target_joint_angles_ = *joint_angles;
        } else {
            // TODO: handle appropriately
        }
    }

    /**
     * Callback function for receiving joint state messages.
     *
     * @param msg       The received JointState message.
     */
    void on_joint_state(const JointState::SharedPtr msg) {
        if (joint_command_.position.empty()) {
            joint_command_.position =
                msg->position; /* use the first received joint state as the initial position for
                                  trajectory generation */
            last_command_time_ = std::chrono::steady_clock::now();
        }
    }

    /**
     * Computes the next joint command and sends it to the joint controller at a fixed rate.
     * This function is called periodically by a timer.
     */
    void compute_and_send_joint_command() {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_command_time_).count();
        last_command_time_ = now;

        if (target_joint_angles_.empty() || joint_command_.position.empty()) {
            return;
        }

        double scale = trajectory::compute_scale(joint_command_.position, target_joint_angles_,
                                                 MAX_SPEED_RAD_S_ * dt);

        for (size_t i = 0; i < target_joint_angles_.size(); ++i) {
            joint_command_.position[i] = trajectory::compute_trajectory(
                joint_command_.position[i], target_joint_angles_[i], scale);
        }

        // TODO: publish joint_command_ to the joint controller topic when it exists
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControlNode>());
    rclcpp::shutdown();
    return 0;
}
