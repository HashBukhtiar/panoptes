#include <cmath>
#include <memory>

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
    MotionControlNode() : Node("motion_control") {
        camera_sub_ = create_subscription<CameraOrientationCommand>(
            "camera_orientation_command", /* name of the topic */
            rclcpp::QoS(1).best_effort(), /* queue of up to 1 message (keeps newest only) */
            [this](const CameraOrientationCommand::SharedPtr msg) { on_camera_command(msg); });

        joint_sub_ = create_subscription<JointState>(
            "joint_state",              /* name of the topic*/
            rclcpp::QoS(10).reliable(), /* queue of up to 10 messages */
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
        target_joint_angles_;     // Desired position from camera orientation command
    JointCommand joint_command_;  // Last commanded sent to the joint controller
    std::chrono::steady_clock::time_point last_command_time_ =
        std::chrono::steady_clock::now();  // Time when the last command was sent

    /**
     * Declares a parameter with the given name and fallback value, and returns its value.
     * If the parameter is already declared, it simply returns its value.
     *
     * @param name The name of the parameter to declare and get.
     * @param fallback The fallback value to use if the parameter is not already declared.
     */
    double declare_and_get(const std::string& name, double fallback) {
        declare_parameter(name, fallback);
        return get_parameter(name).as_double();
    }

    /**
     * Callback function for receiving camera orientation commands.
     *
     * @param msg  The received CameraOrientationCommand message.
     */
    void on_camera_command(const CameraOrientationCommand::SharedPtr msg) {
        kinematics::CameraOrientation orientation{msg->yaw, msg->pitch, msg->roll};
        auto joint_angles = kinematics::inverse_kinematics(orientation);

        if (joint_angles) {
            target_joint_angles_ = *joint_angles;
        } else {
            RCLCPP_INFO(get_logger(),
                        "[MC] failed to compute joint angles for the given orientation");
        }
    }

    /**
     * Callback function for receiving joint state messages.
     *
     * @param msg  The received JointState message.
     */
    void on_joint_state(const JointState::SharedPtr msg) {
        // Use the first received joint state as the initial position for trajectory generation
        if (joint_command_.position.empty()) {
            joint_command_.position = msg->position;
            last_command_time_ = std::chrono::steady_clock::now();
        }
    }

    /**
     * Generates the next joint position by moving toward the target
     * at a fixed maximum velocity.
     *
     * @param curr          Current commanded joint position.
     * @param target        Target joint position.
     * @param max_velocity  Maximum movement velocity (rad/second).
     * @param dt            Time since the previous update (seconds).
     * @return              Next commanded joint position.
     */
    double compute_trajectory(double curr, double target, double max_velocity, double dt) {
        double max_step = max_velocity * dt;
        double error = target - curr;

        // TODO: consider acceleration limit, and/or implement a more sophisticated trajectory
        // generator (e.g., trapezoidal velocity profile)

        if (std::abs(error) <= max_step) {
            return target;
        }

        return curr + std::copysign(max_step, error);
    }

    /**
     * Computes the next joint command and sends it to the joint controller.
     */
    void compute_and_send_joint_command() {
        if (target_joint_angles_.empty() || joint_command_.position.empty()) {
            RCLCPP_INFO(get_logger(),
                        "[MC] waiting for both camera orientation command and initial joint state");
            return;
        }

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_command_time_).count();
        last_command_time_ = now;

        for (size_t i = 0; i < target_joint_angles_.size(); ++i) {
            joint_command_.position[i] = compute_trajectory(
                joint_command_.position[i], target_joint_angles_[i], MAX_SPEED_RAD_S_, dt);
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
