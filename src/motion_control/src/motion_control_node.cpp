#include "motion_control/motion_control_node.hpp"

#include "motion_control/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

MotionControlNode::MotionControlNode() : Node("motion_control") {
    camera_sub_ = create_subscription<CameraOrientationCommand>(
        "camera_orientation_command", /* name of the topic */
        rclcpp::QoS(1).best_effort(), /* queue of up to 1 message (keeps newest only) */
        [this](const CameraOrientationCommand::SharedPtr msg) { on_camera_command(msg); });

    joint_state_sub_ = create_subscription<JointState>(
        "joint_state",                /* name of the topic*/
        rclcpp::QoS(1).best_effort(), /* queue of up to 1 message (keeps newest only) */
        [this](const JointState::SharedPtr msg) { on_joint_state(msg); });

    joint_command_pub_ = create_publisher<JointCommand>(
        "joint_command",             /* name of the topic*/
        rclcpp::QoS(1).best_effort() /* queue of up to 1 message (keeps newest only) */
    );

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / LOOP_RATE_HZ_),
                               [this]() { compute_and_send_joint_command(); });
}

/**
 * Declares a parameter with the given name and fallback value, and returns its value.
 *
 * @param name      The name of the parameter to declare and get.
 * @param fallback  The fallback value to use if the parameter is not already declared.
 */
double MotionControlNode::declare_and_get(const std::string& name, double fallback) {
    declare_parameter(name, fallback);
    return get_parameter(name).as_double();
}

/**
 * Callback function for receiving camera orientation commands.
 *
 * @param msg       The received CameraOrientationCommand message.
 */
void MotionControlNode::on_camera_command(const CameraOrientationCommand::SharedPtr msg) {
    kinematics::CameraOrientation orientation{msg->yaw, msg->pitch, msg->roll};

    auto joint_angles = kinematics::inverse_kinematics(orientation);

    if (joint_angles) {
        target_joint_angles_ = *joint_angles;
        target_joint_angles_time_ = msg->header.stamp;
    } else {
        // TODO: handle appropriately
    }
}

/**
 * Callback function for receiving joint state messages.
 *
 * @param msg       The received JointState message.
 */
void MotionControlNode::on_joint_state(const JointState::SharedPtr msg) {
    if (last_joint_command_.position.empty()) {
        last_joint_command_.position =
            msg->position; /* use the first received joint state as the initial position for
                              trajectory generation */
        last_joint_command_time_ = std::chrono::steady_clock::now();
    }
}

/**
 * Computes the next joint command and sends it to the joint controller at a fixed rate.
 * This function is called periodically by a timer.
 */
void MotionControlNode::compute_and_send_joint_command() {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_joint_command_time_).count();
    last_joint_command_time_ = now;

    if (target_joint_angles_.empty() || last_joint_command_.position.empty()) {
        return;
    }

    trajectory::step_rate_limited(last_joint_command_.position, target_joint_angles_,
                                  MAX_SPEED_RAD_S_, dt);

    last_joint_command_.header.stamp = target_joint_angles_time_;
    joint_command_pub_->publish(last_joint_command_);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto logger = rclcpp::get_logger("main_logger");
    RCLCPP_INFO(logger, "Initializing motion_control node...");
    rclcpp::spin(std::make_shared<MotionControlNode>());

    rclcpp::shutdown();
    return 0;
}
