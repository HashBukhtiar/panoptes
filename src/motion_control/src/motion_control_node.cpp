#include "motion_control/motion_control_node.hpp"

#include "motion_control/timing.hpp"
#include "motion_control/trajectory.hpp"
#include "motion_control/validation.hpp"

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
 * Declares an array parameter with the given name and fallback value, and returns its value.
 *
 * @param name      The name of the parameter to declare and get.
 * @param fallback  The fallback value to use if the parameter is not already declared.
 */
std::vector<double> MotionControlNode::declare_and_get_vec(const std::string& name,
                                                           const std::vector<double>& fallback) {
    declare_parameter(name, fallback);
    return get_parameter(name).as_double_array();
}

/**
 * Callback function for receiving camera orientation commands.
 *
 * @param msg       The received CameraOrientationCommand message.
 */
void MotionControlNode::on_camera_command(const CameraOrientationCommand::SharedPtr msg) {
    if (!msg) {
        // TODO: handle
        return;
    }

    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());

    if (timing::is_too_old(timing::get_age_ms(now(), stamp), CAMERA_COMMAND_MAX_AGE_MS_)) {
        // TODO: handle
        return;
    }

    if (timing::is_out_of_order(stamp, last_camera_command_stamp_)) {
        // TODO: handle
        return;
    }

    kinematics::CameraOrientation orientation{msg->yaw, msg->pitch, msg->roll};

    if (!msg->is_absolute) {
        auto base = kinematics::forward_kinematics(target_joint_angles_);
        if (!base) {
            // TODO: handle
            return;
        }
        orientation.yaw += base->yaw;
        orientation.pitch += base->pitch;
        orientation.roll += base->roll;
    }

    if (!validation::valid_camera_orientation(orientation.yaw, orientation.pitch, orientation.roll,
                                              CAMERA_MIN_RAD_, CAMERA_MAX_RAD_)) {
        // TODO: handle
        return;
    }

    auto joint_angles = kinematics::inverse_kinematics(orientation);

    if (!joint_angles ||
        !validation::valid_joint_positions(*joint_angles, JOINT_MIN_RAD_, JOINT_MAX_RAD_)) {
        // TODO: handle
        return;
    }

    last_camera_command_stamp_ = stamp;
    target_joint_angles_ = *joint_angles;
}

/**
 * Callback function for receiving joint state messages.
 *
 * @param msg       The received JointState message.
 */
void MotionControlNode::on_joint_state(const JointState::SharedPtr msg) {
    if (!msg) {
        // TODO: handle
        return;
    }

    if (!validation::valid_joint_positions(msg->position, JOINT_MIN_RAD_, JOINT_MAX_RAD_)) {
        // TODO: handle
        return;
    }

    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());

    if (timing::is_too_old(timing::get_age_ms(now(), stamp), JOINT_STATE_MAX_AGE_MS_)) {
        // TODO: handle
        return;
    }

    if (timing::is_out_of_order(stamp, last_joint_state_stamp_)) {
        // TODO: handle
        return;
    }

    const auto now_steady = std::chrono::steady_clock::now();

    last_joint_state_stamp_ = stamp;
    last_good_joint_state_time_ = now_steady;

    if (last_joint_command_.position.empty()) {
        last_joint_command_.position =
            msg->position; /* use the first received joint state as the initial position for
                              trajectory generation */
        last_joint_command_time_ = now_steady;
    }
}

/**
 * Computes the next joint command and sends it to the joint controller at a fixed rate.
 * This function is called periodically by a timer.
 */
void MotionControlNode::compute_and_send_joint_command() {
    auto now_steady = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now_steady - last_joint_command_time_).count();
    last_joint_command_time_ = now_steady;

    if (target_joint_angles_.empty() || last_joint_command_.position.empty()) {
        return;
    }

    /* checks if there was no new joint state message after N ms */
    if (timing::is_too_old(timing::get_ms_between(last_good_joint_state_time_, now_steady),
                           JOINT_STATE_MAX_GAP_MS_)) {
        // TODO: handle
    }

    trajectory::step_rate_limited(last_joint_command_.position, target_joint_angles_,
                                  MAX_SPEED_RAD_S_, dt);

    last_joint_command_.header.stamp = *last_camera_command_stamp_;
    joint_command_pub_->publish(last_joint_command_);
}