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
 * Stops motion and alerts.
 */
void MotionControlNode::hold() {
    alerter_.raise("[WARN] incoming messages are faulty for too long, holding");
    state_ = State::HOLDING;
    if (!hold_start_time_) {
        hold_start_time_ = std::chrono::steady_clock::now();
    }
}

/**
 * Re-seeds the last joint command from the latest joint state, then resumes.
 */
void MotionControlNode::recalibrate() {
    last_joint_command_.position = last_joint_state_;
    last_target_joint_angles_.reset();
    alerter_.raise("[INFO] incoming messages now good, recalibrating");
    state_ = State::NORMAL;
    hold_start_time_.reset();
}

/**
 * Stops motion permanently and raises an urgent alert: a restart is required.
 * The loop keeps republishing the last command unchanged, and nothing leaves this state.
 *
 * @param reason    Why the node is terminating, included in the alert.
 */
void MotionControlNode::terminate(const std::string& reason) {
    if (state_ == State::TERMINAL) {
        return;
    }
    alerter_.raise("[URGENT] restart required, terminating: " + reason);
    state_ = State::TERMINAL;
}

/**
 * Begins a timer for joint state being faulty; clears the timer for joint state being good.
 */
void MotionControlNode::start_joint_state_fault_timer() {
    if (!joint_state_fault_start_time_) {
        joint_state_fault_start_time_ = std::chrono::steady_clock::now();
    }
    joint_state_good_start_time_.reset();
}

/**
 * Begins a timer for joint state being good; clears the timer for joint state being faulty.
 */
void MotionControlNode::start_joint_state_good_timer() {
    if (!joint_state_good_start_time_) {
        joint_state_good_start_time_ = std::chrono::steady_clock::now();
    }
    joint_state_fault_start_time_.reset();
}

/**
 * Checks all timers and responds to state changes accordingly.
 *
 * @param now       Current local time.
 */
void MotionControlNode::update_state(LocalClockTime now) {
    if (state_ == State::CALIBRATING || state_ == State::TERMINAL) {
        return;
    }

    /* checks if there was no new joint state message after N ms */
    if (timing::is_too_old(timing::get_ms_between(last_joint_state_time_, now),
                           JOINT_STATE_MAX_GAP_MS_)) {
        start_joint_state_fault_timer();
    }

    if (state_ == State::NORMAL) {
        if (joint_state_fault_start_time_ &&
            timing::get_ms_between(*joint_state_fault_start_time_, now) >=
                JOINT_STATE_FAULT_STREAK_MAX_MS_) {
            hold();
        }
    } else if (state_ == State::HOLDING) {
        if (hold_start_time_ &&
            timing::get_ms_between(*hold_start_time_, now) >= HOLD_MAX_DURATION_MS_) {
            terminate("hold duration exceeded tolerance");
        } else if (joint_state_good_start_time_ &&
                   timing::get_ms_between(*joint_state_good_start_time_, now) >=
                       JOINT_STATE_GOOD_STREAK_MIN_MS_) {
            recalibrate();
        }
    }
}

/**
 * Callback function for receiving camera orientation commands.
 *
 * @param msg       The received CameraOrientationCommand message.
 */
void MotionControlNode::on_camera_command(const CameraOrientationCommand::SharedPtr msg) {
    if (!msg || !validation::valid_camera_orientation(msg->yaw, msg->pitch, msg->roll,
                                                      CAMERA_MIN_RAD_, CAMERA_MAX_RAD_)) {
        alerter_.raise("[WARN] received null/invalid camera command, dropping this message");
        return;
    }

    const RosClockTime stamp(msg->header.stamp, get_clock()->get_clock_type());

    if (timing::is_too_old(timing::get_age_ms(now(), stamp), CAMERA_COMMAND_MAX_AGE_MS_)) {
        alerter_.raise("[WARN] received old camera command, dropping this message");
        return;
    }

    if (timing::is_out_of_order(stamp, last_camera_command_stamp_)) {
        /* not alerting here */
        return;
    }

    kinematics::CameraOrientation orientation{msg->yaw, msg->pitch, msg->roll};

    if (!msg->is_absolute) {
        std::optional<kinematics::CameraOrientation> base;
        if (last_target_joint_angles_) {
            base = kinematics::forward_kinematics(*last_target_joint_angles_);
        } else {
            // TODO: if we recalibrated (i.e., lose last relative target), start from joint
            // state...?
        }
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
        alerter_.raise(
            "[WARN] kinematics library made invalid transformation, dropping this message");
        return;
    }

    last_camera_command_stamp_ = stamp;
    last_target_joint_angles_ = *joint_angles;
}

/**
 * Callback function for receiving joint state messages.
 *
 * @param msg       The received JointState message.
 */
void MotionControlNode::on_joint_state(const JointState::SharedPtr msg) {
    if (!msg || !validation::valid_joint_positions(msg->position, JOINT_MIN_RAD_, JOINT_MAX_RAD_)) {
        start_joint_state_fault_timer();
        return;
    }

    const RosClockTime stamp(msg->header.stamp, get_clock()->get_clock_type());

    if (timing::is_too_old(timing::get_age_ms(now(), stamp), JOINT_STATE_MAX_AGE_MS_)) {
        start_joint_state_fault_timer();
        return;
    }

    if (timing::is_out_of_order(stamp, last_joint_state_stamp_)) {
        /* not alerting here */
        return;
    }

    const LocalClockTime now = std::chrono::steady_clock::now();

    last_joint_state_ = msg->position;
    last_joint_state_stamp_ = stamp;
    last_joint_state_time_ = now;
    start_joint_state_good_timer();

    /* use the first received joint state as the initial position for trajectory generation */
    if (state_ == State::CALIBRATING) {
        last_joint_command_.position = msg->position;
        last_joint_command_time_ = now;
        state_ = State::NORMAL;
    }
}

/**
 * Computes the next joint command and sends it to the joint controller at a fixed rate.
 * This function is called periodically by a timer.
 */
void MotionControlNode::compute_and_send_joint_command() {
    LocalClockTime now = std::chrono::steady_clock::now();
    update_state(now);

    double dt = std::chrono::duration<double>(now - last_joint_command_time_).count();
    last_joint_command_time_ = now;

    if (last_joint_command_.position.empty()) {
        return;
    }

    if (state_ == State::NORMAL && last_target_joint_angles_) {
        trajectory::step_rate_limited(last_joint_command_.position, *last_target_joint_angles_,
                                      MAX_SPEED_RAD_S_, dt);
        last_joint_command_.header.stamp = *last_camera_command_stamp_;
    }

    joint_command_pub_->publish(last_joint_command_);
}