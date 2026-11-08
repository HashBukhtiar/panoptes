#include <memory>

#include "interfaces/msg/camera_orientation_command.hpp"
#include "kinematics/kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;
using JointState = sensor_msgs::msg::JointState;

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
    }

   private:
    rclcpp::Subscription<CameraOrientationCommand>::SharedPtr camera_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_sub_;

    void on_camera_command(const CameraOrientationCommand::SharedPtr msg) {
        RCLCPP_INFO(get_logger(),
                    "[MC] received camera orientation command: yaw=%f, pitch=%f, roll=%f", msg->yaw,
                    msg->pitch, msg->roll);

        kinematics::CameraOrientation orientation{msg->yaw, msg->pitch, msg->roll};
        auto joint_angles = kinematics::inverse_kinematics(orientation);

        if (joint_angles) {
            RCLCPP_INFO(get_logger(), "[MC] computed joint angles: ");
        } else {
            RCLCPP_INFO(get_logger(),
                        "[MC] failed to compute joint angles for the given orientation");
        }
    }

    void on_joint_state(const JointState::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "[MC] received joint state: %f, %f, %f", msg->position[0],
                    msg->position[1], msg->position[2]);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControlNode>());
    rclcpp::shutdown();
    return 0;
}
