#include <memory>

#include "interfaces/msg/camera_orientation_command.hpp"
#include "rclcpp/rclcpp.hpp"

using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;

class MotionControlNode : public rclcpp::Node {
   public:
    MotionControlNode() : Node("motion_control") {
        sub_ = create_subscription<CameraOrientationCommand>(
            "camera_orientation_command", /* name of the topic */
            rclcpp::QoS(1).best_effort(), /* queue of up to 1 message (keeps newest only) */
            [this](const CameraOrientationCommand::SharedPtr msg) { on_command(msg); });
    }

   private:
    rclcpp::Subscription<CameraOrientationCommand>::SharedPtr sub_;

    void on_command(const CameraOrientationCommand::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "[MC] received camera orientation command: yaw=%f, pitch=%f",
                    msg->yaw, msg->pitch);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControlNode>());
    rclcpp::shutdown();
    return 0;
}
