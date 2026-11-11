#include <memory>

#include "motion_control/motion_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto logger = rclcpp::get_logger("main_logger");
    RCLCPP_INFO(logger, "Initializing motion_control node...");
    rclcpp::spin(std::make_shared<MotionControlNode>());

    rclcpp::shutdown();
    return 0;
}
