#include "rclcpp/rclcpp.hpp"

class DummyActuator : public rclcpp::Node {
   public:
    DummyActuator() : Node("dummy_actuator") {}
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyActuator>());
    rclcpp::shutdown();
    return 0;
}