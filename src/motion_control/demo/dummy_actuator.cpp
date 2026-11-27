#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "interfaces/msg/joint_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using JointCommand = interfaces::msg::JointCommand;
using JointState = sensor_msgs::msg::JointState;

class DummyActuator : public rclcpp::Node {
   public:
    DummyActuator() : Node("dummy_actuator") {
        declare_parameter("version", "valid");
        version_ = get_parameter("version").as_string();

        subscription_ = create_subscription<JointCommand>(
            "joint_command", rclcpp::QoS(1).best_effort(),
            [this](const JointCommand::SharedPtr msg) { on_joint_command(msg); });

        publisher_ = create_publisher<JointState>("joint_state", rclcpp::QoS(1).best_effort());

        start_time_ = now();
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / 90.0),
                                   [this]() { publish_state(); });
    }

   private:
    std::string version_;

    rclcpp::Time start_time_;
    rclcpp::Subscription<JointCommand>::SharedPtr subscription_;
    rclcpp::Publisher<JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> positions_{0.0, 0.0, 0.0};
    size_t receive_log_counter_ = 0;
    size_t publish_log_counter_ = 0;

    void on_joint_command(const JointCommand::SharedPtr msg) {
        positions_ = msg->position;

        if (++receive_log_counter_ >= 30) {
            std::ostringstream position;
            for (size_t i = 0; i < msg->position.size(); ++i) {
                if (i > 0) {
                    position << ", ";
                }
                position << msg->position[i];
            }

            RCLCPP_INFO(get_logger(), "Received joint command: position=[%s]",
                        position.str().c_str());
            receive_log_counter_ = 0;
        }
    }

    void publish_state() {
        JointState msg;
        msg.header.stamp = now();
        msg.position = positions_;

        const double t = (now() - start_time_).seconds();

        if (version_ == "invalid" && t >= 1.0 && t < 1.1) {
            /* Publishes invalid states from t=1 to t=1.1 */
            if (!msg.position.empty()) {
                msg.position[0] = std::numeric_limits<double>::quiet_NaN();
            }
        } else if (version_ == "old") {
            /* Publishes stale states permanently */
            msg.header.stamp = now() - rclcpp::Duration::from_seconds(1);
        }

        publisher_->publish(msg);

        if (++publish_log_counter_ >= 30) {
            std::ostringstream position;
            for (size_t i = 0; i < msg.position.size(); ++i) {
                if (i > 0) {
                    position << ", ";
                }
                position << msg.position[i];
            }

            RCLCPP_INFO(get_logger(), "Published joint state: position=[%s], stamp=%u.%09u",
                        position.str().c_str(), msg.header.stamp.sec, msg.header.stamp.nanosec);
            publish_log_counter_ = 0;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyActuator>());
    rclcpp::shutdown();
    return 0;
}