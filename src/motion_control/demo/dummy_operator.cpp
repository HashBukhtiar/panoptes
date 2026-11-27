#include <chrono>
#include <cmath>
#include <limits>

#include "interfaces/msg/camera_orientation_command.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;

class DummyOperator : public rclcpp::Node {
   public:
    DummyOperator() : Node("dummy_operator") {
        declare_parameter("version", "valid");
        version_ = get_parameter("version").as_string();

        publisher_ = create_publisher<interfaces::msg::CameraOrientationCommand>(
            "camera_orientation_command", rclcpp::QoS(1).best_effort());

        start_time_ = now();
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / 90.0),
                                   [this]() { publish_command(); });
    }

   private:
    std::string version_;

    rclcpp::Time start_time_;
    rclcpp::Publisher<CameraOrientationCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t log_counter_ = 0;

    void publish_command() {
        CameraOrientationCommand msg;
        msg.header.stamp = now();
        msg.mode = 0;
        msg.is_absolute = true;

        const double t = (now() - start_time_).seconds();
        const double angle = 0.5 * std::sin(2.0 * M_PI * t / 4.0);

        msg.yaw = angle;
        msg.pitch = angle * 0.5;
        msg.roll = angle * 0.2;

        if (version_ == "invalid" && t >= 1.0 && t < 1.1) {
            /* Publishes invalid commands from t=1 to t=1.1 */
            msg.yaw = std::numeric_limits<double>::quiet_NaN();
        } else if (version_ == "old") {
            /* Publishes stsale commands permanently */
            msg.header.stamp = now() - rclcpp::Duration::from_seconds(1);
        }

        publisher_->publish(msg);

        if (++log_counter_ >= 30) {
            RCLCPP_INFO(
                get_logger(), "Published command: yaw=%.3f, pitch=%.3f, roll=%.3f, stamp=%u.%09u",
                msg.yaw, msg.pitch, msg.roll, msg.header.stamp.sec, msg.header.stamp.nanosec);
            log_counter_ = 0;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyOperator>());
    rclcpp::shutdown();
    return 0;
}
