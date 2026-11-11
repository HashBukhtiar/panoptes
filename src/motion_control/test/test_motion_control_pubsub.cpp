#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <vector>

#include "interfaces/msg/camera_orientation_command.hpp"
#include "interfaces/msg/joint_command.hpp"
#include "motion_control/motion_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;
using JointCommand = interfaces::msg::JointCommand;
using JointState = sensor_msgs::msg::JointState;

class MotionControlPubSubTest : public ::testing::Test {
   protected:
    std::shared_ptr<MotionControlNode> motion_control_node_;
    std::shared_ptr<rclcpp::Node> peer_node_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<CameraOrientationCommand>::SharedPtr camera_pub_;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<JointCommand>::SharedPtr joint_command_sub_;

    std::vector<JointCommand> received_;

    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        motion_control_node_ = std::make_shared<MotionControlNode>();
        peer_node_ = std::make_shared<rclcpp::Node>("peer");

        camera_pub_ = peer_node_->create_publisher<CameraOrientationCommand>(
            "camera_orientation_command", rclcpp::QoS(1).best_effort());
        joint_state_pub_ =
            peer_node_->create_publisher<JointState>("joint_state", rclcpp::QoS(1).best_effort());
        joint_command_sub_ = peer_node_->create_subscription<JointCommand>(
            "joint_command", rclcpp::QoS(1).best_effort(),
            [this](const JointCommand::SharedPtr msg) { received_.push_back(*msg); });

        executor_.add_node(motion_control_node_);
        executor_.add_node(peer_node_);

        ASSERT_TRUE(spin_until([this] {
            return camera_pub_->get_subscription_count() > 0 &&
                   joint_state_pub_->get_subscription_count() > 0;
        }));
    }

    bool spin_until(const std::function<bool()>& condition,
                    std::chrono::milliseconds timeout = 2s) {
        const auto start = std::chrono::steady_clock::now();

        while (!condition()) {
            const auto elapsed = std::chrono::steady_clock::now() - start;

            if (elapsed >= timeout) {
                return false;
            }

            executor_.spin_some(10ms);
        }

        return true;
    }

    CameraOrientationCommand make_camera_command_msg() {
        CameraOrientationCommand msg;
        msg.header.stamp.sec = 123;
        msg.header.stamp.nanosec = 456;
        return msg;
    }

    JointState make_joint_state_msg() {
        JointState msg;
        msg.header.stamp.sec = 123;
        msg.header.stamp.nanosec = 456;
        return msg;
    }
};

TEST_F(MotionControlPubSubTest, PublishesJointCommandWithOriginalTimestamp) {
    const auto camera_command = make_camera_command_msg();
    const auto joint_state = make_joint_state_msg();

    ASSERT_TRUE(spin_until([&] {
        camera_pub_->publish(camera_command);
        joint_state_pub_->publish(joint_state);
        return !received_.empty();
    }));

    const auto& joint_command = received_.front();
    EXPECT_EQ(joint_command.header.stamp.sec, camera_command.header.stamp.sec);
    EXPECT_EQ(joint_command.header.stamp.nanosec, camera_command.header.stamp.nanosec);
}