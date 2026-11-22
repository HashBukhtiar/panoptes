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
        /* Override normal configuration limits since this test uses dummy inputs, not real data */
        const std::vector<const char*> args = {
            "test_motion_control_pubsub",
            "--ros-args",
            "-p",
            "camera_command_max_age_ms:=1e15", /* arbitrarily high to avoid test messages being too
                                                  old */
            "-p",
            "joint_state_max_age_ms:=1e15", /* arbitrarily high to avoid test messages being too
                                                  old */
            "-p",
            "joint_min_rad:=[-3.0,-3.0,-3.0]",
            "-p",
            "joint_max_rad:=[3.0,3.0,3.0]",
            "-p",
            "max_speed_rad_s:=1e6", /* arbitrarily high so that all test joint commands can be
                                       accomplished in one step */
        };
        rclcpp::init(static_cast<int>(args.size()), args.data());
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

    void spin_for(std::chrono::milliseconds duration) {
        spin_until([] { return false; }, duration);
    }

    CameraOrientationCommand make_camera_command_msg(double yaw = 0.0, double pitch = 0.0,
                                                     double roll = 0.0, bool is_absolute = true,
                                                     int32_t sec = 123) {
        CameraOrientationCommand msg;
        msg.header.stamp.sec =
            sec; /* must increase between commands or it is dropped as out-of-order */
        msg.header.stamp.nanosec = 456;
        msg.yaw = yaw;
        msg.pitch = pitch;
        msg.roll = roll;
        msg.is_absolute = is_absolute;
        return msg;
    }

    JointState make_joint_state_msg() {
        JointState msg;
        msg.header.stamp.sec = 123;
        msg.header.stamp.nanosec = 456;
        msg.position = {1.5, 1.5, 1.5};
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

TEST_F(MotionControlPubSubTest, SeedsStartPositionWithJointState) {
    const auto camera_command = make_camera_command_msg();
    const auto joint_state = make_joint_state_msg();

    ASSERT_TRUE(spin_until([&] {
        camera_pub_->publish(camera_command);
        joint_state_pub_->publish(joint_state);
        return !received_.empty();
    }));

    const auto& joint_command = received_.front();
    for (double val : joint_command.position) {
        EXPECT_LE(val, 3.0); /* from make_joint_state_msg */
        EXPECT_GE(val, 1.0); /* from stub_kinematics.cpp */
    }
}

TEST_F(MotionControlPubSubTest, NoOutputWithoutJointState) {
    camera_pub_->publish(make_camera_command_msg());
    spin_for(300ms);
    EXPECT_TRUE(received_.empty());
}

TEST_F(MotionControlPubSubTest, NoOutputWithoutCameraOrientationCommand) {
    joint_state_pub_->publish(make_joint_state_msg());
    spin_for(300ms);
    EXPECT_TRUE(received_.empty());
}

TEST_F(MotionControlPubSubTest, RelativeCommandIsAppliedOnTopOfLastCommand) {
    const double x = 0.1, d = 0.05;

    /* Send one command, wait for the loop to publish, then return the most recent joint command. */
    auto send = [&](const CameraOrientationCommand& cmd) {
        received_.clear();
        camera_pub_->publish(cmd);
        joint_state_pub_->publish(make_joint_state_msg());

        spin_until([&] { return !received_.empty(); });

        EXPECT_FALSE(received_.empty());
        return received_.empty() ? JointCommand{} : received_.back();
    };

    send(make_camera_command_msg(x, x, x, true, 1));
    const auto relative_out = send(make_camera_command_msg(d, d, d, false, 2));
    const auto absolute_out = send(make_camera_command_msg(x + d, x + d, x + d, true, 3));

    ASSERT_EQ(relative_out.position.size(), absolute_out.position.size());

    for (size_t i = 0; i < relative_out.position.size(); ++i) {
        EXPECT_NEAR(relative_out.position[i], absolute_out.position[i], 1e-6);
    }
}