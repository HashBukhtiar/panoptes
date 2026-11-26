#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "motion_control/motion_control_node.hpp"

using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;
using JointCommand = interfaces::msg::JointCommand;
using JointState = sensor_msgs::msg::JointState;

class MotionControlTest : public ::testing::Test {
   protected:
    std::shared_ptr<MotionControlNode> motion_control_node_;
    std::shared_ptr<rclcpp::Node> peer_node_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<CameraOrientationCommand>::SharedPtr camera_pub_;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<JointCommand>::SharedPtr joint_command_sub_;

    std::vector<JointCommand> joint_commands_received_;

    /* Override normal configuration limits since tests use dummy inputs, not real data */
    static std::vector<std::string> default_param_args() {
        return {"camera_command_max_age_ms:=1e15",  "joint_state_max_age_ms:=1e15",
                "joint_min_rad:=[-3.0,-3.0,-3.0]",  "joint_max_rad:=[3.0,3.0,3.0]",
                "camera_min_rad:=[-3.0,-3.0,-3.0]", "camera_max_rad:=[3.0,3.0,3.0]"};
    }

    static void init_ros(const std::vector<std::string>& param_overrides) {
        std::vector<std::string> storage = {"test_motion_control", "--ros-args"};
        for (const auto& p : param_overrides) {
            storage.push_back("-p");
            storage.push_back(p);
        }

        std::vector<const char*> args;
        args.reserve(storage.size());
        for (const auto& s : storage) {
            args.push_back(s.c_str());
        }

        rclcpp::init(static_cast<int>(args.size()), args.data());
    }

    static void SetUpTestSuite() {
        init_ros(default_param_args());
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
            [this](const JointCommand::SharedPtr msg) {
                joint_commands_received_.push_back(*msg);
            });

        executor_.add_node(motion_control_node_);
        executor_.add_node(peer_node_);

        ASSERT_TRUE(spin_until([this] {
            return camera_pub_->get_subscription_count() > 0 &&
                   joint_state_pub_->get_subscription_count() > 0;
        }));
    }

    bool spin_until(const std::function<bool()>& condition,
                    std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
        const auto start = std::chrono::steady_clock::now();
        while (!condition()) {
            const auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed >= timeout) {
                return false;
            }
            executor_.spin_some(std::chrono::milliseconds(10));
        }
        return true;
    }

    void spin_for(std::chrono::milliseconds duration) {
        spin_until([] { return false; }, duration);
    }

    virtual CameraOrientationCommand make_camera_command_msg() {
        CameraOrientationCommand msg;
        msg.header.stamp = peer_node_->now();
        return msg;
    }

    virtual JointState make_joint_state_msg() {
        JointState msg;
        msg.header.stamp = peer_node_->now();
        msg.position = {2.0, 2.0, 2.0};
        return msg;
    }
};