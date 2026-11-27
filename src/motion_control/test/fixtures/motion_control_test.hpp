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

    static std::vector<std::string> default_param_args() {
        return {
            "camera_command_max_age_ms:=1e15", /* Disable staleness checks unless overriden */
            "joint_state_max_age_ms:=1e15",

            "joint_state_good_streak_min_ms:=1e15", /* Disable timing checks unless overriden */
            "joint_state_fault_streak_max_ms:=1e15",
            "hold_max_duration_ms:=1e15",
            "joint_state_max_gap_ms:=1e15",

            "joint_min_rad:=[-1e9,-1e9,-1e9]", /* Disable numerical checks unless overriden */
            "joint_max_rad:=[1e9,1e9,1e9]",
            "camera_min_rad:=[-1e9,-1e9,-1e9]",
            "camera_max_rad:=[1e9,1e9,1e9]",

            "max_speed_rad_s:=1000.0", /* Disable ramping unless overriden */
        };
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

    CameraOrientationCommand make_camera_command_msg(double yaw = 0.0, double pitch = 0.0,
                                                     double roll = 0.0, bool is_absolute = true) {
        CameraOrientationCommand msg;
        msg.header.stamp = peer_node_->now();
        msg.yaw = yaw;
        msg.pitch = pitch;
        msg.roll = roll;
        msg.is_absolute = is_absolute;
        return msg;
    }

    JointState make_joint_state_msg(const std::vector<double>& position = {2.0, 2.0, 2.0}) {
        JointState msg;
        msg.header.stamp = peer_node_->now();
        msg.position = position;
        return msg;
    }

    void publish_camera_command() {
        camera_pub_->publish(make_camera_command_msg()); /* default (valid) */
    }

    void publish_camera_command(const CameraOrientationCommand& msg) {
        camera_pub_->publish(msg);
    }

    void publish_joint_state() {
        joint_state_pub_->publish(make_joint_state_msg()); /* default (valid) */
    }

    void publish_joint_state(const JointState& msg) {
        joint_state_pub_->publish(msg);
    }

    void force_hold() {
        motion_control_node_->hold();
    }

    void force_recalibrate() {
        motion_control_node_->recalibrate();
    }

    void force_terminate() {
        motion_control_node_->terminate("forced by test");
    }

    /**
     * Seeds the node with an initial valid joint state and waits for the first joint command to be
     * published. Clears joint_commands_received_ so subsequent checks only see commands published
     * after seeding.
     *
     * @return The initial joint positions, which the first joint command should match.
     */
    std::vector<double> seed_with_joint_state() {
        const auto msg = make_joint_state_msg();
        publish_joint_state(msg);

        EXPECT_TRUE(spin_until([&] { return !joint_commands_received_.empty(); }));

        joint_commands_received_.clear();
        return msg.position;
    }

    /**
     * Checks that every joint command received so far is at some expected position.
     * Fails if nothing has been received or if any command differs in size or value.
     *
     * @param expected Positions every received joint command must match.
     */
    void expect_all_joint_commands_to_equal(const std::vector<double>& expected) {
        ASSERT_FALSE(joint_commands_received_.empty());

        for (const auto& cmd : joint_commands_received_) {
            ASSERT_EQ(cmd.position.size(), expected.size());
            for (size_t i = 0; i < expected.size(); ++i) {
                EXPECT_DOUBLE_EQ(cmd.position[i], expected[i]);
            }
        }
    }
};