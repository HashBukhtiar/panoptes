#include <gtest/gtest.h>

#include <chrono>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;
using MotionControlPubSubTest = MotionControlTest;

TEST_F(MotionControlPubSubTest, PublishesJointCommandWithOriginalTimestamp) {
    const auto camera_command = make_camera_command_msg();
    const auto joint_state = make_joint_state_msg();

    ASSERT_TRUE(spin_until([&] {
        camera_pub_->publish(camera_command);
        joint_state_pub_->publish(joint_state);
        return !joint_commands_received_.empty();
    }));

    const auto& joint_command = joint_commands_received_.front();
    EXPECT_EQ(joint_command.header.stamp.sec, camera_command.header.stamp.sec);
    EXPECT_EQ(joint_command.header.stamp.nanosec, camera_command.header.stamp.nanosec);
}

TEST_F(MotionControlPubSubTest, SeedsStartPositionWithJointState) {
    const auto camera_command = make_camera_command_msg();
    const auto joint_state = make_joint_state_msg();

    ASSERT_TRUE(spin_until([&] {
        camera_pub_->publish(camera_command);
        joint_state_pub_->publish(joint_state);
        return !joint_commands_received_.empty();
    }));

    const auto& joint_command = joint_commands_received_.front();
    for (double val : joint_command.position) {
        EXPECT_LE(val, 2.0); /* from make_joint_state_msg */
        EXPECT_GE(val, 1.0); /* from stub_kinematics.cpp */
    }
}

TEST_F(MotionControlPubSubTest, NoOutputWithoutJointState) {
    camera_pub_->publish(make_camera_command_msg());
    spin_for(300ms);
    EXPECT_TRUE(joint_commands_received_.empty());
}

TEST_F(MotionControlPubSubTest, OutputEvenWithoutCameraOrientationCommand) {
    joint_state_pub_->publish(make_joint_state_msg());
    spin_for(300ms);
    EXPECT_FALSE(joint_commands_received_.empty());
}

TEST_F(MotionControlPubSubTest, RelativeCommandIsAppliedOnTopOfLastCommand) {
    const double x = 0.1, d = 0.05;

    /* Send one command, wait for the loop to publish, then return the most recent joint command. */
    auto send = [&](const CameraOrientationCommand& cmd) {
        joint_commands_received_.clear();
        camera_pub_->publish(cmd);
        joint_state_pub_->publish(make_joint_state_msg());

        spin_until([&] { return !joint_commands_received_.empty(); });

        EXPECT_FALSE(joint_commands_received_.empty());
        return joint_commands_received_.empty() ? JointCommand{} : joint_commands_received_.back();
    };

    send(make_camera_command_msg(x, x, x, true, 1));
    const auto relative_out = send(make_camera_command_msg(d, d, d, false, 2));
    const auto absolute_out = send(make_camera_command_msg(x + d, x + d, x + d, true, 3));

    ASSERT_EQ(relative_out.position.size(), absolute_out.position.size());

    for (size_t i = 0; i < relative_out.position.size(); ++i) {
        EXPECT_NEAR(relative_out.position[i], absolute_out.position[i], 1e-6);
    }
}