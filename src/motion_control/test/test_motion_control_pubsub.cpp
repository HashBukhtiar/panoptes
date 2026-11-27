#include <gtest/gtest.h>

#include <chrono>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;

class MotionControlPubSubTest : public MotionControlTest {};

TEST_F(MotionControlPubSubTest, PublishesJointCommandWithOriginalTimestamp) {
    seed_with_joint_state();

    const auto camera_command = make_camera_command_msg();
    publish_camera_command(camera_command);

    ASSERT_TRUE(spin_until([&] { return !joint_commands_received_.empty(); }));

    const auto& joint_command = joint_commands_received_.front();
    EXPECT_EQ(joint_command.header.stamp.sec, camera_command.header.stamp.sec);
    EXPECT_EQ(joint_command.header.stamp.nanosec, camera_command.header.stamp.nanosec);
}

TEST_F(MotionControlPubSubTest, SeedsStartPositionWithJointState) {
    const auto joint_state = make_joint_state_msg();
    publish_joint_state(joint_state);

    ASSERT_TRUE(spin_until([&] { return !joint_commands_received_.empty(); }));

    const auto& joint_command = joint_commands_received_.front();
    EXPECT_EQ(joint_command.position, joint_state.position);
}

TEST_F(MotionControlPubSubTest, DoesNotPublishWithoutJointState) {
    publish_camera_command();
    spin_for(200ms);

    EXPECT_TRUE(joint_commands_received_.empty());
}

TEST_F(MotionControlPubSubTest, PublishesEvenWithoutCameraOrientationCommand) {
    publish_joint_state();

    EXPECT_TRUE(spin_until([this] { return !joint_commands_received_.empty(); }));
}

TEST_F(MotionControlPubSubTest, AppliesRelativeCommandOnTopOfLastCommand) {
    const double x = 0.1, d = 0.05;

    seed_with_joint_state();

    auto publish_and_get_joint_command = [&](const CameraOrientationCommand& msg) {
        joint_commands_received_.clear();
        publish_camera_command(msg);

        EXPECT_TRUE(spin_until([this] { return !joint_commands_received_.empty(); }));

        return joint_commands_received_.back();
    };

    publish_and_get_joint_command(make_camera_command_msg(x, x, x, true));
    const auto relative_out =
        publish_and_get_joint_command(make_camera_command_msg(d, d, d, false));
    const auto absolute_out =
        publish_and_get_joint_command(make_camera_command_msg(x + d, x + d, x + d, true));

    ASSERT_EQ(relative_out.position.size(), absolute_out.position.size());

    for (size_t i = 0; i < relative_out.position.size(); ++i) {
        EXPECT_NEAR(relative_out.position[i], absolute_out.position[i], 1e-6);
    }
}