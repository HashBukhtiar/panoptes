#include <gtest/gtest.h>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;
using State = MotionControlNode::State;

class MotionControlStateTest : public MotionControlTest {
   protected:
    /**
     * Seeds the node with a valid joint state and waits until it publishes.
     *
     * Republishes joint state until the first joint command arrives, then clears
     * joint_commands_received_ so later checks only see output from after seeding.
     *
     * @return The positions that were published, which the seeded joint command should now hold.
     */
    std::vector<double> seed_with_joint_state() {
        const auto msg = make_joint_state_msg();
        EXPECT_TRUE(spin_until([&] {
            joint_state_pub_->publish(make_joint_state_msg());
            return !joint_commands_received_.empty();
        }));
        joint_commands_received_.clear();
        return msg.position;  // TODO: might remove for this one? dk if needed?
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

TEST_F(MotionControlStateTest, NormalStatePublishesNewCameraOrientationCommand) {
    const auto initial_state = seed_with_joint_state();

    publish_camera_command(make_camera_command_msg());
    spin_for(300ms);

    ASSERT_FALSE(joint_commands_received_.empty());
    EXPECT_NE(joint_commands_received_.back().position, initial_state);
}

TEST_F(MotionControlStateTest, HoldStateRepublishesOldCameraOrientationCommand) {
    const auto initial_state = seed_with_joint_state();

    force_hold();
    publish_camera_command(make_camera_command_msg());
    spin_for(300ms);

    expect_all_joint_commands_to_equal(initial_state);
}

TEST_F(MotionControlStateTest, HoldToNormalReseedsJointState) {
    GTEST_SKIP() << "[TODO] not implemented yet";
}

TEST_F(MotionControlStateTest, HoldToNormalClearsOldCameraOrientationCommand) {
    GTEST_SKIP() << "[TODO] not implemented yet";
}

TEST_F(MotionControlStateTest, CalibratingStateDoesNotPublish) {
    GTEST_SKIP() << "[N/A] tested by `NoOutputWithoutJointState`";
}

TEST_F(MotionControlStateTest, TerminalStateRepublishesOldCameraOrientationCommand) {
    GTEST_SKIP() << "[TODO] not implemented yet";
}

TEST_F(MotionControlStateTest, TerminalStateIsNotExitable) {
    GTEST_SKIP() << "[TODO] not implemented yet";
}