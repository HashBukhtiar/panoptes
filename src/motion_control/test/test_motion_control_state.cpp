#include <gtest/gtest.h>

#include "motion_control_test.hpp"

using State = MotionControlNode::State;

class MotionControlStateTest : public MotionControlTest {
   protected:
    static constexpr std::chrono::milliseconds DRAIN_WINDOW{30};
    static constexpr std::chrono::milliseconds OBSERVATION_WINDOW{50};

    /**
     * Waits for the node to publish commands reflecting the new state after a state change.
     *
     * Commands from the previous state may still be in flight after the state change. Therefore,
     * this first drains and discards old commands (DRAIN_WINDOW), then collects fresh commands
     * (OBSERVATION_WINDOW) in joint_commands_received_.
     *
     * Contrast with spin_for(), which collects all commands, including in-flight commands from
     * before the state change.
     */
    void spin_for_fresh_commands() {
        spin_for(DRAIN_WINDOW);
        joint_commands_received_.clear();
        spin_for(OBSERVATION_WINDOW);
    }
};

TEST_F(MotionControlStateTest, NormalStatePublishesNewCameraOrientationCommand) {
    const auto initial_state = seed_with_joint_state();

    publish_camera_command();

    ASSERT_TRUE(spin_until([&] {
        return !joint_commands_received_.empty() &&
               joint_commands_received_.back().position != initial_state;
    }));
    ASSERT_EQ(motion_control_node_->get_state(), State::NORMAL);
}

TEST_F(MotionControlStateTest, HoldStateRepublishesOldCameraOrientationCommand) {
    const auto initial_state = seed_with_joint_state();

    force_hold();
    publish_camera_command(make_camera_command_msg());
    spin_for(OBSERVATION_WINDOW);

    ASSERT_EQ(motion_control_node_->get_state(), State::HOLDING);
    expect_all_joint_commands_to_equal(initial_state);
}

TEST_F(MotionControlStateTest, HoldToNormalReseedsJointState) {
    const auto initial_state = seed_with_joint_state();

    std::vector<double> new_state = initial_state;
    for (double& value : new_state) {
        value += 0.1;
    }

    publish_joint_state(make_joint_state_msg(new_state));

    /* During hold, the last commanded positions are maintained */
    force_hold();
    spin_for_fresh_commands();

    ASSERT_EQ(motion_control_node_->get_state(), State::HOLDING);
    expect_all_joint_commands_to_equal(initial_state);

    /* After recalibration, joint command is reseeded to latest joint state */
    force_recalibrate();
    spin_for_fresh_commands();

    ASSERT_EQ(motion_control_node_->get_state(), State::NORMAL);
    expect_all_joint_commands_to_equal(new_state);
}

TEST_F(MotionControlStateTest, HoldToNormalClearsOldCameraOrientationCommand) {
    const auto initial_state = seed_with_joint_state();

    /* Before hold, the camera command changes the joint positions */
    publish_camera_command();

    ASSERT_TRUE(spin_until([&] {
        return !joint_commands_received_.empty() &&
               joint_commands_received_.back().position != initial_state;
    }));
    ASSERT_EQ(motion_control_node_->get_state(), State::NORMAL);

    /* During hold, the last commanded positions are maintained */
    force_hold();
    spin_for_fresh_commands();

    ASSERT_EQ(motion_control_node_->get_state(), State::HOLDING);
    ASSERT_FALSE(joint_commands_received_.empty());
    ASSERT_NE(joint_commands_received_.back().position, initial_state);

    /* After recalibration, the old camera command is invalidated */
    force_recalibrate();
    spin_for_fresh_commands();

    ASSERT_EQ(motion_control_node_->get_state(), State::NORMAL);
    expect_all_joint_commands_to_equal(initial_state);
}

TEST_F(MotionControlStateTest, CalibratingStateDoesNotPublish) {
    /* By default, the node begins in calibrating state */
    publish_camera_command(make_camera_command_msg());
    spin_for(OBSERVATION_WINDOW);

    ASSERT_EQ(motion_control_node_->get_state(), State::CALIBRATING);
    ASSERT_TRUE(joint_commands_received_.empty());
}

TEST_F(MotionControlStateTest, TerminalStateRepublishesOldCameraOrientationCommand) {
    const auto initial_state = seed_with_joint_state();

    force_terminate();
    publish_camera_command(make_camera_command_msg());
    spin_for(OBSERVATION_WINDOW);

    ASSERT_EQ(motion_control_node_->get_state(), State::TERMINAL);
    expect_all_joint_commands_to_equal(initial_state);
}