#include <gtest/gtest.h>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;
using State = MotionControlNode::State;

class MotionControlStateTest : public MotionControlTest {
   protected:
    /* Time to spin after a state change, before draining joint_commands_received_, so commands the
     * node published BEFORE the change but that are still in transit are delivered and discarded.
     */
    static constexpr std::chrono::milliseconds DRAIN_WINDOW{30};

    /* Time to spin and collect commands after draining joint_commands_received_, before asserting
     * on them. Long enough to gather several commands. */
    static constexpr std::chrono::milliseconds OBSERVATION_WINDOW{50};

    /**
     * Waits for the node to publish commands that reflect the current state, then collects them in
     * joint_commands_received_. Call it right after an action (e.g., a state change) whose effect
     * you want to check.
     *
     * Commands published before the action may still be in transit and would otherwise show up in
     * the buffer. This discards them first (DRAIN_WINDOW), then collects fresh ones
     * (OBSERVATION_WINDOW).
     *
     * Contrast with spin_for(), which collects everything, including in-flight commands from before
     * the action.
     */
    void spin_for_fresh_commands() {
        spin_for(DRAIN_WINDOW);
        joint_commands_received_.clear();
        spin_for(OBSERVATION_WINDOW);
    }

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

    ASSERT_TRUE(spin_until([&] {
        publish_camera_command(make_camera_command_msg());
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
    ASSERT_TRUE(spin_until([&] {
        publish_camera_command(make_camera_command_msg());
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

TEST_F(MotionControlStateTest, TerminalStateIsNotExitable) {
    GTEST_SKIP() << "[TODO] not implemented yet";
}