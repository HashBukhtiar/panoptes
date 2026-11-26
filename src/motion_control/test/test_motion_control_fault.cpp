#include <gtest/gtest.h>

#include <chrono>
#include <functional>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;
using State = MotionControlNode::State;

/* Defined in test/stubs/stub_kinematics.cpp */
namespace stub_kinematics {
extern bool ik_fails;
extern bool ik_out_of_range;
}  // namespace stub_kinematics

class MotionControlFaultTest : public MotionControlTest {
   protected:
    static void SetUpTestSuite() {
        auto params = default_param_args();
        params.push_back("camera_command_max_age_ms:=5000.0");
        params.push_back("joint_state_max_age_ms:=5000.0");
        params.push_back("joint_state_max_gap_ms:=100.0");
        params.push_back("joint_state_fault_streak_max_ms:=500.0");
        params.push_back("joint_state_good_streak_min_ms:=500.0");
        params.push_back("hold_max_duration_ms:=2000.0");
        init_ros(params);
    }

    void TearDown() override {
        stub_kinematics::ik_fails = false;
        stub_kinematics::ik_out_of_range = false;
    }

    void publish_camera_command() {
        publish_camera_command(make_camera_command_msg()); /* default (valid) */
    }

    void publish_camera_command(const CameraOrientationCommand& msg) {
        camera_pub_->publish(msg);
    }

    void publish_joint_state() {
        publish_joint_state(make_joint_state_msg()); /* default (valid) */
    }

    void publish_joint_state(const JointState& msg) {
        joint_state_pub_->publish(msg);
    }

    /**
     * Spins the executor for the duration like spin_for(), but also runs `each_tick` on every spin
     * step,to keep something  happening while time passes (e.g., publishing a message every step).
     * Use spin_for() when nothing needs to happen during the wait.
     *
     * @param duration How long to spin.
     * @param each_tick Callable run on every spin step.
     */
    void spin_for_while(std::chrono::milliseconds duration,
                        const std::function<void()>& each_tick) {
        spin_until(
            [&] {
                each_tick();
                return false;
            },
            duration);
    }

    /**
     * Spins the executor for the window, running `each_tick` on every spin step, and expects `text`
     * to be logged during it. Use spin_for_and_expect_alert() instead when nothing needs to happen
     * while waiting.
     *
     * Logging is captured only for the duration of the call, so earlier logs (e.g., from seeding)
     * don't count. The node only processes messages while the executor spins.
     *
     * @param duration How long to spin and watch for the alert.
     * @param each_tick Callable run on every spin step, including the first.
     * @param text Substring expected in the log. Defaults to "[WARN]".
     */
    void spin_for_while_and_expect_alert(std::chrono::milliseconds duration,
                                         const std::function<void()>& each_tick,
                                         const std::string& text = "[WARN]") {
        testing::internal::CaptureStderr();
        spin_for_while(duration, each_tick);
        const std::string log = testing::internal::GetCapturedStderr();

        EXPECT_NE(log.find(text), std::string::npos) << "log was:\n" << log;
    }

    /**
     * Spins the executor for the window and expects `text` to be logged during it.
     * Publish first, then call this. Use spin_for_while_and_expect_alert() instead when something
     * must keep happening while waiting (e.g., publishing every step).
     *
     * Logging is captured only for the duration of the call, so earlier logs (e.g., from seeding)
     * don't count. The node only processes messages while the executor spins.
     *
     * @param duration How long to spin and watch for the alert.
     * @param text Substring expected in the log. Defaults to "[WARN]".
     */
    void spin_for_and_expect_alert(std::chrono::milliseconds duration = 300ms,
                                   const std::string& text = "[WARN]") {
        spin_for_while_and_expect_alert(duration, [] {}, text);
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
            publish_joint_state(msg);
            return !joint_commands_received_.empty();
        }));
        joint_commands_received_.clear();
        return msg.position;
    }

    /**
     * Checks that every joint command received so far still holds the expected positions.
     * Fails if nothing has been received or if any command differs in size or value.
     *
     * @param expected Positions every received joint command must match.
     */
    void expect_joint_commands_unchanged(const std::vector<double>& expected) {
        ASSERT_FALSE(joint_commands_received_.empty());

        for (const auto& cmd : joint_commands_received_) {
            ASSERT_EQ(cmd.position.size(), expected.size());
            for (size_t i = 0; i < expected.size(); ++i) {
                EXPECT_DOUBLE_EQ(cmd.position[i], expected[i]);
            }
        }
    }

    /**
     * Publishes once, watches for a fixed window, and expects no joint command to arrive.
     *
     * @param publish Callable that publishes the messages under test.
     * @param window How long to watch for output after publishing.
     */
    void expect_no_output_after(const std::function<void()>& publish,
                                std::chrono::milliseconds window = 300ms) {
        publish();
        spin_for(window);
        EXPECT_TRUE(joint_commands_received_.empty());
    }
};

/* Controls */
TEST_F(MotionControlFaultTest, ValidCameraCommandMovesJointCommand) {
    const auto seeded = seed_with_joint_state();

    publish_camera_command(make_camera_command_msg());
    spin_for(300ms);

    ASSERT_FALSE(joint_commands_received_.empty());
    EXPECT_NE(joint_commands_received_.back().position, seeded);
}

/* Single CameraOrientationCommand faults */
TEST_F(MotionControlFaultTest, MistypedCameraCommandIsDropped) {
    GTEST_SKIP() << "[N/A] type mismatch is rejected by ROS2 pub/sub, never reaches the node";
}

TEST_F(MotionControlFaultTest, InvalidCameraCommandIsDroppedAndAlerts) {
    const auto seeded = seed_with_joint_state();

    auto msg = make_camera_command_msg();
    msg.yaw = 99.0;
    publish_camera_command(msg);

    spin_for_and_expect_alert(300ms);
    expect_joint_commands_unchanged(seeded);
}

TEST_F(MotionControlFaultTest, TooOldCameraCommandIsDroppedAndAlerts) {
    const auto seeded = seed_with_joint_state();

    auto msg = make_camera_command_msg();
    msg.header.stamp = peer_node_->now() - rclcpp::Duration::from_seconds(10.0);
    publish_camera_command(msg);

    spin_for_and_expect_alert(300ms);
    expect_joint_commands_unchanged(seeded);
}

TEST_F(MotionControlFaultTest, OutOfOrderCameraCommandIsDropped) {
    seed_with_joint_state();

    const auto t0 = peer_node_->now();

    auto younger = make_camera_command_msg();
    younger.header.stamp = t0;

    auto older = make_camera_command_msg();
    older.header.stamp = t0 - rclcpp::Duration::from_seconds(1.0);

    auto has_stamp = [](const JointCommand& cmd, const builtin_interfaces::msg::Time& stamp) {
        return cmd.header.stamp.sec == stamp.sec && cmd.header.stamp.nanosec == stamp.nanosec;
    };

    /* younger command is accepted, so output carries its stamp */
    ASSERT_TRUE(spin_until([&] {
        publish_camera_command(younger);
        return !joint_commands_received_.empty() &&
               has_stamp(joint_commands_received_.back(), younger.header.stamp);
    }));

    /* older command is ignored, so output keeps the younger stamp */
    joint_commands_received_.clear();
    publish_camera_command(older);
    spin_for(300ms);

    ASSERT_FALSE(joint_commands_received_.empty());

    for (const auto& cmd : joint_commands_received_) {
        EXPECT_TRUE(has_stamp(cmd, younger.header.stamp));
    }
}

/* Single kinematics library faults */
TEST_F(MotionControlFaultTest, ValidCameraCommandButFailedKinematicsIsDroppedAndAlerts) {
    const auto seeded = seed_with_joint_state();

    stub_kinematics::ik_fails = true;
    publish_camera_command(make_camera_command_msg());

    spin_for_and_expect_alert(300ms);
    expect_joint_commands_unchanged(seeded);
}

TEST_F(MotionControlFaultTest, ValidCameraCommandButInvalidKinematicsIsDroppedAndAlerts) {
    const auto seeded = seed_with_joint_state();

    stub_kinematics::ik_out_of_range = true;
    publish_camera_command(make_camera_command_msg());

    spin_for_and_expect_alert(300ms);
    expect_joint_commands_unchanged(seeded);
}

/* Single JointState faults */
TEST_F(MotionControlFaultTest, NullJointStateIsDropped) {
    expect_no_output_after([&] { publish_joint_state(JointState{}); });
}

TEST_F(MotionControlFaultTest, MistypedJointStateIsDropped) {
    GTEST_SKIP() << "[N/A] type mismatch is rejected by ROS2 pub/sub, never reaches the node";
}

TEST_F(MotionControlFaultTest, InvalidJointStateIsDropped) {
    auto msg = make_joint_state_msg();
    msg.position = {5.0, 5.0, 5.0};

    expect_no_output_after([&] { publish_joint_state(msg); });
}

TEST_F(MotionControlFaultTest, TooOldJointStateIsDropped) {
    auto msg = make_joint_state_msg();
    msg.header.stamp = peer_node_->now() - rclcpp::Duration::from_seconds(10.0);

    expect_no_output_after([&] { publish_joint_state(msg); });
}

TEST_F(MotionControlFaultTest, OutOfOrderJointStateIsDropped) {
    GTEST_SKIP() << "[TODO] not observable via pub/sub (seeding only happens on recalibration); "
                    "maybe unit test the stamp check instead";
}

/* Continuous (timed) JointState faults */
TEST_F(MotionControlFaultTest, ProlongedPausedJointStateEntersHold) {
    seed_with_joint_state();

    /* Bad streak timer running, not in hold yet */
    spin_for(300ms);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::NORMAL);

    /* Hold entered at approximately 100ms + 500ms */
    spin_for_and_expect_alert(700ms);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);
}

TEST_F(MotionControlFaultTest, BadJointStateStreakEntersHold) {
    seed_with_joint_state();

    auto publish_bad_msg = [this] {
        auto msg = make_joint_state_msg();
        msg.position = {5.0, 5.0, 5.0};
        publish_joint_state(msg);
    };

    /* Bad streak timer running, not in hold yet */
    spin_for_while(300ms, publish_bad_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::NORMAL);

    /* Hold entered at approximately 500ms */
    spin_for_while_and_expect_alert(600ms, publish_bad_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);
}

TEST_F(MotionControlFaultTest, GoodJointStateStreakExitsHold) {
    seed_with_joint_state();

    auto publish_bad_msg = [this] {
        auto msg = make_joint_state_msg();
        msg.position = {5.0, 5.0, 5.0};
        publish_joint_state(msg);
    };

    auto publish_good_msg = [this] { publish_joint_state(make_joint_state_msg()); };

    /* Bad streak timer running, not in hold yet */
    spin_for_while(300ms, publish_bad_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::NORMAL);

    /* Hold entered at approximately 500ms */
    spin_for_while_and_expect_alert(600ms, publish_bad_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);

    /* Good streak timer running, not exited hold yet */
    spin_for_while(300ms, publish_good_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);

    /* Hold exited after approximately additional 500ms */
    spin_for_while(600ms, publish_good_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::NORMAL);
}

TEST_F(MotionControlFaultTest, ExtendedHoldTerminatesAndAlerts) {
    seed_with_joint_state();

    /* Hold entered at approximately 100ms + 500ms */
    spin_for(700ms);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);

    /* Terminal after approximately 2000ms */
    spin_for_and_expect_alert(2100ms, "[URGENT]");
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::TERMINAL);
}