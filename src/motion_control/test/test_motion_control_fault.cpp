#include <gtest/gtest.h>

#include <chrono>
#include <functional>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;
using State = MotionControlNode::State;

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

    /* Spins for the window, running `each_tick` on every step (e.g., to keep publishing). */
    void spin_for_while(std::chrono::milliseconds window, const std::function<void()>& each_tick) {
        spin_until(
            [&] {
                each_tick();
                return false;
            },
            window);
    }

    /**
     * Watches the log for a fixed window, starting from when it's called, and expects a "[WARN]"
     * alert. Note that the node only processes messages while the executor spins, so the alert
     * fires inside this window as long as nothing spins in between!
     *
     * @param window How long to watch for the alert.
     */
    void spin_for_and_expect_alert(std::chrono::milliseconds window = 300ms) {
        testing::internal::CaptureStderr();
        spin_for(window);
        const std::string log = testing::internal::GetCapturedStderr();

        EXPECT_NE(log.find("[WARN]"), std::string::npos);
    }

    /**
     * Runs `body` and expects a "[WARN]" alert to be logged while it runs.
     *
     * Use this when the test must do more than just wait inside the window, e.g., keep publishing
     * on every spin step (`expect_alert_during([&] { spin_for_while(700ms, publish_bad_msg); });`).
     * `body` is responsible for spinning; the node only processes messages while the executor
     * spins, so a `body` that never spins will see no alert.
     *
     * Contrast with spin_for_and_expect_alert(), which is the simple case: it does the spinning
     * itself for a fixed window and publishes nothing. Use that one when you have already published
     * and only need to wait for the alert.
     *
     * @param body Callable run while stderr is captured. Must spin the executor.
     */
    void expect_alert_during(const std::function<void()>& body,
                             const std::string& text = "[WARN]") {
        testing::internal::CaptureStderr();
        body();
        const std::string log = testing::internal::GetCapturedStderr();

        EXPECT_NE(log.find(text), std::string::npos) << "log was:\n" << log;
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

    /* Hold entered at approximately 100ms + 500ms + 0-10ms */
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
    expect_alert_during([&] { spin_for_while(700ms, publish_bad_msg); });
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
    expect_alert_during([&] { spin_for_while(700ms, publish_bad_msg); });
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);

    /* Good streak timer running, not exited hold yet */
    spin_for_while(300ms, publish_good_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);

    /* Hold exited after approximately additional 500ms */
    spin_for_while(500ms, publish_good_msg);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::NORMAL);
}

TEST_F(MotionControlFaultTest, ExtendedHoldTerminatesAndAlerts) {
    seed_with_joint_state();

    /* Hold entered at approximately 600ms */
    spin_for(700ms);
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::HOLDING);

    /* Terminal at approximately 2600ms */
    expect_alert_during([&] { spin_for(2700ms); }, "[URGENT]");
    EXPECT_EQ(motion_control_node_->get_state(), MotionControlNode::State::TERMINAL);
}