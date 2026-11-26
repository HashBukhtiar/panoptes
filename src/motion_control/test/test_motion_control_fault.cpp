#include <gtest/gtest.h>

#include <chrono>
#include <functional>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;

class MotionControlFaultTest : public MotionControlTest {
   protected:
    static void SetUpTestSuite() {
        auto params = default_param_args();
        params.push_back("camera_command_max_age_ms:=500.0");
        params.push_back("joint_state_max_age_ms:=500.0");
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

    bool no_output_after(const std::function<void()>& publish,
                         std::chrono::milliseconds window = 300ms) {
        publish();
        spin_for(window);
        return joint_commands_received_.empty();
    }
};

/* Single CameraOrientationCommand faults */
TEST_F(MotionControlFaultTest, MistypedCameraCommandIsDropped) {
    GTEST_SKIP() << "not implemented";
}

TEST_F(MotionControlFaultTest, InvalidCameraCommandIsDroppedAndAlerts) {
    // auto msg = make_camera_command_msg();
    // msg.yaw = 99.0;

    // EXPECT_TRUE(no_output_after([&] {
    //     publish_camera_command(msg);
    //     publish_joint_state();
    // }));

    GTEST_SKIP() << "not implemented";
}

TEST_F(MotionControlFaultTest, TooOldCameraCommandIsDroppedAndAlerts) {
    GTEST_SKIP() << "not implemented";
}

TEST_F(MotionControlFaultTest, OutOfOrderCameraCommandIsDropped) {
    GTEST_SKIP() << "not implemented";
}

/* Single JointState faults */
TEST_F(MotionControlFaultTest, NullJointStateIsDropped) {
    EXPECT_TRUE(no_output_after([&] { publish_joint_state(JointState{}); }));
}

TEST_F(MotionControlFaultTest, MistypedJointStateIsDropped) {
    GTEST_SKIP() << "not implemented";
}

TEST_F(MotionControlFaultTest, InvalidJointStateIsDropped) {
    auto msg = make_joint_state_msg();
    msg.position = {5.0, 5.0, 5.0};

    EXPECT_TRUE(no_output_after([&] { publish_joint_state(msg); }));
}

TEST_F(MotionControlFaultTest, TooOldJointStateIsDropped) {
    auto msg = make_joint_state_msg();
    msg.header.stamp = peer_node_->now() - rclcpp::Duration::from_seconds(5.0);

    EXPECT_TRUE(no_output_after([&] { publish_joint_state(msg); }));
}

TEST_F(MotionControlFaultTest, OutOfOrderJointStateIsDropped) {
    GTEST_SKIP() << "not implemented";
}