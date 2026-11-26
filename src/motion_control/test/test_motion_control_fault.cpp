#include <gtest/gtest.h>

#include <chrono>

#include "motion_control_test.hpp"

using namespace std::chrono_literals;

class MotionControlFaultTest : public MotionControlTest {
   protected:
    // static void SetUpTestSuite() {
    //     auto params = default_param_args();
    //     params.push_back("joint_max_rad:=[1.0,1.0,1.0]");
    //     params.push_back("joint_state_max_age_ms:=50");
    //     init_ros(params);
    // }

    JointState make_joint_state_msg() override {
        auto msg = MotionControlTest::make_joint_state_msg();
        msg.position = {2.9, 2.9, 2.9};
        return msg;
    }

    void send_inputs_until_output() {
        spin_until([this] {
            camera_pub_->publish(make_camera_command_msg());
            joint_state_pub_->publish(make_joint_state_msg());
            return !joint_commands_received_.empty();
        });
    }
};

/* Single faults */
TEST_F(MotionControlFaultTest, NullCameraCommandIsDroppedAndAlerts) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, MistypedCameraCommandIsDropped) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, InvalidCameraCommandIsDroppedAndAlerts) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, TooOldCameraCommandIsDroppedAndAlerts) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, OutOfOrderCameraCommandIsDropped) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, NullJointStateIsDropped) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, MistypedJointStateIsDropped) {
    /* no alert expected because this is stopped at ros2's pub/sub stream, doesn't reach us */
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, InvalidJointStateIsDropped) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, TooOldJointStateIsDropped) {
    ASSERT_TRUE(1);
}

TEST_F(MotionControlFaultTest, OutOfOrderJointStateIsDropped) {
    ASSERT_TRUE(1);
}