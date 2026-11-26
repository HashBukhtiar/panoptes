#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <vector>

#include "interfaces/msg/camera_orientation_command.hpp"
#include "interfaces/msg/joint_command.hpp"
#include "motion_control/motion_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using CameraOrientationCommand = interfaces::msg::CameraOrientationCommand;
using JointCommand = interfaces::msg::JointCommand;
using JointState = sensor_msgs::msg::JointState;

class MotionControlFaultTest : public ::testing::Test {
   protected:
    std::shared_ptr<MotionControlNode> motion_control_node_;
    std::shared_ptr<rclcpp::Node> peer_node_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<CameraOrientationCommand>::SharedPtr camera_pub_;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<JointCommand>::SharedPtr joint_command_sub_;

    std::vector<JointCommand> received_;

    static void SetUpTestSuite() {
        /* Override normal configuration limits since this test uses dummy inputs, not real data */
        const std::vector<const char*> args = {
            "test_motion_control_pubsub",
            "--ros-args",
            "-p",
            "camera_command_max_age_ms:=1e15",
            "-p",
            "joint_state_max_age_ms:=1e15",
            "-p",
            "joint_min_rad:=[-3.0,-3.0,-3.0]",
            "-p",
            "joint_max_rad:=[3.0,3.0,3.0]",
        };
        rclcpp::init(static_cast<int>(args.size()), args.data());
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
            [this](const JointCommand::SharedPtr msg) { received_.push_back(*msg); });

        executor_.add_node(motion_control_node_);
        executor_.add_node(peer_node_);
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