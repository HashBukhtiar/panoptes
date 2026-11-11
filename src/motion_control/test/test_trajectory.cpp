#include <gtest/gtest.h>

#include <vector>

#include "motion_control/trajectory.hpp"

using trajectory::compute_scale;
using trajectory::compute_trajectory;

/**
 * @brief Test suite for the compute_trajectory function.
 */
TEST(ComputeTrajectory, MoveForward) {
    EXPECT_DOUBLE_EQ(compute_trajectory(0.0, 1.0, 0.5), 0.5);
}

TEST(ComputeTrajectory, MoveBackward) {
    EXPECT_DOUBLE_EQ(compute_trajectory(1.0, 0.0, 0.5), 0.5);
}

TEST(ComputeTrajectory, AlreadyAtTarget) {
    EXPECT_DOUBLE_EQ(compute_trajectory(1.0, 1.0, 0.5), 1.0);
}

TEST(ComputeTrajectory, FullMovement) {
    EXPECT_DOUBLE_EQ(compute_trajectory(0.0, 1.0, 1.0), 1.0);
}

/**
 * @brief Test suite for the compute_scale function.
 */
TEST(ComputeScale, MovementWithinLimit) {
    std::vector<double> current = {0.1, 0.2};
    std::vector<double> target = {0.2, 0.3};

    EXPECT_DOUBLE_EQ(compute_scale(current, target, 0.5), 1.0);
}

TEST(ComputeScale, MovementExceedsLimit) {
    std::vector<double> current = {0.0, 0.0};
    std::vector<double> target = {1.0, 0.5};

    EXPECT_DOUBLE_EQ(compute_scale(current, target, 0.5), 0.5);
}

TEST(ComputeScale, DifferentJointErrors) {
    std::vector<double> current = {0.0, 0.0};
    std::vector<double> target = {1.0, 0.25};

    EXPECT_DOUBLE_EQ(compute_scale(current, target, 0.25), 0.25);
}

TEST(ComputeScale, NoMovementRequired) {
    std::vector<double> current = {1.0, 2.0};
    std::vector<double> target = {1.0, 2.0};

    EXPECT_DOUBLE_EQ(compute_scale(current, target, 0.5), 1.0);
}

/**
 * @brief Test suite for combined trajectory functions.
 */
TEST(TrajectoryIntegration, SynchronizedJointMovement) {
    std::vector<double> current = {0.0, 0.0};
    std::vector<double> target = {1.0, 0.5};

    const double max_step = 0.25;

    const std::vector<std::vector<double>> expected = {{0.25, 0.125},
                                                       {0.50, 0.250},
                                                       {0.75, 0.375},
                                                       {1.00, 0.500}};

    for (const auto& expected_positions : expected) {
        double scale = compute_scale(current, target, max_step);

        for (size_t i = 0; i < current.size(); ++i) {
            current[i] = compute_trajectory(current[i], target[i], scale);
        }

        for (size_t i = 0; i < current.size(); ++i) {
            EXPECT_NEAR(current[i], expected_positions[i], 1e-9);
        }
    }
}
