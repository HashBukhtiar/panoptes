#include "kinematics/kinematics.hpp"

#include <iostream>

namespace kinematics {
std::optional<JointAngles> inverse_kinematics(const CameraOrientation& orientation) {
    std::clog << "[KIN] received camera orientation command: yaw=" << orientation.yaw
              << ", pitch=" << orientation.pitch << ", roll=" << orientation.roll << std::endl;
    // TODO: implement actual logic for inverse kinematics
    return std::nullopt;
};

std::optional<CameraOrientation> forward_kinematics(const JointAngles& joint_angles) {
    std::clog << "[KIN] received joint angles command: ";
    for (const auto& angle : joint_angles) {
        std::clog << angle << " ";
    }
    std::clog << std::endl;
    // TODO: implement actual logic for forward kinematics
    return std::nullopt;
};
}  // namespace kinematics