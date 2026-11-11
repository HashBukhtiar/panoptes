#pragma once

#include <optional>
#include <vector>

namespace kinematics {
struct CameraOrientation {
    double yaw;    // rad
    double pitch;  // rad
    double roll;   // rad
};

using JointAngles = std::vector<double>;

std::optional<JointAngles> inverse_kinematics(const CameraOrientation& orientation);
std::optional<CameraOrientation> forward_kinematics(const JointAngles& joint_angles);
}  // namespace kinematics
