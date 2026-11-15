#include "kinematics/kinematics.hpp"

namespace kinematics {
std::optional<JointAngles> inverse_kinematics(const CameraOrientation&) {
    return JointAngles{1.0, 1.0, 1.0};
}
std::optional<CameraOrientation> forward_kinematics(const JointAngles&) {
    return std::nullopt;
}
}  // namespace kinematics