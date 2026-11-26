#include "kinematics/kinematics.hpp"

namespace stub_kinematics {
bool ik_fails = false;
bool ik_out_of_range = false;
}  // namespace stub_kinematics

namespace kinematics {
std::optional<JointAngles> inverse_kinematics(const CameraOrientation& o) {
    if (stub_kinematics::ik_fails) {
        return std::nullopt;
    }
    if (stub_kinematics::ik_out_of_range) {
        return JointAngles{99.0, 99.0, 99.0};
    }
    return JointAngles{o.yaw + 1.0, o.pitch + 1.0, o.roll + 1.0};
}

std::optional<CameraOrientation> forward_kinematics(const JointAngles& j) {
    if (j.size() != 3) {
        return std::nullopt;
    }
    return CameraOrientation{j[0] - 1.0, j[1] - 1.0, j[2] - 1.0};
}
}  // namespace kinematics