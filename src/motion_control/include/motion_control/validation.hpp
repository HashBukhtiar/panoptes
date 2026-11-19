#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace validation {
/**
 * Checks that every value is finite (not NaN or inf).
 *
 * @param vals  Values to check.
 * @return      True if all values are finite.
 */
inline bool all_finite(const std::vector<double>& vals) {
    for (double x : vals) {
        if (!std::isfinite(x)) {
            return false;
        }
    }
    return true;
}

/**
 * Checks that values match the limits' size and are finite and within [lo, hi].
 *
 * @param vals  Values to check.
 * @param min   Per-element lower limits.
 * @param max   Per-element upper limits.
 * @return      True if sizes match and every value is finite and within its limits.
 */
inline bool valid_numbers(const std::vector<double>& vals, const std::vector<double>& min,
                          const std::vector<double>& max) {
    if (vals.size() != min.size() || vals.size() != max.size()) {
        return false;
    }
    for (size_t i = 0; i < vals.size(); ++i) {
        if (!std::isfinite(vals[i]) || vals[i] < min[i] || vals[i] > max[i]) {
            return false;
        }
    }
    return true;
}

/**
 * Checks a camera orientation command against the camera FOV limits.
 *
 * @param yaw       Yaw (rad).
 * @param pitch     Pitch (rad).
 * @param roll      Roll (rad).
 * @param fov_min   Lower limits, ordered {yaw, pitch, roll}.
 * @param fov_max   Upper limits, ordered {yaw, pitch, roll}.
 * @return          True if all angles are finite and within the FOV limits.
 */
inline bool valid_camera_orientation(double yaw, double pitch, double roll,
                                     const std::vector<double>& fov_min,
                                     const std::vector<double>& fov_max) {
    return valid_numbers({yaw, pitch, roll}, fov_min, fov_max);
}

/**
 * Checks joint positions against joint limits.
 *
 * @param positions     Joint positions (rad).
 * @param joint_min     Per-joint lower limits (rad).
 * @param joint_max     Per-joint upper limits (rad).
 * @return              True if sizes match and all positions are finite and within limits.
 */
inline bool valid_joint_positions(const std::vector<double>& positions,
                                  const std::vector<double>& joint_min,
                                  const std::vector<double>& joint_max) {
    return valid_numbers(positions, joint_min, joint_max);
}
}  // namespace validation