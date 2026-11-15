#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace trajectory {
/**
 * Calculates a common scaling factor to limit the maximum joint movement.
 *
 * @param current    Current commanded joint positions.
 * @param target     Target joint positions.
 * @param max_step   Maximum movement allowed per joint this tick (rad).
 * @return           Scale in (0, 1].
 */
template <typename Current, typename Target>
double compute_scale(const Current& current, const Target& target, double max_step) {
    double max_err = 0.0;

    for (size_t i = 0; i < target.size(); ++i) {
        max_err = std::max(max_err, std::abs(target[i] - current[i]));
    }

    return (max_err > max_step) ? max_step / max_err : 1.0;
}

/**
 * Moves one joint toward its target by a fraction of its remaining error.
 *
 * @param curr      Current commanded joint position.
 * @param target    Target joint position.
 * @param scale     Fraction of the remaining error to cover this tick.
 * @return          Next commanded joint position.
 */
inline double compute_trajectory(double curr, double target, double scale) {
    return curr + (target - curr) * scale;
}

/**
 * Moves `positions` toward `target`, limited to max_speed_rad_s over dt seconds.
 *
 * @param positions         Current commanded joint positions, updated in place.
 * @param target            Target joint positions.
 * @param max_speed_rad_s   Maximum speed per joint (rad/s).
 * @param dt                Time since the last step (s).
 */
template <typename Positions, typename Target>
void step_rate_limited(Positions& positions, const Target& target, double max_speed_rad_s,
                       double dt) {
    const double scale = compute_scale(positions, target, max_speed_rad_s * dt);

    for (size_t i = 0; i < target.size(); ++i) {
        positions[i] = compute_trajectory(positions[i], target[i], scale);
    }
}

}  // namespace trajectory
