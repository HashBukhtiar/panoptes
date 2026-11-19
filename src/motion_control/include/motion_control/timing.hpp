#pragma once

#include <chrono>
#include <optional>

namespace timing {
/**
 * Computes message age (now - original timestamp).
 *
 * @tparam Time     Time type.
 * @param now       Current time.
 * @param stamp     Original timestamp of the message.
 * @return          Message age in ms.
 */
template <typename Time>
double get_age_ms(const Time& now, const Time& stamp) {
    return (now - stamp).seconds() * 1000.0;
}

/**
 * Checks whether a message is stale (we are receiving old data).
 *
 * @param message_age_ms    Message age (ms).
 * @param tolerance_ms      Maximum allowed age (ms).
 * @return                  True if the age exceeds the tolerance.
 */
inline bool is_too_old(double message_age_ms, double tolerance_ms) {
    return message_age_ms > tolerance_ms;
}

/**
 * Checks whether a message is older than the newest accepted one (out-of-order data).
 *
 * @tparam Time         Time type.
 * @param stamp         Timestamp of the received message.
 * @param last_accepted Timestamp of the newest accepted message, if any.
 * @return              True if a message was accepted before and stamp is older than it.
 */
template <typename Time>
bool is_out_of_order(const Time& stamp, const std::optional<Time>& last_accepted) {
    return last_accepted.has_value() && stamp < *last_accepted;
}

/**
 * Computes elapsed time between two steady-clock points.
 *
 * @param from      Earlier time point.
 * @param to        Later time point.
 * @return          Elapsed time in ms.
 */
inline double get_ms_between(std::chrono::steady_clock::time_point from,
                             std::chrono::steady_clock::time_point to) {
    return std::chrono::duration<double, std::milli>(to - from).count();
}
}  // namespace timing