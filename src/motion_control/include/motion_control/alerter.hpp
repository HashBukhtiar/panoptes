#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

/**
 * Logs alerts from a background thread.
 * Repeats of the same message are throttled to 1 per second.
 */
class Alerter {
   public:
    explicit Alerter(rclcpp::Logger logger) : logger_(logger), worker_([this] { run(); }) {}

    ~Alerter() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            queue_.clear();
        }
        cv_.notify_one();
        worker_.join();
    }

    Alerter(const Alerter&) = delete;
    Alerter& operator=(const Alerter&) = delete;

    /**
     * Queues an alert for the background thread to log.
     *
     * @param msg   The alert message.
     */
    void raise(const std::string& msg) {
        {
            if (stop_) return;
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() >= kMaxQueuedAlerts_) {
                queue_.pop_front();
            }
            queue_.push_back(msg);
        }
        cv_.notify_one();
    }

   private:
    void run() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
            if (stop_) {
                queue_.clear();
                return;
            }

            const std::string msg = std::move(queue_.front());
            queue_.pop_front();

            lock.unlock();
            log(msg);
            lock.lock();
        }
    }

    void log(const std::string& msg) {
        const auto now = std::chrono::steady_clock::now();
        auto it = last_logged_.find(msg);
        if (it != last_logged_.end() && now - it->second < std::chrono::seconds(1)) {
            return; /* throttled */
        }

        for (it = last_logged_.begin(); it != last_logged_.end();) {
            if (now - it->second >= std::chrono::seconds(1)) {
                it = last_logged_.erase(it);
            } else {
                ++it;
            }
        }

        if (last_logged_.size() >= kMaxTrackedAlerts_) {
            last_logged_.erase(last_logged_.begin());
        }

        last_logged_[msg] = now;
        RCLCPP_WARN(logger_, "%s", msg.c_str());
    }

    static constexpr size_t kMaxQueuedAlerts_ = 64;
    static constexpr size_t kMaxTrackedAlerts_ = 64;

    rclcpp::Logger logger_;

    std::mutex mutex_;
    std::condition_variable cv_;

    std::deque<std::string> queue_;
    bool stop_ = false;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_logged_;

    std::thread worker_; /* starts after everything above is initialized */
};