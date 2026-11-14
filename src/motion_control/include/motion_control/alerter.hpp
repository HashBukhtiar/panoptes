#pragma once

#include <chrono>
#include <condition_variable>
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
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_back(msg);
        }
        cv_.notify_one();
    }

   private:
    void run() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
            if (queue_.empty()) {
                return; /* only reachable when stopping */
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
        last_logged_[msg] = now;
        RCLCPP_WARN(logger_, "%s", msg.c_str());
    }

    rclcpp::Logger logger_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;
    bool stop_ = false;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_logged_;
    std::thread worker_; /* starts after everything above is initialized */
};