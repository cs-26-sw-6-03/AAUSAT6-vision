#pragma once

/* 
 * threadsafequeue.hpp
 *
 * A thread-safe queue implementation using mutexes and condition variables.
 * 
 * TODO: Needs review
 */

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <chrono>

template <typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_size = 32)
        : max_size_(max_size) {}

    // Block until space is available, then push
    void push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_not_full_.wait(lock, [this] {
            return queue_.size() < max_size_ || stopped_;
        });
        if (stopped_) return;
        queue_.push(std::move(item));
        cv_not_empty_.notify_one();
    }

    // Non-blocking push — drops item if full, returns false
    bool try_push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_ || stopped_) return false;
        queue_.push(std::move(item));
        cv_not_empty_.notify_one();
        return true;
    }

    // Block until item is available
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_not_empty_.wait(lock, [this] {
            return !queue_.empty() || stopped_;
        });
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop();
        cv_not_full_.notify_one();
        return item;
    }

    // Block with timeout — returns nullopt if timed out
    std::optional<T> pop_for(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_not_empty_.wait_for(lock, timeout, [this] {
                return !queue_.empty() || stopped_;
            })) {
            return std::nullopt;
        }
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop();
        cv_not_full_.notify_one();
        return item;
    }

    // Non-blocking pop
    std::optional<T> try_pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop();
        cv_not_full_.notify_one();
        return item;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    // Signal all waiting threads to unblock and return nullopt/false
    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        cv_not_empty_.notify_all();
        cv_not_full_.notify_all();
    }

    bool is_stopped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_;
    }

private:
    std::queue<T>           queue_;
    mutable std::mutex      mutex_;
    std::condition_variable cv_not_empty_;
    std::condition_variable cv_not_full_;
    size_t                  max_size_;
    bool                    stopped_ = false;
};