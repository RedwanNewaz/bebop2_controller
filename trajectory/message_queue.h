//
// Created by redwan on 9/30/23.
//

#ifndef BEBOP2_CONTROLLER_MESSAGE_QUEUE_H
#define BEBOP2_CONTROLLER_MESSAGE_QUEUE_H

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>

class MessageQueue {
public:
    MessageQueue()
    {
        isTerminated_ = isPaused_ = isQuit_ = false;
    }
    void push(const std::vector<double>& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        messageQueue_.push(message);
        cv_.notify_one();
    }

    void setTerminate(bool status)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        isTerminated_ = status;
    }

    void setPause(bool status)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        isPaused_ = status;
    }

    void setQuit(bool status)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        isQuit_ = status;
    }

    bool isTerminated() const
    {
        return isTerminated_;
    }

    bool isPaused() const
    {
        return isPaused_;
    }

    bool isQuitting() const
    {
        return isQuit_;
    }

    bool pop(std::vector<double>& message) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !messageQueue_.empty(); });

        if (!messageQueue_.empty()) {
            message = messageQueue_.front();
            messageQueue_.pop();
            return true;
        }

        return false;
    }

private:
    std::queue<std::vector<double>> messageQueue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool isTerminated_, isPaused_, isQuit_;
};

#endif //BEBOP2_CONTROLLER_MESSAGE_QUEUE_H
