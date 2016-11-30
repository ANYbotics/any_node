/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Philipp Leemann
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file    ThreadedPublisher.hpp
 * @author  Remo Diethelm
 * @date    Nov 30, 2016
 */

#pragma once


// c++
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

// ros
#include <ros/ros.h>

// message logger
#include "message_logger/message_logger.hpp"


namespace any_node {


template <typename MessageType>
class ThreadedPublisher
{
protected:
    std::mutex publisherMutex_;
    ros::Publisher publisher_;

    std::mutex messageBufferMutex_;
    std::queue<MessageType> messageBuffer_;
    unsigned int messageBufferMaxSize_ = 0;

    std::thread thread_;
    std::mutex notifyThreadMutex_;
    std::condition_variable notifyThreadCv_;
    std::atomic<bool> threadIsReady_;
    std::atomic<bool> notifiedThread_;
    std::atomic<bool> shutdownRequested_;

public:
    ThreadedPublisher(const ros::Publisher& publisher, unsigned int messageBufferMaxSize = 10)
    :   publisher_(publisher),
        messageBufferMaxSize_(messageBufferMaxSize),
        threadIsReady_(false),
        notifiedThread_(false),
        shutdownRequested_(false)
    {
        thread_ = std::thread(&ThreadedPublisher::threadedPublish, this);
    }

    virtual ~ThreadedPublisher()
    {
        shutdown();
    }

    void publish(const boost::shared_ptr<MessageType>& message)
    {
        addMessageToBuffer(*message);
    }

    void publish(const MessageType& message)
    {
        addMessageToBuffer(message);
    }

    void shutdown()
    {
        shutdownRequested_ = true;
        notifyThread();
        thread_.join();
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        publisher_.shutdown();
    }

    std::string getTopic() const
    {
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        return publisher_.getTopic();
    }

    uint32_t getNumSubscribers() const
    {
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        return publisher_.getNumSubscribers();
    }

    bool isLatched() const
    {
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
        return publisher_.isLatched();
    }

protected:
    void addMessageToBuffer(const MessageType& message)
    {
        {
            std::lock_guard<std::mutex> messageBufferLock(messageBufferMutex_);
            if (messageBuffer_.size() == messageBufferMaxSize_)
            {
                MELO_ERROR("Threaded publisher: Message buffer reached max size, discarding oldest message without publishing.");
                messageBuffer_.pop();
            }
            messageBuffer_.push(message);
        }
        notifyThread();
    }

    void notifyThread()
    {
        std::unique_lock<std::mutex> lock(notifyThreadMutex_);
        notifiedThread_ = true;
        notifyThreadCv_.notify_all();
    }

    void threadedPublish()
    {
        while (true)
        {
            // Wait for the notification.
            {
                std::unique_lock<std::mutex> notifyThreadLock(notifyThreadMutex_);
                while (!notifiedThread_) // Additional bool protecting against spurious wake ups.
                    notifyThreadCv_.wait(notifyThreadLock);
                notifiedThread_ = false;
            }

            // Publish all messages in the buffer.
            while (true)
            {
                // Stop the thread in case of a shutdown.
                if (shutdownRequested_)
                    break;

                // Execute the publishing with a copied message object.
                MessageType message;
                {
                    std::lock_guard<std::mutex> messageBufferLock(messageBufferMutex_);
                    if (messageBuffer_.empty())
                        break;
                    message = messageBuffer_.front();
                }
                {
                    std::lock_guard<std::mutex> publisherLock(publisherMutex_);
                    publisher_.publish(message);
                }
            }

            // Stop the thread in case of a shutdown.
            if (shutdownRequested_)
                break;
        }
    }
};


} // any_node
