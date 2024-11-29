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
#ifndef ROS2_BUILD
#include <ros/ros.h>
#else /* ROS2_BUILD */
#include "rclcpp/rclcpp.hpp"
#endif /* ROS2_BUILD */

#include <message_logger/message_logger.hpp>

namespace any_node {

template <typename MessageType>
class ThreadedPublisher {
 protected:
  mutable std::mutex publisherMutex_;
#ifndef ROS2_BUILD
  ros::Publisher publisher_;
#else  /* ROS2_BUILD */
  typename rclcpp::Publisher<MessageType>::SharedPtr publisher_;
#endif /* ROS2_BUILD */

  std::mutex messageBufferMutex_;
  std::queue<MessageType> messageBuffer_;
  unsigned int maxMessageBufferSize_{0};
  bool autoPublishRos_{true};

  std::thread thread_;
  std::mutex notifyThreadMutex_;
  std::condition_variable notifyThreadCv_;
  std::atomic<bool> notifiedThread_{false};
  std::atomic<bool> shutdownRequested_{false};

 public:
#ifndef ROS2_BUILD
  explicit ThreadedPublisher(const ros::Publisher& publisher, unsigned int maxMessageBufferSize = 10, bool autoPublishRos = true)
#else  /* ROS2_BUILD */
  explicit ThreadedPublisher(const typename rclcpp::Publisher<MessageType>::SharedPtr& publisher, unsigned int maxMessageBufferSize = 10,
                             bool autoPublishRos = true)
#endif /* ROS2_BUILD */
      : publisher_(publisher), maxMessageBufferSize_(maxMessageBufferSize), autoPublishRos_(autoPublishRos) {
    if (autoPublishRos_) {
      thread_ = std::thread(&ThreadedPublisher::threadedPublish, this);
    }
  }

  virtual ~ThreadedPublisher() { shutdown(); }

#ifndef ROS2_BUILD
  void publish(const boost::shared_ptr<MessageType>& message) { addMessageToBuffer(*message); }
#else  /* ROS2_BUILD */
  void publish(const std::shared_ptr<MessageType>& message) { addMessageToBuffer(*message); }
#endif /* ROS2_BUILD */

  void publish(const MessageType& message) { addMessageToBuffer(message); }

  void shutdown() {
    // Prohibit shutting down twice.
    if (shutdownRequested_) {
      return;
    }

    // Shutdown thread if autopublishing
    if (autoPublishRos_) {
      shutdownRequested_ = true;
      notifyThread();
      thread_.join();
    }

    std::lock_guard<std::mutex> publisherLock(publisherMutex_);
#ifndef ROS2_BUILD
    publisher_.shutdown();
#else  /* ROS2_BUILD */
    publisher_.reset();
#endif /* ROS2_BUILD */
  }

  std::string getTopic() const {
    std::lock_guard<std::mutex> publisherLock(publisherMutex_);
    return publisher_.getTopic();
  }

  uint32_t getNumSubscribers() const {
    std::lock_guard<std::mutex> publisherLock(publisherMutex_);
    return publisher_.getNumSubscribers();
  }

  bool isLatched() const {
    std::lock_guard<std::mutex> publisherLock(publisherMutex_);
    return publisher_.isLatched();
  }

  /*!
   * Send all messages in the buffer.
   */
  void sendRos() {
    // Publish all messages in the buffer; stop the thread in case of a shutdown.
    while (!shutdownRequested_) {
      // Execute the publishing with a copied message object.
      MessageType message;
      {
        std::lock_guard<std::mutex> messageBufferLock(messageBufferMutex_);
        if (messageBuffer_.empty()) {
          break;
        }
        message = messageBuffer_.front();
        messageBuffer_.pop();
      }
      {
        std::lock_guard<std::mutex> publisherLock(publisherMutex_);
#ifndef ROS2_BUILD
        publisher_.publish(message);
#else  /* ROS2_BUILD */
        publisher_->publish(message);
#endif /* ROS2_BUILD */
      }
    }
  }

 protected:
  void addMessageToBuffer(const MessageType& message) {
    {
      std::lock_guard<std::mutex> messageBufferLock(messageBufferMutex_);
      if (messageBuffer_.size() == maxMessageBufferSize_) {
#ifndef ROS2_BUILD
        auto topicName = publisher_.getTopic();
#else  /* ROS2_BUILD */
        auto topicName = publisher_->get_topic_name();
#endif /* ROS2_BUILD */
        MELO_ERROR_STREAM(
            "Threaded publisher: Message buffer reached max size, discarding oldest message without publishing. Topic: " << topicName);
        messageBuffer_.pop();
      }
      messageBuffer_.push(message);
    }
    notifyThread();
  }

  void notifyThread() {
    std::unique_lock<std::mutex> lock(notifyThreadMutex_);
    notifiedThread_ = true;
    notifyThreadCv_.notify_all();
  }

  void threadedPublish() {
    // Stop the thread in case of a shutdown.
    while (!shutdownRequested_) {
      // Wait for the notification.
      {
        std::unique_lock<std::mutex> notifyThreadLock(notifyThreadMutex_);
        while (!notifiedThread_) {  // Additional bool protecting against spurious wake ups.
          notifyThreadCv_.wait(notifyThreadLock);
        }
        notifiedThread_ = false;
      }

      // Publish all messages in the buffer; stop the thread in case of a shutdown.
      sendRos();
    }
  }
};

template <typename MessageType>
using ThreadedPublisherPtr = std::shared_ptr<ThreadedPublisher<MessageType>>;

}  // namespace any_node
