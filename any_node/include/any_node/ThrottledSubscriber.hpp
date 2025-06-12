/*!
 * @file    ThrottledSubscriber.hpp
 * @author  Gabriel Hottiger
 * @date    Jan 19, 2017
 */

#pragma once

// STL
#include <chrono>
#include <functional>

// ros
#ifndef ROS2_BUILD
#include <ros/ros.h>
#else /* ROS2_BUILD */
#include "rclcpp/rclcpp.hpp"
#endif /* ROS2_BUILD */

#ifndef ROS2_BUILD
#include <message_logger/message_logger.hpp>
#endif

namespace any_node {

template <typename MessageType, typename CallbackClass>
class ThrottledSubscriber {
 public:
#ifndef ROS2_BUILD
  ThrottledSubscriber() : subscriber_(), fp_(nullptr), obj_(nullptr), lastTime_(), timeStep_() {}
#else /* ROS2_BUILD */
  ThrottledSubscriber() : subscriber_(), fp_(nullptr), obj_(nullptr), lastTime_(), timeStep_(rclcpp::Duration::from_seconds(0)) {}
#endif

#ifndef ROS2_BUILD
  ThrottledSubscriber(const double timeStep, ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                      void (CallbackClass::*fp)(const boost::shared_ptr<MessageType const>&), CallbackClass* obj,
                      const ros::TransportHints& transport_hints = ros::TransportHints())
#else  /* ROS2_BUILD */
  ThrottledSubscriber(const double timeStep, rclcpp::Node& nh, const std::string& topic, uint32_t queue_size,
                      void (CallbackClass::*fp)(const std::shared_ptr<MessageType const>&), CallbackClass* obj)
#endif /* ROS2_BUILD */
#ifndef ROS2_BUILD
      : fp_(fp), obj_(obj), lastTime_(ros::TIME_MIN), timeStep_(ros::Duration().fromSec(timeStep)) {
#else  /* ROS2_BUILD */
      : fp_(fp), obj_(obj), lastTime_(), timeStep_(rclcpp::Duration::from_seconds(timeStep)) {
#endif /* ROS2_BUILD */
    subscriber_ =
#ifndef ROS2_BUILD
        nh.subscribe(topic, queue_size, &ThrottledSubscriber<MessageType, CallbackClass>::internalCallback, this, transport_hints);
#else  /* ROS2_BUILD */
        nh.create_subscription(topic, queue_size, &ThrottledSubscriber<MessageType, CallbackClass>::internalCallback, this);
#endif /* ROS2_BUILD */
  }

  virtual ~ThrottledSubscriber() { shutdown(); }

  void shutdown() { subscriber_.shutdown(); }

#ifndef ROS2_BUILD
  void internalCallback(const boost::shared_ptr<MessageType const>& msg) {
    ros::Time now = ros::Time::now();
#else  /* ROS2_BUILD */
  void internalCallback(const std::shared_ptr<MessageType const>& msg) {
    rclcpp::Time now = rclcpp::Clock{RCL_ROS_TIME}.now();
#endif /* ROS2_BUILD */
    if ((now - lastTime_) >= timeStep_) {
      (*obj_.*fp_)(msg);
      lastTime_ = now;
    }
  }

 protected:
#ifndef ROS2_BUILD
  ros::Subscriber subscriber_;
  void (CallbackClass::*fp_)(const boost::shared_ptr<MessageType const>&);
#else  /* ROS2_BUILD */
  typename rclcpp::Subscription<MessageType>::SharedPtr subscriber_;
  void (CallbackClass::*fp_)(const std::shared_ptr<MessageType const>&);
#endif /* ROS2_BUILD */
  CallbackClass* obj_;
#ifndef ROS2_BUILD
  ros::Time lastTime_;
  ros::Duration timeStep_;
#else  /* ROS2_BUILD */
  rclcpp::Time lastTime_;
  rclcpp::Duration timeStep_;
#endif /* ROS2_BUILD */
};

template <typename MessageType, typename CallbackClass>
using ThrottledSubscriberPtr = std::shared_ptr<ThrottledSubscriber<MessageType, CallbackClass>>;

}  // namespace any_node
