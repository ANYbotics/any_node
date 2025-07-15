/*!
 * @file	Topic.hpp
 * @author	Philipp Leemann
 * @date	Jun 30, 2016
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#ifndef ROS2_BUILD
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#else /* ROS2_BUILD */
#include <acl_config/acl_config.hpp>
#include <rclcpp/rclcpp.hpp>
#endif /* ROS2_BUILD */

#include "any_node/Param.hpp"
#include "any_node/ThreadedPublisher.hpp"
#include "any_node/ThrottledSubscriber.hpp"

namespace any_node {

template <typename msg>
#ifndef ROS2_BUILD
ros::Publisher advertise(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic, uint32_t queue_size,
                         bool latch = false) {
  return nh.advertise<msg>(param<std::string>(nh, "publishers/" + name + "/topic", defaultTopic),
                           param<int>(nh, "publishers/" + name + "/queue_size", queue_size),
                           param<bool>(nh, "publishers/" + name + "/latch", latch));
#else  /* ROS2_BUILD */
[[deprecated("Use the acl_node_helpers instead.")]] typename rclcpp::Publisher<msg>::SharedPtr advertise(
    rclcpp::Node& nh, const std::string& name, [[deprecated]] const std::string& /* defaultTopic */,
    [[deprecated]] uint32_t /* queue_size */, bool latch = false) {

  auto parameterInterface{nh.get_node_parameters_interface()};

  auto topic{acl::config::getParameter<std::string>(*parameterInterface, "publishers." + name + ".topic")};
  auto queueSize{acl::config::getParameter<int>(*parameterInterface, "publishers." + name + ".queue_size")};

  // nh.create_publisher(topic, q_size) ctor. is equivalent to KeepLast(q_size).
  auto qos_profile{rclcpp::QoS(rclcpp::KeepLast(queueSize))};
  if (latch) {
    // Latching behaviour needs TransientLocal. Unclear if Reliable also needs to be set
    // https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
    qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
  }

  return nh.create_publisher<msg>(topic, qos_profile);
#endif /* ROS2_BUILD */
}

template <typename msg>
#ifndef ROS2_BUILD
ThreadedPublisherPtr<msg> threadedAdvertise(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic,
#else  /* ROS2_BUILD */
[[deprecated("Use the acl_node_helpers instead.")]] ThreadedPublisherPtr<msg> threadedAdvertise(
    rclcpp::Node& nh, const std::string& name, const std::string& defaultTopic,
#endif /* ROS2_BUILD */
                                            uint32_t queue_size, bool latch = false, unsigned int maxMessageBufferSize = 10) {
  return ThreadedPublisherPtr<msg>(
      new ThreadedPublisher<msg>(advertise<msg>(nh, name, defaultTopic, queue_size, latch), maxMessageBufferSize));
}

template <class M, class T>
#ifndef ROS2_BUILD
ros::Subscriber subscribe(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic, uint32_t queue_size,
                          void (T::*fp)(const boost::shared_ptr<M const>&), T* obj,
                          const ros::TransportHints& transport_hints = ros::TransportHints()) {
#else  /* ROS2_BUILD */
[[deprecated("Use the acl_node_helpers instead.")]] typename rclcpp::Subscription<M>::SharedPtr subscribe(
    rclcpp::Node& nh, const std::string& name, [[deprecated]] const std::string& /* defaultTopic */,
    [[deprecated]] uint32_t /* queue_size */, void (T::*fp)(const std::shared_ptr<M const>&), T* obj,
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
#endif /* ROS2_BUILD */
#ifndef ROS2_BUILD
  if (nh.param<bool>("subscribers/" + name + "/deactivate", false)) {
    return ros::Subscriber();  // return empty subscriber
  } else {
    return nh.subscribe(param<std::string>(nh, "subscribers/" + name + "/topic", defaultTopic),
                        param<int>(nh, "subscribers/" + name + "/queue_size", queue_size), fp, obj, transport_hints);
  }
#else  /* ROS2_BUILD */
  auto parameterInterface{nh.get_node_parameters_interface()};

  auto topic = acl::config::getParameter<std::string>(*parameterInterface, "subscribers." + name + ".topic");
  auto queueSize = acl::config::getParameter<int>(*parameterInterface, "subscribers." + name + ".queue_size");
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  return nh.create_subscription(topic, queueSize, std::bind(fp, obj, std::placeholders::_1), options);
#endif /* ROS2_BUILD */
}

template <class M, class T>
#ifndef ROS2_BUILD
ThrottledSubscriberPtr<M, T> throttledSubscribe(double timeStep, ros::NodeHandle& nh, const std::string& name,
                                                const std::string& defaultTopic, uint32_t queue_size,
                                                void (T::*fp)(const boost::shared_ptr<M const>&), T* obj,
                                                const ros::TransportHints& transport_hints = ros::TransportHints()) {
#else  /* ROS2_BUILD */
[[deprecated("Use the acl_node_helpers instead.")]] ThrottledSubscriberPtr<M, T> throttledSubscribe(
    double timeStep, rclcpp::Node& nh, const std::string& name, [[deprecated]] const std::string& /* defaultTopic */,
    [[deprecated]] uint32_t /* queue_size */, void (T::*fp)(const std::shared_ptr<M const>&), T* obj) {
#endif /* ROS2_BUILD */
#ifndef ROS2_BUILD
  if (nh.param<bool>("subscribers/" + name + "/deactivate", false)) {
#else                                                      /* ROS2_BUILD */
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr paramInterface = nh.get_node_parameters_interface();
  if (paramInterface == nullptr) {
    return std::make_shared<ThrottledSubscriber<M, T>>();
  }

  if (acl::config::getParameter<bool>(*paramInterface, "subscribers/" + name + "/deactivate")) {
#endif                                                     /* ROS2_BUILD */
    return std::make_shared<ThrottledSubscriber<M, T>>();  // return empty subscriber
  } else {
#ifndef ROS2_BUILD
    return std::make_shared<ThrottledSubscriber<M, T>>(timeStep, nh, param<std::string>(nh, "subscribers/" + name + "/topic", defaultTopic),
                                                       param<int>(nh, "subscribers/" + name + "/queue_size", queue_size), fp, obj,
                                                       transport_hints);
#else  /* ROS2_BUILD */
    return std::make_shared<ThrottledSubscriber<M, T>>(
        timeStep, nh, acl::config::getParameter<std::string>(*paramInterface, "subscribers." + name + ".topic"),
        acl::config::getParameter<int>(*paramInterface, "subscribers." + name + ".queue_size"), fp, obj);
#endif /* ROS2_BUILD */
  }
}

#ifndef ROS2_BUILD
template <class T, class MReq, class MRes>
ros::ServiceServer advertiseService(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService,
                                    bool (T::*srv_func)(MReq&, MRes&), T* obj) {
  return nh.advertiseService(param<std::string>(nh, "servers/" + name + "/service", defaultService), srv_func, obj);
}
#else  /* ROS2_BUILD */
template <class T, class Service>
[[deprecated("Use the acl_node_helpers instead.")]] auto advertiseService(
    rclcpp::Node& nh, const std::string& name, const std::string& /* defaultService */,
    void (T::*srv_func)(const std::shared_ptr<typename Service::Request>, std::shared_ptr<typename Service::Response>), T* obj,
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  auto service = acl::config::getParameter<std::string>(*nh.get_node_parameters_interface(), "servers." + name + ".service");
  using namespace std::placeholders;
  return nh.create_service<Service>(service, std::bind(srv_func, obj, _1, _2), rclcpp::ServicesQoS(), group);
}
#endif /* ROS2_BUILD */

// NOTE(rverschueren): I'm not sure this signature is possible anymore in ROS2. You need the service
// type.
#ifndef ROS2_BUILD
template <class MReq, class MRes>
ros::ServiceClient serviceClient(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService,
                                 const ros::M_string& header_values = ros::M_string()) {
  return nh.serviceClient<MReq, MRes>(param<std::string>(nh, "clients/" + name + "/service", defaultService),
                                      param<bool>(nh, "clients/" + name + "/persistent", false), header_values);
}
#endif /* ROS2_BUILD */

template <class Service>
#ifndef ROS2_BUILD
ros::ServiceClient serviceClient(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService,
                                 const ros::M_string& header_values = ros::M_string()) {
#else  /* ROS2_BUILD */
[[deprecated("Use the acl_node_helpers instead.")]] auto serviceClient(rclcpp::Node& nh, const std::string& name,
                                                                       const std::string& /* defaultService */,
                                                                       rclcpp::CallbackGroup::SharedPtr group = nullptr) {
#endif /* ROS2_BUILD */
#ifndef ROS2_BUILD
  return nh.serviceClient<Service>(param<std::string>(nh, "clients/" + name + "/service", defaultService),
                                   param<bool>(nh, "clients/" + name + "/persistent", false), header_values);
#else  /* ROS2_BUILD */
  auto service = acl::config::getParameter<std::string>(*nh.get_node_parameters_interface(), "clients." + name + ".service");
  return nh.create_client<Service>(service, rclcpp::ServicesQoS(), group);
#endif /* ROS2_BUILD */
}

}  // namespace any_node
