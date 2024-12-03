/*!
 * @file	Node.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#ifndef ROS2_BUILD
#include <ros/ros.h>
#else /* ROS2_BUILD */
#include "rclcpp/rclcpp.hpp"
#endif /* ROS2_BUILD */
#include <sched.h>
#include <unistd.h>  // for getpid()
#include <memory>    // for std::shared_ptr

#include <any_worker/WorkerManager.hpp>
#include <any_worker/WorkerOptions.hpp>

#include "any_node/Param.hpp"
#include "any_node/Topic.hpp"

namespace any_node {

bool setProcessPriority(int priority);

class Node {
 public:
#ifndef ROS2_BUILD
  using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;
#else  /* ROS2_BUILD */
  using NodeHandlePtr = std::shared_ptr<rclcpp::Node>;
#endif /* ROS2_BUILD */

  Node() = delete;
  explicit Node(NodeHandlePtr nh);
  virtual ~Node() = default;

  /*
   * (abstract) interface functions
   */

  /*!
   * Init function, used to initialize all members and starting workers (if any).
   * @return      True if successful. Returning false indicates that the node shall shut down.
   */
  virtual bool init() = 0;
  /*!
   * Pre-Cleanup function, which is called by Nodewrap _before_ stopping workers. (Thread safety up to the user!).
   * This function is called even if init() returned false.
   */
  virtual void preCleanup() {}
  /*!
   * Cleanup function, called by Nodewrap _after_ stopping workers.
   * This function is called even if init() returned false.
   */
  virtual void cleanup() = 0;

  /*
   * general
   */

  /*!
   * Method to signal nodewrap to shutdown the node.
   */
  void shutdown();

  /*!
   * Helper functions to add Workers to the WorkerManager
   */
  template <class T>
  inline bool addWorker(const std::string& name, const double timestep, bool (T::*fp)(const any_worker::WorkerEvent&), T* obj,
                        const int priority = 0, const int affinity = -1) {
    return workerManager_.addWorker(name, timestep, fp, obj, priority, true, affinity);
  }

  inline bool addWorker(const any_worker::WorkerOptions& options) { return workerManager_.addWorker(options); }

  /*!
   * Check if WorkerManager is managing a Worker with given name
   * @param name  Name of the worker
   * @return      True if worker was found
   */
  inline bool hasWorker(const std::string& name) { return workerManager_.hasWorker(name); }

  /*!
   * Stop a worker managed by the WorkerManager
   * @param name  Name of the worker
   * @param wait  Whether to wait until the worker has finished or return immediately
   */
  inline void stopWorker(const std::string& name, const bool wait = true) { workerManager_.stopWorker(name, wait); }

  /*!
   * Start a worker managed by the WorkerManager.
   * @param name  Name of the worker
   */
  inline void startWorker(const std::string& name) { workerManager_.startWorker(name); }

  /*!
   * Stop and delete a worker managed by the WorkerManager
   * @param name  Name of the worker
   * @param wait  Whether to wait until the worker has finished or return immediately
   */
  inline void cancelWorker(const std::string& name, const bool wait = true) { workerManager_.cancelWorker(name, wait); }

  /*!
   * Method to stop all workers managed by the WorkerManager
   */
  inline void stopAllWorkers() { stopAllWorkers(true); }
  inline void stopAllWorkers(bool wait) { workerManager_.cancelWorkers(wait); }

  /*
   * accessors
   */
#ifndef ROS2_BUILD
  inline ros::NodeHandle& getNodeHandle() const { return *nh_; }
  // An alias to symplify the ROS2 migration
  inline ros::NodeHandle& getNode() const { return *nh_; }
#else  /* ROS2_BUILD */
  inline rclcpp::Node::SharedPtr getNodeHandle() { return nh_; }
  inline rclcpp::Node& getNode() { return *nh_; }
#endif /* ROS2_BUILD */

  /*
   * forwarding to Topic.hpp functions
   */
  template <typename msg>
#ifndef ROS2_BUILD
  inline ros::Publisher advertise(const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false) {
#else  /* ROS2_BUILD */
  inline typename rclcpp::Publisher<msg>::SharedPtr advertise(const std::string& name, const std::string& defaultTopic, uint32_t queue_size,
                                                              bool latch = false) {
#endif /* ROS2_BUILD */
    return any_node::advertise<msg>(*nh_, name, defaultTopic, queue_size, latch);
  }

  template <typename msg>
  inline ThreadedPublisherPtr<msg> threadedAdvertise(const std::string& name, const std::string& defaultTopic, uint32_t queue_size,
                                                     bool latch = false, unsigned int maxMessageBufferSize = 10) {
    return any_node::threadedAdvertise<msg>(*nh_, name, defaultTopic, queue_size, latch, maxMessageBufferSize);
  }

  template <class M, class T>
#ifndef ROS2_BUILD
  inline ros::Subscriber subscribe(const std::string& name, const std::string& defaultTopic, uint32_t queue_size,
                                   void (T::*fp)(const boost::shared_ptr<M const>&), T* obj,
                                   const ros::TransportHints& transport_hints = ros::TransportHints()) {
#else  /* ROS2_BUILD */
  inline typename rclcpp::Subscription<M>::SharedPtr subscribe(const std::string& name, const std::string& defaultTopic,
                                                               uint32_t queue_size, void (T::*fp)(const std::shared_ptr<M const>&),
                                                               T* obj) {
#endif /* ROS2_BUILD */
#ifndef ROS2_BUILD
    return any_node::subscribe(*nh_, name, defaultTopic, queue_size, fp, obj, transport_hints);
#else  /* ROS2_BUILD */
    return any_node::subscribe(*nh_, name, defaultTopic, queue_size, fp, obj);
#endif /* ROS2_BUILD */
  }

  template <class M, class T>
  inline ThrottledSubscriberPtr<M, T> throttledSubscribe(double timeStep, const std::string& name, const std::string& defaultTopic,
#ifndef ROS2_BUILD
                                                         uint32_t queue_size, void (T::*fp)(const boost::shared_ptr<M const>&), T* obj,
                                                         const ros::TransportHints& transport_hints = ros::TransportHints()) {

#else  /* ROS2_BUILD */
                                                         uint32_t queue_size, void (T::*fp)(const std::shared_ptr<M const>&), T* obj) {
#endif /* ROS2_BUILD */
    return any_node::throttledSubscribe<M, T>(timeStep, *nh_, name, defaultTopic, queue_size, fp, obj);
  }

  template <class T, class MReq, class MRes>
#ifndef ROS2_BUILD
  inline ros::ServiceServer advertiseService(const std::string& name, const std::string& defaultService, bool (T::*srv_func)(MReq&, MRes&),
                                             T* obj) {
#else  /* ROS2_BUILD */
  inline auto advertiseService(const std::string& name, const std::string& defaultService, bool (T::*srv_func)(MReq&, MRes&), T* obj) {
#endif /* ROS2_BUILD */
    return any_node::advertiseService(*nh_, name, defaultService, srv_func, obj);
  }

#ifndef ROS2_BUILD
  template <class MReq, class MRes>
  inline ros::ServiceClient serviceClient(const std::string& name, const std::string& defaultService,
                                          const ros::M_string& header_values = ros::M_string()) {
    return any_node::serviceClient<MReq, MRes>(*nh_, name, defaultService, header_values);
  }
#endif /* ROS2_BUILD */

  template <class Service>
#ifndef ROS2_BUILD
  inline ros::ServiceClient serviceClient(const std::string& name, const std::string& defaultService,
                                          const ros::M_string& header_values = ros::M_string()) {
#else  /* ROS2_BUILD */
  inline rclcpp::Client<Service> serviceClient(const std::string& name, const std::string& defaultService) {
#endif /* ROS2_BUILD */
#ifndef ROS2_BUILD
    return any_node::serviceClient<Service>(*nh_, name, defaultService, header_values);
#else  /* ROS2_BUILD */
    return any_node::serviceClient<Service>(*nh_, name, defaultService);
#endif /* ROS2_BUILD */
  }

#ifndef ROS2_BUILD
  /*
   * forwarding to Param.hpp functions
   */
  template <typename ParamT>
  inline bool getParam(const std::string& key, ParamT& param_val) {
    return any_node::getParam(*nh_, key, param_val);
  }

  template <typename ParamT>
  inline ParamT param(const std::string& key, const ParamT& defaultValue) {
    return any_node::param(*nh_, key, defaultValue);
  }

  template <typename ParamT>
  inline void setParam(const std::string& key, const ParamT& param) {
    any_node::setParam(*nh_, key, param);
  }
#endif /* ROS2_BUILD */

 protected:
  NodeHandlePtr nh_;

 private:
  any_worker::WorkerManager workerManager_;
};

}  // namespace any_node
