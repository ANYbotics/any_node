/*!
 * @file	Nodewrap.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>

#include <message_logger/message_logger.hpp>
#include "any_node/Param.hpp"
#include "any_worker/WorkerOptions.hpp"
#include "signal_handler/SignalHandler.hpp"

namespace any_node {

template <class NodeImpl>
class Nodewrap {
 public:
  Nodewrap() = delete;
  /*!
   * @param argc
   * @param argv
   * @param nodeName                name of the node
   * @param numSpinners             number of async ros spinners. Set to -1 to get value from ros params. A value of 0 means to use the
   * number of processor cores.
   * @param installSignalHandler    set to False to use the ros internal signal handler instead
   */
  Nodewrap(int argc, char** argv, const std::string& nodeName, int numSpinners = -1, const bool installSignalHandler = true)
      : signalHandlerInstalled_(installSignalHandler) {
    if (signalHandlerInstalled_) {
      ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);
    } else {
      ros::init(argc, argv, nodeName);
    }

    // Explicitly call ros::start here, to avoid the ROS node handles to call this implicitly (as well as calling ros::shutdown implicitly).
    // See https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/node_handle.cpp#L178
    // and https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/node_handle.cpp#L194
    ros::start();

    nh_ = std::make_shared<ros::NodeHandle>("~");

    if (numSpinners == -1) {
      numSpinners = param<unsigned int>(*nh_, "num_spinners", 2);
    }

    spinner_.reset(new ros::AsyncSpinner(numSpinners));
    impl_.reset(new NodeImpl(nh_));

    checkSteadyClock();
  }

  virtual ~Nodewrap() {
    // Call ros::shutdown here explicitly, as we also called ros::start explicitly in the constructor.
    ros::shutdown();
  };

  /*!
   * blocking call, executes init, run (if init() was successful) and cleanup (independent of the success of init()).
   */
  bool execute() {
    const bool initSuccess{init()};
    if (initSuccess) {
      run();
    }
    cleanup();
    return initSuccess;
  }

  /*!
   * Initializes the node
   */
  bool init() {
    if (signalHandlerInstalled_) {
      signal_handler::SignalHandler::bindAll(&Nodewrap::signalHandler, this);
    }

    spinner_->start();
    if (!impl_->init()) {
      MELO_ERROR("Failed to init Node %s!", ros::this_node::getName().c_str());
      return false;
    }

    running_ = true;
    return true;
  }

  /*!
   * blocking call, returns when the program should shut down
   */
  void run() {
    // returns if running_ is false
    std::unique_lock<std::mutex> lk(mutexRunning_);
    cvRunning_.wait(lk, [this] { return !running_; });
  }

  /*!
   * Stops the workers, ros spinners and calls cleanup of the underlying instance of any_node::Node
   */
  void cleanup() {
    if (signalHandlerInstalled_) {
      signal_handler::SignalHandler::unbindAll(&Nodewrap::signalHandler, this);
    }

    impl_->preCleanup();
    impl_->stopAllWorkers();
    spinner_->stop();
    impl_->cleanup();
  }

  /*!
   * Stops execution of the run(..) function.
   */
  void stop() {
    std::lock_guard<std::mutex> lk(mutexRunning_);
    running_ = false;
    cvRunning_.notify_all();
  }

 public:  /// INTERNAL FUNCTIONS
  void signalHandler(const int signum) {
    MELO_DEBUG_STREAM("Signal: " << signum)
    stop();

    if (signum == SIGSEGV) {
      signal(signum, SIG_DFL);
      kill(getpid(), signum);
    }
  }

  static void checkSteadyClock() {
    if (std::chrono::steady_clock::period::num != 1 || std::chrono::steady_clock::period::den != 1000000000) {
      MELO_ERROR("std::chrono::steady_clock does not have a nanosecond resolution!")
    }
    if (std::chrono::system_clock::period::num != 1 || std::chrono::system_clock::period::den != 1000000000) {
      MELO_ERROR("std::chrono::system_clock does not have a nanosecond resolution!")
    }
  }

  NodeImpl* getImplPtr() { return impl_.get(); }

 protected:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::unique_ptr<NodeImpl> impl_;

  bool signalHandlerInstalled_{false};

  std::atomic<bool> running_{false};
  std::condition_variable cvRunning_;
  std::mutex mutexRunning_;
};

}  // namespace any_node
