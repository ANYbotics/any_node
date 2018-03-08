/*!
 * @file	Nodewrap.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include <mutex>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <chrono>
#include <ros/ros.h>

#include "any_node/Param.hpp"
#include "any_worker/WorkerOptions.hpp"
#include "signal_handler/SignalHandler.hpp"
#include "message_logger/message_logger.hpp"

namespace any_node {

static inline void basicSigintHandler(int sig) {
    ros::requestShutdown();
}

template <class NodeImpl>
class Nodewrap {
public:
  Nodewrap() = delete;
  /*!
   * @param argc
   * @param argv
   * @param nodeName    name of the node
   * @param numSpinners number of async ros spinners. Set to 0 to get value from ros params.
   */
  Nodewrap(int argc, char **argv, const std::string& nodeName, int numSpinners = 0):
      nh_(nullptr),
      spinner_(nullptr),
      impl_(nullptr),
      signalHandlerInstalled_(false),
      isStandalone_(false),
      timeStep_(0.01),
      running_(false),
      cvRunning_(),
      mutexRunning_()
  {
      ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);
      nh_ = std::make_shared<ros::NodeHandle>("~");

      isStandalone_ = nh_->param("standalone", false);
      timeStep_ = nh_->param("time_step", 0.01);

      if(numSpinners == 0) {
          numSpinners = nh_->param("num_spinners", 2);
      }

      spinner_.reset(new ros::AsyncSpinner(numSpinners));
      impl_.reset(new NodeImpl(nh_));

      checkSteadyClock();
  }

  /*!
   * @param argc
   * @param argv
   * @param nodeName        name of the node
   * @param isStandalone    Set to true if a worker should be set up, which calls the update function with the given timestep
   * @param timeStep        Timestep with which the update worker is called. Only used if isStandalone=true
   * @param numSpinners     number of async ros spinners. Set to 0 to get value from ros params.
   */
  Nodewrap(int argc, char **argv, const std::string& nodeName, bool isStandalone, double timeStep = 0.01, int numSpinners = 0):
      Nodewrap(argc, argv, nodeName, numSpinners)
  {
      isStandalone_ = isStandalone;
      timeStep_ = timeStep;
  }

  // not necessary to call ros::shutdown in the destructor, this is done as soon as the last nodeHandle
  // is destructed
  virtual ~Nodewrap() = default;

  /*!
   * blocking call, executes init, run and cleanup
   * @param priority                priority of the worker calling the update function. Only used if isStandalone=true
   * @param installSignalHandler    Enable installing signal handlers (SIGINT, ...).
   */
  void execute(const int priority=0, const bool installSignalHandler=true) {
      init(installSignalHandler);
      run(priority);
      cleanup();
  }

  /*!
   * Initializes the node
   * @param installSignalHandler  Enable installing signal handlers (SIGINT, ...).
   */
  void init(const bool installSignalHandler=true) {
      if(installSignalHandler) {
          signal_handler::SignalHandler::bindAll(&Nodewrap::signalHandler, this);
          signalHandlerInstalled_ = true;
      }else{
          signal(SIGINT, basicSigintHandler);
      }

      spinner_->start();
      impl_->init();
      running_ = true;
  }

  /*!
   * blocking call, returns when the program should shut down
   * @param priority    priority of the worker calling the update function. Only used if isStandalone=true
   */
  virtual void run(const int priority=0) {
      if(isStandalone_) {
          impl_->addWorker("updateWorker", timeStep_, static_cast<bool(NodeImpl::*)(const any_worker::WorkerEvent&)>(&NodeImpl::update), impl_.get(), priority);
      }
      // returns if running_ is false
      std::unique_lock<std::mutex> lk(mutexRunning_);
      cvRunning_.wait(lk, [this]{ return !running_; });
  }

  /*!
   * Stops the workers, ros spinners and calls cleanup of the underlying instance of any_node::Node
   */
  void cleanup() {
      if(signalHandlerInstalled_) {
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

public: /// INTERNAL FUNCTIONS
  void signalHandler(const int signum) {
      stop();

      if (signum == SIGSEGV) {
          signal(signum, SIG_DFL);
          kill(getpid(), signum);
      }
  }


  void enforceTimestep(const double timestep) { isStandalone_ = true; timeStep_ = timestep; }

  static void checkSteadyClock() {
      if(std::chrono::steady_clock::period::num != 1 || std::chrono::steady_clock::period::den != 1000000000) {
          MELO_ERROR("std::chrono::steady_clock does not have a nanosecond resolution!")
      }
      if(std::chrono::system_clock::period::num != 1 || std::chrono::system_clock::period::den != 1000000000) {
          MELO_ERROR("std::chrono::system_clock does not have a nanosecond resolution!")
      }
  }

  NodeImpl* getImplPtr() {
    return impl_.get();
  }
protected:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::unique_ptr<NodeImpl> impl_;

  bool signalHandlerInstalled_;

  bool isStandalone_; // if true, executes update() every timeStep_ seconds
  double timeStep_; // [s], only used if isStandalone_ is true

  std::atomic<bool> running_;
  std::condition_variable cvRunning_;
  std::mutex mutexRunning_;
};

} // namespace any_node
