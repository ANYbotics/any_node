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

#include "any_node/SignalHandler.hpp"
#include "any_node/Param.hpp"
#include "any_worker/WorkerOptions.hpp"
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

      spinner_ = new ros::AsyncSpinner(numSpinners);
      impl_ = new NodeImpl(nh_);

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

  virtual ~Nodewrap() {
      // not necessary to call ros::shutdown, this is done as soon as the last nodeHandle
      // is destructed
      if(impl_) {
          delete impl_;
      }

      if(spinner_) {
          delete spinner_;
      }
  }

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
      spinner_->start();
      impl_->init();
      running_ = true;

      if(installSignalHandler) {
          SignalHandler::bindAll(&Nodewrap::signalHandler, this);
          signalHandlerInstalled_ = true;
      }else{
          signal(SIGINT, basicSigintHandler);
      }
  }

  /*!
   * blocking call, returns when the program should shut down
   * @param priority    priority of the worker calling the update function. Only used if isStandalone=true
   */
  virtual void run(const int priority=0) {
      if(isStandalone_) {
          impl_->addWorker("updateWorker", timeStep_, static_cast<bool(NodeImpl::*)(const any_worker::WorkerEvent&)>(&NodeImpl::update), impl_, priority);
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
          SignalHandler::unbindAll(&Nodewrap::signalHandler, this);
      }

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
    return impl_;
  }
protected:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::AsyncSpinner* spinner_;
  NodeImpl* impl_;

  bool signalHandlerInstalled_;

  bool isStandalone_; // if true, executes update() every timeStep_ seconds
  double timeStep_; // [s], only used if isStandalone_ is true

  std::atomic<bool> running_;
  std::condition_variable cvRunning_;
  std::mutex mutexRunning_;
};

} // namespace any_node
