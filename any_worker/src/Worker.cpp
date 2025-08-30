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
 * @file	Worker.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#include "any_worker/Worker.hpp"

#include <pthread.h>
#include <sys/resource.h>  // setpriority
#include <cstring>         // strerror(..)
#include <ctime>
#include <message_logger/message_logger.hpp>

namespace any_worker {

Worker::Worker(const std::string& name, const double timestep, const WorkerCallback& callback)
    : Worker(WorkerOptions(name, timestep, callback)) {}

Worker::Worker(const std::string& name, const double timestep, const WorkerCallback& callback,
               const WorkerCallbackFailureReaction& callbackFailureReaction)
    : Worker(WorkerOptions(name, timestep, callback, callbackFailureReaction)) {}

Worker::Worker(const WorkerOptions& options)
    : options_(options),
      rate_(options)  // NOLINT(cppcoreguidelines-slicing)
{}

Worker::Worker(Worker&& other)
    : options_(std::move(other.options_)),
      running_(other.running_.load()),
      done_(other.done_.load()),
      thread_(std::move(other.thread_)),
      rate_(std::move(other.rate_)) {}

Worker::~Worker() {
  stop(true);
}

bool Worker::start(const int priority) {
  if (running_) {
    MELO_ERROR("Worker [%s] cannot be started, already/still running.", options_.name_.c_str());
    done_ = true;
    return false;
  }
  if (options_.timeStep_ < 0.0) {
    MELO_ERROR("Worker [%s] cannot be started, invalid timestep: %f", options_.name_.c_str(), options_.timeStep_.load());
    done_ = true;
    return false;
  }

  running_ = true;
  done_ = false;

  thread_ = std::thread(&Worker::run, this);

  sched_param sched{};
  sched.sched_priority = 0;
  if (priority != 0) {
    sched.sched_priority = priority;
  } else if (options_.defaultPriority_ != 0) {
    sched.sched_priority = options_.defaultPriority_;
  }
  // Set real-time scheduling priority and policy
  if (sched.sched_priority != 0) {
    if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
      MELO_WARN("Failed to set thread priority for worker [%s]: %s", options_.name_.c_str(), strerror(errno));
    }
  }

  // Set affinity if there is one
  if (options_.schedAffinity_ != -1) {
    if (options_.schedAffinity_ >= CPU_SETSIZE) {
      MELO_ERROR_STREAM("Selected affinity of " << options_.schedAffinity_ << "is too high. Max allowed is " << CPU_SETSIZE);
    } else {
      // Use a CPU set as stated in the manpages
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(options_.schedAffinity_, &cpuset);
      auto s = pthread_setaffinity_np(thread_.native_handle(), sizeof(cpuset), &cpuset);
      if (s != 0) {
        MELO_ERROR_STREAM("Failed to set thread affinity to " << options_.schedAffinity_);
      }
    }
  }

  MELO_INFO("Worker [%s] started", options_.name_.c_str());
  return true;
}

void Worker::stop(const bool wait) {
  running_ = false;

  // Only wait to stop is not called from within the worker itself (= same thread ID as worker)
  if (thread_.get_id() != std::this_thread::get_id()) {
    if (wait && thread_.joinable()) {
      thread_.join();
    }
  }
}

void Worker::setTimestep(const double timeStep) {
  if (timeStep <= 0.0) {
    MELO_ERROR("Cannot change timestep of Worker [%s] to %f, invalid value.", options_.name_.c_str(), timeStep);
    return;
  }
  options_.timeStep_ = timeStep;
  if (!std::isinf(timeStep)) {
    // We will use the rate, so we set its parameters.
    rate_.getOptions().timeStep_ = timeStep;
  }
}

void Worker::setEnforceRate(const bool enforceRate) {
  options_.enforceRate_ = enforceRate;
  rate_.getOptions().enforceRate_ = enforceRate;
}

void Worker::run() {
  // Adjust thread scheduling priortiy under default scheduling policy
  if (options_.defaultPriority_ == 0 && options_.defaultNiceness_ != WorkerNiceness::DEFAULT) {
    // Note: To adjust the scheduling priority under linux either the unique TID is required or to use 0
    // as indicator for the calling process. There is no direct conversion from std::thread::native_handle()
    // to a TID as the latter is system-specifc identifier. Therefore, to explicitly use the correct kernel thread ID,
    // the thread priority is adjusted within the thread execution context.
    // TODO(lneuner): To be evaluted wether this is required for single callback execution.
    if (setpriority(PRIO_PROCESS, 0 /*calling_tid*/, static_cast<int>(options_.defaultNiceness_)) != 0) {
      MELO_WARN("Failed to set thread niceness for worker [%s]: %s", options_.name_.c_str(), strerror(errno));
    }
  }

  if (std::isinf(options_.timeStep_)) {
    // Run the callback once.
    static timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (!options_.callback_(WorkerEvent(options_.timeStep_, now))) {
      MELO_WARN("Worker [%s] callback returned false. Calling failure reaction.", options_.name_.c_str());
      options_.callbackFailureReaction_();
    }
  } else {
    // Reset the rate step time.
    rate_.reset();

    // Run the callback repeatedly.
    do {
      if (!options_.callback_(WorkerEvent(options_.timeStep_, rate_.getSleepEndTime()))) {
        MELO_WARN("Worker [%s] callback returned false. Calling failure reaction.", options_.name_.c_str());
        options_.callbackFailureReaction_();
      }

      rate_.sleep();

    } while (running_);
  }

  MELO_INFO("Worker [%s] terminated.", options_.name_.c_str());
  done_ = true;
}

}  // namespace any_worker
