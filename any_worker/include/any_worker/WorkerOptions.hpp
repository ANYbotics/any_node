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
 * @file	WorkerOptions.hpp
 * @author	Philipp Leemann
 * @date	Jul 8, 2016
 */

#pragma once

#include <functional>
#include <string>

#include "any_worker/RateOptions.hpp"
#include "any_worker/WorkerEvent.hpp"

namespace any_worker {

/*!
 * Nice levels are multiplicative, with a 10% CPU time slice for a nice-level of 1, which is
 * achieved through a multiplier of 1.25. https://www.man7.org/linux/man-pages/man7/sched.7.html
 * The default niceness on Linux is 0.
 */
enum class WorkerNiceness : int { HIGH = -10, DEFAULT = 0 };

using WorkerCallback = std::function<bool(const WorkerEvent&)>;
using WorkerCallbackFailureReaction = std::function<void(void)>;

struct WorkerOptions : public RateOptions {
  WorkerOptions() : callbackFailureReaction_([]() {}) {}

  WorkerOptions(const std::string& name, const double timestep, WorkerCallback callback, const int priority = 0,
                const int schedAffinity = -1)
      : RateOptions(name, timestep),
        callback_(std::move(callback)),
        callbackFailureReaction_([]() {}),
        defaultPriority_(priority),
        defaultNiceness_(WorkerNiceness::DEFAULT),
        destructWhenDone_(false),
        schedAffinity_(schedAffinity) {}

  WorkerOptions(const std::string& name, const double timestep, WorkerCallback callback, const WorkerNiceness niceness)
      : RateOptions(name, timestep),
        callback_(std::move(callback)),
        callbackFailureReaction_([]() {}),
        defaultPriority_(0),
        defaultNiceness_(niceness),
        destructWhenDone_(false),
        schedAffinity_(-1) {}

  WorkerOptions(const std::string& name, const double timestep, WorkerCallback callback,
                WorkerCallbackFailureReaction callbackFailureReaction, const int priority = 0, const int schedAffinity = -1)
      : RateOptions(name, timestep),
        callback_(std::move(callback)),
        callbackFailureReaction_(std::move(callbackFailureReaction)),
        defaultPriority_(priority),
        defaultNiceness_(WorkerNiceness::DEFAULT),
        destructWhenDone_(false),
        schedAffinity_(schedAffinity) {}

  WorkerOptions(const WorkerOptions& other) = default;

  WorkerOptions(WorkerOptions&& other)
      : RateOptions(std::move(other)),
        callback_(std::move(other.callback_)),
        callbackFailureReaction_(std::move(other.callbackFailureReaction_)),
        defaultPriority_(other.defaultPriority_),
        defaultNiceness_(other.defaultNiceness_),
        destructWhenDone_(other.destructWhenDone_),
        schedAffinity_(other.schedAffinity_) {}

  /*!
   * The primary worker callback to be called
   */
  WorkerCallback callback_;

  /*!
   * The reaction callback to be called when the primary indicates error (returns false)
   */
  WorkerCallbackFailureReaction callbackFailureReaction_;

  /*!
   * Static scheduling priority of the underlying thread. Static priority 0 implies default
   * time-share scheduling, whereas 1 (low) to 99 (high) are required for real-time policies.
   */
  int defaultPriority_{0};

  /*!
   * Niceness of the underlying thread for default non-rt scheduling. Time-share scheduling
   * requires a static scheduling priority of 0.
   */
  WorkerNiceness defaultNiceness_{WorkerNiceness::DEFAULT};

  /*!
   * if set to true and timestep=0 (run callback only once), the worker will be destructed by the WorkerManager
   */
  bool destructWhenDone_{false};

  /*!
   * scheduling affinity of the underlying thread, integer between 0 and number of CPUs - 1. A scheduling affinity
   * of "-1" means no affinity is set.
   */
  int schedAffinity_{-1};
};

}  // namespace any_worker
