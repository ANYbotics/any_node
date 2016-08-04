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

#include <atomic>
#include <string>

#include "any_worker/WorkerEvent.hpp"

namespace any_worker {

using WorkerCallback = std::function<bool(const WorkerEvent&)>;

struct WorkerOptions {
    WorkerOptions():
        name_(),
        timeStep_(0.0),
        callback_(),
        defaultPriority_(0),
        destructWhenDone_(false)
    {
    }

    WorkerOptions(const std::string& name, const double timestep, const WorkerCallback& callback, const int priority=0, const bool destructWhenDone=false):
        name_(name),
        timeStep_(timestep),
        callback_(callback),
        defaultPriority_(priority),
        destructWhenDone_(destructWhenDone)
    {

    }

    WorkerOptions(const WorkerOptions& other):
        name_(other.name_),
        timeStep_(other.timeStep_.load()),
        callback_(other.callback_),
        defaultPriority_(other.defaultPriority_),
        destructWhenDone_(other.destructWhenDone_)
    {

    }

    WorkerOptions(WorkerOptions&& other):
        name_(std::move(other.name_)),
        timeStep_(std::move(other.timeStep_.load())),
        callback_(std::move(other.callback_)),
        defaultPriority_(other.defaultPriority_),
        destructWhenDone_(other.destructWhenDone_)
    {

    }


    std::string name_;
    std::atomic<double> timeStep_;
    WorkerCallback callback_;

    /*!
     * priority of the underlying thread, integer between 0 and 99 with 0 beeing the lowest priority.
     */
    int defaultPriority_;

    /*!
     * if set to true and timestep=0 (run callback only once), the worker will be destructed by the WorkerManager
     */
    bool destructWhenDone_;
};

} // namespace any_worker
