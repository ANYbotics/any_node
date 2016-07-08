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

#include <pthread.h>
#include <chrono>

#include "any_worker/Worker.hpp"
#include "message_logger/message_logger.hpp"

namespace any_worker {

Worker::Worker(const std::string& name, const double timestep, const WorkerCallback& callback):
    name_(name),
    timeStep_(timestep),
    callback_(callback),
    running_(false),
    thread_()
{

}


Worker::Worker(Worker&& other):
    name_(std::move(other.name_)),
    timeStep_(std::move(other.timeStep_)),
    callback_(std::move(other.callback_)),
    running_(std::move(other.running_.load())),
    thread_(std::move(other.thread_))
{

}


Worker::~Worker() {
    stop(true);
}


bool Worker::start(const int priority) {
    if(timeStep_ < 0.0) {
        MELO_ERROR("Worker [%s] cannot be started, invalid timestep: %f", name_.c_str(), timeStep_);
        return false;
    }else if(timeStep_ == 0.0) {
        running_ = false; // thread loop will exit after first execution
    }else{
        running_ = true;
    }

    thread_ = std::thread(&Worker::run, this);

    if(priority != 0) {
        sched_param sched;
        sched.sched_priority = priority;
        if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
            MELO_WARN("Failed to set thread priority for worker [%s]: %s", name_.c_str(), strerror(errno));
        }
    }

    MELO_INFO("Worker [%s] started", name_.c_str());
    return true;
}

void Worker::stop(const bool wait) {
    running_ = false;
    if(wait && thread_.joinable()) {
        thread_.join();
    }
}

void Worker::run() {
    auto nextLoop = std::chrono::steady_clock::now();
    WorkerEvent event;

    do {
        if(!callback_(event)) {
            MELO_WARN("Worker [%s] callback returned false.", name_.c_str());
        }

        nextLoop += std::chrono::nanoseconds(static_cast<long int>(timeStep_*1e9));
        std::this_thread::sleep_until(nextLoop);
    }while(running_);

    MELO_INFO("Worker [%s] terminated.", name_.c_str());
}


} // namespace any_worker
