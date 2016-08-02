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
#include <time.h>

#include "any_worker/Worker.hpp"
#include "message_logger/message_logger.hpp"

namespace any_worker {

Worker::Worker(const std::string& name, const double timestep, const WorkerCallback& callback):
    Worker(WorkerOptions(name, timestep, callback))
{

}

Worker::Worker(const WorkerOptions& options):
    options_(options),
    running_(false),
    thread_()
{

}


Worker::Worker(Worker&& other):
    options_(std::move(other.options_)),
    running_(std::move(other.running_.load())),
    thread_(std::move(other.thread_))
{

}


Worker::~Worker() {
    stop(true);
}


bool Worker::start(const int priority) {
    if(options_.timeStep_ < 0.0) {
        MELO_ERROR("Worker [%s] cannot be started, invalid timestep: %f", options_.name_.c_str(), options_.timeStep_);
        return false;
    }else if(options_.timeStep_ == 0.0) {
        running_ = false; // thread loop will exit after first execution
    }else{
        running_ = true;
    }

    thread_ = std::thread(&Worker::run, this);

    sched_param sched;
    sched.sched_priority = 0;
    if(priority != 0) {
        sched.sched_priority = priority;
    }else if(options_.defaultPriority_ != 0) {
        sched.sched_priority = options_.defaultPriority_;
    }

    if(sched.sched_priority != 0) {
        if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
            MELO_WARN("Failed to set thread priority for worker [%s]: %s", options_.name_.c_str(), strerror(errno));
        }
    }

    MELO_INFO("Worker [%s] started", options_.name_.c_str());
    return true;
}

void Worker::stop(const bool wait) {
    running_ = false;
    if(wait && thread_.joinable()) {
        thread_.join();
    }
}

void Worker::run() {
    struct timespec   ts;
    struct timespec   tp;
    long int elapsedTimeNs;
    const long int timeStepNs = static_cast<long int>(options_.timeStep_*1e9);
    WorkerEvent event(options_.timeStep_);

    clock_gettime(CLOCK_MONOTONIC, &ts);

    do {
        if(!options_.callback_(event)) {
            MELO_WARN("Worker [%s] callback returned false.", options_.name_.c_str());
        }

        if(options_.timeStep_ != 0.0) {
            ts.tv_nsec += timeStepNs;
            ts.tv_sec  += ts.tv_nsec/1000000000;
            ts.tv_nsec  = ts.tv_nsec%1000000000;

            // check for too slow processing (or slow system clock)
            clock_gettime(CLOCK_MONOTONIC, &tp);
            elapsedTimeNs = (tp.tv_sec-ts.tv_sec)*1000000000 + (tp.tv_nsec-ts.tv_nsec);
            if(elapsedTimeNs > timeStepNs*10) {
                MELO_ERROR("Worker [%s]: Computation took more than 10 times the maximum allowed computation time (%lf s)!", options_.name_.c_str(), options_.timeStep_);
            }else if (elapsedTimeNs > 0) {
                MELO_WARN_THROTTLE(1.0, "Worker [%s]: Too slow processing! Took %lf s, should have finished in %lf s", options_.name_.c_str(), static_cast<double>(elapsedTimeNs+timeStepNs)/1000000000., options_.timeStep_);
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        }

    }while(running_);

    MELO_INFO("Worker [%s] terminated.", options_.name_.c_str());
}


} // namespace any_worker
