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
#include <ros/ros.h>

#include "any_node/SignalHandler.hpp"

namespace any_node {

    template <class NodeImpl>
    class Nodewrap {
    public:
        Nodewrap():
            running_(false),
            cvRunning_(),
            mutexRunning_()
        {

        }

        virtual ~Nodewrap() {

        }

        /*
         * blocking call, executes init, run and cleanup
         * @param nodeName              Name of the nodeName
         * @param numSpinners           Number of AsyncSpinners to create. Setting this to the number of subscribed topics+services is generally a good idea
         * @param installSignalHandler  Enable installing signal handlers (SIGINT, ...). IF SET TO FALSE, THE USER HAS TO INSTALL HIS OWN SIGNAL HANDLER. Otherwise the program wont be closed in a clean way.
         */
        void start(int argc, char **argv, const std::string nodeName, const unsigned int numSpinners=1, const bool installSignalHandler=true) {
            ros::init(argc, argv, nodeName, installSignalHandler ? ros::init_options::NoSigintHandler : 0);
            std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");
            ros::AsyncSpinner spinner(numSpinners); // use 1 thread
            spinner.start();

            if(installSignalHandler) {
                SignalHandler::bindAll(&Nodewrap::signalHandler, this);
            }

            const bool isStandalone = nh->param<bool>("standalone", false);
            const double timeStep = nh->param<double>("time_step", 0.01);

            running_ = true;
            NodeImpl impl(nh, isStandalone, timeStep);
            impl.init();

            // returns if running_ is false
            std::unique_lock<std::mutex> lk(mutexRunning_);
            cvRunning_.wait(lk, [this]{ return !running_; });

            impl.stopAllWorkers();
            impl.cleanup();

            if(installSignalHandler) {
                SignalHandler::unbindAll(&Nodewrap::signalHandler, this);
            }

            ros::shutdown();
        }

        void stop() {
            std::lock_guard<std::mutex> lk(mutexRunning_);
            running_ = false;
            cvRunning_.notify_all();
        }

        void signalHandler(const int signum) {
            stop();
        }

    private:
        std::atomic<bool> running_;
        std::condition_variable cvRunning_;
        std::mutex mutexRunning_;
    };
} // namespace any_node
