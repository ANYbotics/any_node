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
 * @file	Node.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include <memory> // for std::shared_ptr
#include <sched.h>
#include <unistd.h> // for getpid()
#include <ros/ros.h>


#include "any_worker/WorkerManager.hpp"
#include "any_worker/WorkerOptions.hpp"
#include "any_node/Param.hpp"
#include "any_node/Topic.hpp"

namespace any_node {

bool setProcessPriority(int priority);

class Node {
 public:
    using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;

    Node() = delete;
    Node(NodeHandlePtr nh);
    virtual ~Node();

    /*
     * abstract interface functions
     */
    virtual void init() = 0;
    virtual void cleanup() = 0;
    virtual bool update(const any_worker::WorkerEvent& event) = 0;


    /*
     * general
     */
    void shutdown();

    template<class T>
    inline bool addWorker(const std::string& name, const double timestep, bool(T::*fp)(const any_worker::WorkerEvent&), T* obj, const int priority=0) {
        return workerManager_.addWorker(name, timestep, fp, obj, priority);
    }

    inline bool addWorker(const any_worker::WorkerOptions& options) {
        return workerManager_.addWorker(options);
    }

    inline void stopAllWorkers() {
        workerManager_.clearWorkers();
    }

    /*
     * accessors
     */
    inline ros::NodeHandle& getNodeHandle() const { return *nh_; }


    /*
     * forwarding to Topic.hpp functions
     */
    template<typename msg>
    inline ros::Publisher advertise(const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false) {
        return any_node::advertise<msg>(*nh_, name, defaultTopic, queue_size, latch);
    }

    template<typename msg>
    inline ThreadedPublisherPtr<msg> threadedAdvertise(const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false, unsigned int maxMessageBufferSize = 10) {
        return any_node::threadedAdvertise<msg>(*nh_, name, defaultTopic, queue_size, latch, maxMessageBufferSize);
    }

    template<class M, class T>
    inline ros::Subscriber subscribe(const std::string& name, const std::string& defaultTopic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
        return any_node::subscribe(*nh_, name, defaultTopic, queue_size, fp, obj, transport_hints);
    }

    template<class T, class MReq, class MRes>
    inline ros::ServiceServer advertiseService(const std::string& name, const std::string& defaultService, bool(T::*srv_func)(MReq &, MRes &), T *obj) {
        return any_node::advertiseService(*nh_, name, defaultService, srv_func, obj);
    }

    template<class MReq, class MRes>
    inline ros::ServiceClient serviceClient(const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string()) {
        return any_node::serviceClient<MReq, MRes>(*nh_, name, defaultService, header_values);
    }

    template<class Service>
    inline ros::ServiceClient serviceClient(const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string()) {
        return any_node::serviceClient<Service>(*nh_, name, defaultService, header_values);
    }

    /*
     * forwarding to Param.hpp functions
     */
     template<typename ParamT>
     inline bool getParam(const std::string& key, ParamT& param_val) {
         return any_node::getParam(*nh_, key, param_val);
     }

     template<typename ParamT>
     inline ParamT param(const std::string& key, const ParamT& defaultValue) {
         return any_node::param(*nh_, key, defaultValue);
     }

     template<typename ParamT>
     inline void setParam(const std::string& key, const ParamT& param) {
         any_node::setParam(*nh_, key, param);
     }

 private:
    NodeHandlePtr nh_;

 private:
    any_worker::WorkerManager workerManager_;

};

} // namespace any_node
