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
 * @file	Topic.hpp
 * @author	Philipp Leemann
 * @date	Jun 30, 2016
 */

#pragma once

#include <string>
#include <cstdint>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <ros/service_client.h>

#include "any_node/Param.hpp"
#include "any_node/ThreadedPublisher.hpp"

namespace any_node {

template<typename msg>
ros::Publisher advertise(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false)
{
	return nh.advertise<msg>(
			param<std::string>(nh, "publishers/"+name+"/topic", defaultTopic),
			param<int>(nh, "publishers/"+name+"/queue_size", queue_size),
			param<bool>(nh, "publishers/"+name+"/latch", latch));
}

template<typename msg>
ThreadedPublisherPtr<msg> threadedAdvertise(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic, uint32_t queue_size, bool latch = false, unsigned int maxMessageBufferSize = 10)
{
    return ThreadedPublisherPtr<msg>(new ThreadedPublisher<msg>(advertise<msg>(nh, name, defaultTopic, queue_size, latch), maxMessageBufferSize));
}

template<class M, class T>
ros::Subscriber subscribe(ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic,
		uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, const ros::TransportHints& transport_hints = ros::TransportHints())
{
    if(nh.param<bool>("subscribers/"+name+"/deactivate", false)) {
        return ros::Subscriber(); // return empty subscriber
    }else{
        return nh.subscribe(
			param<std::string>(nh, "subscribers/"+name+"/topic", defaultTopic),
			param<int>(nh, "subscribers/"+name+"/queue_size", queue_size),
			fp, obj,
			transport_hints);
    }
}


template<class T, class MReq, class MRes>
ros::ServiceServer advertiseService(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService, bool(T::*srv_func)(MReq &, MRes &), T *obj)
{
	return nh.advertiseService(
			param<std::string>(nh, "servers/"+name+"/service", defaultService),
			srv_func, obj);

}

template<class MReq, class MRes>
ros::ServiceClient serviceClient(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string())
{
    return nh.serviceClient<MReq, MRes>(
            param<std::string>(nh, "clients/"+name+"/service", defaultService),
            param<bool>(nh, "clients/"+name+"/persistent", false), header_values);
}

template<class Service>
ros::ServiceClient serviceClient(ros::NodeHandle& nh, const std::string& name, const std::string& defaultService, const ros::M_string& header_values = ros::M_string())
{
    return nh.serviceClient<Service>(
            param<std::string>(nh, "clients/"+name+"/service", defaultService),
			param<bool>(nh, "clients/"+name+"/persistent", false), header_values);
}

} // namespace any_node
