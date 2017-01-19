/*!
 * @file    ThrottledSubscriber.hpp
 * @author  Gabriel Hottiger
 * @date    Jan 19, 2017
 */

#pragma once

// STL
#include <functional>
#include <chrono>


// ros
#include <ros/ros.h>

// message logger
#include "message_logger/message_logger.hpp"

namespace any_node {

template <typename MessageType, typename CallbackClass>
class ThrottledSubscriber
{
protected:
    ros::Subscriber subscriber_;

public:
    template <typename T>
    ThrottledSubscriber(const double timeStep, ros::NodeHandle& nh, const std::string& name, const std::string& defaultTopic,
                        uint32_t queue_size, void(T::*fp)(const boost::shared_ptr<MessageType const>&), T* obj,
                        const ros::TransportHints& transport_hints = ros::TransportHints())
    :   subscriber_(nh.subscribe())
    {

    }

    virtual ~ThrottledSubscriber()
    {
        shutdown();
    }

    void shutdown() {
      subscriber_.shutdown();
    }


protected:

};

template <typename MessageType>
using ThreadedPublisherPtr = std::shared_ptr<ThreadedPublisher<MessageType>>;


} // any_node
