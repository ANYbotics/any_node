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
 * @file	Node.cpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#include <csignal>

#include "any_node/Node.hpp"
#include "message_logger/message_logger.hpp"

namespace any_node {

Node::Node(NodeHandlePtr nh):
    nh_(nh),
    workerManager_()
{

}

Node::~Node() {

}

void Node::shutdown() {
    raise(SIGINT);
}

bool setProcessPriority(int priority) {
    sched_param params;
    params.sched_priority = priority;
    if(sched_setscheduler(getpid(), SCHED_FIFO, &params) != 0) {
        MELO_WARN("Failed to set process priority to %d: %s. Check /etc/security/limits.conf for the rights.", priority, std::strerror(errno));
        return false;
    }
    return true;
}


} // namespace any_node
