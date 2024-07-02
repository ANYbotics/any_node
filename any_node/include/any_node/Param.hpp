/*!
 * @file	Param.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#ifndef ROS2_BUILD

#include "param_io/get_param.hpp"
#include "param_io/set_param.hpp"

namespace any_node {

using namespace param_io;  // NOLINT

}  // namespace any_node

#else

#include <acl_config/acl_config.hpp>

#endif
