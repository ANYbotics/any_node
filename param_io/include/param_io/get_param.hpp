/*
 * get_param.hpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once


// ros
#include <ros/ros.h>
#include <XmlRpc.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


namespace param_io {


/*
 * Interfaces:
 *
 * 1) bool   getParam(const ros::NodeHandle& nh, const std::string& key, ParamT& param);
 *
 * 2) ParamT getParam(const ros::NodeHandle& nh, const std::string& key);
 *
 *
 *
 * Examples:
 *
 * 1a) double myParam = 0;
 *     bool success = getParam(nh, "my_param", myParam);
 *
 * 1b) double myParam1 = 0;
 *     double myParam2 = 0;
 *     bool success = true;
 *     success = success && getParam(nh, "my_param1", myParam1);
 *     success = success && getParam(nh, "my_param2", myParam2);
 *
 * 2)  double myParam = getParam<double>(nh, "my_param");
 */


// 1)
template<typename ParamT>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, ParamT& param)
{
  if (!nh.getParam(key, param))
  {
    ROS_WARN_STREAM("Could not acquire parameter " << key << " from server.");
    return false;
  }
  return true;
}

// 2)
template<typename ParamT>
inline ParamT getParam(const ros::NodeHandle& nh, const std::string& key)
{
  ParamT param;
  getParam(nh, key, param);
  return param;
}


template<typename ParamT>
inline bool param(const ros::NodeHandle& nh, const std::string& key, ParamT& param_val, const ParamT& defaultValue=ParamT())
{
  if (!nh.getParam(key, param_val))
  {
    ROS_WARN_STREAM("Could not acquire parameter " << key << " from server. Using default value: " << defaultValue);
    param_val = defaultValue;
    return false;
  }
  return true;
}

template<typename ParamT>
inline ParamT param(const ros::NodeHandle& nh, const std::string& key, const ParamT& defaultValue=ParamT())
{
  ParamT param_val;
  param(nh, key, param_val, defaultValue);
  return param_val;
}

// primitive types
template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, uint32_t& param)
{
  int32_t value = 0;
  bool success = getParam(nh, key, value);
  if (value < 0)
  {
    ROS_ERROR_STREAM("Parameter " << key << " is smaller than 0, cannot be stored inside an unsigned int.");
    return false;
  }
  param = value;
  return success;
}

// ros
template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, ros::Time& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/sec", param.sec);
  success = success && getParam(nh, key + "/nsec", param.nsec);
  return success;
}

// std_msgs
template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, std_msgs::Header& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/stamp", param.stamp);
  success = success && getParam(nh, key + "/seq", param.seq);
  success = success && getParam(nh, key + "/frame_id", param.frame_id);
  return success;
}

// geometry_msgs
template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Vector3& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", param.x);
  success = success && getParam(nh, key + "/y", param.y);
  success = success && getParam(nh, key + "/z", param.z);
  return success;
}

template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Point& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", param.x);
  success = success && getParam(nh, key + "/y", param.y);
  success = success && getParam(nh, key + "/z", param.z);
  return success;
}

template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Quaternion& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/w", param.w);
  success = success && getParam(nh, key + "/x", param.x);
  success = success && getParam(nh, key + "/y", param.y);
  success = success && getParam(nh, key + "/z", param.z);
  return success;
}

template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Pose& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/position", param.position);
  success = success && getParam(nh, key + "/orientation", param.orientation);
  return success;
}

template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::PoseStamped& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/header", param.header);
  success = success && getParam(nh, key + "/pose", param.pose);
  return success;
}

template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Twist& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/linear", param.linear);
  success = success && getParam(nh, key + "/angular", param.angular);
  return success;
}

template<>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::TwistStamped& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/header", param.header);
  success = success && getParam(nh, key + "/twist", param.twist);
  return success;
}

template<typename T>
T getMember(XmlRpc::XmlRpcValue param, const std::string& key)
{
  try
  {
    if (!param.hasMember(key))
    {
      ROS_ERROR_STREAM("XmlRpcValue does not contain member '" << key << "'.");
      return T();
    }
    return static_cast<T>(param[key]);
  }
  catch (const XmlRpc::XmlRpcException& exception)
  {
    ROS_ERROR_STREAM("Caught an XmlRpc exception while getting member '" << key << "'.");
    return T();
  }
}


} // param_io
