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
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


namespace param_io {


/*
 * Interfaces:
 *
 * bool   getParam(const ros::NodeHandle& nh, const std::string& key, ParamT& param);
 * ParamT getParam(const ros::NodeHandle& nh, const std::string& key);
 * ParamT getParam(const ros::NodeHandle& nh, const std::string& key, bool& success);
 */


// primitive types
template<typename ParamT>
bool getParam(const ros::NodeHandle& nh, const std::string& key, ParamT& param)
{
  if (!nh.getParam(key, param))
  {
    ROS_WARN_STREAM("Could not acquire parameter " << key << " from server.");
    return false;
  }
  return true;
}

template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, uint32_t& param)
{
  bool success = true;
  int32_t value = 0;
  success = success && getParam(nh, key, value);
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
bool getParam(const ros::NodeHandle& nh, const std::string& key, ros::Time& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/sec", param.sec);
  success = success && getParam(nh, key + "/nsec", param.nsec);
  return success;
}

// std_msgs
template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, std_msgs::Header& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/stamp", param.stamp);
  success = success && getParam(nh, key + "/seq", param.seq);
  success = success && getParam(nh, key + "/frame_id", param.frame_id);
  return success;
}

// geometry_msgs
template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Vector3& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", param.x);
  success = success && getParam(nh, key + "/y", param.y);
  success = success && getParam(nh, key + "/z", param.z);
  return success;
}

template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Point& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/x", param.x);
  success = success && getParam(nh, key + "/y", param.y);
  success = success && getParam(nh, key + "/z", param.z);
  return success;
}

template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Quaternion& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/w", param.w);
  success = success && getParam(nh, key + "/x", param.x);
  success = success && getParam(nh, key + "/y", param.y);
  success = success && getParam(nh, key + "/z", param.z);
  return success;
}

template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Pose& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/position", param.position);
  success = success && getParam(nh, key + "/orientation", param.orientation);
  return success;
}

template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::PoseStamped& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/header", param.header);
  success = success && getParam(nh, key + "/pose", param.pose);
  return success;
}

template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::Twist& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/linear", param.linear);
  success = success && getParam(nh, key + "/angular", param.angular);
  return success;
}

template<>
bool getParam(const ros::NodeHandle& nh, const std::string& key, geometry_msgs::TwistStamped& param)
{
  bool success = true;
  success = success && getParam(nh, key + "/header", param.header);
  success = success && getParam(nh, key + "/twist", param.twist);
  return success;
}

// alternative function interfaces
template<typename ParamT>
ParamT getParam(const ros::NodeHandle& nh, const std::string& key)
{
  return getParam<ParamT>(nh, key);
}

template<typename ParamT>
ParamT getParam(const ros::NodeHandle& nh, const std::string& key, bool& success)
{
  ParamT param;
  success = getParam(nh, key, param);
  return param;
}


} // param_io
