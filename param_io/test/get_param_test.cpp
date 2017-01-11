/*
 * get_param_test.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: Christian Gehring
 */




#include <gtest/gtest.h>
#include <ros/ros.h>

#include <param_io/get_param.hpp>

TEST(GetParam, getParam) {
  ros::NodeHandle nh("~");
  double testdouble = 0;

  testdouble = 3.0;
  ASSERT_FALSE(param_io::getParam(nh, "double", testdouble));
  ASSERT_EQ(3.0, testdouble);

  testdouble = 1.0;
  ASSERT_FALSE(param_io::getParam(nh, "double", testdouble, 4.0));
  ASSERT_EQ(4.0, testdouble);

  testdouble = 1.0;
  testdouble = param_io::getParam(nh, "double", 2.0);
  ASSERT_EQ(2.0, testdouble);
}
