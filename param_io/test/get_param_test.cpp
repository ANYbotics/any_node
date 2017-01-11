/*
 * get_param_test.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: Christian Gehring
 */




#include <gtest/gtest.h>
#include <ros/ros.h>

#include <param_io/get_param.hpp>

TEST(GetParam, getParam_double) {
  ros::NodeHandle nh("~");
  double testdouble = 0.0;

  testdouble = 3.0;
  ASSERT_FALSE(param_io::getParam(nh, "double", testdouble));
  ASSERT_EQ(3.0, testdouble);

  testdouble = 1.0;
  ASSERT_FALSE(param_io::getParam(nh, "double", testdouble, 4.0));
  ASSERT_EQ(4.0, testdouble);

  testdouble = 1.0;
  double testdefault = 100;
  ASSERT_FALSE(param_io::getParam(nh, "double", testdouble, testdefault));
  ASSERT_EQ(testdefault, testdouble);


}

TEST(GetParam, param_double) {
  ros::NodeHandle nh("~");
  double testdouble = 0.0;

  testdouble = 1.0;
  testdouble = param_io::param(nh, "double", 2.0);
  ASSERT_EQ(2.0, testdouble);

  testdouble = 1.0;
  double testdefault = 100;
  double res = param_io::param(nh, "double", testdefault);
  ASSERT_EQ(testdefault, res);

}


TEST(GetParam, param_bool) {
  ros::NodeHandle nh("~");
  bool testbool = true;

  testbool = true;
  testbool = param_io::param(nh, "bool", false);
  ASSERT_EQ(false, testbool);

  testbool = true;
  bool testdefault = false;
  bool res = param_io::param(nh, "bool", testdefault);
  ASSERT_EQ(testdefault, res);

}
