cmake_minimum_required(VERSION 2.8.3)
project(param_io)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    rostest
    std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rostest
    std_msgs
)

if (CATKIN_ENABLE_TESTING)
  include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
  )
  add_rostest_gtest(test_param_io
    test/test_param_io.test
    test/test_main.cpp
    test/get_param_test.cpp
  )
  target_link_libraries(test_param_io
    ${catkin_LIBRARIES}
  )
endif()

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
