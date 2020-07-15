cmake_minimum_required(VERSION 3.5.1)
project(param_io)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
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
    std_msgs
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${rostest_INCLUDE_DIRS}
  )
  add_rostest_gtest(test_${PROJECT_NAME}_node
    test/param_io.test
    test/main.cpp
    test/GetParam.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${rostest_LIBRARIES}
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_rostest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()

##########
## Test ##
##########
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
