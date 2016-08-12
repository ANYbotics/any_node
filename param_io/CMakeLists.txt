cmake_minimum_required(VERSION 2.8.3)
project(param_io)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    std_msgs
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
