cmake_minimum_required(VERSION 3.5.1)
project(any_node)

find_package(catkin REQUIRED
  COMPONENTS
    any_worker
    message_logger
    param_io
    roscpp
    signal_handler
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    pthread
  CATKIN_DEPENDS
    any_worker
    message_logger
    param_io
    roscpp
    signal_handler
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/Node.cpp
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##########
## Test ##
##########
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
