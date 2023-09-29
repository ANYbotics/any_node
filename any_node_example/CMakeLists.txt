cmake_minimum_required(VERSION 3.5.1)
project(any_node_example)

find_package(catkin REQUIRED
  COMPONENTS
    any_node
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    any_node
    std_msgs
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/ExampleNode.cpp
)

target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/any_node_example_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########
## Test ##
##########
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
