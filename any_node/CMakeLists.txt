cmake_minimum_required(VERSION 3.5.1)
project(any_node)



if("$ENV{ROS_VERSION}" STREQUAL "1")



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
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/EmptyTests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries( test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      gtest_main
    )
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()



else() # ROS version 2



find_package(acl_config_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(any_worker REQUIRED)
find_package(rclcpp REQUIRED)
find_package(signal_handler REQUIRED)

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



add_library(${PROJECT_NAME} src/Node.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ament_cmake
  acl_config_cpp any_worker rclcpp signal_handler)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME} test/EmptyTests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

  if(TARGET test_${PROJECT_NAME})
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()

ament_export_dependencies(acl_config_cpp)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(any_worker)
ament_export_dependencies(rclcpp)
ament_export_dependencies(signal_handler)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(TARGETS DESTINATION lib/${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

endif()
