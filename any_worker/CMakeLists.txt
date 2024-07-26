cmake_minimum_required(VERSION 3.5.1)
project(any_worker)

if("$ENV{ROS_VERSION}" STREQUAL "1")

find_package(catkin REQUIRED COMPONENTS
  message_logger
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} pthread
  CATKIN_DEPENDS message_logger
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
  src/Rate.cpp
  src/Worker.cpp
  src/WorkerManager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

##########
## Test ##
##########
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    test_${PROJECT_NAME}
    test/${PROJECT_NAME}_test.cpp
    test/RateTest.cpp
  )
endif()

if(TARGET test_${PROJECT_NAME})
  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()

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
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

else() # ROS version 2

###############
## Configure ##
###############

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
  message_logger
)

find_package(ament_cmake REQUIRED)
foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
  src/Rate.cpp
  src/Worker.cpp
  src/WorkerManager.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
  ${PACKAGE_DEPENDENCIES}
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/${PROJECT_NAME}_test.cpp
    test/RateTest.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()


ament_package()

endif()
