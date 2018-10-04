cmake_minimum_required(VERSION 2.8.3)
project(any_worker)

find_package(catkin QUIET)

if(catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS
    message_logger
  )
else()
  find_package(message_logger REQUIRED)
endif()

###################################
## catkin specific configuration ##
###################################
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} pthread
    CATKIN_DEPENDS message_logger
  )
endif()

###########
## Build ##
###########
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
if(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
else()
  include_directories(
    ${message_logger_INCLUDE_DIRS}
  )
endif()

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
    ${PROJECT_NAME}_test
    test/${PROJECT_NAME}_test.cpp
    test/RateTest.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
endif()

#############
## Install ##
#############
if(catkin_FOUND)
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

### Local Build ###

if(NOT catkin_FOUND)
  set(external_libs
    ${message_logger_LIBRARIES}
  )

  # Register the local build in case on doesn't use "make install"
  export(PACKAGE any_worker)

  # Create variable with the library location
  set(LIB_NAME any_worker)
  set(any_worker_library $<TARGET_FILE:${LIB_NAME}>)

  # Create variable for the local build tree
  get_property(any_worker_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  # Configure config file for local build tree
  configure_file(any_workerConfig.cmake.in
    "${PROJECT_BINARY_DIR}/any_workerConfig.cmake" @ONLY
  )

### Install Build ###

  # Change the include location for the case of an install location
  set(any_worker_include_dirs
    ${CMAKE_INSTALL_PREFIX}/include
    ${message_logger_INCLUDE_DIRS}
  )

  # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
  configure_file(any_workerConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/any_workerConfig.cmake" @ONLY
  )

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/any_workerConfig.cmake"
    DESTINATION share/any_worker/cmake COMPONENT dev
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/any_worker/
    DESTINATION include/any_worker
  )
endif()