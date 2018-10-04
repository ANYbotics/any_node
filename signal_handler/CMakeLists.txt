cmake_minimum_required(VERSION 2.8.3)
project(signal_handler)

find_package(catkin QUIET)

###################################
## catkin specific configuration ##
###################################
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}
  )
endif()

###########
## Build ##
###########
add_definitions(-std=c++11)

include_directories(
  include
)
if(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif()

add_library(
  ${PROJECT_NAME}
  src/SignalHandler.cpp
)

#############
## Install ##
#############
if(catkin_FOUND)
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

### Local Build ###

if(NOT catkin_FOUND)
  # Register the local build in case on doesn't use "make install"
  export(PACKAGE signal_handler)

  # Create variable with the library location
  set(LIB_NAME signal_handler)
  set(signal_handler_library $<TARGET_FILE:${LIB_NAME}>)

  # Create variable for the local build tree
  get_property(signal_handler_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  # Configure config file for local build tree
  configure_file(signal_handlerConfig.cmake.in
    "${PROJECT_BINARY_DIR}/signal_handlerConfig.cmake" @ONLY
  )

### Install Build ###

  # Change the include location for the case of an install location
  set(signal_handler_include_dirs
    ${CMAKE_INSTALL_PREFIX}/include
  )

  # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
  configure_file(signal_handlerConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/signal_handlerConfig.cmake" @ONLY
  )

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/signal_handlerConfig.cmake"
    DESTINATION share/signal_handler/cmake COMPONENT dev
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/signal_handler/
    DESTINATION include/signal_handler
  )
endif()