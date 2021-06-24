cmake_minimum_required(VERSION 2.8.3)
project(passenger_car_kinematic_model)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  lib_vehicle_model
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS lib_vehicle_model
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${lib_vehicle_model_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
## Declare as a SHARED library for dynamic linking
 add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/EntryPoint.cpp
  src/${PROJECT_NAME}/PassengerCarKinematicModel.cpp
 )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  # Add testing only dependencies
  find_package ( tf2 REQUIRED)
  find_package ( rosbag REQUIRED)
  find_package ( message_filters REQUIRED)
  find_package ( geometry_msgs REQUIRED)
  find_package ( sensor_msgs REQUIRED)
  find_package ( pacmod_msgs REQUIRED)
  find_package ( autoware_msgs REQUIRED)
  find_package ( automotive_platform_msgs REQUIRED)
  find_package ( roscpp REQUIRED)
  find_package ( Boost REQUIRED)
  find_package ( model_test_tools REQUIRED)
  find_package ( sigpack REQUIRED)
  find_package ( gps_common REQUIRED)

  include_directories(
    ${tf2_INCLUDE_DIRS}
    ${rosbag_INCLUDE_DIRS}
    ${message_filters_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${pacmod_msgs_INCLUDE_DIRS}
    ${autoware_msgs_INCLUDE_DIRS}
    ${automotive_platform_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${model_test_tools_INCLUDE_DIRS}
    ${sigpack_INCLUDE_DIRS}
    ${gps_common_INCLUDE_DIRS}
  )

  catkin_add_gmock(${PROJECT_NAME}-test 
    test/TestEntryPoint.cpp
    test/${PROJECT_NAME}/PassengerCarKinematicModelTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
  )

  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    ${tf2_LIBRARIES}
    ${rosbag_LIBRARIES}
    ${message_filters_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${pacmod_msgs_LIBRARIES}
    ${autoware_msgs_LIBRARIES}
    ${automotive_platform_msgs_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${Boost_LIBRARIES}
    ${model_test_tools_LIBRARIES}
    ${sigpack_LIBRARIES}
    ${gps_common_LIBRARIES}
  )

  add_dependencies( ${PROJECT_NAME}-test 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    ${tf2_EXPORTED_TARGETS}
    ${rosbag_EXPORTED_TARGETS}
    ${message_filters_EXPORTED_TARGETS}
    ${geometry_msgs_EXPORTED_TARGETS}
    ${sensor_msgs_EXPORTED_TARGETS}
    ${pacmod_msgs_EXPORTED_TARGETS}
    ${autoware_msgs_EXPORTED_TARGETS}
    ${automotive_platform_msgs_EXPORTED_TARGETS}
    ${roscpp_EXPORTED_TARGETS}
    ${Boost_EXPORTED_TARGETS}
    ${model_test_tools_EXPORTED_TARGETS}
    ${sigpack_EXPORTED_TARGETS}
    ${gps_common_EXPORTED_TARGETS}
  )

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
