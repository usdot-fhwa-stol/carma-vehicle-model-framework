cmake_minimum_required(VERSION 2.8.3)
project(passenger_car_kinematic_model)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  lib_vehicle_model
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/temp_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  lib_vehicle_model
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
  )

  catkin_add_gmock(${PROJECT_NAME}-test 
    test/TestEntryPoint.cpp
    test/${PROJECT_NAME}/PassengerCarKinematicModelTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
  )

  target_link_libraries(${PROJECT_NAME}-test lib_vehicle_model 
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
  )

  add_dependencies( ${PROJECT_NAME}-test lib_vehicle_model 
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
  )

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)