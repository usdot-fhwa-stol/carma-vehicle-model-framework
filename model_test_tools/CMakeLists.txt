cmake_minimum_required(VERSION 2.8.3)
project(model_test_tools)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  lib_vehicle_model
  tf2
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS lib_vehicle_model
)

###########
## Build ##
###########

include_directories(
 include
 ${lib_vehicle_model_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

