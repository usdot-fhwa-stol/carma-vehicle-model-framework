cmake_minimum_required(VERSION 2.8.3)

#
# Copyright (C) 2018-2019 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#

project(temp_filter_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(DEPS 
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  wgs84_utils
  cav_msgs
  cav_srvs
  message_filters
  novatel_gps_msgs
  carma_utils
  sensor_msgs
  sigpack
  dynamic_reconfigure
  jsk_recognition_msgs
)
find_package(catkin REQUIRED COMPONENTS
  ${DEPS}
)

generate_dynamic_reconfigure_options(
  cfg/Filter.cfg
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS ${DEPS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

