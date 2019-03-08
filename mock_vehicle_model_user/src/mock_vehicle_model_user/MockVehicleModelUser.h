#pragma once
/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <mutex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <lib_vehicle_model/LibVehicleModel.h>

/**
 * @class MockVehicleModelUser
 * @brief Is the class which demonstrates usage of the lib_vehicle_model library
 * 
 */
class MockVehicleModelUser 
{
private:
  // Shutdown flag and mutex
  std::mutex shutdown_mutex_;
  bool shutting_down_ = false;
  // Members used in ROS behavior
  int default_spin_rate_ = 1;
  std::shared_ptr<ros::NodeHandle> default_nh_;
  ros::Publisher exception_alert_pub_;
public:
  /**
   * @brief Constructor
   * @param argc - command line argument count
   * @param argv - command line arguments
   */
  MockVehicleModelUser(int argc, char** argv) {
    ros::init(argc,argv,"mock_vehicle_model_user_node"); // Initialize ROS connection
  };

  /**
   * @brief Execution function which will start the ROS subscriptions and publications. Exits on node shutdown.
   */
  int run();

private:

  /**
   * @brief Initializes the subscribers and publishers for this node
   */
  void initialize();

  /**
   * @brief Shutsdown this node
   */
  void shutdown();

  /**
   * @brief Handles caught exceptions which have reached the top level of this node
   * 
   * @param message The exception to handle
   * 
   * If an exception reaches the top level of this node it should be passed to this function.
   * The function will try to log the exception and publish a message to exception_alert before shutting itself down.
   */
  void handleException(const std::exception& e);
};