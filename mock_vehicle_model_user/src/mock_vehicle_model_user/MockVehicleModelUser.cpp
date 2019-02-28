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

/**
 * CPP File containing MockVehicleModelUser method definitions
 */

#include "MockVehicleModelUser.h"

int MockVehicleModelUser::run() {
  bool goodInit = false;
  // Initialize Node
  try {
    ROS_INFO_STREAM("Before init");
    initialize();
    goodInit = true;
  }
  catch(const std::exception& e) {
    ROS_ERROR_STREAM("Failed to initialize node with exception: " << e.what());
    return -6;
  }

  if (goodInit) {
    ROS_INFO_STREAM("Initialization complete!");
  }
  
  // Continuosly process callbacks for default_nh_ using the GlobalCallbackQueue
  ros::Rate r(default_spin_rate_);
  while (ros::ok() && !shutting_down_)
  {
    ros::spinOnce();

    lib_vehicle_model::VehicleState state;
    state.x_pos = 10;
    std::vector<lib_vehicle_model::VehicleState> results = lib_vehicle_model::predict(state, 0.1, 0.2);
    std::ostringstream msg;
    msg << "Returned xPos = " << results.at(0).x_pos << " Given input = " << state.x_pos;
    std_msgs::String str_msg;
    str_msg.data = msg.str();
    exception_alert_pub_.publish(str_msg);

    ROS_INFO_STREAM(msg.str());

    r.sleep();
  }
  // Request ros node shutdown before exit
  ros::shutdown();

  return 0; 
}

void MockVehicleModelUser::initialize() {
  // Setup node handles here if needed
  default_nh_.reset(new ros::NodeHandle());

  // Exception Publisher
  exception_alert_pub_ = default_nh_->advertise<std_msgs::String>("exception_alert", 10, true);

  // Try to load the vehicle model
  lib_vehicle_model::init(default_nh_);
}

void MockVehicleModelUser::handleException(const std::exception& e) {
  // Create alert message
  std_msgs::String alert_msg;
  alert_msg.data = "Uncaught Exception in " + ros::this_node::getName() + " exception: " + e.what();
 
  ROS_ERROR_STREAM(alert_msg.data); // Log exception

  exception_alert_pub_.publish(alert_msg); // Notify subscribers

  ros::Duration(0.05).sleep(); // Leave a small amount of time for the alert to be published
  shutdown(); // Shutdown this node
}

void MockVehicleModelUser::shutdown() {
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  ROS_WARN_STREAM("Node shutting down");
  shutting_down_ = true;
}
