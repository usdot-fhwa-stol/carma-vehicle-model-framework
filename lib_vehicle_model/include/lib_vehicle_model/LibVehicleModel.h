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

#include <memory>
#include <exception>
#include <ros/ros.h>
#include "VehicleState.h"
#include "VehicleMotionModel.h"
#include "VehicleModelControlInput.h"
#include "ParameterServer.h"
#include "KinematicsSolver.h"
#include "KinematicsProperty.h"

namespace lib_vehicle_model {

  //
  // Public Namespace
  //

  /**
   * @brief Initialization function for vehicle model. Loads the vehicle model as specified by parameters 
   * 
   * When provided with a NodeHandle this function uses the ROSParameterServer instead of a user specified parameter server
   * 
   * @param nh A pointer to the node handle which will be used to initialize the ROSParameterServer
   * 
   * @throws std::invalid_argument If the model could not be loaded or parameters could not be read
   * @throws std::runtime_error If this function is called more than once within the same process execution
   * 
   */ 
  void init(std::shared_ptr<ros::NodeHandle> nh);

  /**
   * @brief Initialization function for class. Loads the library as specified by parameters 
   * 
   * @param parameter_server A reference to the parameter server which vehicle models will use to load parameters
   * 
   * @throws std::invalid_argument If the model could not be loaded or parameters could not be read
   * @throws std::runtime_error If this function is called more than once within the same process execution
   * 
   */ 
  void init(std::shared_ptr<ParameterServer> parameter_server);

  //
  // Functions matching the VehicleMotionModel interface
  //

  /**
   * @brief Predict vehicle motion assuming no change in control input
   * 
   * @param initial_state The starting state of the vehicle
   * @param timestep The time increment between returned traversed states
   * @param delta_t The time to project the motion forward for
   * 
   * @return A list of traversed states seperated by the timestep
   * 
   * @throws std::runtime_error If this function is called before the init() function
   * 
   * NOTE: This function header must match a predict function found in the VehicleMotionModel interface
   * 
   */
  std::vector<VehicleState> predict(VehicleState initial_state,
    double timestep, double delta_t); 

  /**
   * @brief Predict vehicle motion given a starting state and list of control inputs
   * 
   * @param initial_state The starting state of the vehicle
   * @param control_inputs A list of control inputs seperated by the provided timestep
   * @param timestep The time increment between returned traversed states and provided control inputs
   * 
   * @return A list of traversed states seperated by the timestep
   * 
   * @throws std::runtime_error If this function is called before the init() function
   * 
   * NOTE: This function header must match a predict function found in the VehicleMotionModel interface
   * 
   */
  std::vector<VehicleState> predict(VehicleState initial_state,
    std::vector<VehicleModelControlInput> control_inputs, double timestep);
}