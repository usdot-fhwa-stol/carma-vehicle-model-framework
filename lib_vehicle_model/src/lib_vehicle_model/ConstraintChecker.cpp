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

#include "ConstraintChecker.h"



/**
 * Cpp containing the implementation of ConstraintChecker
 */
using namespace lib_vehicle_model;

ConstraintChecker::ConstraintChecker(std::shared_ptr<ParameterServer> parameter_server) {

  // Load Parameters
  // TODO remove reverse parameters from class diagram
  bool speedParam = parameter_server->getParam("max_forward_speed", max_forward_speed_);
  bool accelParam = parameter_server->getParam("forward_acceleration_limit", forward_acceleration_limit_);
  bool decelParam = parameter_server->getParam("forward_deceleration_limit", forward_deceleration_limit_);
  bool maxSteerParam = parameter_server->getParam("max_steering_angle", max_steering_angle_);
  bool minSteerParam = parameter_server->getParam("min_steering_angle", min_steering_angle_);
  bool maxSteerRateParam = parameter_server->getParam("max_steering_angle_rate", max_steering_angle_rate_);
  bool maxTrailerAngleParam = parameter_server->getParam("max_trailer_angle", max_trailer_angle_);
  bool minTrailerAngleParam = parameter_server->getParam("min_trailer_angle", min_trailer_angle_);

  // Check if all the required parameters could be loaded
  if (!(speedParam && accelParam && decelParam && maxSteerParam && minSteerParam
         && maxSteerRateParam && maxTrailerAngleParam && minTrailerAngleParam)) {

    std::ostringstream msg;
    msg << "One of the required parameters could not be found or read " 
      << " max_forward_speed: " << speedParam 
      << " forward_acceleration_limit: " << accelParam 
      << " forward_deceleration_limit: " << decelParam 
      << " max_steering_angle: " << maxSteerParam 
      << " min_steering_angle: " << minSteerParam 
      << " max_steering_angle_rate: " << maxSteerRateParam
      << " max_trailer_angle: " << maxTrailerAngleParam 
      << " min_trailer_angle: " << minTrailerAngleParam;

    throw std::invalid_argument(msg.str());
  }
}

void ConstraintChecker::validateInitialState(const VehicleState& initial_state) {
  std::ostringstream msg;

  if (initial_state.steering_angle < min_steering_angle_) {
    msg << "Invalid initial_state with steering angle: " << initial_state.steering_angle << " is below min of: " << min_steering_angle_;
    throw std::invalid_argument(msg.str());
  }
  
  if (initial_state.steering_angle > max_steering_angle_) {
    msg << "Invalid initial_state with steering angle: " << initial_state.steering_angle << " is above max of: " << max_steering_angle_;
    throw std::invalid_argument(msg.str());
  }

  if (initial_state.trailer_angle < min_trailer_angle_) {
    msg << "Invalid initial_state with trailer angle: " << initial_state.trailer_angle << " is below min of: " << min_trailer_angle_;
    throw std::invalid_argument(msg.str());
  }

  if (initial_state.trailer_angle > max_trailer_angle_) {
    msg << "Invalid initial_state with trailer angle: " << initial_state.trailer_angle << " is above max of: " << max_trailer_angle_;
    throw std::invalid_argument(msg.str());
  }

} 

void ConstraintChecker::validateControlInputs(const VehicleState& initial_state, const std::vector<VehicleModelControlInput>& control_inputs, const double timestep) {

  // Last steering angle used to compute rate of steering angle change between control inputs
  double last_steer_angle = initial_state.steering_angle;

  size_t count = 0;
  std::ostringstream msg;
  // Validate each control input in sequence
  for (const VehicleModelControlInput& control : control_inputs) {
    if (control.target_acceleration < forward_deceleration_limit_) {
      msg << "Invalid control_input " << count << " with target_acceleration: " << control.target_acceleration << " is below min of: " << forward_deceleration_limit_;
      throw std::invalid_argument(msg.str());
    }

    if (control.target_acceleration > forward_acceleration_limit_) {
      msg << "Invalid control_input " << count << " with target_acceleration: " << control.target_acceleration << " is above max of: " << forward_acceleration_limit_;
      throw std::invalid_argument(msg.str());
    }

    if (control.target_steering_angle < min_steering_angle_) {
      msg << "Invalid control_input " << count << " with target_steering_angle: " << control.target_steering_angle << " is below min of: " << min_steering_angle_;
      throw std::invalid_argument(msg.str());
    }

    if (control.target_steering_angle > max_steering_angle_) {
      msg << "Invalid control_input " << count << " with target_steering_angle: " << control.target_steering_angle << " is above max of: " << max_steering_angle_;
      throw std::invalid_argument(msg.str());
    }

    const double delta_steer = control.target_steering_angle - last_steer_angle;
    const double steering_rate = abs(delta_steer / timestep);
    if (steering_rate > max_steering_angle_rate_) {
      msg << "Invalid control_input " << count << " with rate of steering change : " << steering_rate << " is above max of: " << max_steering_angle_rate_;
      throw std::invalid_argument(msg.str());
    }

    last_steer_angle = control.target_steering_angle;
    count++;
  }
}
