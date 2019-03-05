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

#include <stdexcept>
#include <vector>
#include <string>
#include <dlfcn.h>
#include <memory>
#include <stdlib.h>
#include <sstream>
#include "lib_vehicle_model/VehicleMotionModel.h"


namespace lib_vehicle_model {
  /**
   * @class ConstraintChecker
   * @brief Class which can validate vehicle states and control inputs
   */
  class ConstraintChecker
  {
    private:
      // Constraints
      double max_forward_speed_;
      double max_reverse_speed_;
      double forward_acceleration_limit_;
      double forward_deceleration_limit_;
      double reverse_acceleration_limit_;
      double reverse_deceleration_limit_;
      double max_steering_angle_;
      double min_steering_angle_;
      double max_steering_angle_rate_;
      double max_trailer_angle_;
      double min_trailer_angle_;

    public:
      ConstraintChecker(std::shared_ptr<ParameterServer> parameter_server);

      /**
       * @brief Helper function to validate the initial vehicle state for a motion prediction
       * 
       * @param initial_state The starting state of the vehicle passed into the prediction function
       * 
       * @throws std::invalid_argument If the initial vehicle state is found to be invalid
       */
      void validateInitialState(const VehicleState& initial_state);  

      /**
       * @brief Helper function to validate the control inputs for a motion prediction
       * 
       * @param initial_state The starting state of the vehicle passed into the prediction function
       * @param control_inputs The control inputs for the vehicle passed into the prediction function
       * @param timestep The difference in time between successive control inputs in seconds
       * 
       * @throws std::invalid_argument If the initial control inputs are found to be invalid
       */
      void validateControlInputs(const VehicleState& initial_state, const std::vector<VehicleModelControlInput>& control_inputs, const double timestep); 
  };
}