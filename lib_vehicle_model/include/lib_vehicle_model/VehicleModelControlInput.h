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

namespace lib_vehicle_model {
  /**
   * @struct VehicleModelControlInput
   * @brief A struct used to define the set of variables used to represent the vehicle control input.
   * 
   * The members of this struct are defined relative to the vehicle state and control models described in the VehicleModelLib documentation
   */
  struct VehicleModelControlInput 
  {
    /**
     * The target forward acceleration of the vehicle in m/s^2
     */
    double target_acceleration;

    /**
     * The target steering angle for the front wheel to make with the longitudinal centerline of the vehicle in rad
     * Left of the centerline is positive
     */
    double target_steering_angle;
  };
}