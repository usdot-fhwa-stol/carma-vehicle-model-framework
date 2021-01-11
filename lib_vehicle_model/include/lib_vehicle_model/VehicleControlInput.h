#pragma once
/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <sstream>

namespace lib_vehicle_model {
  /**
   * @struct VehicleControlInput
   * @brief A struct used to define the set of variables used to represent the vehicle control input.
   * 
   * The members of this struct are defined relative to the vehicle state and control models described in the VehicleModelLib documentation
   */
  struct VehicleControlInput 
  {

    /**
     * The target steering angle for the front wheel to make with the longitudinal centerline of the vehicle in rad
     * Left of the centerline is positive
     */
    double target_steering_angle = 0;

    /** 
     * The target velocity command in m/s
     */
    double target_velocity = 0;

    /**
     * Overload of << operation so struct will output as strings in print functions
     * 
     */ 
    friend std::ostream& operator<<( std::ostream& os, const VehicleControlInput& c )
    {
      os << "VehicleControlInput [ " << 
        c.target_steering_angle << ", " <<
        c.target_velocity << " ]";

      return os;
    }
  };
}