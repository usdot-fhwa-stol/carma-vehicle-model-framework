#pragma once
/*
 * Copyright (C) 2018-2020 LEIDOS.
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
   * @struct VehicleState
   * @brief A struct used to define the set of variables used to represent the vehicle state
   * 
   * The members of this struct are defined relative to the vehicle state and control models described in the VehicleModelLib documentation
   */
  struct VehicleState
  {
    /**
     * 2d x-axis position of the vehicle center of gravity in meters
     * This position is in a fixed inertial frame which vehicle motion is described in
     */ 
    double X_pos_global = 0;
    /**
     * 2d y-axis position of the vehicle center of gravity in meters
     * This position is in a fixed inertial frame which vehicle motion is described in
     */ 
    double Y_pos_global = 0;
    /**
     * The orientation of the vehicle's longitudinal axis in radians
     * This orientation is in a fixed inertial frame which vehicle motion is described in
     */ 
    double orientation = 0;
    /**
     * longitudinal velocity of the vehicle center of gravity in m/s in its body frame
     */ 
    double longitudinal_vel = 0;
    /**
     * lateral velocity of the vehicle center of gravity in m/s in its body frame
     */ 
    double lateral_vel = 0;
    /**
     * The yaw rate of the vehicle in rad/s in the body frame which is equivalent to the rate of change of orientation
     */ 
    double yaw_rate = 0;
    /** 
     * The angular velocity of the front wheel in rad/s
     */ 
    double front_wheel_rotation_rate = 0;
    /**
     * The angular velocity of the rear wheel in rad/s
     */ 
    double rear_wheel_rotation_rate = 0;
    /**
     * The steering angle of the front wheel in rad with left being positive
     */ 
    double steering_angle = 0;
    /**
     * The angle the trailer makes with the vehicle longitudinal axis in rad with left being positive
     */ 
    double trailer_angle = 0;
    /**
     * The previous velocity command in m/s
     */ 
    double prev_vel_cmd = 0;
    /**
     * The previous steering command in rad
     */ 
    double prev_steering_cmd = 0;

    /**
     * Overload of << operation so struct will output as strings in print functions
     * 
     */ 
    friend std::ostream& operator<<( std::ostream& os, const VehicleState& v )
    {
      os << "VehicleState [ " << 
        v.X_pos_global << ", " <<
        v.Y_pos_global << ", " <<
        v.orientation << ", " <<
        v.longitudinal_vel << ", " <<
        v.lateral_vel << ", " <<
        v.yaw_rate << ", " <<
        v.front_wheel_rotation_rate << ", " <<
        v.rear_wheel_rotation_rate << ", " <<
        v.steering_angle << ", " <<
        v.trailer_angle << ", " <<
        v.prev_steering_cmd << ", " <<
        v.prev_vel_cmd << " ]";

      return os;
    }
  };
}