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
   * @struct VehicleState
   * @brief A struct used to define the set of variables used to represent the vehicle state
   * 
   * The members of this struct are defined relative to the vehicle state and control models described in the VehicleModelLib documentation
   */
  struct VehicleState
  {
    /**
     * 2d x-axis position of the vehicle center of gravity in meters
     */ 
    double x_pos;
    /**
     * 2d y-axis position of the vehicle center of gravity in meters
     */ 
    double y_pos;
    /**
     * 2d x-axis velocity of the vehicle center of gravity in m/s
     */ 
    double x_vel;
    /**
     * 2d y-axis velocity of the vehicle center of gravity in m/s
     */ 
    double y_vel;
    /**
     * 2d x-axis acceleration of the vehicle center of gravity in m/s^2
     */ 
    double x_accel;
    /**
     * 2d y-axis acceleration of the vehicle center of gravity in m/s^2
     */ 
    double y_accel;
    /**
     * The angle of the vehicle centerline with respect to the X axis in rad
     */ 
    double angle_with_x_axis;
    /**
     * The angular velocity of the vehicle centerline with respect to the X axis in rad/s
     */ 
    double angular_vel; //TODO this field is not in the class diagram
    /**
     * The angular acceleration of the vehicle centerline with respect to the X axis in rad/s^2
     */ 
    double angular_accel;
    /**
     * The current steering angle in rad of the wheels relative to the vehicle center line. Positive angles will be to the left of the vehicle.
     */ 
    double steering_angle;
    /**
     * The current angle in rad a trailer (if any) makes with the vehicle center line. Positive angles will be to the left of the cab centerline
     */ 
    double trailer_angle;
  };
}