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

#include "passenger_car_kinematic_model/PassengerCarKinematicModel.h"

/**
 * This Cpp file defines the two functions used as entry and exit points from the vehicle model shared library
 * Define functions with C symbols (create/destroy PassengerCarKinematicModel instance).
 */ 
/**
 * @brief Creates a new PassengerCarKinematicModel instance and returns a raw pointer to that instance 
 * 
 * This function is the access hook for getting a PassengerCarKinematicModel from this shared lib
 * 
 * @return A raw pointer to a new PassengerCarKinematicModel instance
 */
extern "C" PassengerCarKinematicModel* create()
{
  return new PassengerCarKinematicModel;
}

/**
 * @brief Destroys the PassengerCarKinematicModel pointed at by the provided raw pointer and frees up its memory
 * 
 * @param model_ptr A pointer to a valid instance of a PassengerCarKinematicModel
 * 
 * This function is the access hook for getting a PassengerCarKinematicModel from this shared lib
 */
extern "C" void destroy(PassengerCarKinematicModel* model_ptr)
{
  delete model_ptr;
}
