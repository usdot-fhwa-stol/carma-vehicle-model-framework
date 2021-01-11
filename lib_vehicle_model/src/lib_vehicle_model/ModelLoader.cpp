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

#include "ModelLoader.h"



/**
 * Cpp containing the implementation of ModelLoader
 */
using namespace lib_vehicle_model;

std::unique_ptr<VehicleMotionModel, ModelLoader::destroy_fnc_ptr> ModelLoader::load(std::string& vehicle_model_lib_path) {
  // Exception output stream
  std::ostringstream error_msg;

  // Load library from path
  void *lib_handle;
  lib_handle = dlopen(vehicle_model_lib_path.c_str(), RTLD_NOW);

  // Check if load successfull
  if (!lib_handle)
  {
    error_msg << "Failed to open vehicle model shared library at " << vehicle_model_lib_path << " Reported Error: " << dlerror();
    throw std::invalid_argument(error_msg.str());
  }

  // Get pointers to the create and destroy functions
  create_fnc_ptr create_fnc = (create_fnc_ptr)dlsym(lib_handle,"create");
  destroy_fnc_ptr destroy_fnc = (destroy_fnc_ptr)dlsym(lib_handle,"destroy");

  // Check if create and destroy functions could be found
  if (!create_fnc)
  {
    error_msg << "Failed to find pointer to vehicle model shared library create function Reported Error: " << dlerror();
    throw std::invalid_argument(error_msg.str());
  }

  if (!destroy_fnc)
  {
    std::string errorLog(dlerror());
    error_msg << "Failed to find pointer to vehicle model shared library destroy function Reported Error: " << errorLog;
    throw std::invalid_argument(error_msg.str());
  }

  // Set the vehicle model to the object returned by create_fnc
  // Pass in the destroy_fnc as the smart pointer deletor
  std::unique_ptr<VehicleMotionModel, destroy_fnc_ptr> vehicle_model(create_fnc(), destroy_fnc);

  return vehicle_model;
}
