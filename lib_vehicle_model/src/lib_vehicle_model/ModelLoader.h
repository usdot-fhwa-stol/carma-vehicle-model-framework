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
   * @class ModelLoader
   * @brief Class which can load a vehicle model
   */
  class ModelLoader
  {
    public:

      // Typedef for function pointers to use with loaded libraries create and destroy functions
      typedef VehicleMotionModel* (*create_fnc_ptr)();
      typedef void (*destroy_fnc_ptr)(VehicleMotionModel*);

      /** 
       * @brief Helper function to load the host vehicle model. Must be called only in constructor
       * 
       * @param vehicle_model_lib_path The path to the shared library containing the vehicle model to load
       * 
       * @return A pointer to the loaded vehicle model
       * 
       * @throws std::invalid_argument If the model could not be loaded 
       */
      static std::shared_ptr<VehicleMotionModel> load(std::string& vehicle_model_lib_path);
  };
}