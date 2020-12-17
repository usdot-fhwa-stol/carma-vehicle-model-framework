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

#include <vector>
#include <string>
#include <memory>
#include <lib_vehicle_model/VehicleState.h>
#include <lib_vehicle_model/VehicleMotionModel.h>
#include <lib_vehicle_model/VehicleControlInput.h>
#include <lib_vehicle_model/ParameterServer.h>

/**
 * @class MockVehicleModel
 * @brief Example class which implements a mock version of VehicleMotionModel interface to demonstrate library linking 
 * 
 * NOTE: This class should not be used in real world execution on a vehicle
 */
class MockVehicleModel: public lib_vehicle_model::VehicleMotionModel
{
  private:
    std::shared_ptr<lib_vehicle_model::ParameterServer> param_server_;

    // Parameters
    double example_param_;

  public:

    /**
     * @brief Constructor 
     * 
     */ 
    MockVehicleModel();

    /**
     * @brief Destructor as required by interface
     * 
     */ 
    ~MockVehicleModel();

    //
    // Overriden interface functions
    //
    void setParameterServer(std::shared_ptr<lib_vehicle_model::ParameterServer> parameter_server) override;

    std::vector<lib_vehicle_model::VehicleState> predict(const lib_vehicle_model::VehicleState& initial_state,
      double timestep, double delta_t) override; 

    std::vector<lib_vehicle_model::VehicleState> predict(const lib_vehicle_model::VehicleState& initial_state,
      const std::vector<lib_vehicle_model::VehicleControlInput>& control_inputs, double timestep) override;
};
