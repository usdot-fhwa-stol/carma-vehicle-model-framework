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

#include <vector>
#include <string>
#include <memory>
#include <lib_vehicle_model/VehicleState.h>
#include <lib_vehicle_model/VehicleMotionModel.h>
#include <lib_vehicle_model/VehicleModelControlInput.h>
#include <lib_vehicle_model/ParameterServer.h>

using namespace lib_vehicle_model;

/**
 * @class MockVehicleModel
 * @brief Example class which implements a mock version of VehicleMotionModel interface to demonstrate library linking 
 * 
 * NOTE: This class should not be used in real world execution on a vehicle
 */
class MockVehicleModel: public VehicleMotionModel
{
  private:
    std::shared_ptr<ParameterServer> param_server_;

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

    void setParameterServer(std::shared_ptr<ParameterServer> parameter_server) override;

    std::vector<VehicleState> predict(VehicleState initial_state,
      double timestep, double delta_t) override; 

    std::vector<VehicleState> predict(VehicleState initial_state,
      std::vector<VehicleModelControlInput> control_inputs, double timestep) override;
};
