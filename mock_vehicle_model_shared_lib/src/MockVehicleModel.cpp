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

#include "MockVehicleModel.h"


using namespace lib_vehicle_model;

/**
 * Cpp containing the implementation of MockVehicleModel
 */

MockVehicleModel::MockVehicleModel() {};

MockVehicleModel::~MockVehicleModel() {};

void MockVehicleModel::setParameterServer(std::shared_ptr<ParameterServer> parameter_server) {
  param_server_ = parameter_server;
  param_server_->getParam("example_param", example_param_);
}

std::vector<VehicleState> MockVehicleModel::predict(const VehicleState& initial_state,
  double timestep, double delta_t) {
    VehicleState vs;
    vs.X_pos_global = initial_state.X_pos_global + 5;// Update x pos to confirm data was processed
    std::vector<VehicleState> states;
    states.push_back(vs);
    return states;
  }

std::vector<VehicleState> MockVehicleModel::predict(const VehicleState& initial_state,
  const std::vector<VehicleControlInput>& control_inputs, double timestep) {

    VehicleState vs;
    vs.X_pos_global = initial_state.X_pos_global + 5;// Update x pos to confirm data was processed
    std::vector<VehicleState> states;
    states.push_back(vs);
    return states;
  }
