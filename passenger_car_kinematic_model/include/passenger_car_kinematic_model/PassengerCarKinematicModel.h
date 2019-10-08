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
#include <stdexcept>
#include <lib_vehicle_model/ODESolver.h>
#include <lib_vehicle_model/VehicleState.h>
#include <lib_vehicle_model/VehicleMotionModel.h>
#include <lib_vehicle_model/VehicleControlInput.h>
#include <lib_vehicle_model/ParameterServer.h>
 
/**
 * @class PassengerCarKinematicModel
 * @brief Class which implements a very simple kinematic vehicle bicycle model for front wheel drive passenger cars
 * 
 * NOTE: This class does not support trailers at this time. The trailer angle will remain unchanged during integration
 *       Additionally, the lateral velocity will remain unchanged
 */
class PassengerCarKinematicModel: public lib_vehicle_model::VehicleMotionModel
{
  private:

    const size_t ODE_STATE_SIZE = 3; // Number of elements of VehicleState that are accounted for in this model
    const size_t FULL_STATE_SIZE = 12; // Total number of elements in a CARMA VehicleState 

    // Handles to callback functions
    lib_vehicle_model::ODESolver::ODEFunction<lib_vehicle_model::VehicleControlInput, double> ode_func_;
    lib_vehicle_model::ODESolver::PostStepFunction<lib_vehicle_model::VehicleControlInput, double> post_step_func_;
    
    // Parameter server used to load vehicle parameters
    std::shared_ptr<lib_vehicle_model::ParameterServer> param_server_;
    
    // Parameters
    double l_; // Wheel base in m
    double R_ef_; // The vertical distance from the front axle to the ground when the vehicle is loaded. 
    double R_er_; // The vertical distance from the rear axle to the ground when the vehicle is loaded. 

    /*
     * @brief Function describing the ODE system which defines the vehicle equations of motion
     * 
     * This function matches the ODESolver::ODEFunction definition
     */ 
    void KinematicCarODE(const lib_vehicle_model::ODESolver::State& state,
      const lib_vehicle_model::VehicleControlInput& control,
      double& prev_time,
      lib_vehicle_model::ODESolver::StateDot& state_dot,
      double t
    ) const;

    /*
     * @brief Function describing the necessary actions after each ODE integration step
     * 
     * This function matches the ODESolver::PostStepFunction definition.
     * This function is used to populate the state vector elements not used in the ODE
     */ 
    void ODEPostStep(const lib_vehicle_model::ODESolver::State& current,
      const lib_vehicle_model::VehicleControlInput& control,
      double& prev_time,
      double t,
      const lib_vehicle_model::ODESolver::State& initial_state,
      lib_vehicle_model::ODESolver::State& output
    ) const;

  public:

    /**
     * @brief Constructor 
     * 
     */ 
    PassengerCarKinematicModel();

    /**
     * @brief Destructor as required by interface
     * 
     */ 
    ~PassengerCarKinematicModel();

    //
    // Overriden interface functions
    //

    /**
     * @throws std::invalid_argment if not all required parameters could be read
     */ 
    void setParameterServer(std::shared_ptr<lib_vehicle_model::ParameterServer> parameter_server) override;

    std::vector<lib_vehicle_model::VehicleState> predict(const lib_vehicle_model::VehicleState& initial_state,
      double timestep, double delta_t) override; 

    std::vector<lib_vehicle_model::VehicleState> predict(const lib_vehicle_model::VehicleState& initial_state,
      const std::vector<lib_vehicle_model::VehicleControlInput>& control_inputs, double timestep) override;
};