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
 * @brief Class which implements a kinematic vehicle bicycle model for front wheel drive passenger cars. This class treats the longitudinal velocity as the total vehicle velocity
 * 
 * This class is based on the kinematic model presented in (R. Rajamani, Vehicle Dynamics and Control. Springer, 2012.)
 * which was reformulated in 
 * (Polack, Philip & Altché, Florent & Novel, Brigitte & de La Fortelle, Arnaud. (2017).
 * The kinematic bicycle model: A consistent model for planning feasible trajectories for autonomous vehicles?. 812-818. 10.1109/IVS.2017.7995816.) 
 * 
 * The forumaltion was modified to compute the acceleration control input based on a proportional response to the error between current and target velocity
 * 
 * NOTE: This class does not support trailers at this time. The trailer angle will remain unchanged during integration
 *       Additionally, the lateral velocity will be set to zero
 */
class PassengerCarKinematicModel: public lib_vehicle_model::VehicleMotionModel
{
  private:

    const size_t ODE_STATE_SIZE = 4; // Number of elements of VehicleState that are accounted for in this model
    const size_t FULL_STATE_SIZE = 12; // Total number of elements in a CARMA VehicleState 

    // Handles to callback functions
    lib_vehicle_model::ODESolver::ODEFunction<lib_vehicle_model::VehicleControlInput, double> ode_func_;
    lib_vehicle_model::ODESolver::PostStepFunction<lib_vehicle_model::VehicleControlInput, double> post_step_func_;
    
    // Parameter server used to load vehicle parameters
    std::shared_ptr<lib_vehicle_model::ParameterServer> param_server_;
    
    // Parameters
    // Initialized with reasonable values but should overwritten with call to setParameterServer
    double l_f_                    = 1.0;   // Distance from front wheel to CG in m
    double l_r_                    = 1.0;   // Distance from rear wheel to CG in m
    double wheel_base_             = 2.0;   // Total wheel base in m
    double ulR_f_                  = 0.34;  // Unloaded radius of front tire in m
    double ulR_r_                  = 0.34;  // Unloaded radius of rear tire in m
    double lR_f_                   = 0.34;  // Loaded radius of front tire in m
    double lR_r_                   = 0.34;  // Loaded radius of rear tire in m
    double R_ef_                   = 0.34;  // The vertical distance from the front axle to the ground when the vehicle is loaded. (m)
    double R_er_                   = 0.34;  // The vertical distance from the rear axle to the ground when the vehicle is loaded. (m)
    double speed_kP_               = 0.8;   // The proportional value used to convert error in current speed into acceleration
    double acceleration_limit_     = 3.0;   // The maximum possible acceleration of the vehicle (m/s^2)
    double deceleration_limit_     = 6.0;   // The maximum possible deceleration of the vehicle (m/s^2)
    double hard_braking_threshold_ = 2.2;   // The speed error required for the controller to enter a hard braking state where is forces maximum deceleration until near the setpoint (m/s)

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

    /**
     * @brief Helper function defines the transfer function which converts new velocity commands into accelerations
     * 
     * @param V The current velocity in m/s
     * @param V_c The new velocity command in m/s
     * 
     * @return the expected acceleration
     */ 
    double predictAccel(const double V, const double V_c) const;
    
    /**
     * @brief Helper function to compute the effective wheel radius from the loaded and unloaded wheel radius.
     * 
     * @param unloaded_radius The unloaded tire radius in m;
     * @param loaded_radius The loaded tire radius in m
     * 
     * @return The effective wheel radius in m;
     */ 
    double computeEffectiveWheelRadius(const double unloaded_radius, const double loaded_radius) const;

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
