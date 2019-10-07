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

#include <stdlib.h>
#include <math.h>
#include <sstream>
#include <functional>
#include "passenger_car_kinematic_model/PassengerCarKinematicModel.h"

/**
 * Cpp containing the implementation of PassengerCarKinematicModel
 */

using namespace lib_vehicle_model;

PassengerCarKinematicModel::PassengerCarKinematicModel() {
  // Bind the callback functions
  ode_func_ = std::bind(&PassengerCarKinematicModel::KinematicCarODE, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
  post_step_func_ = std::bind(&PassengerCarKinematicModel::ODEPostStep, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
};

PassengerCarKinematicModel::~PassengerCarKinematicModel() {};

void PassengerCarKinematicModel::setParameterServer(std::shared_ptr<ParameterServer> parameter_server) {
  param_server_ = parameter_server;
  
  // Load Parameters
  bool l_param   = param_server_->getParam("wheel_base", l_);
  bool R_f_param = param_server_->getParam("effective_wheel_radius_f", R_ef_);
  bool R_r_param = param_server_->getParam("effective_wheel_radius_r", R_er_);

  // Check if all the required parameters could be loaded
  if (!(l_param && R_f_param && R_r_param)) {

    std::ostringstream msg;
    msg << "One of the required parameters could not be found or read " 
      << " wheel_base: " << l_param
      << " effective_wheel_radius_f: " << R_f_param 
      << " effective_wheel_radius_r: " << R_r_param;

    throw std::invalid_argument(msg.str());
  }
}

std::vector<VehicleState> PassengerCarKinematicModel::predict(const VehicleState& initial_state,
  double timestep, double delta_t) {

    // Populate control inputs
    // This predict function takes in no new control inputs so extract the old ones from the state vector
    VehicleControlInput control_input;
    control_input.target_steering_angle = initial_state.prev_steering_cmd;
    control_input.target_velocity = initial_state.prev_vel_cmd;

    // Ensure we run at least 1 step
    size_t num_steps;
    if (delta_t <= timestep) {
      num_steps = 1;
    } else {
      num_steps = delta_t / timestep;
    }

    std::vector<VehicleControlInput> control_inputs(num_steps, control_input);

    // Call default predict method
    return predict(initial_state, control_inputs, timestep);
  }

std::vector<VehicleState> PassengerCarKinematicModel::predict(const VehicleState& initial_state,
  const std::vector<VehicleControlInput>& control_inputs, double timestep) {
        
    // Construct output vector
    std::vector<VehicleState> resulting_states;
    resulting_states.reserve(control_inputs.size());

    // Construct ode output vector
    std::vector<std::tuple<double, ODESolver::State>> ode_outputs;
    ode_outputs.reserve(control_inputs.size());

    // Populate initial condition
    ODESolver::State state(ODE_STATE_SIZE, 0);
    state[0]  = initial_state.X_pos_global;
    state[1]  = initial_state.Y_pos_global;
    state[2]  = initial_state.orientation;

    double prev_time = 0.0;

    // STATE
    // x,y, theta

    // Integrate ODE
    ODESolver::rk4<VehicleControlInput, double>(
      ode_func_,
      control_inputs.size(),
      timestep,
      state,
      control_inputs,
      ode_outputs,
      post_step_func_,
      prev_time
    );

    // Convert result to target output
    for (size_t j = 0; j < ode_outputs.size(); j++) {
      const ODESolver::State new_state = std::get<1>(ode_outputs[j]);

      if (new_state.size() != FULL_STATE_SIZE) {
        throw std::invalid_argument("Too small");
      }
      // Save result
      VehicleState result;
      result.X_pos_global              = new_state[0];
      result.Y_pos_global              = new_state[1];
      result.orientation               = new_state[2];
      result.longitudinal_vel          = new_state[3];
      result.lateral_vel               = new_state[4];
      result.yaw_rate                  = new_state[5];
      result.front_wheel_rotation_rate = new_state[6];
      result.rear_wheel_rotation_rate  = new_state[7];
      result.steering_angle            = new_state[8];
      result.trailer_angle             = initial_state.trailer_angle;
      result.prev_steering_cmd         = new_state[10];
      result.prev_vel_cmd              = new_state[11];

      resulting_states.push_back(result);
    }

    return resulting_states;
  }

void PassengerCarKinematicModel::KinematicCarODE(const lib_vehicle_model::ODESolver::State& state,
    const lib_vehicle_model::VehicleControlInput& control,
    double& prev_time,
    lib_vehicle_model::ODESolver::StateDot& state_dot,
    double t
  ) const
{
  // Extract control values
  const double d_fc = control.target_steering_angle; // Steering angle commend
  const double V_c = control.target_velocity; // Velocity command

  // State = [X, Y, Theta]
  const  double X     = state[0];
  const  double Y     = state[1];
  const  double Theta = state[2];

  // Ensure state_dot is the same size as state.  Zero out all values.
  state_dot.resize(state.size(), 0);
  
  // Compute state_dot
  state_dot[0] = V_c * cos(Theta);        // X-dot
  state_dot[1] = V_c * sin(Theta);        // Y-dot
  state_dot[2] = (V_c / l_) * tan(d_fc);  // Theta-dot

}

void PassengerCarKinematicModel::ODEPostStep(const lib_vehicle_model::ODESolver::State& current,
    const lib_vehicle_model::VehicleControlInput& control,
    double& prev_time,
    double t,
    const lib_vehicle_model::ODESolver::State& prev_state,
    lib_vehicle_model::ODESolver::State& output
  ) const
{
  // Copy state contents
  output = current;
  output.resize(FULL_STATE_SIZE);

  // Copy over unstimulated values
  double dt = t - prev_time;
  output[3]  = control.target_velocity; // This model assumes command velocity change is instantaneous
  output[4]  = 0; // This model assumes no lateral velocity
  output[5]  = dt == 0.0 ? 0.0 : (current[2] - prev_state[2]) / dt; // Yaw rate equals dTheta/dt
  output[6]  = output[3] / R_ef_; // Assume no slip. Velocity / radius = rotation rate
  output[7]  = output[3] / R_er_; // Assume no slip. 
  output[8]  = control.target_steering_angle; // This model assumes instantaneous steering change
  output[9]  = 0; // Model ignores trailer angle
  output[10] = control.target_steering_angle;
  output[11] = control.target_velocity;

  // Update tracker
  prev_time = t;
}

