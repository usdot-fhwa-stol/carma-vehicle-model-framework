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

#include <stdlib.h>
#include <math.h>
#include <sstream>
#include <functional>
#include "passenger_car_dynamic_model/PassengerCarDynamicModel.h"

/**
 * Cpp containing the implementation of PassengerCarDynamicModel
 */

using namespace lib_vehicle_model;

PassengerCarDynamicModel::PassengerCarDynamicModel() {
  // Bind the callback functions
  ode_func_ = std::bind(&PassengerCarDynamicModel::DynamicCarODE, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
  post_step_func_ = std::bind(&PassengerCarDynamicModel::ODEPostStep, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
};

PassengerCarDynamicModel::~PassengerCarDynamicModel() {};

void PassengerCarDynamicModel::setParameterServer(std::shared_ptr<ParameterServer> parameter_server) {
  param_server_ = parameter_server;
  
  // Load Parameters
  bool l_f_param        = param_server_->getParam("length_to_f", l_f_);
  bool l_r_param        = param_server_->getParam("length_to_r", l_r_);
  bool R_f_param        = param_server_->getParam("effective_wheel_radius_f", R_ef_);
  bool R_r_param        = param_server_->getParam("effective_wheel_radius_r", R_er_);
  bool long_stiff_param = param_server_->getParam("tire_longitudinal_stiffness", C_sx_);
  bool lat_stiff_param  = param_server_->getParam("tire_cornering_stiffness", C_ay_);
  bool inertia_param    = param_server_->getParam("moment_of_inertia", I_z_);
  bool mass_param       = param_server_->getParam("vehicle_mass", m_);

  // Check if all the required parameters could be loaded
  if (!(l_f_param && l_r_param && R_f_param && R_r_param
         && long_stiff_param && lat_stiff_param && inertia_param && mass_param)) {

    std::ostringstream msg;
    msg << "One of the required parameters could not be found or read " 
      << " length_to_f: " << l_f_param 
      << " length_to_r: " << l_r_param 
      << " effective_wheel_radius_f: " << R_f_param 
      << " effective_wheel_radius_r: " << R_r_param 
      << " tire_longitudinal_stiffness: " << long_stiff_param
      << " tire_cornering_stiffness: " << lat_stiff_param 
      << " moment_of_inertia: " << inertia_param
      << " vehicle_mass: " << mass_param;

    throw std::invalid_argument(msg.str());
  }
}

std::vector<VehicleState> PassengerCarDynamicModel::predict(const VehicleState& initial_state,
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

std::vector<VehicleState> PassengerCarDynamicModel::predict(const VehicleState& initial_state,
  const std::vector<VehicleControlInput>& controls, double timestep) {
    // Copy control inputs vector to modifieable vector
    std::vector<VehicleControlInput> control_inputs = controls;
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
    state[3]  = initial_state.longitudinal_vel;
    state[4]  = initial_state.lateral_vel;
    state[5]  = initial_state.yaw_rate;
    state[6]  = initial_state.front_wheel_rotation_rate;
    state[7]  = initial_state.rear_wheel_rotation_rate;
    state[8]  = initial_state.steering_angle;

    // Integrate ODE
    double prev_time = 0.0;
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

      if (new_state.size() != 12) {
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

void PassengerCarDynamicModel::DynamicCarODE(const ODESolver::State& state,
  const VehicleControlInput& control,
  double& prev_time,
  ODESolver::StateDot& state_dot,
  double t) const
{
  // Extract control values
  const double d_fc = control.target_steering_angle; // Steering angle commend
  const double V_c = control.target_velocity; // Velocity command

  // Extract the state values.  The data of ompl::base::SE2StateSpace is mapped as:
  // [X, Y, Theta, v_xc, v_yc, r, w_f, w_r, d_f, sigma]
  const  double X     = state[0];
  const  double Y     = state[1];
  const  double Theta = state[2];
         double v_xc  = state[3]; // Must be non-const to account for divide by zero
  const  double v_yc  = state[4];
  const  double r     = state[5];
  const  double w_f   = state[6];
  const  double w_r   = state[7];
  const  double d_f   = state[8];

  // Ensure state_dot is the same size as state.  Zero out all values.
  state_dot.resize(state.size(), 0);

  // Compute state_dot

  // Compute acceleration state
  // A rolling wheel going straight with no slip has a velocity of v = w * R
  const double no_steer_no_slip_vel = w_f * R_ef_;

  // If our ideal speed is lower than our target speed we will consider that to mean we are accelerating
  const bool isAccelerating = no_steer_no_slip_vel <=  V_c;


  // Evaluate 0 value edge cases for v_xc, w_f, and w_r
  const double FLOATING_POINT_EPSILON = 0.000001;

  double F_xf;
  double F_xr;

  double F_yf;
  double F_yr;
  bool non_zero_force_f = true;
  bool non_zero_force_r = true;

  if (abs(v_xc) < FLOATING_POINT_EPSILON) {
    // If we have no forward velocity with no slip then force is 0
    if (abs(w_f) < FLOATING_POINT_EPSILON) {
      F_xf = 0.0;
      F_yf = 0.0;
      non_zero_force_f = false;
    } else { // Assume minute forward velocity to make math work
      v_xc = FLOATING_POINT_EPSILON;
    }

    if (abs(w_r) < FLOATING_POINT_EPSILON) {
      F_xr = 0.0;
      F_yr = 0.0;
      non_zero_force_r = false;
    } else { // Assume minute forward velocity to make math work
      v_xc = FLOATING_POINT_EPSILON;
    }
  }

  // Compute forces
  if (non_zero_force_f) {
    // Compute longitudinal slip ratio
    const double sigma_f = isAccelerating ?
      (no_steer_no_slip_vel - v_xc) / v_xc :   // When accelerating
      (no_steer_no_slip_vel - v_xc) / (no_steer_no_slip_vel); // When braking
    // Compute lateral slip angle
    const double a_f = atan((v_yc + r * l_f_) / v_xc) + d_f;

    F_xf = C_sx_ * sigma_f;
    F_yf = -C_ay_ * a_f;
  }

  if (non_zero_force_r) {
    // Compute longitudinal slip ratio
    const double sigma_r = isAccelerating ?
      (R_er_ * w_r - v_xc) / v_xc :   // When accelerating
      (R_er_ * w_r - v_xc) / (R_er_ * w_r); // When braking
    // Compute lateral slip angle
    const double a_r = atan((v_yc - r * l_r_) / v_xc);

    F_xr = C_sx_ * sigma_r;
    F_yr = -C_ay_ * a_r;
  }

  const double cos_Theta = cos(Theta);
  const double sin_Theta = sin(Theta);
  const double cos_d_f = cos(d_f);
  const double sin_d_f = sin(d_f);
  
  // Compute state_dot
  state_dot[0] = v_xc * cos_Theta - v_yc * sin_Theta;                                   // X-dot
  state_dot[1] = v_xc * sin_Theta + v_yc * cos_Theta;                                   // Y-dot
  state_dot[2] = r;                                                                     // Theta-dot
  state_dot[3] = ((F_xf * cos_d_f + F_xr - F_yf * sin_d_f) / m_) + r * v_yc;            // v_xc-dot
  state_dot[4] = ((F_yf * cos_d_f + F_yr + F_xf * sin_d_f) / m_) - r * v_xc;            // v_yc-dot
  state_dot[5] = (l_f_ * F_yf * cos_d_f + l_f_ * F_xf * sin_d_f - l_r_ * F_yr) / I_z_;  // r-dot
  state_dot[6] = funcW_f(w_f, w_r, V_c);                                                // w_f-dot
  state_dot[7] = funcW_r(w_f, w_r, V_c);                                                // w_r-dot
  state_dot[8] = funcD_f(d_f, d_fc);                                                    // d_f-dot
}


void PassengerCarDynamicModel::ODEPostStep(const ODESolver::State& current, const VehicleControlInput& control, double& prev_time, double t, const ODESolver::State& initial_state, ODESolver::State& output) const {
  // Copy state contents
  output = current;
  output.resize(FULL_STATE_SIZE);

  // Copy over un-simulated values
  output[9]  = 0;
  output[10] = control.target_steering_angle;
  output[11] = control.target_velocity;
}

double PassengerCarDynamicModel::funcW_f(const double w_f, const double w_r, const double V_c) const {
  return 0; // TODO Testing needs to be conducted on each vehicle to fill out this portion of the model
}

double PassengerCarDynamicModel::funcW_r(const double w_f, const double w_r, const double V_c) const {
  return 0; // TODO Testing needs to be conducted on each vehicle to fill out this portion of the model
}

double PassengerCarDynamicModel::funcD_f(const double d_f, const double d_fc) const {
  return 0; // TODO Testing needs to be conducted on each vehicle to fill out this portion of the model
}

