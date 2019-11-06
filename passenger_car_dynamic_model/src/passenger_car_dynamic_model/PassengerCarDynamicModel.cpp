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
#include "passenger_car_dynamic_model/TwoStepPID.h"
#include "passenger_car_dynamic_model/PassengerCarDynamicModel.h"
#include <algorithm>

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
  bool l_f_param               = param_server_->getParam("length_to_f", l_f_);
  bool l_r_param               = param_server_->getParam("length_to_r", l_r_);
  bool ulR_f_param             = param_server_->getParam("unloaded_wheel_radius_f", ulR_f_);
  bool ulR_r_param             = param_server_->getParam("unloaded_wheel_radius_r", ulR_r_);
  bool lR_f_param              = param_server_->getParam("loaded_wheel_radius_f", lR_f_);
  bool lR_r_param              = param_server_->getParam("loaded_wheel_radius_r", lR_r_);
  bool f_long_stiff_param      = param_server_->getParam("tire_longitudinal_stiffness_f", C_sxf_);
  bool r_long_stiff_param      = param_server_->getParam("tire_longitudinal_stiffness_r", C_sxr_);
  bool f_lat_stiff_param       = param_server_->getParam("tire_cornering_stiffness_f", C_ayf_);
  bool r_lat_stiff_param       = param_server_->getParam("tire_cornering_stiffness_r", C_ayr_);
  bool inertia_param           = param_server_->getParam("moment_of_inertia", I_z_);
  bool mass_param              = param_server_->getParam("vehicle_mass", m_);
  bool steer_p_param           = param_server_->getParam("steering_kP", steer_P_);
  bool steer_i_param           = param_server_->getParam("steering_kI", steer_I_);
  bool steer_d_param           = param_server_->getParam("steering_kD", steer_D_);
  bool wheel_p_param           = param_server_->getParam("wheel_kP", wheel_P_);
  bool wheel_i_param           = param_server_->getParam("wheel_kI", wheel_I_);
  bool wheel_d_param           = param_server_->getParam("wheel_kD", wheel_D_);
  bool max_steering_rate_param = param_server_->getParam("max_steering_rate", max_steering_rate_);
  bool top_speed_param         = param_server_->getParam("top_speed", top_speed_);
  //bool low_speed_lag_param     = param_server_->getParam("low_speed_acceleration_response_lag", low_speed_acceleration_response_lag_);
  bool low_speed_lag_param = true;
  // Check if all the required parameters could be loaded
  if (!(l_f_param && l_r_param && ulR_f_param && ulR_r_param
         && lR_f_param && lR_r_param
         && f_long_stiff_param && r_long_stiff_param && f_lat_stiff_param && r_lat_stiff_param 
         && inertia_param && mass_param 
         && steer_p_param && steer_i_param && steer_d_param
         && wheel_p_param && wheel_i_param && wheel_d_param
         && max_steering_rate_param && top_speed_param && low_speed_lag_param
      )) {

    std::ostringstream msg;
    msg << "One of the required parameters could not be found or read " 
      << " length_to_f: " << l_f_param 
      << " length_to_r: " << l_r_param 
      << " unloaded_wheel_radius_f: " << ulR_f_param 
      << " unloaded_wheel_radius_r: " << ulR_r_param 
      << " loaded_wheel_radius_f: " << lR_f_param 
      << " loaded_wheel_radius_r: " << lR_r_param
      << " tire_longitudinal_stiffness_f: " << f_long_stiff_param
      << " tire_longitudinal_stiffness_r: " << r_long_stiff_param
      << " tire_cornering_stiffness_f: " << f_lat_stiff_param 
      << " tire_cornering_stiffness_r: " << r_lat_stiff_param 
      << " moment_of_inertia: " << inertia_param
      << " vehicle_mass: " << mass_param
      << " steering_kP: " << steer_p_param
      << " steering_kI: " << steer_i_param
      << " steering_kD: " << steer_d_param
      << " wheel_kP: " << wheel_p_param
      << " wheel_kI: " << wheel_i_param
      << " wheel_kD: " << wheel_d_param
      << " max_steering_rate" << max_steering_rate_param
      << " top_speed: " << top_speed_param;
 //     << " low_speed_acceleration_response_lag" << low_speed_lag_param;

    throw std::invalid_argument(msg.str());
  }

  // If params are valid compute the max wheel rotation rate
  max_wheel_rotation_rate_ = top_speed_ / R_ef_; // (m/s) / radius = rad/s // TODO I actually need max wheel angular acceleration
  // Compute the effective wheel radius
  R_ef_ = computeEffectiveWheelRadius(ulR_f_, lR_f_);
  R_er_ = computeEffectiveWheelRadius(ulR_r_, lR_r_);
  std::cerr <<"Ref " << R_ef_ << " Rer " << R_er_ << std::endl;

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
  const std::vector<VehicleControlInput>& control_inputs, double timestep) {
    auto mutable_control_inputs = control_inputs;
    // Construct output vector
    std::vector<VehicleState> resulting_states;
    resulting_states.reserve(control_inputs.size());

    // Construct ode output vector
    std::vector<std::tuple<double, ODESolver::State>> ode_outputs;
    ode_outputs.reserve(control_inputs.size());

    // Tracker used to update PID control loops during integration
    // Vector description 
    // pid_tracker[0] = steer_pid
    // pid_tracker[1] = front_wheel_pid

    TwoStepPID steer_pid(steer_P_, steer_I_, steer_D_);
    steer_pid.setOutputMax(max_steering_rate_);
    steer_pid.setOutputMin(-max_steering_rate_);

    TwoStepPID front_wheel_pid(wheel_P_, wheel_I_, wheel_D_);
    front_wheel_pid.setOutputMax(6.13); // TODO this used to say max wheel rotation rate however is should say max wheel angular acceleration
    front_wheel_pid.setOutputMin(-6.13); // This model does not predict motion in reverse

    TwoStepLagModel tslm(0,0);
    TrackerType pid_tracker = std::make_tuple(steer_pid, front_wheel_pid, tslm);


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
    ODESolver::rk4<VehicleControlInput, TrackerType>(
      ode_func_,
      control_inputs.size(),
      timestep,
      state,
      mutable_control_inputs,
      ode_outputs,
      post_step_func_,
      pid_tracker
    );

    // Convert result to target output
    for (size_t j = 0; j < ode_outputs.size(); j++) {
      const ODESolver::State new_state = std::get<1>(ode_outputs[j]);

      if (new_state.size() != FULL_STATE_SIZE) {
        throw std::invalid_argument("ODESolver result is Too small");
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
  const VehicleControlInput& control, TrackerType& pid_tracker,
  ODESolver::StateDot& state_dot,
  double t) const
{

  // Extract control values
  const double d_fc = control.target_steering_angle; // Steering angle commend
  const double V_c = control.target_velocity; // Velocity command

  // Extract the state values.
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
  // TODO these values
  double B = 7.1;
  double C = 1.4;
  double D = 0.8;
  double E = 1.01;

  // Compute forces
  if (non_zero_force_f) {
    // Compute longitudinal slip ratio
    const double sigma_f = isAccelerating ?
      (no_steer_no_slip_vel - v_xc) / v_xc :   // When accelerating
      (no_steer_no_slip_vel - v_xc) / (no_steer_no_slip_vel); // When braking
    // Compute lateral slip angle
    const double a_f = atan((v_yc + r * l_f_) / v_xc) + d_f;

    double F_zf = 0.5538 * m_ * 9.80665;
    //F_xf = C_sxf_ * sigma_f; // TODO this is the function that was replaced with the magic formula
    double slip = -((R_ef_ * w_f) - v_xc)  / fabs(v_xc);
    F_xf = F_zf * D * sin(C * atan(B*slip - E*(B*slip - atan(B*slip))));
    F_yf = -C_ayf_ * a_f;
  }

  if (non_zero_force_r) {
    // Compute longitudinal slip ratio
    const double rear_no_steer_no_slip_vel = R_er_ * w_r;
    const double sigma_r = isAccelerating ?
      (rear_no_steer_no_slip_vel - v_xc) / v_xc :   // When accelerating
      (rear_no_steer_no_slip_vel - v_xc) / (rear_no_steer_no_slip_vel); // When braking
    // Compute lateral slip angle
    const double a_r = atan((v_yc - r * l_r_) / v_xc);
    double F_zr = 0.4462 * m_ * 9.80665;

    double slip = -((R_er_ * w_r) - v_xc)  / fabs(v_xc);
    //F_xr = C_sxr_ * sigma_r; // TODO this is the function that was replaced with the magic formula
    F_xr = F_zr * D * sin(C * atan(B*slip - E*(B*slip - atan(B*slip))));
    F_yr = -C_ayr_ * a_r;
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
  state_dot[6] = funcW_f(w_f, w_r, V_c, t, pid_tracker);                                // w_f-dot
  state_dot[7] = funcW_r(w_f, w_r, V_c, t, pid_tracker);                                // w_r-dot
  state_dot[8] = funcD_f(d_f, d_fc, t, pid_tracker);                                    // d_f-dot
}


void PassengerCarDynamicModel::ODEPostStep(const ODESolver::State& current, const VehicleControlInput& control, TrackerType& pid_tracker,
  double t, const ODESolver::State& prev_final_state, ODESolver::State& output) const {
  // Copy state contents
  output = current;
  output.resize(FULL_STATE_SIZE);

  // Update steer PID
  // TODO uncomment
  // std::get<STEER_PID>(pid_tracker).setSetpoint(control.target_steering_angle);
  // std::get<STEER_PID>(pid_tracker).updateMemory(current[8], t);

  // Update front wheel PID
  //std::cerr << "Cmd Speed: " << control.target_velocity << " Setpoint: " << control.target_velocity / R_ef_ << " Radius: " << R_ef_ << std::endl;
  // TODO remove
  // double wheel_speed_setpoint = control.target_velocity / R_ef_;
  // double maxError = 5.0;
  // double frac = fabs(wheel_speed_setpoint - current[6]) / fabs(maxError);
  // double minFrac = std::min(1.0,frac);
  // std::get<WHEEL_PID>(pid_tracker).setConstants(wheel_P_ * minFrac, wheel_I_,  wheel_D_);
  std::get<WHEEL_PID>(pid_tracker).setSetpoint(control.target_velocity / R_ef_);
  std::get<WHEEL_PID>(pid_tracker).updateMemory(current[6], t);

  // Copy over un-simulated values
  output[9]  = 0;
  output[10] = control.target_steering_angle;
  //std::cerr << "Prev " << prev_final_state[10]  << " Expected Next Prev " << output[10] << std::endl;
  output[11] = control.target_velocity;
}

double PassengerCarDynamicModel::funcW_f(const double w_f, const double w_r, const double V_c, const double t, TrackerType& pid_tracker) const {
  // TODO: This seems to work well enough to evaluate for predicting rotation rate. Move onto computing speed
  double current_speed = R_ef_ * w_f;
  double kP = wheel_P_;
  bool braking_hard = current_speed > (V_c + 2.2); // If current speed is greater than the velocity command + 2.2 m/s
  double max_value = 8.13;
  if (braking_hard)  {
    std::cerr << "Hard braking" << std::endl;
    kP = kP * 2.0; // Increase value of kP by 10%
    max_value = 12.0;
    // NOTE: It seems that the check for hard braking will disconnect when we get near the setpoint
    // This is expected based on the boolean. But the data does not mimick this behavior. 
    // I have confirmed that the integrator does NOT force the output to reach the setpoint at the end of the prediction TODO comment cleanup
  }

  double P = kP * ((V_c / R_ef_) - w_f);

  return std::min(std::max(P, -max_value), max_value);//funcW_f(w_f, w_r, V_c, t, pid_tracker);  // TODO get these max output values set correctly
  // std::get<WHEEL_PID>(pid_tracker).setSetpoint(V_c / R_ef_);
  // return std::get<WHEEL_PID>(pid_tracker).computeOutput(w_f, t);
}

double PassengerCarDynamicModel::funcW_r(const double w_f, const double w_r, const double V_c, const double t, TrackerType& pid_tracker) const {
  return funcW_f(w_f, w_r, V_c, t, pid_tracker); // TODO should use different wheel radius: Assuming that engine torque is distributed evenly between front and rear axel
}

double PassengerCarDynamicModel::funcD_f(const double d_f, const double d_fc, const double t, TrackerType& pid_tracker) const {

  std::get<STEER_PID>(pid_tracker).setSetpoint(d_fc);
  return std::get<STEER_PID>(pid_tracker).computeOutput(d_f, t);
}

/**
 * Math based on figure 3.35 in R. N. Jazar, "Vehicle dynamics: theory and application" Springer, 2008.
 */ 
inline double PassengerCarDynamicModel::computeEffectiveWheelRadius(const double unloaded_radius, const double loaded_radius) const {
  return unloaded_radius - ((unloaded_radius - loaded_radius) / 3.0);
}

