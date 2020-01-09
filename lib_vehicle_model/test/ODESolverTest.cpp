/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <gtest/gtest.h>
#include <cmath>
#include "lib_vehicle_model/ODESolver.h"

using namespace lib_vehicle_model;

/**
 * Tests the rk4 solver function of the ODESolver
 */ 
TEST(ODESOlver, rk4)
{
  
  std::vector<double> control_inputs(5, 0);

  ODESolver::State initial_state;
  initial_state.push_back(0);
  initial_state.push_back(1);

  std::vector<std::tuple<double, ODESolver::State>> ode_outputs;
  double timestep = 0.1;

  // ODE Defined as
  // x[2]_dot = 4e^(.8t) - 0.5x[0]
  // x[1]_dot = 4e^(0.8t) - 3x[1]

  // Solution generated with MATLAB ode45 solver

  // Integrate ODE
  int tracker = 0;
  ODESolver::rk4<double,int>(
    [this](const ODESolver::State& state, const double& control, int tracker, ODESolver::StateDot& state_dot, const double t) -> void {
      state_dot[0] = 4 * exp(0.8*t) - 0.5*state[0];
      state_dot[1] = 4 * exp(0.8*t) - 3*state[1];
    },
    control_inputs.size(),
    timestep,
    initial_state,
    control_inputs,
    ode_outputs,
    [this](const ODESolver::State& current, const double& control, int tracker, const double t, const ODESolver::State& initial_state, ODESolver::State& output) -> void {
      output = current;
    },
    tracker
  );


  // Expected Result
  std::vector<std::tuple<double, ODESolver::State>> expected_result;
  ODESolver::State r0 = {0.40633, 1.10131};
  ODESolver::State r1 = {0.82669, 1.20639};
  ODESolver::State r2 = {1.26320, 1.31676};
  ODESolver::State r3 = {1.71814, 1.43376};
  ODESolver::State r4 = {2.19392, 1.55860};
  std::tuple<double, ODESolver::State> t0(0.10000, r0);
  std::tuple<double, ODESolver::State> t1(0.20000, r1);
  std::tuple<double, ODESolver::State> t2(0.30000, r2);
  std::tuple<double, ODESolver::State> t3(0.40000, r3);
  std::tuple<double, ODESolver::State> t4(0.50000, r4);
  expected_result.push_back(t0);
  expected_result.push_back(t1);
  expected_result.push_back(t2);
  expected_result.push_back(t3);
  expected_result.push_back(t4);

  // Check matching size
  ASSERT_EQ(expected_result.size(), ode_outputs.size());

  // Check matching data
  int i = 0;
  for (auto tu: expected_result) {
    ASSERT_NEAR(std::get<0>(tu), std::get<0>(ode_outputs[i]), 0.000001);
    ASSERT_NEAR(std::get<1>(tu)[0], std::get<1>(ode_outputs[i])[0], 0.00001);
    ASSERT_NEAR(std::get<1>(tu)[1], std::get<1>(ode_outputs[i])[1], 0.00001);
    i++;
  }

}
