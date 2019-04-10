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
#include <stddef.h>
#include <functional>

namespace lib_vehicle_model {
  /**
   * @namespace ODESolver
   * @brief A namspace containing functions which can be used to solve sets of ordinary differential equations using numerical methods
   */
  namespace ODESolver {
    
    /**
     * @brief Type alias for ODE State vectors
     */ 
    using State = std::vector<double>;
    /**
     * @brief Type alias for ODE State derivative vectors
     */ 
    using StateDot = std::vector<double>;

    /**
     * @brief Type alias for a function which describes the first order ODEs
     * 
     * @tparam C The data type of the control variable
     *  
     * @param state The vector of values which define the initial state. Each element of this state must match an element of the state_dot vector
     * @param control An optional control object whose contents will be treated as constants during a step of integration
     * @param state_dot The vector which contains the current rates of change of the state vector with respect to t
     * @param t The independant variable which the derivatives are derived against such as dx/dt.
     */
    template<typename C>
    using ODEFunction = std::function<void(const State& state, const C& control, StateDot& state_dot, double t)>;

    /**
     * @brief Type alias for a function which will be called after each integration step. 
     * 
     * This function can be used to set state variables which are not being considered during integration
     * 
     * @tparam C The data type of the control variable
     *  
     * @param current The vector of values which define the initial state. Each element of this state must match an element of the state_dot vector
     * @param control A control object whose contents will be treated as constants during a step of integration
     * @param t The value of the independant variable which the derivatives are derived against such as dx/dt.
     * @param initial_state The initial state of the whole integration process
     * @param output The updated state vector which will be stored in the final integration output
     */
    template<typename C>
    using PostStepFunction = std::function<void(const State& current, const C& control, double t, const State& initial_state, State& output)>;

    /**
     * @brief Solve ODEs using Runge-Kutta 4th Order Integration
     * 
     * @tparam C The data type of the control variable
     * 
     * @param num_steps The number of samples which will be returned. 
     * @param step_size The step size between independant variable samples of the ODE. 
     * @param initial_state The vector of values which define the initial state. The initial condition is defined as (0, initial_state).
     * @param controls A list of controls which will be applied as a constant during each integration step. If the list is shorter than the integration size the last element will be used for the remainder of the integration
     * @param output A list of output states seperated by step_size with an added length equal to num_steps. Elements of the list are tuples of (independant variable, state)
     * @param post_step_fun A function which will be called after each integration step. This function can be used to set state variables which are not being considered during integration
     */

    template<typename C>
    void rk4(const ODEFunction<C>& ode_func,
      double num_steps,
      double step_size,
      State& initial_state,
      const std::vector<C>& controls,
      std::vector<std::tuple<double, State>>& output,
      const PostStepFunction<C>& post_step_func
    );
  }
}

// Template functions cannot be linked unless the implementation is provided
// Therefore include implementation to allow for template functions
#include "../../src/lib_vehicle_model/ODESolver.cpp"
