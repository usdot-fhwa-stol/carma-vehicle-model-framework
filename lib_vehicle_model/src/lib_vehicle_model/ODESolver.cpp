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
#include <boost/numeric/odeint.hpp>
#include "lib_vehicle_model/ODESolver.h"

// CPP File containing the implementations of the functions in the ODESolver namespace
namespace lib_vehicle_model {
  namespace ODESolver 
  {
    // Private namsepace
    namespace {

      // Functor for use with odeint which wraps the ode function to allow for injection of control constants into the ode function
      template<class C>
      struct ODEFunctor
      {
        const ODEFunction<C> ode_func_;
        C control_;

        ODEFunctor(const ODEFunction<C>& ode_func) : ode_func_(std::move(ode_func))
        {}

        void setControl(const C& control) {
          control_ = control;
        }


        // Callback for ode function
        void operator()(const State& current, StateDot& output, double t)
        {
            ode_func_(current, control_, output, t);
        }
      };

      // Functor for use with odeint which calls the PostStepFunction and accumulates the resulting output data
      template<class C>
      struct PostStepFunctor
      {
        const PostStepFunction<C> post_step_func_;
        const std::vector<C>& controls_;
        const State& initial_state_;
        std::vector<std::tuple<double, State>>& output_vec_;
        ODEFunctor<C>& ode_functor_;


        PostStepFunctor(const PostStepFunction<C>& post_step_func, ODEFunctor<C>& ode_functor, const std::vector<C>& controls, const State& initial_state, std::vector<std::tuple<double, State>>& output_vec) :  
          post_step_func_(std::move(post_step_func)), ode_functor_(ode_functor), controls_(controls), initial_state_(initial_state), output_vec_(output_vec)
        {
          ode_functor_.setControl(controls[0]); // Set initial control
        }

        // Callback for ode observer during integration
        void operator()(const State& current, double t)
        {
          // If this is the initial odeint callback for our starting condition we don't need to record it
          if (t == 0) {
            return;
          }

          // Call the post step function
          State updated_state;

          post_step_func_(current, controls_[output_vec_.size()], t, initial_state_, updated_state);

          // Set the final output
          output_vec_.push_back(std::tuple<double,State>(t, updated_state));

          // Update the control value for the next step
          if (controls_.size() > output_vec_.size()) {
            ode_functor_.setControl(controls_[output_vec_.size()]); // Update the control input
          }
        }
      };
    }

    //
    // Public Namespace
    //

    template<typename C>
    void rk4(const ODEFunction<C>& ode_func,
      double num_steps,
      double step_size,
      State& initial_state,
      const std::vector<C>& controls,
      std::vector<std::tuple<double, State>>& output,
      const PostStepFunction<C>& post_step_func
    ) {

      boost::numeric::odeint::runge_kutta4<State> solver; // Get RK4 solver
      
      ODEFunctor<C> ode(ode_func); // Build ODE functor

      double start_time = 0; // Set start time

      PostStepFunctor<C> ps_func(post_step_func, ode, controls, initial_state, output); // Build post step functor

      // Intrgrate the function
      boost::numeric::odeint::integrate_n_steps(
        solver, ode, initial_state, 
        start_time, step_size, num_steps, 
        ps_func
      );
    }
  }
}
