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


      template<class C> // TODO comment
      class ControlWrapper {
        public:
          C control;
      };

      // Functor for use with odeint which wraps the ode function to allow for injection of control constants into the ode function
      template<class C, class T>
      struct ODEFunctor
      {
        const ODEFunction<C,T> ode_function;
        ControlWrapper<C>* control_input_; // TODO To make this work I need the control input to be stored in the post step functor and this to point to that TODO does this should probably be made safer
        T& tracker_;

        ODEFunctor(const ODEFunction<C,T>& ode_func, T& tracker) : ode_function(std::move(ode_func)), tracker_(tracker)
        {}

        void setControlInputPtr(ControlWrapper<C>* control_ptr) {
          control_input_ = control_ptr;
        }

        // Callback for ode function
        void operator()(const State& current, StateDot& output, double t)
        {
          if (control_input_->control.target_velocity > 0) {
         // std::cerr << "Control ODE Vel: " << control_input_->control.target_velocity << " " << control_input_ << std::endl;

          }
          ode_function(current, control_input_->control, tracker_, output, t);
        }
      };


      // Functor for use with odeint which calls the PostStepFunction and accumulates the resulting output data
      template<class C, class T>
      struct PostStepFunctor
      {
        const PostStepFunction<C,T> post_step_function;
        std::vector<C>& control_inputs;
        State prev_final_state;
        std::vector<std::tuple<double, State>>& outputs;
        ODEFunctor<C,T>& ode_functor_obj;
        ControlWrapper<C>& current_control_;

        PostStepFunctor(const PostStepFunction<C, T>& post_step_func, ODEFunctor<C,T>& ode_functor, std::vector<C>& controls, T& tracker, const State& prev_final_state, std::vector<std::tuple<double, State>>& output_vec, ControlWrapper<C>& control_wrapper) :  
          post_step_function(std::move(post_step_func)), control_inputs(controls), prev_final_state(prev_final_state), outputs(output_vec), ode_functor_obj(ode_functor), current_control_(control_wrapper)
        {
          current_control_.control = controls[0]; // Set initial control input
          ode_functor_obj.setControlInputPtr(&current_control_); // Set control address
        }

        // Callback for ode observer during integration
        void operator()(const State& current, double t)
        {
          // If this is the initial odeint callback for our starting condition we don't need to record it
          // TODO commented old code
          if (t == 0) {
            return;
          }

          // Call the post step function
          State updated_state;

          post_step_function(current, control_inputs[outputs.size()], ode_functor_obj.tracker_, t, prev_final_state, updated_state);
          prev_final_state = updated_state;
          //std::cerr << "Actual Next Prev " << prev_final_state[10] << std::endl;
          // Set the final output
          outputs.push_back(std::tuple<double,State>(t, updated_state));

          // Update the control value for the next step
          //std::cerr << "Time: " << t<<std::endl;
          if (control_inputs.size() > outputs.size()) {
            current_control_.control = control_inputs[outputs.size()]; // Update the control input
          }
        }
      };
    }

    //
    // Public Namespace
    //

    template<typename C, typename T>
    void rk4(const ODEFunction<C,T>& ode_func,
      double num_steps,
      double step_size,
      State& initial_state,
      std::vector<C>& controls,
      std::vector<std::tuple<double, State>>& output,
      const PostStepFunction<C,T>& post_step_func,
      T& tracker
    ) {

      boost::numeric::odeint::runge_kutta4<State> solver; // Get RK4 solver
      
      ODEFunctor<C,T> ode(ode_func, tracker); // Build ODE functor
      double start_time = 0; // Set start time

      // Wrapper for the control variable
      ControlWrapper<C> control_wrapper;
      // TODO: The PSF is being copied
      PostStepFunctor<C,T> ps_func(post_step_func, ode, controls, tracker, initial_state, output, control_wrapper); // Build post step functor

      // Intrgrate the function
      boost::numeric::odeint::integrate_n_steps(
        solver, ode, initial_state, 
        start_time, step_size, num_steps, 
        ps_func
      );
    }
  }
}
